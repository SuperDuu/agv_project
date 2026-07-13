package comm;

import java.lang.reflect.Method;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;

/**
 * UartManager — serial port abstraction (jSerialComm via reflection).
 *
 * Protocol V2.1 frame builder:
 *   [SOF1][SOF2][DEST][SRC][LEN_L][LEN_H][CMD][SEQ][PAYLOAD...][CRC_L][CRC_H]
 *   All multi-byte fields: Little-Endian.
 *   LEN = payload byte count.
 *   CRC-16/CCITT-FALSE covers bytes [DEST .. last PAYLOAD byte].
 */
public class UartManager {

    // -----------------------------------------------------------------------
    // Protocol V2.1 constants
    // -----------------------------------------------------------------------
    private static final int SOF1 = 0xAA;
    private static final int SOF2 = 0x55;

    /** Node addresses */
    public static final int ADDR_MAIN      = 0x01;
    public static final int ADDR_ARM_LEFT  = 0x02;
    public static final int ADDR_ARM_RIGHT = 0x03;
    public static final int ADDR_PC_APP    = 0x20;

    /** Command IDs */
    public static final int CMD_ARM_JOINT   = 0x20;
    public static final int CMD_ARM_GRIPPER = 0x21;

    /** Motion modes */
    public static final int MOTION_ABSOLUTE = 1;
    public static final int MOTION_ESTOP    = 4;

    /** Arm flags */
    public static final int ARM_FLAG_VALID = 0x01;

    /**
     * Default max delta per frame at 50 Hz → 3.00°/frame ≈ 150°/s.
     * Slave firmware drops frame if any joint exceeds this.
     */
    public static final int DEFAULT_MAX_DELTA_X100 = 300;

    private static final int FRAME_OVERHEAD = 10; // 2 SOF + DEST + SRC + 2 LEN + CMD + SEQ + 2 CRC

    // -----------------------------------------------------------------------
    // Sequence counter (wraps 0..255)
    // -----------------------------------------------------------------------
    private int seqCounter = 0;

    private int nextSeq() {
        int s = seqCounter;
        seqCounter = (seqCounter + 1) & 0xFF;
        return s;
    }

    // -----------------------------------------------------------------------
    // CRC-16/CCITT-FALSE
    // poly=0x1021, init=0xFFFF, refin=false, refout=false, xorout=0x0000
    // -----------------------------------------------------------------------
    private static int crc16(byte[] data, int offset, int length) {
        int crc = 0xFFFF;
        for (int i = offset; i < offset + length; i++) {
            crc ^= (data[i] & 0xFF) << 8;
            for (int bit = 0; bit < 8; bit++) {
                crc = ((crc & 0x8000) != 0) ? ((crc << 1) ^ 0x1021) : (crc << 1);
                crc &= 0xFFFF;
            }
        }
        return crc;
    }

    // -----------------------------------------------------------------------
    // Frame builder
    // -----------------------------------------------------------------------

    /**
     * Builds a complete Protocol V2.1 frame.
     *
     * @param dest       destination node address (ADDR_ARM_LEFT / ADDR_ARM_RIGHT)
     * @param cmd        command ID (CMD_ARM_JOINT, etc.)
     * @param payload    payload bytes (Little-Endian encoded)
     * @return complete frame as byte[]
     */
    private byte[] buildFrame(int dest, int cmd, byte[] payload) {
        int payloadLen = (payload != null) ? payload.length : 0;
        byte[] frame = new byte[FRAME_OVERHEAD + payloadLen];

        frame[0] = (byte) SOF1;
        frame[1] = (byte) SOF2;
        frame[2] = (byte) dest;
        frame[3] = (byte) ADDR_PC_APP;
        // LEN at [4:5], Little-Endian
        frame[4] = (byte) (payloadLen & 0xFF);
        frame[5] = (byte) ((payloadLen >> 8) & 0xFF);
        frame[6] = (byte) cmd;
        frame[7] = (byte) nextSeq();

        if (payloadLen > 0) {
            System.arraycopy(payload, 0, frame, 8, payloadLen);
        }

        // CRC covers: DEST, SRC, LEN_L, LEN_H, CMD, SEQ, PAYLOAD
        // = bytes [2 .. 8 + payloadLen - 1] = 6 + payloadLen bytes
        int crc = crc16(frame, 2, 6 + payloadLen);
        frame[8 + payloadLen]     = (byte) (crc & 0xFF);         // CRC_L
        frame[8 + payloadLen + 1] = (byte) ((crc >> 8) & 0xFF);  // CRC_H

        return frame;
    }

    /**
     * Builds a Protocol V2.1 Arm Joint Command frame (CMD = 0x20, 22-byte payload).
     *
     * Payload layout (all Little-Endian):
     *   [0]     motion_mode
     *   [1]     arm_flags
     *   [2..3]  q1_x100   (int16 LE)
     *   [4..5]  q2_x100
     *   [6..7]  q3_x100
     *   [8..9]  q4_x100
     *   [10..11] q5_x100
     *   [12..13] q6_x100
     *   [14..15] move_time_ms (uint16 LE)
     *   [16..17] max_delta_x100 (uint16 LE)
     *   [18..21] reserved = 0
     *
     * Note: arm_id has been removed. DEST field (0x02 / 0x03) identifies the arm.
     *
     * @param dest           ADDR_ARM_LEFT or ADDR_ARM_RIGHT
     * @param qActuator_deg  6-element actuator-space angles in degrees
     * @param maxDeltaX100   Slave-side Δθ guard (x100 fixed-point), e.g. 300 = 3.00°
     * @return complete binary frame ready to send over UART
     */
    public byte[] buildArmJointFrame(int dest, double[] qActuator_deg, int maxDeltaX100) {
        ByteBuffer payload = ByteBuffer.allocate(22);
        payload.order(ByteOrder.LITTLE_ENDIAN);

        payload.put((byte) MOTION_ABSOLUTE); // motion_mode
        payload.put((byte) ARM_FLAG_VALID);  // arm_flags

        for (int i = 0; i < 6; i++) {
            int x100 = degToX100(qActuator_deg[i]);
            payload.putShort((short) x100);  // q_x100 [LE]
        }

        payload.putShort((short) 0);             // move_time_ms = 0 (use arm default)
        payload.putShort((short) maxDeltaX100);  // max_delta_x100

        // reserved [18..21]
        payload.putInt(0);

        return buildFrame(dest, CMD_ARM_JOINT, payload.array());
    }

    /**
     * Convenience overload using the default max-delta (300 = 3.00°/frame at 50 Hz).
     */
    public byte[] buildArmJointFrame(int dest, double[] qActuator_deg) {
        return buildArmJointFrame(dest, qActuator_deg, DEFAULT_MAX_DELTA_X100);
    }

    /**
     * Builds a Protocol V2.1 Arm Gripper Command frame (CMD = 0x21, 4-byte payload).
     *
     * Payload layout:
     *   [0] grip_action: 0=release, 1=grip, 2=toggle
     *   [1..3] reserved = 0
     */
    public byte[] buildArmGripperFrame(int dest, int gripAction) {
        ByteBuffer payload = ByteBuffer.allocate(4);
        payload.order(ByteOrder.LITTLE_ENDIAN);
        payload.put((byte) (gripAction & 0xFF));
        payload.put((byte) 0);
        payload.put((byte) 0);
        payload.put((byte) 0);
        return buildFrame(dest, CMD_ARM_GRIPPER, payload.array());
    }

    /**
     * Converts a floating-point degree value to int16 x100 fixed-point,
     * using round-half-up (not truncation).
     */
    private static int degToX100(double deg) {
        return (int) Math.round(deg * 100.0);
    }

    // -----------------------------------------------------------------------
    // Serial port (jSerialComm via reflection for optional dependency)
    // -----------------------------------------------------------------------
    private volatile Object port;
    private final boolean serialAvailable;
    private final Class<?> serialPortClass;

    public UartManager() {
        Class<?> clazz = null;
        boolean available = false;
        try {
            clazz = Class.forName("com.fazecast.jSerialComm.SerialPort");
            available = true;
        } catch (Throwable t) {
            System.err.println("Warning: jSerialComm is not available: " + t.getMessage());
        }
        this.serialPortClass = clazz;
        this.serialAvailable = available;
    }

    public static List<String> getAvailablePorts() {
        List<String> portList = new ArrayList<>();
        try {
            Class<?> clazz = Class.forName("com.fazecast.jSerialComm.SerialPort");
            Object ports = clazz.getMethod("getCommPorts").invoke(null);
            Object[] arr = (Object[]) ports;
            for (Object p : arr) {
                portList.add((String) clazz.getMethod("getSystemPortName").invoke(p));
            }
        } catch (Throwable t) {
            System.err.println("Warning: Could not get serial ports: " + t.getMessage());
        }
        return portList;
    }

    private final java.util.concurrent.BlockingQueue<byte[]> writeQueue = new java.util.concurrent.LinkedBlockingQueue<>();
    private Thread writeThread;
    private volatile boolean writeThreadRunning = false;

    private void startWriteThread() {
        writeQueue.clear();
        writeThreadRunning = true;
        writeThread = new Thread(() -> {
            while (writeThreadRunning && isConnected()) {
                try {
                    byte[] data = writeQueue.take();
                    if (port != null) {
                        try {
                            int written = (int) serialPortClass.getMethod("writeBytes", byte[].class, int.class)
                                    .invoke(port, data, data.length);
                            if (written < 0) {
                                System.err.println("UART Write Error: Connection lost. Closing port.");
                                disconnect();
                                break;
                            }
                        } catch (Throwable t) {
                            System.err.println("UART Write Exception: " + t.getMessage() + ". Closing port.");
                            disconnect();
                            break;
                        }
                    }
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }
            writeThreadRunning = false;
        });
        writeThread.setDaemon(true);
        writeThread.setName("UART-Write-Thread");
        writeThread.start();
    }

    public boolean connect(String portName, int baudRate) {
        if (!serialAvailable || serialPortClass == null) {
            System.err.println("Serial library not available. Cannot connect to " + portName);
            return false;
        }
        try {
            port = serialPortClass.getMethod("getCommPort", String.class).invoke(null, portName);
            serialPortClass.getMethod("setComPortParameters", int.class, int.class, int.class, int.class)
                    .invoke(port, baudRate, 8, 1, 0);
            serialPortClass.getMethod("setComPortTimeouts", int.class, int.class, int.class)
                    .invoke(port, serialPortClass.getField("TIMEOUT_NONBLOCKING").getInt(null), 0, 0);

            boolean opened = (boolean) serialPortClass.getMethod("openPort").invoke(port);
            if (opened) {
                System.out.println("Connected to STM32 on " + portName + " at " + baudRate + " baud.");
                startWriteThread();
                return true;
            }
            System.out.println("Failed to open " + portName);
        } catch (Throwable t) {
            System.err.println("Serial connect error: " + t.getMessage());
        }
        return false;
    }

    public void disconnect() {
        writeThreadRunning = false;
        if (writeThread != null) {
            writeThread.interrupt();
            writeThread = null;
        }
        writeQueue.clear();
        if (port != null) {
            try {
                boolean isOpen = (boolean) serialPortClass.getMethod("isOpen").invoke(port);
                if (isOpen) {
                    serialPortClass.getMethod("closePort").invoke(port);
                    System.out.println("Disconnected from STM32.");
                }
            } catch (Throwable t) {
                System.err.println("Serial disconnect error: " + t.getMessage());
            }
        }
    }

    private boolean isImportantCommand(byte[] data) {
        if (data == null || data.length == 0) return false;
        String str = new String(data).trim();
        if (str.contains("ESTOP") || str.contains("GRIP") || str.contains("RELEASE")) {
            return true;
        }
        if (data.length >= 8 && (data[0] & 0xFF) == SOF1 && (data[1] & 0xFF) == SOF2) {
            int cmd = data[6] & 0xFF;
            if (cmd == CMD_ARM_GRIPPER) {
                return true;
            }
        }
        return false;
    }

    /**
     * Sends a raw byte array over the serial port by adding it to the background writer queue.
     * Used for Protocol V2.1 binary frames.
     *
     * @return true if added to queue successfully
     */
    public boolean sendBytes(byte[] data) {
        if (port == null || !isConnected()) return false;
        
        // Prevent queue pile-up (latency) if serial is slow/congested.
        if (writeQueue.size() > 5) {
            java.util.List<byte[]> temp = new java.util.ArrayList<>();
            writeQueue.drainTo(temp);
            for (byte[] item : temp) {
                if (isImportantCommand(item)) {
                    writeQueue.offer(item);
                }
            }
        }
        
        writeQueue.offer(data);
        return true;
    }

    /**
     * Sends a text string over the serial port.
     * Kept for legacy compatibility (config commands, ESTOP text, etc.).
     */
    public boolean sendData(String data) {
        return sendBytes(data.getBytes());
    }

    public boolean isConnected() {
        if (port == null || serialPortClass == null) return false;
        try {
            return (boolean) serialPortClass.getMethod("isOpen").invoke(port);
        } catch (Throwable t) {
            return false;
        }
    }
}
