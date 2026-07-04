package comm;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;

public class UartManager {

    private Object port;
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
                return true;
            }
            System.out.println("Failed to open " + portName);
        } catch (Throwable t) {
            System.err.println("Serial connect error: " + t.getMessage());
        }
        return false;
    }

    public void disconnect() {
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

    public boolean sendData(String data) {
        if (port != null) {
            try {
                boolean isOpen = (boolean) serialPortClass.getMethod("isOpen").invoke(port);
                if (isOpen) {
                    byte[] buffer = data.getBytes();
                    int written = (int) serialPortClass.getMethod("writeBytes", byte[].class, int.class)
                            .invoke(port, buffer, buffer.length);
                    if (written < 0) {
                        System.err.println("UART Write Error: Connection lost. Closing port.");
                        disconnect();
                        return false;
                    }
                    return written == buffer.length;
                }
            } catch (Throwable t) {
                System.err.println("UART Write Exception: " + t.getMessage() + ". Closing port.");
                disconnect();
                return false;
            }
        }
        return false;
    }

    public boolean isConnected() {
        if (port == null || serialPortClass == null) {
            return false;
        }
        try {
            return (boolean) serialPortClass.getMethod("isOpen").invoke(port);
        } catch (Throwable t) {
            return false;
        }
    }
}
