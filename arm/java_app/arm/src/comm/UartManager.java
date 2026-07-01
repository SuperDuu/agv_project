package comm;

import com.fazecast.jSerialComm.SerialPort;
import java.util.ArrayList;
import java.util.List;

public class UartManager {

    private SerialPort port;

    public UartManager() {
    }

    public static List<String> getAvailablePorts() {
        List<String> portList = new ArrayList<>();
        try {
            SerialPort[] ports = SerialPort.getCommPorts();
            for (SerialPort p : ports) {
                portList.add(p.getSystemPortName());
            }
        } catch (Throwable t) {
            System.err.println("Warning: Could not get serial ports: " + t.getMessage());
        }
        return portList;
    }

    public boolean connect(String portName, int baudRate) {
        port = SerialPort.getCommPort(portName);
        port.setComPortParameters(baudRate, 8, 1, 0);
        port.setComPortTimeouts(SerialPort.TIMEOUT_NONBLOCKING, 0, 0);

        if (port.openPort()) {
            System.out.println("Connected to STM32 on " + portName + " at " + baudRate + " baud.");
            return true;
        } else {
            System.out.println("Failed to open " + portName);
            return false;
        }
    }

    public void disconnect() {
        if (port != null && port.isOpen()) {
            port.closePort();
            System.out.println("Disconnected from STM32.");
        }
    }

    public boolean sendData(String data) {
        if (port != null && port.isOpen()) {
            try {
                byte[] buffer = data.getBytes();
                int written = port.writeBytes(buffer, buffer.length);
                if (written < 0) {
                    System.err.println("UART Write Error: Connection lost. Closing port.");
                    disconnect();
                    return false;
                }
                return written == buffer.length;
            } catch (Exception e) {
                System.err.println("UART Write Exception: " + e.getMessage() + ". Closing port.");
                disconnect();
                return false;
            }
        }
        return false;
    }

    public boolean isConnected() {
        return port != null && port.isOpen();
    }
}
