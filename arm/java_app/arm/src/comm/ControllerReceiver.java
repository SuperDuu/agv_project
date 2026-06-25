package comm;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.util.Arrays;

public class ControllerReceiver implements Runnable {
    private final int port;
    private volatile boolean running = true;
    private DatagramSocket socket;
    private Thread thread;

    // Latest received values (thread-safe, read-only from outside)
    private final double[] axes = new double[16];
    private final int[] buttons = new int[32];
    private final int[] hats = new int[4];
    private volatile long lastPacketTime = 0;
    
    private gui.MainFrame mainFrame;

    public void setMainFrame(gui.MainFrame mainFrame) {
        this.mainFrame = mainFrame;
    }

    public ControllerReceiver(int port) {
        this.port = port;
    }

    public void start() {
        thread = new Thread(this, "ControllerReceiverThread");
        thread.setDaemon(true);
        thread.start();
    }

    public void stop() {
        running = false;
        if (socket != null) {
            socket.close();
        }
    }

    @Override
    public void run() {
        try {
            socket = new DatagramSocket(port);
            byte[] buffer = new byte[2048];
            while (running) {
                DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
                try {
                    socket.receive(packet);
                    String message = new String(packet.getData(), 0, packet.getLength(), "UTF-8");
                    parseMessage(message);
                    lastPacketTime = System.currentTimeMillis();
                } catch (IOException e) {
                    if (!running) break;
                    System.err.println("Error receiving controller packet: " + e.getMessage());
                }
            }
        } catch (SocketException e) {
            System.err.println("ControllerReceiver DatagramSocket error: " + e.getMessage());
        }
    }

    private void parseMessage(String message) {
        try {
            if (message.startsWith("PICK:")) {
                String[] tokens = message.substring(5).split(",");
                if (tokens.length >= 3) {
                    double px = Double.parseDouble(tokens[0].trim());
                    double py = Double.parseDouble(tokens[1].trim());
                    double pz = Double.parseDouble(tokens[2].trim());
                    if (mainFrame != null) {
                        mainFrame.triggerPickAndPlace(px, py, pz);
                    }
                }
                return;
            } else if (message.contains("\"pick\":")) {
                int startBracket = message.indexOf("[", message.indexOf("\"pick\":"));
                int endBracket = message.indexOf("]", startBracket);
                if (startBracket != -1 && endBracket != -1) {
                    String[] tokens = message.substring(startBracket + 1, endBracket).split(",");
                    if (tokens.length >= 3) {
                        double px = Double.parseDouble(tokens[0].trim());
                        double py = Double.parseDouble(tokens[1].trim());
                        double pz = Double.parseDouble(tokens[2].trim());
                        if (mainFrame != null) {
                            mainFrame.triggerPickAndPlace(px, py, pz);
                        }
                    }
                }
                return;
            }

            // Very simple JSON parser for:
            // {"axes": [x, y, ...], "buttons": [b0, b1, ...], "hats": [h0, h1]}
            
            // 1. Extract axes
            int axesStart = message.indexOf("\"axes\":");
            if (axesStart != -1) {
                int startBracket = message.indexOf("[", axesStart);
                int endBracket = message.indexOf("]", startBracket);
                if (startBracket != -1 && endBracket != -1) {
                    String raw = message.substring(startBracket + 1, endBracket);
                    String[] tokens = raw.split(",");
                    synchronized (axes) {
                        Arrays.fill(axes, 0.0);
                        for (int i = 0; i < Math.min(tokens.length, axes.length); i++) {
                            String t = tokens[i].trim();
                            if (!t.isEmpty()) {
                                axes[i] = Double.parseDouble(t);
                            }
                        }
                    }
                }
            }

            // 2. Extract buttons
            int buttonsStart = message.indexOf("\"buttons\":");
            if (buttonsStart != -1) {
                int startBracket = message.indexOf("[", buttonsStart);
                int endBracket = message.indexOf("]", startBracket);
                if (startBracket != -1 && endBracket != -1) {
                    String raw = message.substring(startBracket + 1, endBracket);
                    String[] tokens = raw.split(",");
                    synchronized (buttons) {
                        Arrays.fill(buttons, 0);
                        for (int i = 0; i < Math.min(tokens.length, buttons.length); i++) {
                            String t = tokens[i].trim();
                            if (!t.isEmpty()) {
                                buttons[i] = Integer.parseInt(t);
                            }
                        }
                    }
                }
            }

            // 3. Extract hats
            int hatsStart = message.indexOf("\"hats\":");
            if (hatsStart != -1) {
                int startBracket = message.indexOf("[", hatsStart);
                int endBracket = message.indexOf("]", startBracket);
                if (startBracket != -1 && endBracket != -1) {
                    String raw = message.substring(startBracket + 1, endBracket);
                    String[] tokens = raw.split(",");
                    synchronized (hats) {
                        Arrays.fill(hats, 0);
                        for (int i = 0; i < Math.min(tokens.length, hats.length); i++) {
                            String t = tokens[i].trim();
                            if (!t.isEmpty()) {
                                hats[i] = Integer.parseInt(t);
                            }
                        }
                    }
                }
            }
        } catch (Exception e) {
            // Silently ignore malformed packets
        }
    }

    // Thread-safe accessors
    public double[] getAxes() {
        double[] copy = new double[axes.length];
        synchronized (axes) {
            System.arraycopy(axes, 0, copy, 0, axes.length);
        }
        return copy;
    }

    public int[] getButtons() {
        int[] copy = new int[buttons.length];
        synchronized (buttons) {
            System.arraycopy(buttons, 0, copy, 0, buttons.length);
        }
        return copy;
    }

    public int[] getHats() {
        int[] copy = new int[hats.length];
        synchronized (hats) {
            System.arraycopy(hats, 0, copy, 0, hats.length);
        }
        return copy;
    }

    public boolean isConnected() {
        return (System.currentTimeMillis() - lastPacketTime) < 1500; // Timeout after 1.5s
    }
}
