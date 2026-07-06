package comm;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.nio.charset.StandardCharsets;
import java.util.Locale;
import java.util.UUID;

public class Ros2BridgeClient {
    private final String host;
    private final int port;
    private final int timeoutMs;

    public Ros2BridgeClient() {
        this(
                System.getenv().getOrDefault("AGV_ROS2_HOST", "127.0.0.1"),
                parseInt(System.getenv().getOrDefault("AGV_ROS2_PORT", "5010"), 5010),
                parseInt(System.getenv().getOrDefault("AGV_ROS2_TIMEOUT_MS", "1500"), 1500));
    }

    public Ros2BridgeClient(String host, int port, int timeoutMs) {
        this.host = host;
        this.port = port;
        this.timeoutMs = timeoutMs;
    }

    public String requestPlanPose(String arm, double x, double y, double z) throws Exception {
        try (DatagramSocket socket = new DatagramSocket()) {
            socket.setSoTimeout(timeoutMs);
            String requestId = UUID.randomUUID().toString();
            int replyPort = socket.getLocalPort();
            String payload = String.format(Locale.US,
                    "{"
                            + "\"type\":\"plan_pose\","
                            + "\"request_id\":\"%s\","
                            + "\"arm\":\"%s\","
                            + "\"target\":{\"x\":%.4f,\"y\":%.4f,\"z\":%.4f},"
                            + "\"reply_host\":\"host.docker.internal\","
                            + "\"reply_port\":%d"
                            + "}",
                    escape(requestId), escape(arm), x, y, z, replyPort);

            byte[] bytes = payload.getBytes(StandardCharsets.UTF_8);
            DatagramPacket packet = new DatagramPacket(bytes, bytes.length, InetAddress.getByName(host), port);
            socket.send(packet);

            byte[] buffer = new byte[65535];
            DatagramPacket response = new DatagramPacket(buffer, buffer.length);
            socket.receive(response);
            return new String(response.getData(), 0, response.getLength(), StandardCharsets.UTF_8);
        }
    }

    private static int parseInt(String value, int fallback) {
        try {
            return Integer.parseInt(value.trim());
        } catch (Exception ignored) {
            return fallback;
        }
    }

    private static String escape(String value) {
        return value == null ? "" : value.replace("\\", "\\\\").replace("\"", "\\\"");
    }
}
