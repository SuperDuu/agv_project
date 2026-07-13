package comm;

import java.io.BufferedReader;
import java.io.File;
import java.io.InputStreamReader;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.nio.charset.StandardCharsets;

/**
 * Manages the ROS2 Docker container lifecycle from within the Java app.
 *
 * On startup:
 *   1. Checks if the container is already running (docker ps).
 *   2. If not, runs `docker compose up -d` to start it in the background.
 *   3. Waits until the UDP bridge on port 5010 responds (ping).
 *
 * On shutdown (registered as JVM shutdown hook):
 *   4. Sends `docker stop agv_ros2_hybrid` to cleanly stop the container.
 *
 * Configuration via environment variables (same as Ros2BridgeClient):
 *   AGV_ROS2_HOST         default 127.0.0.1
 *   AGV_ROS2_PORT         default 5010
 *   AGV_ROS2_COMPOSE_DIR  path to the ros2_hybrid directory (default: auto-detected relative to working dir)
 */
public class Ros2DockerManager {

    private static final String CONTAINER_NAME = "agv_ros2_hybrid";
    private static final int PING_TIMEOUT_MS = 1000;
    private static final int PING_MAX_RETRIES = 120; // wait up to 120s (includes colcon build time)
    private static final int PING_INTERVAL_MS = 1000;

    private final String composeDir;
    private final String host;
    private final int port;

    // Singleton
    private static Ros2DockerManager instance;

    public static synchronized Ros2DockerManager getInstance() {
        if (instance == null) {
            instance = new Ros2DockerManager();
        }
        return instance;
    }

    private Ros2DockerManager() {
        this.host = System.getenv().getOrDefault("AGV_ROS2_HOST", "127.0.0.1");
        this.port = parseInt(System.getenv().getOrDefault("AGV_ROS2_PORT", "5010"), 5010);

        // Auto-detect compose dir: try env var first, then relative paths
        String envDir = System.getenv("AGV_ROS2_COMPOSE_DIR");
        if (envDir != null && new File(envDir, "docker-compose.yml").exists()) {
            this.composeDir = envDir;
        } else {
            // Try common relative paths from the working directory
            String cwd = System.getProperty("user.dir");
            String[] candidates = {
                cwd + File.separator + "ros2_hybrid",
                cwd + File.separator + ".." + File.separator + "ros2_hybrid",
                cwd + File.separator + ".." + File.separator + ".." + File.separator + "ros2_hybrid",
                "C:\\Users\\DELL\\agv_project\\ros2_hybrid"
            };
            String found = null;
            for (String candidate : candidates) {
                File f = new File(candidate, "docker-compose.yml");
                if (f.exists()) {
                    found = new File(candidate).getAbsolutePath();
                    break;
                }
            }
            this.composeDir = found;
        }
    }

    /**
     * Start the ROS2 container if not already running.
     * Blocks until the UDP bridge is ready (or times out after ~60 seconds).
     * Call this from the app's startup code (e.g., after MainFrame is shown).
     *
     * @return true if the bridge is reachable after startup
     */
    public boolean ensureRunning() {
        if (composeDir == null) {
            System.err.println("[ROS2Docker] Cannot find ros2_hybrid/docker-compose.yml. Set AGV_ROS2_COMPOSE_DIR env var.");
            return false;
        }

        System.out.println("[ROS2Docker] Compose dir: " + composeDir);

        // 1. Check if container already running
        if (isContainerRunning()) {
            System.out.println("[ROS2Docker] Container '" + CONTAINER_NAME + "' already running.");
        } else {
            System.out.println("[ROS2Docker] Starting ROS2 container via docker compose...");
            if (!startContainer()) {
                System.err.println("[ROS2Docker] Failed to start container.");
                return false;
            }
        }

        // 2. Register shutdown hook to stop container when JVM exits
        registerShutdownHook();

        // 3. Wait for UDP bridge to respond
        System.out.println("[ROS2Docker] Waiting for ROS2 bridge on " + host + ":" + port + " ...");
        boolean ready = waitForBridge();
        if (ready) {
            System.out.println("[ROS2Docker] ROS2 bridge is READY.");
        } else {
            System.err.println("[ROS2Docker] ROS2 bridge did NOT respond after " + PING_MAX_RETRIES + "s.");
        }
        return ready;
    }

    /**
     * Returns true if the agv_ros2_hybrid container is currently running.
     */
    public boolean isContainerRunning() {
        try {
            Process p = new ProcessBuilder("docker", "ps", "--filter",
                    "name=" + CONTAINER_NAME, "--filter", "status=running", "--format", "{{.Names}}")
                    .redirectErrorStream(true)
                    .start();
            String output = readAll(p);
            p.waitFor();
            return output.contains(CONTAINER_NAME);
        } catch (Exception e) {
            System.err.println("[ROS2Docker] isContainerRunning error: " + e.getMessage());
            return false;
        }
    }

    /**
     * Runs `docker compose up -d` in the compose directory to start the container.
     */
    private boolean startContainer() {
        try {
            ProcessBuilder pb = new ProcessBuilder("docker", "compose", "up", "-d")
                    .directory(new File(composeDir))
                    .redirectErrorStream(true);
            Process p = pb.start();
            // Stream output to console
            Thread logThread = new Thread(() -> {
                try (BufferedReader br = new BufferedReader(
                        new InputStreamReader(p.getInputStream()))) {
                    String line;
                    while ((line = br.readLine()) != null) {
                        System.out.println("[ROS2Docker] " + line);
                    }
                } catch (Exception ignored) {}
            });
            logThread.setDaemon(true);
            logThread.start();
            int exitCode = p.waitFor();
            logThread.join(3000);
            return exitCode == 0;
        } catch (Exception e) {
            System.err.println("[ROS2Docker] startContainer error: " + e.getMessage());
            return false;
        }
    }

    /**
     * Polls the UDP bridge with a ping packet until it responds or timeout.
     */
    private boolean waitForBridge() {
        byte[] pingPayload = "{\"type\":\"ping\"}".getBytes(StandardCharsets.UTF_8);
        for (int i = 0; i < PING_MAX_RETRIES; i++) {
            try (DatagramSocket socket = new DatagramSocket()) {
                socket.setSoTimeout(PING_TIMEOUT_MS);
                DatagramPacket packet = new DatagramPacket(
                        pingPayload, pingPayload.length,
                        InetAddress.getByName(host), port);
                socket.send(packet);
                byte[] buf = new byte[256];
                socket.receive(new DatagramPacket(buf, buf.length));
                return true; // got any response = ready
            } catch (Exception ignored) {
                // No response yet
            }
            try {
                Thread.sleep(PING_INTERVAL_MS);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return false;
            }
            if (i % 5 == 4) {
                System.out.println("[ROS2Docker] Still waiting... (" + (i + 1) + "s)");
            }
        }
        return false;
    }

    /**
     * Stops the container. Called by the shutdown hook.
     */
    public void stopContainer() {
        System.out.println("[ROS2Docker] Stopping container '" + CONTAINER_NAME + "'...");
        try {
            Process p = new ProcessBuilder("docker", "stop", CONTAINER_NAME)
                    .redirectErrorStream(true)
                    .start();
            readAll(p);
            p.waitFor();
            System.out.println("[ROS2Docker] Container stopped.");
        } catch (Exception e) {
            System.err.println("[ROS2Docker] stopContainer error: " + e.getMessage());
        }
    }

    private void registerShutdownHook() {
        Runtime.getRuntime().addShutdownHook(new Thread(() -> {
            if (isContainerRunning()) {
                stopContainer();
            }
        }, "ros2-docker-shutdown"));
    }

    private static String readAll(Process p) throws Exception {
        StringBuilder sb = new StringBuilder();
        try (BufferedReader br = new BufferedReader(
                new InputStreamReader(p.getInputStream()))) {
            String line;
            while ((line = br.readLine()) != null) {
                sb.append(line).append("\n");
            }
        }
        return sb.toString();
    }

    private static int parseInt(String value, int fallback) {
        try {
            return Integer.parseInt(value.trim());
        } catch (Exception ignored) {
            return fallback;
        }
    }
}
