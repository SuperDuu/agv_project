/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Main.java to edit this template
 */
package app;

import javax.swing.*;

/**
 *
 * @author DELL
 */
public class App {

    /**
     * @param args the command line arguments
     */
    public static void main(String[] args) {
        // Resolve paths relative to the application's working directory (portable)
        String appDir = System.getProperty("user.dir");

        // Prefer the bundled native folder only when it contains this OS' binary.
        // Otherwise let jSerialComm fall back to its built-in native extraction/loading.
        java.io.File nativeDir = new java.io.File(appDir, "lib/native");
        java.io.File nativeFile = new java.io.File(nativeDir, getJSerialCommNativeName());
        if (nativeFile.isFile()) {
            System.setProperty("jSerialComm.library.path", nativeDir.getAbsolutePath());
        }

        // Create uart_temp directory if it doesn't exist
        java.io.File tmpDir = new java.io.File(appDir, "uart_temp");
        if (!tmpDir.exists()) {
            tmpDir.mkdirs();
        }
        System.setProperty("java.io.tmpdir", tmpDir.getAbsolutePath());
        System.out.println("IK solver mode: " + getIkSolverModeName());

        try {
            UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
        } catch (ClassNotFoundException | IllegalAccessException | InstantiationException
                | UnsupportedLookAndFeelException ignored) {
        }

        SwingUtilities.invokeLater(() -> {
            new gui.MainFrame().setVisible(true);

            // Tự động khởi chạy script Python điều khiển PS5
            try {
                String[] pythonCommand = findPythonCommand();
                java.util.List<String> command = new java.util.ArrayList<>();
                java.util.Collections.addAll(command, pythonCommand);
                command.add("scripts/ps5_controller.py");

                ProcessBuilder pb = new ProcessBuilder(command);
                pb.directory(new java.io.File(appDir));
                pb.redirectErrorStream(true); // Gộp lỗi và output để không bị nghẽn buffer
                Process process = pb.start();
                
                // Tiêu diệt tiến trình Python khi tắt ứng dụng Java
                Runtime.getRuntime().addShutdownHook(new Thread(() -> {
                    if (process != null && process.isAlive()) {
                        System.out.println("Đang tắt tiến trình PS5 Controller...");
                        process.destroyForcibly();
                    }
                }));

                // Đọc luồng output của Python để chống nghẽn bộ đệm (Buffer Overflow) gây treo script
                new Thread(() -> {
                    try (java.io.BufferedReader reader = new java.io.BufferedReader(new java.io.InputStreamReader(process.getInputStream()))) {
                        String line;
                        while ((line = reader.readLine()) != null) {
                            // Chỉ dùng khi cần debug script Python
                            // System.out.println("[PS5 Python] " + line);
                        }
                    } catch (Exception e) {}
                }).start();

                System.out.println("Đã khởi chạy nền script ps5_controller.py thành công.");
            } catch (Exception ex) {
                System.err.println("Không thể khởi chạy ps5_controller.py: " + ex.getMessage());
                System.err.println("Hãy chắc chắn Python/Python3 đã được cài đặt và thêm vào biến môi trường PATH.");
                System.err.println("Có thể đặt biến môi trường AGV_PYTHON trỏ tới file python nếu cần.");
            }
        });
    }

    private static String[] findPythonCommand() {
        String configured = System.getenv("AGV_PYTHON");
        if (configured != null && configured.trim().length() > 0) {
            return new String[] { configured.trim() };
        }

        String[][] candidates = isWindows()
                ? new String[][] { { "python" }, { "py", "-3" }, { "python3" } }
                : new String[][] { { "python3" }, { "python" } };

        for (String[] candidate : candidates) {
            if (canRun(candidate)) {
                return candidate;
            }
        }
        return candidates[0];
    }

    private static boolean canRun(String[] baseCommand) {
        java.util.List<String> command = new java.util.ArrayList<>();
        java.util.Collections.addAll(command, baseCommand);
        command.add("--version");

        try {
            Process process = new ProcessBuilder(command).redirectErrorStream(true).start();
            return process.waitFor() == 0;
        } catch (java.io.IOException ex) {
            return false;
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
            return false;
        }  
    }

    private static boolean isWindows() {
        return System.getProperty("os.name", "").toLowerCase(java.util.Locale.ROOT).contains("win");
    }

    private static boolean isMac() {
        return System.getProperty("os.name", "").toLowerCase(java.util.Locale.ROOT).contains("mac");
    }

    private static String getJSerialCommNativeName() {
        if (isWindows()) {
            return "jSerialComm.dll";
        }
        if (isMac()) {
            return "libjSerialComm.dylib";
        }
        return "libjSerialComm.so";
    }

    private static String getIkSolverModeName() {
        return switch (kinematics.Kinematics.solverMode) {
            case 1 -> "C++ JNI Numerical";
            case 2 -> "C++ JNI IKFast";
            default -> "Java Numerical";
        };
    }
}
