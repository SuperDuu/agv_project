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
        System.setProperty("os.arch_full", "x86_64");
        System.setProperty("jSerialComm.library.path", appDir + "/lib/native");

        // Create uart_temp directory if it doesn't exist
        java.io.File tmpDir = new java.io.File(appDir, "uart_temp");
        if (!tmpDir.exists()) {
            tmpDir.mkdirs();
        }
        System.setProperty("java.io.tmpdir", tmpDir.getAbsolutePath());

        try {
            UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
        } catch (ClassNotFoundException | IllegalAccessException | InstantiationException
                | UnsupportedLookAndFeelException ignored) {
        }

        SwingUtilities.invokeLater(() -> {
            new gui.MainFrame().setVisible(true);

            // Tự động khởi chạy script Python điều khiển PS5
            try {
                ProcessBuilder pb = new ProcessBuilder("python", "scripts/ps5_controller.py");
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
                System.out.println("Đã khởi chạy nền script ps5_controller.py thành công.");
            } catch (Exception ex) {
                System.err.println("Không thể khởi chạy ps5_controller.py: " + ex.getMessage());
                System.err.println("Hãy chắc chắn Python đã được cài đặt và thêm vào biến môi trường PATH.");
            }
        });
    }
}
