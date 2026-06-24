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
        System.setProperty("os.arch_full", "x86_64");
        System.setProperty("jSerialComm.library.path", "C:/Users/DELL/Documents/NetBeans/BTL/lib/native");
        System.setProperty("java.io.tmpdir", "C:/Users/DELL/Documents/NetBeans/BTL/uart_temp");

        try {
            UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
        } catch (ClassNotFoundException | IllegalAccessException | InstantiationException
                | UnsupportedLookAndFeelException ignored) {
        }

        SwingUtilities.invokeLater(() -> {
            new gui.MainFrame().setVisible(true);
        });
    }
}
