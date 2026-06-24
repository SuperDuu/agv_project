package utils;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

/**
 * Utility class to save robot workspace points to a CSV file.
 */
public class WorkspaceLogger {
    private final String fileName = "workspace_points.csv";
    private PrintWriter writer;

    /**
     * Initializes the logger. Deletes any existing file and starts a new one with a
     * CSV header.
     */
    public void init() {
        try {
            // FileWriter(fileName, false) ensures the file is overwritten
            writer = new PrintWriter(new FileWriter(fileName, false));
            writer.println("X,Y,Z");
        } catch (IOException e) {
            System.err.println("Error initializing WorkspaceLogger: " + e.getMessage());
        }
    }

    /**
     * Appends a coordinate point to the CSV file.
     */
    public synchronized void logPoint(double x, double y, double z) {
        if (writer != null) {
            writer.printf("%.2f,%.2f,%.2f\n", x, y, z);
        }
    }

    /**
     * Closes the file writer.
     */
    public void close() {
        if (writer != null) {
            writer.flush();
            writer.close();
            writer = null;
        }
    }
}
