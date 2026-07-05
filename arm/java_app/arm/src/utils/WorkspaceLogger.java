package utils;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Locale;

/**
 * Saves reachable workspace samples to a CSV file that can later be used as a
 * trajectory fallback map.
 */
public class WorkspaceLogger {
    public static final String DEFAULT_FILE_NAME = "workspace_points.csv";
    public static final String MODEL_VERSION = "arm6dof_v1";

    public static final String HEADER = String.join(",",
            "model_version",
            "arm",
            "x",
            "y",
            "z",
            "q1",
            "q2",
            "q3",
            "q4",
            "q5",
            "q6",
            "alpha",
            "yaw_offset",
            "gripper_mode",
            "config",
            "reach_class",
            "pos_error",
            "joint_margin",
            "manipulability");

    private final String fileName;
    private PrintWriter writer;

    public WorkspaceLogger() {
        this(DEFAULT_FILE_NAME);
    }

    public WorkspaceLogger(String fileName) {
        this.fileName = fileName;
    }

    /**
     * Initializes the logger. Existing data is overwritten.
     */
    public void init() {
        try {
            writer = new PrintWriter(new FileWriter(fileName, false));
            writer.println(HEADER);
        } catch (IOException e) {
            System.err.println("Error initializing WorkspaceLogger: " + e.getMessage());
        }
    }

    /**
     * Backward-compatible point logging for callers that only know XYZ.
     */
    public synchronized void logPoint(double x, double y, double z) {
        double[] unknownQ = { Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN };
        logRecord("?", x, y, z, unknownQ, Double.NaN, Double.NaN, "UNKNOWN", "?",
                0, Double.NaN, Double.NaN, Double.NaN);
    }

    public synchronized void logRecord(
            String arm,
            double x,
            double y,
            double z,
            double[] qDeg,
            double alphaDeg,
            double yawOffsetDeg,
            String gripperMode,
            String config,
            int reachClass,
            double posError,
            double jointMargin,
            double manipulability) {
        if (writer == null || qDeg == null || qDeg.length < 6) {
            return;
        }

        writer.printf(Locale.US,
                "%s,%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%s,%s,%d,%.6f,%.6f,%.9f%n",
                MODEL_VERSION,
                csv(arm),
                x,
                y,
                z,
                qDeg[0],
                qDeg[1],
                qDeg[2],
                qDeg[3],
                qDeg[4],
                qDeg[5],
                alphaDeg,
                yawOffsetDeg,
                csv(gripperMode),
                csv(config),
                reachClass,
                posError,
                jointMargin,
                manipulability);
    }

    public void close() {
        if (writer != null) {
            writer.flush();
            writer.close();
            writer = null;
        }
    }

    private static String csv(String value) {
        if (value == null) {
            return "";
        }
        String escaped = value.replace("\"", "\"\"");
        if (escaped.indexOf(',') >= 0 || escaped.indexOf('"') >= 0 || escaped.indexOf('\n') >= 0) {
            return "\"" + escaped + "\"";
        }
        return escaped;
    }
}
