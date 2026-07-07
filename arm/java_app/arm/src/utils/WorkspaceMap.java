package utils;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Read-only lookup table for reachable workspace samples exported by
 * WorkspaceLogger.
 */
public class WorkspaceMap {
    private final List<Entry> entries;

    private WorkspaceMap(List<Entry> entries) {
        this.entries = entries;
    }

    public static WorkspaceMap loadDefault() throws IOException {
        return load(new File(WorkspaceLogger.DEFAULT_FILE_NAME));
    }

    public static WorkspaceMap load(File file) throws IOException {
        List<Entry> entries = new ArrayList<>();
        if (file == null || !file.isFile()) {
            return new WorkspaceMap(entries);
        }

        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
            String headerLine = reader.readLine();
            if (headerLine == null) {
                return new WorkspaceMap(entries);
            }

            List<String> headers = parseCsvLine(headerLine);
            Map<String, Integer> headerIndex = new HashMap<>();
            for (int i = 0; i < headers.size(); i++) {
                headerIndex.put(headers.get(i), i);
            }

            String line;
            while ((line = reader.readLine()) != null) {
                if (line.trim().isEmpty()) {
                    continue;
                }
                Entry entry = parseEntry(parseCsvLine(line), headerIndex);
                if (entry != null) {
                    entries.add(entry);
                }
            }
        }

        return new WorkspaceMap(entries);
    }

    public int size() {
        return entries.size();
    }

    public Entry findBestReplacement(
            double x,
            double y,
            double z,
            boolean isRight,
            String gripperMode,
            double[] qRef,
            double maxDistance,
            double maxJointJump) {
        String arm = isRight ? "R" : "L";
        Entry best = null;
        double bestCost = Double.MAX_VALUE;

        for (Entry entry : entries) {
            if (!WorkspaceLogger.MODEL_VERSION.equals(entry.modelVersion)) {
                continue;
            }
            if (!arm.equals(entry.arm)) {
                continue;
            }
            if (gripperMode != null && !gripperMode.equals(entry.gripperMode) && !"ANY".equals(entry.gripperMode)) {
                continue;
            }
            if (entry.reachClass <= 0 || !entry.hasJointSolution()) {
                continue;
            }

            double dist = entry.distanceTo(x, y, z);
            if (dist > maxDistance) {
                continue;
            }

            double maxJump = entry.maxJointJump(qRef);
            if (maxJump > maxJointJump) {
                continue;
            }

            double jumpSum = entry.jointJumpSum(qRef);
            double marginPenalty = Math.max(0.0, 3.0 - entry.jointMargin) * 20.0;
            double manipulabilityBonus = Math.min(0.1, Math.max(0.0, entry.manipulability)) * 100.0;
            double classPenalty = entry.reachClass == 2 ? 0.0 : 200.0;
            double cost = dist * 120.0 + jumpSum * 1.8 + marginPenalty + classPenalty - manipulabilityBonus;

            if (cost < bestCost) {
                bestCost = cost;
                best = entry;
            }
        }

        return best;
    }

    private static Entry parseEntry(List<String> values, Map<String, Integer> headerIndex) {
        try {
            Entry entry = new Entry();
            entry.modelVersion = get(values, headerIndex, "model_version");
            entry.arm = get(values, headerIndex, "arm");
            entry.x = getDouble(values, headerIndex, "x");
            entry.y = getDouble(values, headerIndex, "y");
            entry.z = getDouble(values, headerIndex, "z");
            entry.q = new double[] {
                    getDouble(values, headerIndex, "q1"),
                    getDouble(values, headerIndex, "q2"),
                    getDouble(values, headerIndex, "q3"),
                    getDouble(values, headerIndex, "q4"),
                    getDouble(values, headerIndex, "q5"),
                    getDouble(values, headerIndex, "q6")
            };
            entry.alphaDeg = getDouble(values, headerIndex, "alpha");
            entry.yawOffsetDeg = getDouble(values, headerIndex, "yaw_offset");
            entry.gripperMode = get(values, headerIndex, "gripper_mode");
            entry.config = get(values, headerIndex, "config");
            entry.reachClass = (int) Math.round(getDouble(values, headerIndex, "reach_class"));
            entry.posError = getDouble(values, headerIndex, "pos_error");
            entry.jointMargin = getDouble(values, headerIndex, "joint_margin");
            entry.manipulability = getDouble(values, headerIndex, "manipulability");
            return entry;
        } catch (RuntimeException ex) {
            return null;
        }
    }

    private static String get(List<String> values, Map<String, Integer> headerIndex, String name) {
        Integer idx = headerIndex.get(name);
        if (idx == null || idx < 0 || idx >= values.size()) {
            return "";
        }
        return values.get(idx);
    }

    private static double getDouble(List<String> values, Map<String, Integer> headerIndex, String name) {
        String value = get(values, headerIndex, name);
        if (value == null || value.isEmpty()) {
            return Double.NaN;
        }
        return Double.parseDouble(value);
    }

    private static List<String> parseCsvLine(String line) {
        List<String> values = new ArrayList<>();
        StringBuilder current = new StringBuilder();
        boolean quoted = false;

        for (int i = 0; i < line.length(); i++) {
            char ch = line.charAt(i);
            if (ch == '"') {
                if (quoted && i + 1 < line.length() && line.charAt(i + 1) == '"') {
                    current.append('"');
                    i++;
                } else {
                    quoted = !quoted;
                }
            } else if (ch == ',' && !quoted) {
                values.add(current.toString());
                current.setLength(0);
            } else {
                current.append(ch);
            }
        }
        values.add(current.toString());
        return values;
    }

    private static double wrappedDegDiff(double a, double b) {
        double d = a - b;
        while (d > 180.0) d -= 360.0;
        while (d < -180.0) d += 360.0;
        return d;
    }

    public static class Entry {
        public String modelVersion;
        public String arm;
        public double x;
        public double y;
        public double z;
        public double[] q;
        public double alphaDeg;
        public double yawOffsetDeg;
        public String gripperMode;
        public String config;
        public int reachClass;
        public double posError;
        public double jointMargin;
        public double manipulability;

        public double distanceTo(double tx, double ty, double tz) {
            double dx = x - tx;
            double dy = y - ty;
            double dz = z - tz;
            return Math.sqrt(dx * dx + dy * dy + dz * dz);
        }

        public boolean hasJointSolution() {
            if (q == null || q.length < 6) {
                return false;
            }
            for (double value : q) {
                if (!Double.isFinite(value)) {
                    return false;
                }
            }
            return true;
        }

        public double maxJointJump(double[] qRef) {
            if (qRef == null || qRef.length < 6) {
                return 0.0;
            }
            double max = 0.0;
            for (int i = 0; i < 6; i++) {
                max = Math.max(max, Math.abs(wrappedDegDiff(q[i], qRef[i])));
            }
            return max;
        }

        public double jointJumpSum(double[] qRef) {
            if (qRef == null || qRef.length < 6) {
                return 0.0;
            }
            double sum = 0.0;
            for (int i = 0; i < 6; i++) {
                sum += Math.abs(wrappedDegDiff(q[i], qRef[i]));
            }
            return sum;
        }

        @Override
        public String toString() {
            return String.format(Locale.US,
                    "%s %s xyz=(%.2f, %.2f, %.2f) class=%d err=%.4f",
                    modelVersion, arm, x, y, z, reachClass, posError);
        }
    }
}
