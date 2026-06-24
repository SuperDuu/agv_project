package gui;

import static kinematics.Kinematics.*;
import javax.swing.*;
import java.awt.*;
import java.awt.event.*;
import java.util.ArrayList;

public class ArmPanel extends JPanel
        implements MouseListener, MouseMotionListener, MouseWheelListener {

    ArrayList<double[]> trail = new ArrayList<>();
    double camAz = -30, camEl = 25, scale = 5.0;
    int lastX, lastY;
    int demoStep = 0;
    double[] clickTarget = null; // To draw where we clicked
    ArrayList<double[]> workspacePoints = new ArrayList<>();
    java.util.HashSet<String> workspaceKeys = new java.util.HashSet<>();

    MainFrame robot;

    public ArmPanel(MainFrame robot) {
        this.robot = robot;
        setBackground(Color.WHITE);
        addMouseListener(this);
        addMouseMotionListener(this);
        addMouseWheelListener(this);
    }

    // --- MouseListener ---
    @Override
    public void mousePressed(MouseEvent e) {
        lastX = e.getX();
        lastY = e.getY();
    }

    @Override
    public void mouseClicked(MouseEvent e) {
        // Fixed-height click-to-move: only when fixedHeightMode is enabled and
        // right-click
        if (robot.fixedHeightMode && SwingUtilities.isRightMouseButton(e)) {
            clickTarget = moveToScreenPoint(e.getX(), e.getY());
        }
    }

    @Override
    public void mouseReleased(MouseEvent e) {
    }

    @Override
    public void mouseEntered(MouseEvent e) {
    }

    @Override
    public void mouseExited(MouseEvent e) {
    }

    // --- MouseMotionListener ---
    @Override
    public void mouseDragged(MouseEvent e) {
        int dx = e.getX() - lastX, dy = e.getY() - lastY;
        if (SwingUtilities.isLeftMouseButton(e)) {
            camAz += dx * 0.5;
            camEl = Math.max(-85, Math.min(85, camEl - dy * 0.5));
        }
        lastX = e.getX();
        lastY = e.getY();
        repaint();
    }

    @Override
    public void mouseMoved(MouseEvent e) {
    }

    // --- MouseWheelListener ---
    @Override
    public void mouseWheelMoved(MouseWheelEvent e) {
        scale = Math.max(2.0, Math.min(8.0, scale - e.getWheelRotation() * 0.5));
        repaint();
    }

    /**
     * Convert a screen point to approximate world XY at the fixed Z height,
     * then call IK to move the arm there.
     */
    private double[] moveToScreenPoint(int sx, int sy) {
        int cx = getWidth() / 2, cy = getHeight() * 2 / 3;
        double fixedZ = robot.getFixedHeight();

        double az = Math.toRadians(camAz), el = Math.toRadians(camEl);
        double cAz = Math.cos(az), sAz = Math.sin(az), cEl = Math.cos(el), sEl = Math.sin(el);

        double scrX = (sx - cx) / scale;
        double scrY = -(sy - cy) / scale;

        if (Math.abs(sEl) < 0.05)
            return null; // Avoid singularity near horizontal view

        double p0 = 0, p1 = 0;
        // Numerical solve for world (p0, p1) given ScreenX, ScreenY and fixed World Z
        for (int iter = 0; iter < 100; iter++) {
            double vz = p0 * cAz * cEl + p1 * sAz * cEl + fixedZ * sEl;
            double f = 10000.0 / (10000.0 + vz);
            double vx = scrX / f;
            double vy_prime = scrY / f - fixedZ * cEl;

            p0 = (-sAz * sEl * vx - cAz * vy_prime) / sEl;
            p1 = (-sAz * vy_prime + cAz * sEl * vx) / sEl;
        }

        double[] targetPos = { p0, p1, fixedZ };
        String prefCfg = robot.configCombo.getSelectedIndex() == 0 ? "+" : "-";
        double[] result = robot.solveIKSmart(p0, p1, fixedZ, prefCfg);

        if (result != null) {
            robot.setTargetAngles(result);
            robot.setGotoStatus(String.format("OK (%.1f, %.1f, %.1f)", p0, p1, fixedZ), new Color(0, 140, 0));
        } else {
            robot.setGotoStatus("Ngoài tầm (Click)", Color.RED);
        }

        repaint();
        return targetPos;
    }

    public double[] getEndEffectorPosition() {
        double[][] pts3d = computeAllJoints3D();
        // Return index 6 (NUM_JOINTS + 1) which is the actual distal tooltip center.
        return pts3d[NUM_JOINTS + 1];
    }

    public String workspaceStatus = "";

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2 = (Graphics2D) g;
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        int cx = getWidth() / 2, cy = getHeight() * 2 / 3;

        if (robot.showGridCb.isSelected())
            drawGrid(g2, cx, cy);

        if (robot.showWorkspace)
            drawWorkspace(g2, cx, cy);

        if (!workspaceStatus.isEmpty()) {
            g2.setFont(new Font("Arial", Font.BOLD, 18));
            g2.setColor(new Color(200, 50, 0));
            g2.drawString(workspaceStatus, getWidth() - 400, 30);
        }

        double[][] pts3d = computeAllJoints3D();

        int[][] s = new int[pts3d.length][2];
        for (int i = 0; i < pts3d.length; i++) {
            int[] proj = project(pts3d[i], cx, cy);
            s[i][0] = proj[0];
            s[i][1] = proj[1];
        }

        if (robot.showTrailCb.isSelected()) {
            double[] currentEE = pts3d[NUM_JOINTS + 1];
            if (trail.isEmpty()) {
                trail.add(currentEE.clone());
            } else {
                double[] last = trail.get(trail.size() - 1);
                double dist = Math.sqrt(Math.pow(currentEE[0] - last[0], 2) + Math.pow(currentEE[1] - last[1], 2)
                        + Math.pow(currentEE[2] - last[2], 2));
                if (dist > 1.0) {
                    trail.add(currentEE.clone());
                    if (trail.size() > 500)
                        trail.remove(0);
                }
            }
            drawTrail(g2, cx, cy);
        }

        double[][] T_end = computeEndEffectorMatrix();

        // --- Depth Sorting and Perspective Scaling ---
        java.util.List<Drawable> drawables = new java.util.ArrayList<>();

        // 1. Collect Links (Tubes) and Joints (Spheres)
        int[] tubeWidths = { 7, 6, 5, 4, 3, 2, 2 };
        Color[] tubeColors = {
                new Color(80, 80, 80),
                new Color(90, 90, 90),
                new Color(100, 100, 100),
                new Color(110, 110, 110),
                new Color(120, 120, 120),
                new Color(130, 130, 130),
                new Color(140, 140, 140)
        };
        for (int i = 0; i < pts3d.length - 1; i++) {
            final int tw = (i < tubeWidths.length) ? tubeWidths[i] : tubeWidths[tubeWidths.length - 1];
            final Color color = (i < tubeColors.length) ? tubeColors[i] : tubeColors[tubeColors.length - 1];

            // Sphere at joint start
            drawables.add(new JointSphere(pts3d[i], tw, new Color(50, 120, 200)));

            // Tube segment
            drawables.add(new TubeSegment(pts3d[i], pts3d[i + 1], tw, color));
        }

        // Add Sphere for Joint 6 (Wrist / Flange)
        drawables.add(new JointSphere(pts3d[6], 2, new Color(170, 170, 180)));

        // Gripper base is at tool tip (pts3d[7])
        double[] pTCP = pts3d[7];

        // Draw Gripper fingers assembly (Visual-only, 3.5 units long)
        drawables.add(new GripperDrawable(T_end, pTCP, 3.5));

        // 3. Sort by depth (vz descending - Painter's Algorithm)
        drawables.sort((a, b) -> Double.compare(b.getDepth(), a.getDepth()));

        // 4. Render everything
        for (Drawable d : drawables) {
            d.draw(g2, cx, cy);
        }

        if (clickTarget != null) {
            int[] sc = project(clickTarget, cx, cy);
            g2.setColor(new Color(255, 0, 0, 180));
            g2.fillOval(sc[0] - 4, sc[1] - 4, 8, 8);
        }

        g2.setColor(Color.BLACK);
        g2.drawString("Mô Phỏng Cánh Tay Robot 6-Dof", 10, 20);
    }

    // L-elbow method removed for 6-DOF robot

    // --- New Depth Sorting and Drawing Helpers ---
    interface Drawable {
        double getDepth();

        void draw(Graphics2D g2, int cx, int cy);
    }

    class TubeSegment implements Drawable {
        double[] p1, p2;
        int baseWidth;
        Color color;
        double depth;

        TubeSegment(double[] p1, double[] p2, int bw, Color c) {
            this.p1 = p1.clone();
            this.p2 = p2.clone();
            this.baseWidth = bw;
            this.color = c;
            this.depth = (getVz(p1) + getVz(p2)) / 2.0;
        }

        @Override
        public double getDepth() {
            return depth;
        }

        @Override
        public void draw(Graphics2D g2, int cx, int cy) {
            int[] s1 = project(p1, cx, cy), s2 = project(p2, cx, cy);
            // Better scale: use the average of scale factors at p1 and p2 and apply camera
            // zoom (scale)
            double f1 = getScaleFactor(p1);
            double f2 = getScaleFactor(p2);
            float tw = (float) (baseWidth * scale * (f1 + f2) / 2.0);
            if (tw < 1)
                tw = 1; // Minimum width

            // Shadow
            g2.setColor(new Color(30, 30, 30, 80));
            g2.setStroke(new BasicStroke(tw + 2, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
            g2.drawLine(s1[0], s1[1], s2[0], s2[1]);
            // Body
            g2.setColor(color.darker());
            g2.setStroke(new BasicStroke(tw, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
            g2.drawLine(s1[0], s1[1], s2[0], s2[1]);
        }
    }

    class JointSphere implements Drawable {
        double[] p;
        int baseWidth;
        Color color;
        double depth;

        JointSphere(double[] p, int bw, Color c) {
            this.p = p.clone();
            this.baseWidth = bw;
            this.color = c;
            this.depth = getVz(p);
        }

        public double getDepth() {
            return depth;
        }

        public void draw(Graphics2D g2, int cx, int cy) {
            int[] s = project(p, cx, cy);
            double f = getScaleFactor(p);
            // Use camera zoom explicitly, reduce size of joints by additional 40% (multiply
            // by 0.6)
            int jr = (int) (((baseWidth / 2.0 + 2) * 0.6) * f * scale);//0.6 scale
            if (jr < 2)
                jr = 2; // Keep at least 2 pixels so it remains visible

            g2.setColor(color);
            g2.fillOval(s[0] - jr, s[1] - jr, jr * 2, jr * 2);
            g2.setColor(color.WHITE);
            g2.setStroke(new BasicStroke(1.0f));
            g2.drawOval(s[0] - jr, s[1] - jr, jr * 2, jr * 2);
        }
    }

    class GripperDrawable implements Drawable {
        double[][] T;
        double[] p3D;
        double depth;
        double wFingerLen;

        GripperDrawable(double[][] T, double[] p, double wFingerLen) {
            this.T = T;
            this.p3D = p.clone();
            this.depth = getVz(p);
            this.wFingerLen = wFingerLen;
        }

        @Override
        public double getDepth() {
            return depth;
        }

        @Override
        public void draw(Graphics2D g2, int cx, int cy) {
            double ux = T[0][2], uy = T[1][2], uz = T[2][2];
            double nx = T[0][0], ny = T[1][0], nz = T[2][0];
            double f = getScaleFactor(p3D);

            // Gripper dimensions (Wider and shorter for better visibility)
            double wOpening = (robot.isGripped ? 0.5 : 2.0);

            // p3D is now treated as the base of the gripper (the end of L6)
            double[] baseL = { p3D[0] + nx * wOpening, p3D[1] + ny * wOpening, p3D[2] + nz * wOpening };
            double[] baseR = { p3D[0] - nx * wOpening, p3D[1] - ny * wOpening, p3D[2] - nz * wOpening };

            // Tip is shifted forward along approach vector
            double[] tipL = { baseL[0] + ux * wFingerLen, baseL[1] + uy * wFingerLen, baseL[2] + uz * wFingerLen };
            double[] tipR = { baseR[0] + ux * wFingerLen, baseR[1] + uy * wFingerLen, baseR[2] + uz * wFingerLen };

            // Project 3D coordinates to screen
            int[] sBaseL = project(baseL, cx, cy);
            int[] sBaseR = project(baseR, cx, cy);
            int[] sTipL = project(tipL, cx, cy);
            int[] sTipR = project(tipR, cx, cy);

            // Strokes and Colors (Thinner for small scale)
            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

            // Base Crossbar
            g2.setColor(Color.DARK_GRAY);
            g2.setStroke(new BasicStroke(Math.max(1, (float) (0.4 * f * scale))));
            g2.drawLine(sBaseL[0], sBaseL[1], sBaseR[0], sBaseR[1]);

            // Fingers
            g2.setColor(Color.ORANGE);
            g2.setStroke(new BasicStroke(Math.max(1, (float) (0.3 * f * scale))));
            g2.drawLine(sBaseL[0], sBaseL[1], sTipL[0], sTipL[1]);
            g2.drawLine(sBaseR[0], sBaseR[1], sTipR[0], sTipR[1]);

            // Highlight Tip contact point
            g2.setColor(Color.RED);
            int[] sTip = project(p3D, cx, cy);
            g2.fillOval(sTip[0] - 2, sTip[1] - 2, 4, 4);
        }
    }

    private double getVz(double[] p) {
        double az = Math.toRadians(camAz), el = Math.toRadians(camEl);
        double cAz = Math.cos(az), sAz = Math.sin(az), cEl = Math.cos(el), sEl = Math.sin(el);
        return p[0] * cAz * cEl + p[1] * sAz * cEl + p[2] * sEl;
    }

    private double getScaleFactor(double[] p) {
        double vz = getVz(p);
        return 1000.0 / (1000.0 + vz);
    }

    void drawGrid(Graphics2D g2, int cx, int cy) {
        int size = 50, step = 4;

        // Draw floor (darker semi-transparent rectangle)
        int[] f1 = project(new double[] { -size, -size, 0 }, cx, cy);
        int[] f2 = project(new double[] { size, -size, 0 }, cx, cy);
        int[] f3 = project(new double[] { size, size, 0 }, cx, cy);
        int[] f4 = project(new double[] { -size, size, 0 }, cx, cy);
        Polygon floor = new Polygon();
        floor.addPoint(f1[0], f1[1]);
        floor.addPoint(f2[0], f2[1]);
        floor.addPoint(f3[0], f3[1]);
        floor.addPoint(f4[0], f4[1]);
        
        g2.setColor(new Color(220, 220, 225)); //floor
//        new Color(205, 164, 52)
        g2.fillPolygon(floor);

        g2.setColor(new Color(180, 180, 190)); //grid
        for (int i = -size; i <= size; i += step) {
            int[] p1 = project(new double[] { i, -size, 0 }, cx, cy),
                    p2 = project(new double[] { i, size, 0 }, cx, cy);
            g2.drawLine(p1[0], p1[1], p2[0], p2[1]);
            int[] p3 = project(new double[] { -size, i, 0 }, cx, cy),
                    p4 = project(new double[] { size, i, 0 }, cx, cy);
            g2.drawLine(p3[0], p3[1], p4[0], p4[1]);
        }
    }

    void drawTrail(Graphics2D g2, int cx, int cy) {
        g2.setColor(Color.ORANGE);
        for (int i = 1; i < trail.size(); i++) {
            int[] p1 = project(trail.get(i - 1), cx, cy), p2 = project(trail.get(i), cx, cy);
            g2.drawLine(p1[0], p1[1], p2[0], p2[1]);
//            g2.drawLine(p1[0]+1, p1[1], p2[0], p2[1]+1);
//            g2.drawLine(p1[0]-1, p1[1], p2[0], p2[1]-1);
        }
    }

    // drawGripper method removed (now handled by GripperDrawable class)

    public double[] computeFK(double q1, double q2, double q3, double q4, double q5, double q6) {
        double[][] T = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
        double[][] params = {
                { 0, L1 + L0, 0, -Math.PI / 2, Math.toRadians(q1) },
                { -Math.PI / 2, L2 + L3, 0, -Math.PI / 2, Math.toRadians(q2) },
                { -Math.PI / 2, 0, 0, -Math.PI, Math.toRadians(q3) },
                { 0, 0, L4, -Math.PI / 2, Math.toRadians(q4) },
                { -Math.PI / 2, L5 + L6, 0, -Math.PI / 2, Math.toRadians(q5) },
                { -Math.PI / 2, 0, 0, 0, Math.toRadians(q6) }
        };
        for (int i = 0; i < NUM_JOINTS; i++) {
            T = multiply4x4(T,
                    getMDHMatrix(params[i][0], params[i][1], params[i][2], params[i][3], params[i][4]));
        }
        T = multiply4x4(T, getToolMatrix());
        return new double[] { T[0][3], T[1][3], T[2][3] };
    }

    double[][] computeEndEffectorMatrix() {
        double[][] T = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
        double[][] params = {
                { 0, L1 + L0, 0, -Math.PI / 2, Math.toRadians(robot.angles[0]) },
                { -Math.PI / 2, L2 + L3, 0, -Math.PI / 2, Math.toRadians(robot.angles[1]) },
                { -Math.PI / 2, 0, 0, -Math.PI, Math.toRadians(robot.angles[2]) },
                { 0, 0, L4, -Math.PI / 2, Math.toRadians(robot.angles[3]) },
                { -Math.PI / 2, L5 + L6, 0, -Math.PI / 2, Math.toRadians(robot.angles[4]) },
                { -Math.PI / 2, 0, 0, 0, Math.toRadians(robot.angles[5]) }
        };
        for (int i = 0; i < NUM_JOINTS; i++) {
            T = multiply4x4(T,
                    getMDHMatrix(params[i][0], params[i][1], params[i][2], params[i][3], params[i][4]));
        }
        return multiply4x4(T, getToolMatrix());
    }

    int[] project(double[] p, int cx, int cy) {
        double az = Math.toRadians(camAz), el = Math.toRadians(camEl);
        double cAz = Math.cos(az), sAz = Math.sin(az), cEl = Math.cos(el), sEl = Math.sin(el);
        double vx = -p[0] * sAz + p[1] * cAz;
        double vy = -p[0] * cAz * sEl - p[1] * sAz * sEl + p[2] * cEl;
        double vz = p[0] * cAz * cEl + p[1] * sAz * cEl + p[2] * sEl;
        double f = 10000.0 / (10000.0 + vz);
        return new int[] { (int) Math.round(cx + scale * vx * f), (int) Math.round(cy - scale * vy * f) };
    }

    public double[][] computeAllJoints3D() {
        double[][] pts = new double[NUM_JOINTS + 2][3];
        double[][] T = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
        pts[0] = new double[] { 0, 0, 0 };

        double[][] params = {
                { 0, L1 + L0, 0, -Math.PI / 2, Math.toRadians(robot.angles[0]) },
                { -Math.PI / 2, L2 + L3, 0, -Math.PI / 2, Math.toRadians(robot.angles[1]) },
                { -Math.PI / 2, 0, 0, -Math.PI, Math.toRadians(robot.angles[2]) },
                { 0, 0, L4, -Math.PI / 2, Math.toRadians(robot.angles[3]) },
                { -Math.PI / 2, L5 + L6, 0, -Math.PI / 2, Math.toRadians(robot.angles[4]) },
                { -Math.PI / 2, 0, 0, 0, Math.toRadians(robot.angles[5]) }
        };

        for (int i = 0; i < NUM_JOINTS; i++) {
            T = multiply4x4(T,
                    getMDHMatrix(params[i][0], params[i][1], params[i][2], params[i][3], params[i][4]));
            pts[i + 1] = new double[] { T[0][3], T[1][3], T[2][3] };
        }

        T = multiply4x4(T, getToolMatrix());
        pts[NUM_JOINTS + 1] = new double[] { T[0][3], T[1][3], T[2][3] };

        return pts;
    }

    void drawWorkspace(Graphics2D g2, int cx, int cy) {
        if (workspacePoints.isEmpty())
            return;
        g2.setColor(new Color(110, 170, 255, 80));
        g2.setStroke(new BasicStroke(1));
        synchronized (workspacePoints) {
            for (int i = 0; i < workspacePoints.size() - 1; i++) {
                int[] p1 = project(workspacePoints.get(i), cx, cy);
                int[] p2 = project(workspacePoints.get(i + 1), cx, cy);
                if (Math.abs(p1[0] - p2[0]) < 150 && Math.abs(p1[1] - p2[1]) < 150) {
                    g2.drawLine(p1[0], p1[1], p2[0], p2[1]);
                }
            }
        }
        g2.setColor(new Color(110, 170, 255, 120));
        synchronized (workspacePoints) {
            for (double[] p : workspacePoints) {
                int[] sc = project(p, cx, cy);
                g2.fillRect(sc[0], sc[1], 2, 2);
            }
        }
    }

    void addWorkspacePoint(double[] p) {
        String key = (int) (p[0] / 2) + "," + (int) (p[1] / 2) + "," + (int) (p[2] / 2);
        if (!workspaceKeys.contains(key)) {
            synchronized (workspacePoints) {
                workspacePoints.add(p.clone());
            }
            workspaceKeys.add(key);
        }
    }
}
