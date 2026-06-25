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
        scale = Math.max(1.0, Math.min(30.0, scale - e.getWheelRotation() * 0.8));
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
        System.out.printf("[DEBUG_CLICK] Click position: (X=%.2f, Y=%.2f, Z=%.2f)\n", p0, p1, fixedZ);
        
        String prefCfgRight = robot.configComboRight.getSelectedIndex() == 0 ? "+" : "-";
        double[] resultRight = robot.solveIKSmartRight(p0, p1, fixedZ, prefCfgRight);
        System.out.printf("[DEBUG_CLICK] Right Arm Solver: %s\n", resultRight == null ? "FAILED" : "SUCCESS");
        
        String prefCfgLeft = robot.configComboLeft.getSelectedIndex() == 0 ? "+" : "-";
        double[] resultLeft = robot.solveIKSmartLeft(p0, p1, fixedZ, prefCfgLeft);
        System.out.printf("[DEBUG_CLICK] Left Arm Solver: %s\n", resultLeft == null ? "FAILED" : "SUCCESS");

        boolean chooseRight = true;
        double[] chosenResult = null;

        if (resultRight != null && resultLeft != null) {
            double costRight = calculateMovementCost(resultRight, robot.getAnglesRight());
            double costLeft = calculateMovementCost(resultLeft, robot.getAnglesLeft());
            System.out.printf("[DEBUG_CLICK] Both reached. costRight=%.2f, costLeft=%.2f\n", costRight, costLeft);
            if (costRight <= costLeft) {
                chooseRight = true;
                chosenResult = resultRight;
            } else {
                chooseRight = false;
                chosenResult = resultLeft;
            }
        } else if (resultRight != null) {
            chooseRight = true;
            chosenResult = resultRight;
        } else if (resultLeft != null) {
            chooseRight = false;
            chosenResult = resultLeft;
        }
        System.out.printf("[DEBUG_CLICK] Decision: Chosen Arm = %s\n", chosenResult == null ? "NONE" : (chooseRight ? "RIGHT" : "LEFT"));

        if (chosenResult != null) {
            robot.trajArmCombo.setSelectedIndex(chooseRight ? 0 : 1);
            if (chooseRight) {
                robot.setTargetAnglesRight(chosenResult);
                
                // Retract Left arm to home, preserving shared Joint 1
                double[] leftHome = { chosenResult[0], 0, -10, 30, 0, 0 };
                robot.setTargetAnglesLeft(leftHome);
                
                robot.setGotoStatusRight(String.format("OK (%.1f, %.1f, %.1f)", p0, p1, fixedZ), new Color(0, 140, 0));
                robot.setGotoStatusLeft("Về Home", Color.BLUE);
            } else {
                robot.setTargetAnglesLeft(chosenResult);
                
                // Retract Right arm to home, preserving shared Joint 1
                double[] rightHome = { chosenResult[0], 0, 10, -30, 0, 0 };
                robot.setTargetAnglesRight(rightHome);
                
                robot.setGotoStatusLeft(String.format("OK (%.1f, %.1f, %.1f)", p0, p1, fixedZ), new Color(0, 140, 0));
                robot.setGotoStatusRight("Về Home", Color.BLUE);
            }
        } else {
            robot.setGotoStatusRight("Ngoài tầm (Click)", Color.RED);
            robot.setGotoStatusLeft("Ngoài tầm (Click)", Color.RED);
        }

        repaint();
        return targetPos;
    }

    private double calculateMovementCost(double[] target, double[] current) {
        double sum = 0;
        for (int i = 0; i < target.length; i++) {
            double diff = target[i] - current[i];
            while (diff > 180) diff -= 360;
            while (diff < -180) diff += 360;
            sum += diff * diff;
        }
        return sum;
    }

    public double[] getEndEffectorPosition() {
        double[][] pts3d = computeAllJoints3D();
        // Return index 6 (NUM_JOINTS + 1) which is the actual distal tooltip center.
        return pts3d[NUM_JOINTS + 1];
    }

    public double[] getRightEndEffectorPosition() {
        double[][] pts3d = computeAllJoints3DRight();
        return pts3d[NUM_JOINTS + 1];
    }

    public double[] getLeftEndEffectorPosition() {
        double[][] pts3d = computeAllJoints3DLeft();
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

        double[][] pts3dRight = computeAllJoints3DRight();
        double[][] pts3dLeft = computeAllJoints3DLeft();

        double[][] pts3dActive = robot.isRightArmSelected ? pts3dRight : pts3dLeft;

        if (robot.showTrailCb.isSelected()) {
            double[] currentEE = pts3dActive[NUM_JOINTS + 1];
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

        double[][] T_end_right = computeEndEffectorMatrixRight();
        double[][] T_end_left = computeEndEffectorMatrixLeft();

        // --- Depth Sorting and Perspective Scaling ---
        java.util.List<Drawable> drawables = new java.util.ArrayList<>();


        // Draw humanoid central vertical torso (spine) and base pedestal
        drawables.add(new TubeSegment(new double[] { 0, 0, 10 }, new double[] { 0, 0, 125 }, 12, new Color(60, 65, 70)));
        drawables.add(new BasePedestal());
        
        // Neck and Head
        drawables.add(new TubeSegment(new double[] { 0, 0, 125 }, new double[] { 0, 0, 138 }, 8, new Color(60, 60, 60)));
        drawables.add(new JointSphere(new double[] { 0, 0, 138 }, 12, new Color(75, 80, 85)));

        int[] tubeWidths = { 9, 8, 7, 6, 5, 4, 4 };
        Color[] tubeColorsRight = {
                new Color(80, 80, 80),
                new Color(90, 90, 90),
                new Color(100, 100, 100),
                new Color(110, 110, 110),
                new Color(120, 120, 120),
                new Color(130, 130, 130),
                new Color(140, 140, 140)
        };
        Color[] tubeColorsLeft = {
                new Color(80, 100, 80),
                new Color(90, 110, 90),
                new Color(100, 120, 100),
                new Color(110, 130, 110),
                new Color(120, 140, 120),
                new Color(130, 150, 130),
                new Color(140, 160, 140)
        };

        // Right Arm segments (skip base to avoid overlapping torso)
        for (int i = 1; i < pts3dRight.length - 2; i++) {
            final int tw = (i < tubeWidths.length) ? tubeWidths[i] : tubeWidths[tubeWidths.length - 1];
            final Color color = (i < tubeColorsRight.length) ? tubeColorsRight[i] : tubeColorsRight[tubeColorsRight.length - 1];
            drawables.add(new JointSphere(pts3dRight[i], tw, new Color(50, 120, 200)));
            drawables.add(new TubeSegment(pts3dRight[i], pts3dRight[i + 1], tw, color));
        }
        // Draw the last wrist Joint 6 sphere manually
        drawables.add(new JointSphere(pts3dRight[6], tubeWidths[5], new Color(50, 120, 200)));
        drawables.add(new GripperDrawable(T_end_right, pts3dRight[7], true));

        // Left Arm segments (skip base to avoid overlapping torso)
        for (int i = 1; i < pts3dLeft.length - 2; i++) {
            final int tw = (i < tubeWidths.length) ? tubeWidths[i] : tubeWidths[tubeWidths.length - 1];
            final Color color = (i < tubeColorsLeft.length) ? tubeColorsLeft[i] : tubeColorsLeft[tubeColorsLeft.length - 1];
            drawables.add(new JointSphere(pts3dLeft[i], tw, new Color(200, 80, 80)));
            drawables.add(new TubeSegment(pts3dLeft[i], pts3dLeft[i + 1], tw, color));
        }
        // Draw the last wrist Joint 6 sphere manually
        drawables.add(new JointSphere(pts3dLeft[6], tubeWidths[5], new Color(200, 80, 80)));
        drawables.add(new GripperDrawable(T_end_left, pts3dLeft[7], false));

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
        g2.drawString("Mô Phỏng Robot Song Arm Humanoid (6-Dof)", 10, 20);
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
            double f1 = getScaleFactor(p1);
            double f2 = getScaleFactor(p2);
            float tw = (float) (baseWidth * scale * (f1 + f2) / 2.0);
            if (tw < 1)
                tw = 1;

            float dx = s2[0] - s1[0];
            float dy = s2[1] - s1[1];
            float len = (float) Math.sqrt(dx * dx + dy * dy);

            if (len > 0.1f) {
                float ux = dx / len;
                float uy = dy / len;
                float gnx = -uy;
                float gny = ux;

                float startX = s1[0] - gnx * tw / 2.0f;
                float startY = s1[1] - gny * tw / 2.0f;
                float endX = s1[0] + gnx * tw / 2.0f;
                float endY = s1[1] + gny * tw / 2.0f;

                LinearGradientPaint gp = new LinearGradientPaint(
                    startX, startY, endX, endY,
                    new float[] { 0.0f, 0.25f, 0.7f, 1.0f },
                    new Color[] { color.darker().darker(), color.brighter(), color, color.darker() }
                );
                
                // Shadow
                g2.setColor(new Color(20, 20, 25, 45));
                g2.setStroke(new BasicStroke(tw + 3, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
                g2.drawLine(s1[0], s1[1] + 2, s2[0], s2[1] + 2);

                // Cylinder
                g2.setPaint(gp);
                g2.setStroke(new BasicStroke(tw, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
                g2.drawLine(s1[0], s1[1], s2[0], s2[1]);
            } else {
                g2.setColor(color);
                g2.setStroke(new BasicStroke(tw, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
                g2.drawLine(s1[0], s1[1], s2[0], s2[1]);
            }
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
            int jr = (int) ((baseWidth * 0.95) * f * scale);
            if (jr < 3)
                jr = 3;

            float radius = jr;
            float centerX = s[0] - jr * 0.3f;
            float centerY = s[1] - jr * 0.3f;
            if (radius > 1) {
                RadialGradientPaint rgp = new RadialGradientPaint(
                    centerX, centerY, radius * 1.6f,
                    new float[] { 0.0f, 0.75f, 1.0f },
                    new Color[] { Color.WHITE, color, color.darker().darker() }
                );
                g2.setPaint(rgp);
            } else {
                g2.setColor(color);
            }
            
            g2.fillOval(s[0] - jr, s[1] - jr, jr * 2, jr * 2);
            
            g2.setColor(color.darker().darker());
            g2.setStroke(new BasicStroke(1.0f));
            g2.drawOval(s[0] - jr, s[1] - jr, jr * 2, jr * 2);
        }
    }

    class BasePedestal implements Drawable {
        double depth;

        BasePedestal() {
            this.depth = getVz(new double[] { 0, 0, 5 });
        }

        @Override
        public double getDepth() {
            return depth;
        }

        private class PedestalFace {
            int[] indices;
            double depth;
            Color color;

            PedestalFace(int[] idx, Color c, double[][] corners) {
                this.indices = idx;
                this.color = c;
                double sum = 0;
                for (int i : idx) {
                    sum += getVz(corners[i]);
                }
                this.depth = sum / idx.length;
            }
        }

        @Override
        public void draw(Graphics2D g2, int cx, int cy) {
            double halfW = 20.0;
            double h = 10.0;
            double[][] corners = {
                { -halfW, -halfW, 0 },
                {  halfW, -halfW, 0 },
                {  halfW,  halfW, 0 },
                { -halfW,  halfW, 0 },
                { -halfW, -halfW, h },
                {  halfW, -halfW, h },
                {  halfW,  halfW, h },
                { -halfW,  halfW, h }
            };

            int[][] sc = new int[8][2];
            for (int i = 0; i < 8; i++) {
                sc[i] = project(corners[i], cx, cy);
            }

            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

            // Draw base shadow
            g2.setColor(new Color(20, 20, 25, 60));
            Polygon shadow = new Polygon();
            shadow.addPoint(sc[0][0], sc[0][1] + 3);
            shadow.addPoint(sc[1][0], sc[1][1] + 3);
            shadow.addPoint(sc[2][0], sc[2][1] + 3);
            shadow.addPoint(sc[3][0], sc[3][1] + 3);
            g2.fillPolygon(shadow);

            // Create faces list
            java.util.List<PedestalFace> faces = new java.util.ArrayList<>();
            faces.add(new PedestalFace(new int[] { 0, 1, 5, 4 }, new Color(50, 50, 55), corners));
            faces.add(new PedestalFace(new int[] { 1, 2, 6, 5 }, new Color(55, 55, 60), corners));
            faces.add(new PedestalFace(new int[] { 2, 3, 7, 6 }, new Color(60, 60, 65), corners));
            faces.add(new PedestalFace(new int[] { 3, 0, 4, 7 }, new Color(45, 45, 50), corners));
            faces.add(new PedestalFace(new int[] { 4, 5, 6, 7 }, new Color(75, 75, 80), corners));

            // Sort faces by depth (furthest drawn first)
            faces.sort((f1, f2) -> Double.compare(f2.depth, f1.depth));

            // Draw the sorted faces
            for (PedestalFace f : faces) {
                Polygon poly = new Polygon();
                for (int idx : f.indices) {
                    poly.addPoint(sc[idx][0], sc[idx][1]);
                }
                g2.setColor(f.color);
                g2.fillPolygon(poly);
                g2.setColor(Color.DARK_GRAY);
                g2.setStroke(new BasicStroke(1.0f));
                g2.drawPolygon(poly);
            }
        }
    }

    class GripperDrawable implements Drawable {
        double[][] T;
        double[] p3D;
        double[] pWrist;
        double depth;
        boolean isRight;

        GripperDrawable(double[][] T, double[] p, boolean isRight) {
            this.T = T;
            this.p3D = p.clone();
            this.isRight = isRight;
            double ux = T[0][2], uy = T[1][2], uz = T[2][2];
            // L7 is the length of the gripper. So pWrist is joint 6.
            this.pWrist = new double[] { p[0] - ux * L7, p[1] - uy * L7, p[2] - uz * L7 };
            // Set the depth to be slightly in front of the wrist center to ensure correct sorting
            this.depth = getVz(pWrist) - 0.5;
        }

        @Override
        public double getDepth() {
            return depth;
        }

        private int[] projectRel(double du, double dn, double db, double ux, double uy, double uz, double nx, double ny, double nz, double bx, double by, double bz, int cx, int cy) {
            double[] pt = {
                pWrist[0] + ux * du + nx * dn + bx * db,
                pWrist[1] + uy * du + ny * dn + by * db,
                pWrist[2] + uz * du + nz * dn + bz * db
            };
            return project(pt, cx, cy);
        }

        private void drawThickLink(Graphics2D g2, double du1, double dn1, double du2, double dn2, double t, Color c, Color border,
                                   double ux, double uy, double uz, double nx, double ny, double nz, double bx, double by, double bz,
                                   double f, int cx, int cy) {
            double du = du2 - du1;
            double dn = dn2 - dn1;
            double len = Math.sqrt(du*du + dn*dn);
            if (len < 0.01) return;
            double pu = -dn / len * t * f;
            double pn = du / len * t * f;
            
            int[] p1 = projectRel(du1 + pu, dn1 + pn, 0, ux, uy, uz, nx, ny, nz, bx, by, bz, cx, cy);
            int[] p2 = projectRel(du1 - pu, dn1 - pn, 0, ux, uy, uz, nx, ny, nz, bx, by, bz, cx, cy);
            int[] p3 = projectRel(du2 - pu, dn2 - pn, 0, ux, uy, uz, nx, ny, nz, bx, by, bz, cx, cy);
            int[] p4 = projectRel(du2 + pu, dn2 + pn, 0, ux, uy, uz, nx, ny, nz, bx, by, bz, cx, cy);
            
            Polygon poly = new Polygon();
            poly.addPoint(p1[0], p1[1]);
            poly.addPoint(p2[0], p2[1]);
            poly.addPoint(p3[0], p3[1]);
            poly.addPoint(p4[0], p4[1]);
            
            g2.setColor(c);
            g2.fillPolygon(poly);
            g2.setColor(border);
            g2.setStroke(new BasicStroke(1.0f));
            g2.drawPolygon(poly);
        }

        @Override
        public void draw(Graphics2D g2, int cx, int cy) {
            double ux = T[0][2], uy = T[1][2], uz = T[2][2]; 
            double nx = T[0][0], ny = T[1][0], nz = T[2][0]; 
            double f = getScaleFactor(p3D);

            double bx = uy * nz - uz * ny;
            double by = uz * nx - ux * nz;
            double bz = ux * ny - uy * nx;
            double blen = Math.sqrt(bx*bx + by*by + bz*bz);
            if (blen > 1e-6) {
                bx /= blen; by /= blen; bz /= blen;
            }

            // 1. Base Flange (silver disc)
            drawThickLink(g2, 0.0, 0.0, 1.5, 0.0, 4.0, new Color(130, 135, 140), new Color(70, 75, 80), ux, uy, uz, nx, ny, nz, bx, by, bz, f, cx, cy);

            // 2. Chassis body (dark industrial graphite)
            drawThickLink(g2, 1.5, 0.0, 5.5, 0.0, 2.8, new Color(55, 57, 61), new Color(30, 32, 35), ux, uy, uz, nx, ny, nz, bx, by, bz, f, cx, cy);
            
            // 3. Guide rods (silver)
            drawThickLink(g2, 1.5, 1.2, 5.5, 1.2, 0.5, new Color(200, 205, 210), new Color(120, 125, 130), ux, uy, uz, nx, ny, nz, bx, by, bz, f, cx, cy);
            drawThickLink(g2, 1.5, -1.2, 5.5, -1.2, 0.5, new Color(200, 205, 210), new Color(120, 125, 130), ux, uy, uz, nx, ny, nz, bx, by, bz, f, cx, cy);

            // 4. Fingers opening logic (Parallel sliding bars)
            double w = isRight ? (robot.isGrippedRight ? 1.0 : 3.5) : (robot.isGrippedLeft ? 1.0 : 3.5); 

            // Left Finger (Parallel bar, CNC Anodized Orange)
            drawThickLink(g2, 5.5, w, L7, w, 0.9, new Color(245, 125, 20), new Color(150, 70, 0), ux, uy, uz, nx, ny, nz, bx, by, bz, f, cx, cy);
            // Left Rubber Pad (inside surface of Left Finger)
            drawThickLink(g2, 8.0, w - 0.3, L7 - 0.5, w - 0.3, 0.4, new Color(30, 30, 30), Color.BLACK, ux, uy, uz, nx, ny, nz, bx, by, bz, f, cx, cy);

            // Right Finger (Parallel bar, CNC Anodized Orange)
            drawThickLink(g2, 5.5, -w, L7, -w, 0.9, new Color(245, 125, 20), new Color(150, 70, 0), ux, uy, uz, nx, ny, nz, bx, by, bz, f, cx, cy);
            // Right Rubber Pad (inside surface of Right Finger)
            drawThickLink(g2, 8.0, -w + 0.3, L7 - 0.5, -w + 0.3, 0.4, new Color(30, 30, 30), Color.BLACK, ux, uy, uz, nx, ny, nz, bx, by, bz, f, cx, cy);

            // TCP Red Dot
            int[] sTip = project(p3D, cx, cy);
            g2.setColor(new Color(255, 30, 30));
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
        int size = 150, step = 15;

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
        
        g2.setColor(new Color(230, 230, 235)); //floor
        g2.fillPolygon(floor);

        g2.setColor(new Color(195, 195, 205)); //grid lines
        for (int i = -size; i <= size; i += step) {
            int[] p1 = project(new double[] { i, -size, 0 }, cx, cy),
                    p2 = project(new double[] { i, size, 0 }, cx, cy);
            g2.drawLine(p1[0], p1[1], p2[0], p2[1]);
            int[] p3 = project(new double[] { -size, i, 0 }, cx, cy),
                    p4 = project(new double[] { size, i, 0 }, cx, cy);
            g2.drawLine(p3[0], p3[1], p4[0], p4[1]);
        }

        // Draw coordinate axes at origin
        int[] origin = project(new double[] { 0, 0, 0 }, cx, cy);
        int[] xAxis = project(new double[] { 35, 0, 0 }, cx, cy);
        int[] yAxis = project(new double[] { 0, 35, 0 }, cx, cy);
        int[] zAxis = project(new double[] { 0, 0, 35 }, cx, cy);

        g2.setStroke(new BasicStroke(2.0f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
        // X-axis: Red
        g2.setColor(new Color(220, 50, 50));
        g2.drawLine(origin[0], origin[1], xAxis[0], xAxis[1]);
        g2.drawString("X", xAxis[0] + 5, xAxis[1] + 2);

        // Y-axis: Green
        g2.setColor(new Color(50, 160, 50));
        g2.drawLine(origin[0], origin[1], yAxis[0], yAxis[1]);
        g2.drawString("Y", yAxis[0] + 5, yAxis[1] + 2);

        // Z-axis: Blue
        g2.setColor(new Color(50, 50, 220));
        g2.drawLine(origin[0], origin[1], zAxis[0], zAxis[1]);
        g2.drawString("Z", zAxis[0] - 2, zAxis[1] - 5);
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
        return computeFK(q1, q2, q3, q4, q5, q6, robot.isRightArmSelected);
    }

    public double[] computeFK(double q1, double q2, double q3, double q4, double q5, double q6, boolean isRight) {
        double[][] T = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
        double d2 = isRight ? (L2 + L3) : -(L2 + L3);
        double[][] params = {
                { 0, L1 + L0, 0, -Math.PI / 2, Math.toRadians(q1) },
                { -Math.PI / 2, d2, 0, -Math.PI / 2, Math.toRadians(q2) },
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
        return robot.isRightArmSelected ? computeEndEffectorMatrixRight() : computeEndEffectorMatrixLeft();
    }

    double[][] computeEndEffectorMatrixRight() {
        double[][] T = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
        double[][] params = {
                { 0, L1 + L0, 0, -Math.PI / 2, Math.toRadians(robot.getAnglesRight()[0]) },
                { -Math.PI / 2, L2 + L3, 0, -Math.PI / 2, Math.toRadians(robot.getAnglesRight()[1]) },
                { -Math.PI / 2, 0, 0, -Math.PI, Math.toRadians(robot.getAnglesRight()[2]) },
                { 0, 0, L4, -Math.PI / 2, Math.toRadians(robot.getAnglesRight()[3]) },
                { -Math.PI / 2, L5 + L6, 0, -Math.PI / 2, Math.toRadians(robot.getAnglesRight()[4]) },
                { -Math.PI / 2, 0, 0, 0, Math.toRadians(robot.getAnglesRight()[5]) }
        };
        for (int i = 0; i < NUM_JOINTS; i++) {
            T = multiply4x4(T,
                    getMDHMatrix(params[i][0], params[i][1], params[i][2], params[i][3], params[i][4]));
        }
        return multiply4x4(T, getToolMatrix());
    }

    double[][] computeEndEffectorMatrixLeft() {
        double[][] T = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
        double[][] params = {
                { 0, L1 + L0, 0, -Math.PI / 2, Math.toRadians(robot.getAnglesLeft()[0]) },
                { -Math.PI / 2, -(L2 + L3), 0, -Math.PI / 2, Math.toRadians(robot.getAnglesLeft()[1]) },
                { -Math.PI / 2, 0, 0, -Math.PI, Math.toRadians(robot.getAnglesLeft()[2]) },
                { 0, 0, L4, -Math.PI / 2, Math.toRadians(robot.getAnglesLeft()[3]) },
                { -Math.PI / 2, L5 + L6, 0, -Math.PI / 2, Math.toRadians(robot.getAnglesLeft()[4]) },
                { -Math.PI / 2, 0, 0, 0, Math.toRadians(robot.getAnglesLeft()[5]) }
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
        return robot.isRightArmSelected ? computeAllJoints3DRight() : computeAllJoints3DLeft();
    }

    public double[][] computeAllJoints3DRight() {
        double[][] pts = new double[NUM_JOINTS + 2][3];
        double[][] T = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
        pts[0] = new double[] { 0, 0, 0 };

        double[][] params = {
                { 0, L1 + L0, 0, -Math.PI / 2, Math.toRadians(robot.getAnglesRight()[0]) },
                { -Math.PI / 2, L2 + L3, 0, -Math.PI / 2, Math.toRadians(robot.getAnglesRight()[1]) },
                { -Math.PI / 2, 0, 0, -Math.PI, Math.toRadians(robot.getAnglesRight()[2]) },
                { 0, 0, L4, -Math.PI / 2, Math.toRadians(robot.getAnglesRight()[3]) },
                { -Math.PI / 2, L5 + L6, 0, -Math.PI / 2, Math.toRadians(robot.getAnglesRight()[4]) },
                { -Math.PI / 2, 0, 0, 0, Math.toRadians(robot.getAnglesRight()[5]) }
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

    public double[][] computeAllJoints3DLeft() {
        double[][] pts = new double[NUM_JOINTS + 2][3];
        double[][] T = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
        pts[0] = new double[] { 0, 0, 0 };

        double[][] params = {
                { 0, L1 + L0, 0, -Math.PI / 2, Math.toRadians(robot.getAnglesLeft()[0]) },
                { -Math.PI / 2, -(L2 + L3), 0, -Math.PI / 2, Math.toRadians(robot.getAnglesLeft()[1]) }, // Symmetrical shoulder offset
                { -Math.PI / 2, 0, 0, -Math.PI, Math.toRadians(robot.getAnglesLeft()[2]) },
                { 0, 0, L4, -Math.PI / 2, Math.toRadians(robot.getAnglesLeft()[3]) },
                { -Math.PI / 2, L5 + L6, 0, -Math.PI / 2, Math.toRadians(robot.getAnglesLeft()[4]) },
                { -Math.PI / 2, 0, 0, 0, Math.toRadians(robot.getAnglesLeft()[5]) }
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
