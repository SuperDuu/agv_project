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
        }        double[][] T_end_right = computeEndEffectorMatrixRight();
        double[][] T_end_left = computeEndEffectorMatrixLeft();

        // Draw shadows of joint spheres on the floor first
        if (robot.showGridCb.isSelected()) {
            for (double[] p : pts3dRight) {
                drawShadow(g2, p, 5.0, cx, cy);
            }
            for (double[] p : pts3dLeft) {
                drawShadow(g2, p, 5.0, cx, cy);
            }
            drawShadow(g2, new double[]{0,0,135}, 5.0, cx, cy); // Head shadow
        }

        // --- Depth Sorting and Perspective Scaling ---
        java.util.List<Renderable3D> drawables = new java.util.ArrayList<>();

        // Add 3D elements
        drawables.addAll(createPedestal(20.0, 10.0));
        drawables.addAll(createCylinder(new double[] { 0, 0, 10 }, new double[] { 0, 0, 125 }, 5.5, new Color(55, 58, 62))); // torso
        drawables.addAll(createCylinder(new double[] { 0, 0, 125 }, new double[] { 0, 0, 135 }, 2.5, new Color(60, 60, 60))); // neck
        drawables.addAll(createSphere(new double[] { 0, 0, 135 }, 4.5, new Color(75, 80, 85))); // head

        // Clavicles (collarbone segments)
        drawables.addAll(createCylinder(new double[] { 0, 0, 125 }, pts3dRight[1], 4.0, new Color(75, 75, 78)));
        drawables.addAll(createCylinder(new double[] { 0, 0, 125 }, pts3dLeft[1], 4.0, new Color(70, 85, 72)));

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
                new Color(75, 90, 75),
                new Color(85, 100, 85),
                new Color(95, 110, 95),
                new Color(105, 120, 105),
                new Color(115, 130, 115),
                new Color(125, 140, 125),
                new Color(135, 150, 135)
        };

        // Right Arm segments and joint spheres
        for (int i = 1; i < pts3dRight.length - 1; i++) {
            final double tw = ((i < tubeWidths.length) ? tubeWidths[i] : tubeWidths[tubeWidths.length - 1]) / 2.0;
            final Color color = (i < tubeColorsRight.length) ? tubeColorsRight[i] : tubeColorsRight[tubeColorsRight.length - 1];
            drawables.addAll(createSphere(pts3dRight[i], tw * 1.25, new Color(30, 100, 185))); // Right joints: anodized blue
            drawables.addAll(createCylinder(pts3dRight[i], pts3dRight[i + 1], tw, color));
        }
        drawables.addAll(createGripper(T_end_right, pts3dRight[7], robot.isRightArmSelected ? robot.isGripped : false));

        // Left Arm segments and joint spheres
        for (int i = 1; i < pts3dLeft.length - 1; i++) {
            final double tw = ((i < tubeWidths.length) ? tubeWidths[i] : tubeWidths[tubeWidths.length - 1]) / 2.0;
            final Color color = (i < tubeColorsLeft.length) ? tubeColorsLeft[i] : tubeColorsLeft[tubeColorsLeft.length - 1];
            drawables.addAll(createSphere(pts3dLeft[i], tw * 1.25, new Color(185, 50, 50))); // Left joints: anodized red
            drawables.addAll(createCylinder(pts3dLeft[i], pts3dLeft[i + 1], tw, color));
        }
        drawables.addAll(createGripper(T_end_left, pts3dLeft[7], !robot.isRightArmSelected ? robot.isGripped : false));

        // 3. Sort by depth (vz descending - Painter's Algorithm)
        drawables.sort((a, b) -> Double.compare(b.getDepth(), a.getDepth()));

        // 4. Render everything
        for (Renderable3D d : drawables) {
            d.render(g2, cx, cy);
        }

        if (clickTarget != null) {
            int[] sc = project(clickTarget, cx, cy);
            g2.setColor(new Color(255, 0, 0, 180));
            g2.fillOval(sc[0] - 4, sc[1] - 4, 8, 8);
        }

        g2.setColor(Color.BLACK);
        g2.drawString("Mô Phỏng Robot Song Arm Humanoid (6-Dof)", 10, 20);
    }

    // --- New 3D Rendering Engine Interface and Classes ---
    interface Renderable3D {
        double getDepth();
        void render(Graphics2D g2, int cx, int cy);
    }

    class PolygonFace implements Renderable3D {
        double[][] vertices;
        double[] normal;
        Color baseColor;
        double depth;

        PolygonFace(double[][] vertices, double[] normal, Color baseColor) {
            this.vertices = vertices;
            this.normal = normal.clone();
            this.baseColor = baseColor;
            
            double sumDepth = 0;
            for (double[] v : vertices) {
                sumDepth += getVz(v);
            }
            this.depth = sumDepth / vertices.length;
        }

        @Override
        public double getDepth() {
            return depth;
        }

        @Override
        public void render(Graphics2D g2, int cx, int cy) {
            int n = vertices.length;
            int[][] sc = new int[n][2];
            for (int i = 0; i < n; i++) {
                sc[i] = project(vertices[i], cx, cy);
            }
            
            // Backface culling
            double area = 0;
            for (int i = 0; i < n; i++) {
                int next = (i + 1) % n;
                area += (sc[i][0] * sc[next][1] - sc[next][0] * sc[i][1]);
            }
            if (area <= 0) {
                double az = Math.toRadians(camAz), el = Math.toRadians(camEl);
                double cAz = Math.cos(az), sAz = Math.sin(az), cEl = Math.cos(el), sEl = Math.sin(el);
                double vx = cAz * cEl;
                double vy = sAz * cEl;
                double vz = sEl;
                double dotView = normal[0]*vx + normal[1]*vy + normal[2]*vz;
                if (dotView < 0.0) {
                    return; // Culled
                }
            }

            // Light vector (from top-right-front in camera space)
            double[] lightDir = { 0.4, -0.4, 0.8 };
            double lightLen = Math.sqrt(lightDir[0]*lightDir[0] + lightDir[1]*lightDir[1] + lightDir[2]*lightDir[2]);
            lightDir[0] /= lightLen; lightDir[1] /= lightLen; lightDir[2] /= lightLen;
            
            double dotLight = normal[0]*lightDir[0] + normal[1]*lightDir[1] + normal[2]*lightDir[2];
            double diffuse = Math.max(0.0, dotLight);
            
            // Specular (Phong model)
            double az = Math.toRadians(camAz), el = Math.toRadians(camEl);
            double cAz = Math.cos(az), sAz = Math.sin(az), cEl = Math.cos(el), sEl = Math.sin(el);
            double vx = cAz * cEl;
            double vy = sAz * cEl;
            double vz = sEl;
            
            double rx = 2 * dotLight * normal[0] - lightDir[0];
            double ry = 2 * dotLight * normal[1] - lightDir[1];
            double rz = 2 * dotLight * normal[2] - lightDir[2];
            
            double dotSpec = rx*vx + ry*vy + rz*vz;
            double spec = 0.0;
            if (dotLight > 0.0 && dotSpec > 0.0) {
                spec = Math.pow(dotSpec, 12.0) * 0.4; // Shininess = 12, shinier specular
            }
            
            double ambient = 0.35;
            double totalLight = ambient + 0.65 * diffuse;
            
            int r = (int) (baseColor.getRed() * totalLight + 255 * spec);
            int g = (int) (baseColor.getGreen() * totalLight + 255 * spec);
            int b = (int) (baseColor.getBlue() * totalLight + 255 * spec);
            
            r = Math.min(255, Math.max(0, r));
            g = Math.min(255, Math.max(0, g));
            b = Math.min(255, Math.max(0, b));
            
            Polygon poly = new Polygon();
            for (int i = 0; i < n; i++) {
                poly.addPoint(sc[i][0], sc[i][1]);
            }
            
            g2.setColor(new Color(r, g, b));
            g2.fillPolygon(poly);
            
            // Subtle edge outlines for professional CAD wireframe feel
            g2.setColor(new Color(r / 2, g / 2, b / 2, 70));
            g2.setStroke(new BasicStroke(0.5f));
            g2.drawPolygon(poly);
        }
    }

    private static double[] crossProduct(double[] a, double[] b) {
        return new double[]{
            a[1]*b[2] - a[2]*b[1],
            a[2]*b[0] - a[0]*b[2],
            a[0]*b[1] - a[1]*b[0]
        };
    }
    
    private static double[] normalize(double[] a) {
        double len = Math.sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
        if (len < 1e-9) return a.clone();
        return new double[]{ a[0]/len, a[1]/len, a[2]/len };
    }

    private java.util.List<Renderable3D> createPedestal(double halfW, double h) {
        java.util.List<Renderable3D> list = new java.util.ArrayList<>();
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
        
        double[][] normals = {
            { 0, -1, 0 },
            { 1, 0, 0 },
            { 0, 1, 0 },
            { -1, 0, 0 }
        };
        
        int[][] sideIndices = {
            { 0, 1, 5, 4 },
            { 1, 2, 6, 5 },
            { 2, 3, 7, 6 },
            { 3, 0, 4, 7 }
        };
        
        Color sideColor = new Color(55, 58, 62);
        for (int i = 0; i < 4; i++) {
            double[][] verts = new double[4][3];
            for (int j = 0; j < 4; j++) verts[j] = corners[sideIndices[i][j]];
            list.add(new PolygonFace(verts, normals[i], sideColor));
        }
        
        double[][] topVerts = new double[4][3];
        for (int j = 0; j < 4; j++) topVerts[j] = corners[4 + j];
        list.add(new PolygonFace(topVerts, new double[]{ 0, 0, 1 }, new Color(75, 78, 84)));
        
        return list;
    }

    private java.util.List<Renderable3D> createCylinder(double[] p1, double[] p2, double radius, Color color) {
        java.util.List<Renderable3D> list = new java.util.ArrayList<>();
        double dx = p2[0] - p1[0];
        double dy = p2[1] - p1[1];
        double dz = p2[2] - p1[2];
        double len = Math.sqrt(dx*dx + dy*dy + dz*dz);
        if (len < 0.1) return list;
        
        double[] axis = { dx / len, dy / len, dz / len };
        double[] temp = Math.abs(axis[2]) < 0.9 ? new double[]{0,0,1} : new double[]{1,0,0};
        double[] u = crossProduct(axis, temp);
        u = normalize(u);
        double[] v = crossProduct(axis, u);
        v = normalize(v);
        
        int segments = 8;
        double[][] circle1 = new double[segments][3];
        double[][] circle2 = new double[segments][3];
        for (int i = 0; i < segments; i++) {
            double angle = i * 2.0 * Math.PI / segments;
            double cos = Math.cos(angle) * radius;
            double sin = Math.sin(angle) * radius;
            circle1[i] = new double[]{
                p1[0] + u[0] * cos + v[0] * sin,
                p1[1] + u[1] * cos + v[1] * sin,
                p1[2] + u[2] * cos + v[2] * sin
            };
            circle2[i] = new double[]{
                p2[0] + u[0] * cos + v[0] * sin,
                p2[1] + u[1] * cos + v[1] * sin,
                p2[2] + u[2] * cos + v[2] * sin
            };
        }
        
        for (int i = 0; i < segments; i++) {
            int next = (i + 1) % segments;
            double[][] vertices = {
                circle1[i], circle1[next], circle2[next], circle2[i]
            };
            double angle = (i + 0.5) * 2.0 * Math.PI / segments;
            double cos = Math.cos(angle);
            double sin = Math.sin(angle);
            double[] n = {
                u[0] * cos + v[0] * sin,
                u[1] * cos + v[1] * sin,
                u[2] * cos + v[2] * sin
            };
            n = normalize(n);
            list.add(new PolygonFace(vertices, n, color));
        }
        return list;
    }

    private java.util.List<Renderable3D> createSphere(double[] center, double radius, Color color) {
        java.util.List<Renderable3D> list = new java.util.ArrayList<>();
        int rings = 6;
        int sectors = 8;
        double[][] verts = new double[(rings + 1) * sectors][3];
        for (int r = 0; r <= rings; r++) {
            double phi = Math.PI * r / rings;
            double sinPhi = Math.sin(phi);
            double cosPhi = Math.cos(phi);
            for (int s = 0; s < sectors; s++) {
                double theta = 2.0 * Math.PI * s / sectors;
                double sinTheta = Math.sin(theta);
                double cosTheta = Math.cos(theta);
                int idx = r * sectors + s;
                verts[idx] = new double[]{
                    center[0] + radius * sinPhi * cosTheta,
                    center[1] + radius * sinPhi * sinTheta,
                    center[2] + radius * cosPhi
                };
            }
        }
        
        for (int r = 0; r < rings; r++) {
            for (int s = 0; s < sectors; s++) {
                int nextS = (s + 1) % sectors;
                int r0 = r * sectors;
                int r1 = (r + 1) * sectors;
                
                double[][] vertices = {
                    verts[r0 + s], verts[r0 + nextS], verts[r1 + nextS], verts[r1 + s]
                };
                
                double[] n = {
                    (vertices[0][0] + vertices[1][0] + vertices[2][0] + vertices[3][0]) / 4.0 - center[0],
                    (vertices[0][1] + vertices[1][1] + vertices[2][1] + vertices[3][1]) / 4.0 - center[1],
                    (vertices[0][2] + vertices[1][2] + vertices[2][2] + vertices[3][2]) / 4.0 - center[2]
                };
                n = normalize(n);
                list.add(new PolygonFace(vertices, n, color));
            }
        }
        return list;
    }

    private java.util.List<Renderable3D> createGripper(double[][] T, double[] p3D, boolean isGripped) {
        java.util.List<Renderable3D> list = new java.util.ArrayList<>();
        double ux = T[0][2], uy = T[1][2], uz = T[2][2];
        double nx = T[0][0], ny = T[1][0], nz = T[2][0];
        
        double wOpening = isGripped ? 0.8 : 2.5;
        
        double[] pActuatorStart = { p3D[0] - ux * 6.0, p3D[1] - uy * 6.0, p3D[2] - uz * 6.0 };
        double[] pRailCenter = { p3D[0] - ux * 4.0, p3D[1] - uy * 4.0, p3D[2] - uz * 4.0 };
        
        // Actuator
        list.addAll(createCylinder(pActuatorStart, pRailCenter, 1.8, new Color(45, 47, 50)));
        
        // Guide Rail
        double[] pRailLeft = { pRailCenter[0] + nx * (wOpening + 0.8), pRailCenter[1] + ny * (wOpening + 0.8), pRailCenter[2] + nz * (wOpening + 0.8) };
        double[] pRailRight = { pRailCenter[0] - nx * (wOpening + 0.8), pRailCenter[1] - ny * (wOpening + 0.8), pRailCenter[2] - nz * (wOpening + 0.8) };
        list.addAll(createCylinder(pRailLeft, pRailRight, 0.8, new Color(170, 172, 178)));
        
        double[] pLeftBase = { pRailCenter[0] + nx * wOpening, pRailCenter[1] + ny * wOpening, pRailCenter[2] + nz * wOpening };
        double[] pRightBase = { pRailCenter[0] - nx * wOpening, pRailCenter[1] - ny * wOpening, pRailCenter[2] - nz * wOpening };
        
        double[] l1 = pLeftBase;
        double[] l2 = { pLeftBase[0] + ux * 2.8 + nx * 0.6, pLeftBase[1] + uy * 2.8 + ny * 0.6, pLeftBase[2] + uz * 2.8 + nz * 0.6 };
        double[] l3 = { p3D[0] + ux * 0.8 + nx * 0.2, p3D[1] + uy * 0.8 + ny * 0.2, p3D[2] + uz * 0.8 + nz * 0.2 };
        
        double[] r1 = pRightBase;
        double[] r2 = { pRightBase[0] + ux * 2.8 - nx * 0.6, pRightBase[1] + uy * 2.8 - ny * 0.6, pRightBase[2] + uz * 2.8 - nz * 0.6 };
        double[] r3 = { p3D[0] + ux * 0.8 - nx * 0.2, p3D[1] + uy * 0.8 - ny * 0.2, p3D[2] + uz * 0.8 - nz * 0.2 };
        
        Color fingerColor = new Color(245, 125, 20);
        list.addAll(createCylinder(l1, l2, 0.8, fingerColor));
        list.addAll(createCylinder(l2, l3, 0.8, fingerColor));
        list.addAll(createCylinder(r1, r2, 0.8, fingerColor));
        list.addAll(createCylinder(r2, r3, 0.8, fingerColor));
        
        double[] lPadStart = l3;
        double[] lPadEnd = { l3[0] - ux * 2.2, l3[1] - uy * 2.2, l3[2] - uz * 2.2 };
        double[] rPadStart = r3;
        double[] rPadEnd = { r3[0] - ux * 2.2, r3[1] - uy * 2.2, r3[2] - uz * 2.2 };
        
        Color padColor = new Color(30, 30, 32);
        list.addAll(createCylinder(lPadStart, lPadEnd, 0.6, padColor));
        list.addAll(createCylinder(rPadStart, rPadEnd, 0.6, padColor));
        
        // Add actual TCP red dot as a tiny sphere
        list.addAll(createSphere(p3D, 0.5, new Color(230, 40, 40)));
        
        return list;
    }

    private void drawShadow(Graphics2D g2, double[] p, double radius, int cx, int cy) {
        int[] sc = project(new double[]{ p[0], p[1], 0 }, cx, cy);
        double f = getScaleFactor(new double[]{ p[0], p[1], 0 });
        int r = (int) (radius * scale * f);
        if (r < 1) return;
        
        g2.setColor(new Color(20, 20, 25, 40));
        g2.fillOval(sc[0] - r, sc[1] - r / 2, 2 * r, r);
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
