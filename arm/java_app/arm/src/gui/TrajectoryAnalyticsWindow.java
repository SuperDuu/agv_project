package gui;

import javax.swing.*;
import java.awt.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.util.ArrayList;
import java.util.List;

public class TrajectoryAnalyticsWindow extends JFrame {
    private final List<double[]> jointPath;
    private final List<double[]> cartesianPath;
    private final List<List<double[]>> candidatesPerStep;
    private final int[] selectedIndices;
    private final boolean isRight;

    public TrajectoryAnalyticsWindow(List<double[]> jointPath, List<double[]> cartesianPath,
                                     List<List<double[]>> candidatesPerStep, int[] selectedIndices, boolean isRight) {
        this.jointPath = jointPath != null ? jointPath : new ArrayList<>();
        this.cartesianPath = cartesianPath != null ? cartesianPath : new ArrayList<>();
        this.candidatesPerStep = candidatesPerStep != null ? candidatesPerStep : new ArrayList<>();
        this.selectedIndices = selectedIndices != null ? selectedIndices : new int[0];
        this.isRight = isRight;

        setTitle("Phân Tích Quỹ Đạo Descartes Toàn Cục & Cấu Hình Khớp — " + (isRight ? "Cánh Tay PHẢI" : "Cánh Tay TRÁI"));
        setSize(1100, 680);
        setLocationRelativeTo(null);
        setDefaultCloseOperation(DISPOSE_ON_CLOSE);

        JTabbedPane tabbedPane = new JTabbedPane();
        tabbedPane.setFont(new Font("Segoe UI", Font.BOLD, 13));

        tabbedPane.addTab("📊 Biểu Đồ Không Gian Khớp", new JointTrajectoryChartPanel());
        tabbedPane.addTab("🕸️ Đồ Thị Bậc Thang Descartes (Ladder Graph)", new DescartesGraphPanel());

        getContentPane().add(tabbedPane, BorderLayout.CENTER);
    }

    // --- Panel 1: Joint Trajectory Line Chart ---
    private class JointTrajectoryChartPanel extends JPanel {
        private int hoverIndex = -1;
        private final Color[] jointColors = {
            new Color(230, 81, 0),    // J1: Cam đậm
            new Color(76, 175, 80),   // J2: Xanh lá
            new Color(33, 150, 243),  // J3: Xanh dương
            new Color(156, 39, 176),  // J4: Tím
            new Color(251, 192, 45),  // J5: Vàng
            new Color(0, 188, 212)    // J6: Xanh lam
        };
        private final String[] jointNames = {"Khớp 1", "Khớp 2", "Khớp 3", "Khớp 4", "Khớp 5", "Khớp 6"};

        public JointTrajectoryChartPanel() {
            setBackground(new Color(25, 25, 30));
            addMouseMotionListener(new MouseAdapter() {
                @Override
                public void mouseMoved(MouseEvent e) {
                    if (jointPath.isEmpty()) return;
                    int w = getWidth();
                    int paddingLeft = 60;
                    int paddingRight = 180;
                    int chartWidth = w - paddingLeft - paddingRight;
                    if (chartWidth <= 0) return;

                    double stepX = (double) chartWidth / Math.max(1, jointPath.size() - 1);
                    int idx = (int) Math.round((e.getX() - paddingLeft) / stepX);
                    if (idx >= 0 && idx < jointPath.size()) {
                        if (hoverIndex != idx) {
                            hoverIndex = idx;
                            repaint();
                        }
                    } else {
                        if (hoverIndex != -1) {
                            hoverIndex = -1;
                            repaint();
                        }
                    }
                }
            });
            addMouseListener(new MouseAdapter() {
                @Override
                public void mouseExited(MouseEvent e) {
                    hoverIndex = -1;
                    repaint();
                }
            });
        }

        @Override
        protected void paintComponent(Graphics g) {
            super.paintComponent(g);
            Graphics2D g2 = (Graphics2D) g;
            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

            int w = getWidth();
            int h = getHeight();

            int paddingTop = 50;
            int paddingBottom = 60;
            int paddingLeft = 60;
            int paddingRight = 180;

            int chartWidth = w - paddingLeft - paddingRight;
            int chartHeight = h - paddingTop - paddingBottom;

            if (chartWidth <= 0 || chartHeight <= 0) return;

            // Draw Background Grid & Axes
            g2.setColor(new Color(40, 40, 48));
            g2.fillRect(paddingLeft, paddingTop, chartWidth, chartHeight);

            g2.setColor(new Color(70, 70, 80));
            g2.setStroke(new BasicStroke(1.0f));
            // Draw horizontal grid lines (every 30 degrees)
            for (int angle = -180; angle <= 180; angle += 30) {
                int y = paddingTop + chartHeight - (int) ((angle + 180.0) / 360.0 * chartHeight);
                g2.drawLine(paddingLeft, y, paddingLeft + chartWidth, y);
                g2.setColor(new Color(150, 150, 160));
                g2.setFont(new Font("Segoe UI", Font.PLAIN, 10));
                g2.drawString(angle + "°", 15, y + 4);
                g2.setColor(new Color(55, 55, 65));
            }

            // Draw vertical grid lines
            if (!jointPath.isEmpty()) {
                g2.setColor(new Color(55, 55, 65));
                int stepSize = Math.max(1, jointPath.size() / 10);
                for (int i = 0; i < jointPath.size(); i += stepSize) {
                    int x = paddingLeft + (int) ((double) i / (jointPath.size() - 1) * chartWidth);
                    g2.drawLine(x, paddingTop, x, paddingTop + chartHeight);
                    g2.setColor(new Color(150, 150, 160));
                    g2.drawString("Pt " + i, x - 12, paddingTop + chartHeight + 20);
                    g2.setColor(new Color(55, 55, 65));
                }
            }

            // Draw Title
            g2.setColor(Color.WHITE);
            g2.setFont(new Font("Segoe UI", Font.BOLD, 16));
            g2.drawString("Đồ Thị Góc Khớp Theo Thời Gian (Joint-Space Trajectory)", paddingLeft, 30);

            // Draw Curves
            if (!jointPath.isEmpty()) {
                double stepX = (double) chartWidth / (jointPath.size() - 1);
                g2.setStroke(new BasicStroke(2.0f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
                
                for (int j = 0; j < 6; j++) {
                    g2.setColor(jointColors[j]);
                    Path2D.Double path = new Path2D.Double();
                    boolean first = true;
                    for (int i = 0; i < jointPath.size(); i++) {
                        double[] q = jointPath.get(i);
                        if (q == null) continue;
                        double val = q[j];
                        double px = paddingLeft + i * stepX;
                        double py = paddingTop + chartHeight - ((val + 180.0) / 360.0 * chartHeight);
                        if (first) {
                            path.moveTo(px, py);
                            first = false;
                        } else {
                            path.lineTo(px, py);
                        }
                    }
                    g2.draw(path);
                }
            }

            // Draw Legend
            int legendX = w - paddingRight + 20;
            int legendY = paddingTop + 10;
            g2.setFont(new Font("Segoe UI", Font.BOLD, 12));
            g2.setColor(Color.LIGHT_GRAY);
            g2.drawString("CHÚ GIẢI", legendX, legendY);
            g2.setFont(new Font("Segoe UI", Font.PLAIN, 12));
            for (int j = 0; j < 6; j++) {
                int y = legendY + 25 + j * 30;
                g2.setColor(jointColors[j]);
                g2.fillRect(legendX, y - 10, 15, 10);
                g2.setColor(Color.WHITE);
                g2.drawString(jointNames[j], legendX + 25, y);
            }

            // Hover indicator
            if (hoverIndex >= 0 && hoverIndex < jointPath.size()) {
                double stepX = (double) chartWidth / (jointPath.size() - 1);
                int hx = paddingLeft + (int) (hoverIndex * stepX);
                g2.setColor(new Color(255, 255, 255, 100));
                g2.setStroke(new BasicStroke(1.0f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_BEVEL, 0, new float[]{4, 4}, 0));
                g2.drawLine(hx, paddingTop, hx, paddingTop + chartHeight);

                // Draw Hover Values Box
                double[] q = jointPath.get(hoverIndex);
                if (q != null) {
                    int boxW = 150;
                    int boxH = 175;
                    int boxX = hx + 15;
                    if (boxX + boxW > w - paddingRight) {
                        boxX = hx - 15 - boxW;
                    }
                    int boxY = h - paddingBottom - boxH - 10;

                    g2.setColor(new Color(40, 40, 50, 240));
                    g2.fillRoundRect(boxX, boxY, boxW, boxH, 8, 8);
                    g2.setColor(Color.WHITE);
                    g2.setStroke(new BasicStroke(1.0f));
                    g2.drawRoundRect(boxX, boxY, boxW, boxH, 8, 8);

                    g2.setFont(new Font("Segoe UI", Font.BOLD, 11));
                    g2.drawString("Điểm số: " + hoverIndex, boxX + 10, boxY + 20);
                    g2.setFont(new Font("Segoe UI", Font.PLAIN, 11));
                    for (int j = 0; j < 6; j++) {
                        g2.setColor(jointColors[j]);
                        g2.drawString(String.format("%s: %.1f°", jointNames[j], q[j]), boxX + 10, boxY + 42 + j * 20);
                    }
                }
            }
        }
    }

    // --- Panel 2: Descartes Ladder Graph ---
    private class DescartesGraphPanel extends JPanel {
        private int scrollOffset = 0;
        private final int colWidth = 100;
        private final int nodeRadius = 8;
        private Point selectedNode = null; // col, idx
        private Point hoveredNode = null;

        public DescartesGraphPanel() {
            setBackground(new Color(20, 20, 25));
            
            // Allow horizontal dragging to scroll if graph is wide
            MouseAdapter dragAdapter = new MouseAdapter() {
                private int startX;
                @Override
                public void mousePressed(MouseEvent e) {
                    startX = e.getX();
                }
                @Override
                public void mouseDragged(MouseEvent e) {
                    int dx = e.getX() - startX;
                    scrollOffset += dx;
                    int maxScroll = 0;
                    int minScroll = -Math.max(0, candidatesPerStep.size() * colWidth - getWidth() + 150);
                    if (scrollOffset > maxScroll) scrollOffset = maxScroll;
                    if (scrollOffset < minScroll) scrollOffset = minScroll;
                    startX = e.getX();
                    repaint();
                }
                @Override
                public void mouseMoved(MouseEvent e) {
                    Point node = findNodeAt(e.getX(), e.getY());
                    if (node != hoveredNode) {
                        hoveredNode = node;
                        repaint();
                    }
                }
                @Override
                public void mouseClicked(MouseEvent e) {
                    selectedNode = findNodeAt(e.getX(), e.getY());
                    repaint();
                }
            };
            addMouseListener(dragAdapter);
            addMouseMotionListener(dragAdapter);
        }

        private Point findNodeAt(int mx, int my) {
            int paddingLeft = 50;
            int startY = 100;
            int rowHeight = 45;

            for (int col = 0; col < candidatesPerStep.size(); col++) {
                int cx = paddingLeft + scrollOffset + col * colWidth;
                List<double[]> candidates = candidatesPerStep.get(col);
                if (candidates == null) continue;
                for (int i = 0; i < candidates.size(); i++) {
                    int cy = startY + i * rowHeight;
                    double dist = Point2D.distance(mx, my, cx, cy);
                    if (dist <= nodeRadius + 4) {
                        return new Point(col, i);
                    }
                }
            }
            return null;
        }

        @Override
        protected void paintComponent(Graphics g) {
            super.paintComponent(g);
            Graphics2D g2 = (Graphics2D) g;
            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

            int w = getWidth();
            int h = getHeight();

            // Draw Header Title
            g2.setColor(Color.WHITE);
            g2.setFont(new Font("Segoe UI", Font.BOLD, 16));
            g2.drawString("Đồ Thị Quy Hoạch Động Toàn Cục Descartes (Ladder Graph)", 30, 30);
            g2.setFont(new Font("Segoe UI", Font.PLAIN, 12));
            g2.setColor(Color.GRAY);
            g2.drawString("Kéo chuột trái để cuộn ngang. Click chọn node để xem thông tin khớp.", 30, 50);

            // Legends
            int legX = w - 280;
            g2.setColor(new Color(40, 180, 100)); // Green path
            g2.fillRect(legX, 20, 20, 4);
            g2.setColor(Color.WHITE);
            g2.drawString("Đường đi tối ưu (Viterbi)", legX + 25, 25);

            g2.setColor(new Color(120, 120, 130)); // Gray edges
            g2.fillRect(legX, 40, 20, 1);
            g2.setColor(Color.WHITE);
            g2.drawString("Các liên kết khả dĩ", legX + 25, 45);

            int paddingLeft = 50;
            int startY = 100;
            int rowHeight = 45;

            // Draw all transition edges first (underneath nodes)
            g2.setStroke(new BasicStroke(1.0f));
            for (int col = 0; col < candidatesPerStep.size() - 1; col++) {
                int cx1 = paddingLeft + scrollOffset + col * colWidth;
                int cx2 = cx1 + colWidth;
                List<double[]> layer1 = candidatesPerStep.get(col);
                List<double[]> layer2 = candidatesPerStep.get(col + 1);
                if (layer1 == null || layer2 == null) continue;

                for (int i = 0; i < layer1.size(); i++) {
                    double[] q1 = layer1.get(i);
                    int cy1 = startY + i * rowHeight;
                    for (int j = 0; j < layer2.size(); j++) {
                        double[] q2 = layer2.get(j);
                        int cy2 = startY + j * rowHeight;

                        // Calculate transition cost to set transparency
                        double maxJump = 0.0;
                        for (int k = 0; k < 6; k++) {
                            double diff = Math.abs(q2[k] - q1[k]);
                            if (diff > maxJump) maxJump = diff;
                        }
                        if (maxJump < 90.0) {
                            int alpha = (int) Math.max(5, 50 * (1.0 - maxJump / 90.0));
                            g2.setColor(new Color(150, 150, 160, alpha));
                            g2.drawLine(cx1, cy1, cx2, cy2);
                        }
                    }
                }
            }

            // Draw Optimal Viterbi Path (thick green line)
            g2.setStroke(new BasicStroke(3.0f));
            g2.setColor(new Color(40, 220, 120));
            int lastX = -1, lastY = -1;
            for (int col = 0; col < candidatesPerStep.size(); col++) {
                int cx = paddingLeft + scrollOffset + col * colWidth;
                int chosenIdx = selectedIndices[col];
                if (chosenIdx >= 0) {
                    int cy = startY + chosenIdx * rowHeight;
                    if (lastX != -1) {
                        g2.drawLine(lastX, lastY, cx, cy);
                    }
                    lastX = cx;
                    lastY = cy;
                } else {
                    lastX = -1;
                    lastY = -1; // path broken by null
                }
            }

            // Draw Nodes
            for (int col = 0; col < candidatesPerStep.size(); col++) {
                int cx = paddingLeft + scrollOffset + col * colWidth;
                List<double[]> candidates = candidatesPerStep.get(col);
                
                // Draw column label
                g2.setColor(Color.GRAY);
                g2.setFont(new Font("Segoe UI", Font.BOLD, 10));
                g2.drawString("Pt " + col, cx - 12, startY - 25);

                if (candidates == null || candidates.isEmpty()) {
                    // Draw red "NULL" node
                    g2.setColor(new Color(239, 83, 80));
                    g2.fillOval(cx - 5, startY - 5, 10, 10);
                    g2.setFont(new Font("Segoe UI", Font.PLAIN, 9));
                    g2.drawString("Empty", cx - 12, startY + 15);
                    continue;
                }

                for (int i = 0; i < candidates.size(); i++) {
                    int cy = startY + i * rowHeight;
                    boolean isChosen = (selectedIndices[col] == i);

                    if (isChosen) {
                        g2.setColor(new Color(40, 220, 120));
                        g2.fillOval(cx - nodeRadius - 2, cy - nodeRadius - 2, (nodeRadius + 2) * 2, (nodeRadius + 2) * 2);
                        g2.setColor(Color.WHITE);
                        g2.fillOval(cx - nodeRadius, cy - nodeRadius, nodeRadius * 2, nodeRadius * 2);
                    } else {
                        g2.setColor(new Color(80, 80, 90));
                        g2.fillOval(cx - nodeRadius, cy - nodeRadius, nodeRadius * 2, nodeRadius * 2);
                        g2.setColor(new Color(180, 180, 190));
                        g2.fillOval(cx - nodeRadius + 2, cy - nodeRadius + 2, (nodeRadius - 2) * 2, (nodeRadius - 2) * 2);
                    }

                    // Hover ring
                    if (hoveredNode != null && hoveredNode.x == col && hoveredNode.y == i) {
                        g2.setColor(Color.WHITE);
                        g2.setStroke(new BasicStroke(1.5f));
                        g2.drawOval(cx - nodeRadius - 4, cy - nodeRadius - 4, (nodeRadius + 4) * 2, (nodeRadius + 4) * 2);
                    }
                }
            }

            // Draw Node detail box (clicked node)
            Point targetNode = selectedNode != null ? selectedNode : hoveredNode;
            if (targetNode != null && targetNode.x < candidatesPerStep.size()) {
                List<double[]> candidates = candidatesPerStep.get(targetNode.x);
                if (candidates != null && targetNode.y < candidates.size()) {
                    double[] q = candidates.get(targetNode.y);
                    int boxW = 200;
                    int boxH = 180;
                    int boxX = w - boxW - 30;
                    int boxY = h - boxH - 50;

                    g2.setColor(new Color(30, 30, 40, 245));
                    g2.fillRoundRect(boxX, boxY, boxW, boxH, 10, 10);
                    g2.setColor(new Color(70, 70, 90));
                    g2.setStroke(new BasicStroke(1.5f));
                    g2.drawRoundRect(boxX, boxY, boxW, boxH, 10, 10);

                    g2.setColor(Color.WHITE);
                    g2.setFont(new Font("Segoe UI", Font.BOLD, 12));
                    g2.drawString(String.format("Node Info [Cột %d, Nghiệm %d]", targetNode.x, targetNode.y), boxX + 15, boxY + 25);
                    g2.setFont(new Font("Segoe UI", Font.PLAIN, 11));
                    g2.setColor(new Color(200, 200, 210));
                    for (int j = 0; j < 6; j++) {
                        g2.drawString(String.format("Khớp %d: %.2f°", j + 1, q[j]), boxX + 20, boxY + 50 + j * 20);
                    }
                }
            }
        }
    }
}
