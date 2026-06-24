package gui;

import javax.swing.*;
import java.util.ArrayList;
import java.util.List;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import static kinematics.Kinematics.*;

public final class MainFrame extends JFrame implements ActionListener, ChangeListener {
    private static final double MAX_IK_POSITION_ERROR = 1.5; // General IK threshold (allow a bit of error)
    private static final double TRAJ_RELAXED_ERROR = 2.50; // Trajectory fallback threshold
    double[] anglesRight = { 0, 0, 0, -30, 0, 0 };
    double[] targetAnglesRight = { 0, 0, 0, -30, 0, 0 };
    double[] lastSentAnglesRight = { -999, -999, -999, -999, -999, -999 };

    double[] anglesLeft = { 0, 0, 0, 30, 0, 0 };
    double[] targetAnglesLeft = { 0, 0, 0, 30, 0, 0 };
    double[] lastSentAnglesLeft = { -999, -999, -999, -999, -999, -999 };

    public double[] angles = anglesRight;
    public double[] targetAngles = targetAnglesRight;
    public double[] lastSentAngles = lastSentAnglesRight;

    public boolean isRightArmSelected = true;

    public double[] getAnglesRight() { return anglesRight; }
    public double[] getAnglesLeft() { return anglesLeft; }

    JPanel controlPanel = new JPanel();
    JPanel topPanel = new JPanel();

    JSlider[] sliders = new JSlider[NUM_JOINTS];
    JLabel[] angleLbls = new JLabel[NUM_JOINTS];

    ArmPanel armPanel;
    JLabel endEffectorLabel = new JLabel("Tọa độ kẹp (X, Y, Z): 0, 0, 0");

    JCheckBox showGridCb = new JCheckBox("Hiện Lưới", true);
    JCheckBox showTrailCb = new JCheckBox("Hiện Vết Quỹ Đạo", false);

    JComboBox<String> configCombo = new JComboBox<>(new String[] { "Right / Elbow Up (+)", "Left / Elbow Down (-)" });
    JSlider alphaSlider = new JSlider(0, 360, 180);
    JCheckBox fixedAlphaCb = new JCheckBox("Alpha cố định", false);

    JButton btnReset = new JButton("Reset");
    JButton btnDemo = new JButton("Quỹ đạo Xoắn ốc");
    JButton btnTopView = new JButton("Hệ Trục (Top)");
    JButton btnPersp = new JButton("3D Perspective");

    JTextField txX = new JTextField("0", 5);
    JTextField txY = new JTextField("0", 5);
    JTextField txZ = new JTextField("0", 5);
    JSlider slX = new JSlider(-50, 50, 0);
    JSlider slY = new JSlider(-50, 50, 0);
    JSlider slZ = new JSlider(-20, 80, 0);
    JButton btnGoto = new JButton("Đến tọa độ");
    JButton btnSyncCoords = new JButton("Lấy tọa độ hiện tại");
    JLabel gotoStatus = new JLabel(" ");
    boolean manualMode = false;
    boolean isUpdatingFromFK = false;

    JComboBox<String> trajTypeCombo = new JComboBox<>(new String[] { "Đường thẳng", "Xoắn ốc" });
    JTextField txtLStartX = new JTextField("-16.3", 4);
    JTextField txtLStartY = new JTextField("13.8", 4);
    JTextField txtLStartZ = new JTextField("20", 4);
    JTextField txtLEndX = new JTextField("-13.3", 4);
    JTextField txtLEndY = new JTextField("-5.5", 4);
    JTextField txtLEndZ = new JTextField("20.1", 4);
    JTextField txtSStartX = new JTextField("-12", 4);
    JTextField txtSStartY = new JTextField("-5.5", 4);
    JTextField txtSStartZ = new JTextField("15", 4);
    JTextField txtSR = new JTextField("5", 4);
    JTextField txtSH = new JTextField("10", 4);
    JTextField txtSK = new JTextField("3", 4);
    JTabbedPane mainTabs;

    JCheckBox fixedHeightCb = new JCheckBox("Chế độ Click (chiều cao cố định)", false);
    JSpinner fixedHeightSpinner = new JSpinner(new SpinnerNumberModel(0.0, -200.0, 500.0, 1.0));
    boolean fixedHeightMode = false;
    JSlider speedSlider = new JSlider(0, 120, 60);
    JLabel speedLabel = new JLabel("60 °/s");
    private static final int MOTION_DT_MS = 30;
    Timer motionTimer;

    boolean showWorkspace = false;
    Thread explorationThread;
    String lastLimitInfo = "";

    boolean isGripped = false;
    Timer trajectoryTimer;
    double[] trajectoryLastQ = null;
    double trajectoryLastAlpha = 0.0;
    String trajectoryLockedCfg = "+";
    boolean ikSelectionLogEnabled = true;

    comm.UartManager uartManager = new comm.UartManager();

    private void trajDebug(String phase, String msg) {
        System.out.println("[TRAJ][" + phase + "] " + msg);
    }

    public MainFrame() {
        armPanel = new ArmPanel(this);

        setLayout(new BorderLayout());
        add(armPanel, BorderLayout.CENTER);

        buildControlPanel();
        buildTopPanel();
        buildMenuBar();

        setSize(1100, 750);
        setTitle("Mô Phỏng cánh tay Robot 6-DOF");
        setLocationRelativeTo(null);
        setDefaultCloseOperation(EXIT_ON_CLOSE);

        startMotionTimer();
        updateArm();
    }

    public double getFixedHeight() {
        return ((Number) fixedHeightSpinner.getValue()).doubleValue();
    }

    private void buildControlPanel() {
        add(BorderLayout.EAST, controlPanel);
        controlPanel.setLayout(new BorderLayout());
        controlPanel.setPreferredSize(new Dimension(340, 0));

        mainTabs = new JTabbedPane();

        JPanel manualPanel = new JPanel();
        manualPanel.setLayout(new BoxLayout(manualPanel, BoxLayout.Y_AXIS));
        manualPanel.setBorder(BorderFactory.createEmptyBorder(5, 5, 5, 5));

        JComboBox<String> armCombo = new JComboBox<>(new String[] { "Cánh Tay Phải (Right Arm)", "Cánh Tay Trái (Left Arm)" });
        armCombo.setMaximumSize(new Dimension(320, 30));
        armCombo.setAlignmentX(Component.LEFT_ALIGNMENT);
        armCombo.addActionListener(e -> {
            boolean rightSelected = armCombo.getSelectedIndex() == 0;
            if (rightSelected) {
                angles = anglesRight;
                targetAngles = targetAnglesRight;
                lastSentAngles = lastSentAnglesRight;
                isRightArmSelected = true;
                // Left arm (inactive) returns to home smoothly
                for (int j = 1; j < NUM_JOINTS; j++) {
                    targetAnglesLeft[j] = (j == 3) ? 30.0 : 0;
                }
            } else {
                angles = anglesLeft;
                targetAngles = targetAnglesLeft;
                lastSentAngles = lastSentAnglesLeft;
                isRightArmSelected = false;
                // Right arm (inactive) returns to home smoothly
                for (int j = 1; j < NUM_JOINTS; j++) {
                    targetAnglesRight[j] = (j == 3) ? -30.0 : 0;
                }
            }
            // Sync sliders and labels to the newly selected arm's angles and limits
            double[] minLimits = isRightArmSelected ? JOINT_MIN_RIGHT : JOINT_MIN_LEFT;
            double[] maxLimits = isRightArmSelected ? JOINT_MAX_RIGHT : JOINT_MAX_LEFT;
            for (int j = 0; j < NUM_JOINTS; j++) {
                sliders[j].removeChangeListener(this);
                int minVal = (int) Math.min(minLimits[j], maxLimits[j]);
                int maxVal = (int) Math.max(minLimits[j], maxLimits[j]);
                sliders[j].setMinimum(minVal);
                sliders[j].setMaximum(maxVal);
                sliders[j].setValue((int) Math.round(angles[j]));
                sliders[j].addChangeListener(this);
                angleLbls[j].setText((int) Math.round(angles[j]) + "°");
            }
            startMotionTimer();
            updateArm();
        });
        manualPanel.add(armCombo);
        manualPanel.add(Box.createVerticalStrut(10));

        for (int i = 0; i < NUM_JOINTS; i++) {
            JPanel jointPanel = new JPanel(new BorderLayout());
            jointPanel.setBorder(BorderFactory.createEmptyBorder(5, 5, 5, 5));

            JLabel nameLabel = new JLabel(JOINT_NAMES[i]);
            jointPanel.add(nameLabel, BorderLayout.NORTH);

            int minVal = (int) Math.min(JOINT_MIN_RIGHT[i], JOINT_MAX_RIGHT[i]);
            int maxVal = (int) Math.max(JOINT_MIN_RIGHT[i], JOINT_MAX_RIGHT[i]);
            sliders[i] = new JSlider(minVal, maxVal, (int) angles[i]);
            sliders[i].setMajorTickSpacing(30);
            sliders[i].setPaintTicks(true);
            sliders[i].addChangeListener(this);

            angleLbls[i] = new JLabel(sliders[i].getValue() + "°");
            angleLbls[i].setPreferredSize(new Dimension(50, 20));
            angleLbls[i].setHorizontalAlignment(SwingConstants.RIGHT);

            jointPanel.add(sliders[i], BorderLayout.CENTER);
            jointPanel.add(angleLbls[i], BorderLayout.EAST);
            manualPanel.add(jointPanel);
        }

        endEffectorLabel.setFont(new Font("Arial", Font.BOLD, 12));
        endEffectorLabel.setBorder(BorderFactory.createEmptyBorder(10, 10, 5, 10));
        manualPanel.add(endEffectorLabel);

        JPanel gotoPanel = new JPanel();
        gotoPanel.setLayout(new BoxLayout(gotoPanel, BoxLayout.Y_AXIS));
        gotoPanel.setBorder(BorderFactory.createTitledBorder("Nhập tọa độ (X, Y, Z)"));

        JPanel coordRow = new JPanel(new GridLayout(3, 1, 4, 2));

        JPanel rowX = new JPanel(new BorderLayout(5, 0));
        rowX.add(new JLabel("X:"), BorderLayout.WEST);
        rowX.add(slX, BorderLayout.CENTER);
        txX.setPreferredSize(new Dimension(50, 20));
        rowX.add(txX, BorderLayout.EAST);

        JPanel rowY = new JPanel(new BorderLayout(5, 0));
        rowY.add(new JLabel("Y:"), BorderLayout.WEST);
        rowY.add(slY, BorderLayout.CENTER);
        txY.setPreferredSize(new Dimension(50, 20));
        rowY.add(txY, BorderLayout.EAST);

        JPanel rowZ = new JPanel(new BorderLayout(5, 0));
        rowZ.add(new JLabel("Z:"), BorderLayout.WEST);
        rowZ.add(slZ, BorderLayout.CENTER);
        txZ.setPreferredSize(new Dimension(50, 20));
        rowZ.add(txZ, BorderLayout.EAST);

        coordRow.add(rowX);
        coordRow.add(rowY);
        coordRow.add(rowZ);
        gotoPanel.add(coordRow);

        slX.addChangeListener(this);
        slY.addChangeListener(this);
        slZ.addChangeListener(this);

        JPanel btnRow = new JPanel(new FlowLayout(FlowLayout.LEFT, 4, 2));
        btnRow.add(btnGoto);
        btnRow.add(btnSyncCoords);
        btnRow.add(gotoStatus);
        gotoPanel.add(btnRow);
        manualPanel.add(gotoPanel);

        btnGoto.addActionListener(this);
        btnSyncCoords.addActionListener(this);
        ActionListener gotoAction = e -> gotoCoordinate();
        txX.addActionListener(gotoAction);
        txY.addActionListener(gotoAction);
        txZ.addActionListener(gotoAction);

        JPanel fixedPanel = new JPanel();
        fixedPanel.setLayout(new BoxLayout(fixedPanel, BoxLayout.Y_AXIS));
        fixedPanel.setBorder(BorderFactory.createTitledBorder("Chế độ Click (IK tại Z cố định)"));

        JPanel fhRow1 = new JPanel(new FlowLayout(FlowLayout.LEFT, 4, 2));
        fhRow1.add(fixedHeightCb);
        fixedPanel.add(fhRow1);

        JPanel fhRow2 = new JPanel(new FlowLayout(FlowLayout.LEFT, 4, 2));
        fhRow2.add(new JLabel("Chiều cao Z cố định:"));
        fixedHeightSpinner.setPreferredSize(new Dimension(80, 25));
        fhRow2.add(fixedHeightSpinner);
        fixedPanel.add(fhRow2);

        JLabel fhHint = new JLabel(
                "<html><i>Nhấp chuột phải trên màn hình<br>để di chuyển cánh tay đến vị trí đó</i></html>");
        fhHint.setBorder(BorderFactory.createEmptyBorder(2, 8, 4, 4));
        fhHint.setForeground(new Color(100, 100, 100));
        fixedPanel.add(fhHint);
        manualPanel.add(fixedPanel);
        fixedHeightCb.addActionListener(this);

        JPanel configPanel = new JPanel(new BorderLayout());
        configPanel.setBorder(BorderFactory.createTitledBorder("Cấu hình (IK Pose)"));
        configPanel.add(configCombo);
        manualPanel.add(configPanel);
        JPanel alphaPanel = new JPanel(new BorderLayout());
        alphaPanel.setBorder(BorderFactory.createTitledBorder("Bending Alpha (Target orientation)"));
        alphaSlider.setMajorTickSpacing(90);
        alphaSlider.setPaintTicks(true);
        alphaSlider.setPaintLabels(true);
        alphaPanel.add(alphaSlider, BorderLayout.CENTER);
        alphaPanel.add(fixedAlphaCb, BorderLayout.SOUTH);
        manualPanel.add(alphaPanel);

        JScrollPane scrollManual = new JScrollPane(manualPanel);
        scrollManual.setBorder(null);
        mainTabs.addTab("Điều khiển", scrollManual);

        mainTabs.addTab("Quỹ đạo", buildTrajectoryPanel());

        controlPanel.add(mainTabs, BorderLayout.CENTER);
    }

    private JPanel buildTrajectoryPanel() {
        JPanel trajPanel = new JPanel();
        trajPanel.setLayout(new BoxLayout(trajPanel, BoxLayout.Y_AXIS));
        trajPanel.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));

        JPanel topP = new JPanel(new FlowLayout(FlowLayout.LEFT));
        topP.add(new JLabel("Loại quỹ đạo: "));
        topP.add(trajTypeCombo);
        trajPanel.add(topP);
        JPanel cards = new JPanel(new CardLayout());

        JPanel pLine = new JPanel(new GridLayout(3, 1, 5, 2));
        pLine.setBorder(BorderFactory.createTitledBorder("Thông số Đường thẳng"));

        JPanel l1 = new JPanel(new FlowLayout(FlowLayout.LEFT));
        l1.add(new JLabel("Điểm ĐẦU (X,Y,Z):"));
        l1.add(txtLStartX);
        l1.add(txtLStartY);
        l1.add(txtLStartZ);

        JPanel l2 = new JPanel(new FlowLayout(FlowLayout.LEFT));
        l2.add(new JLabel("Điểm CUỐI (X,Y,Z):"));
        l2.add(txtLEndX);
        l2.add(txtLEndY);
        l2.add(txtLEndZ);

        pLine.add(l1);
        pLine.add(l2);
        cards.add(pLine, "Đường thẳng");

        // --- Card 2: Spiral ---
        JPanel pSpiral = new JPanel(new GridLayout(4, 1, 5, 2));
        pSpiral.setBorder(BorderFactory.createTitledBorder("Thông số Xoắn ốc"));

        JPanel s1 = new JPanel(new FlowLayout(FlowLayout.LEFT));
        s1.add(new JLabel("Tâm (X,Y,Z):"));
        s1.add(txtSStartX);
        s1.add(txtSStartY);
        s1.add(txtSStartZ);

        JPanel s2 = new JPanel(new FlowLayout(FlowLayout.LEFT));
        s2.add(new JLabel("Bk R:"));
        s2.add(txtSR);
        s2.add(new JLabel(" Cao H:"));
        s2.add(txtSH);

        JPanel s3 = new JPanel(new FlowLayout(FlowLayout.LEFT));
        s3.add(new JLabel("Số vòng K:"));
        s3.add(txtSK);

        pSpiral.add(s1);
        pSpiral.add(s2);
        pSpiral.add(s3);
        cards.add(pSpiral, "Xoắn ốc");

        trajPanel.add(cards);

        trajTypeCombo.addItemListener(e -> {
            CardLayout cl = (CardLayout) (cards.getLayout());
            cl.show(cards, (String) e.getItem());
        });

        JPanel botP = new JPanel(new FlowLayout(FlowLayout.LEFT, 10, 5));
        JButton btnStartTraj = new JButton("Bắt đầu");
        JButton btnStopTraj = new JButton("Dừng");
        JButton btnCheckLine = new JButton("Kiểm tra đoạn thẳng");

        btnStopTraj.addActionListener(e -> {
            if (trajectoryTimer != null)
                trajectoryTimer.stop();
            setTitle("Mô Phỏng cánh tay Robot 6-DOF");
        });

        btnStartTraj.addActionListener(e -> {
            trajDebug("START_CLICK", "Start trajectory button clicked");
            if (trajectoryTimer != null)
                trajectoryTimer.stop();
            boolean isLine = trajTypeCombo.getSelectedIndex() == 0;
            try {
                if (isLine) {
                    double sx = Double.parseDouble(txtLStartX.getText());
                    double sy = Double.parseDouble(txtLStartY.getText());
                    double sz = Double.parseDouble(txtLStartZ.getText());
                    double ex = Double.parseDouble(txtLEndX.getText());
                    double ey = Double.parseDouble(txtLEndY.getText());
                    double ez = Double.parseDouble(txtLEndZ.getText());
                    trajDebug("INPUT_LINE", String.format("S=(%.2f,%.2f,%.2f) E=(%.2f,%.2f,%.2f)",
                            sx, sy, sz, ex, ey, ez));
                    runLineTrajectory(sx, sy, sz, ex, ey, ez);
                } else {
                    double cx = Double.parseDouble(txtSStartX.getText());
                    double cy = Double.parseDouble(txtSStartY.getText());
                    double cz = Double.parseDouble(txtSStartZ.getText());
                    double r = Double.parseDouble(txtSR.getText());
                    double h = Double.parseDouble(txtSH.getText());
                    double k = Double.parseDouble(txtSK.getText());
                    trajDebug("INPUT_SPIRAL", String.format("C=(%.2f,%.2f,%.2f) R=%.2f H=%.2f K=%.2f",
                            cx, cy, cz, r, h, k));
                    runSpiralTrajectoryParam(cx, cy, cz, r, h, k);
                }
            } catch (Exception ex) {
                trajDebug("INPUT_ERROR", ex.toString());
                JOptionPane.showMessageDialog(this, "Vui lòng nhập số hợp lệ!", "Lỗi", JOptionPane.ERROR_MESSAGE);
            }
        });

        btnCheckLine.addActionListener(e -> {
            try {
                double sx = Double.parseDouble(txtLStartX.getText());
                double sy = Double.parseDouble(txtLStartY.getText());
                double sz = Double.parseDouble(txtLStartZ.getText());
                double ex = Double.parseDouble(txtLEndX.getText());
                double ey = Double.parseDouble(txtLEndY.getText());
                double ez = Double.parseDouble(txtLEndZ.getText());

                String cfg = configCombo.getSelectedIndex() == 0 ? "+" : "-";
                trajectoryLockedCfg = cfg;
                trajectoryLastQ = null;
                trajectoryLastAlpha = alphaSlider.getValue();

                String report = buildLineFeasibilityReport(sx, sy, sz, ex, ey, ez);
                trajDebug("LINE_CHECK", report.replace('\n', ' '));
                JOptionPane.showMessageDialog(this, report, "Kết quả kiểm tra đoạn thẳng",
                        JOptionPane.INFORMATION_MESSAGE);
            } catch (Exception ex) {
                JOptionPane.showMessageDialog(this, "Vui lòng nhập số hợp lệ!", "Lỗi", JOptionPane.ERROR_MESSAGE);
            }
        });

        botP.add(btnCheckLine);
        botP.add(btnStartTraj);
        botP.add(btnStopTraj);
        trajPanel.add(botP);

        trajPanel.add(Box.createVerticalGlue());

        return trajPanel;
    }

    private void buildTopPanel() {
        add(BorderLayout.NORTH, topPanel);

        JButton btnGripper = new JButton("Đóng / Mở Kẹp");
        btnGripper.addActionListener(e -> {
            isGripped = !isGripped;
            armPanel.repaint();
            setGotoStatus(isGripped ? "Đã ĐÓNG kẹp" : "Đã MỞ kẹp", Color.BLUE);
        });
        topPanel.add(btnGripper);

        topPanel.add(new JLabel("  Tốc độ di chuyển:"));
        speedSlider.setPreferredSize(new Dimension(140, 25));
        speedSlider.setMajorTickSpacing(30);
        speedSlider.setPaintTicks(true);
        speedSlider.addChangeListener(ev -> speedLabel.setText(speedSlider.getValue() + " °/s"));
        topPanel.add(speedSlider);
        topPanel.add(speedLabel);

        topPanel.add(new JSeparator(SwingConstants.VERTICAL));
        topPanel.add(new JLabel(" COM:"));
        JComboBox<String> comPortCombo = new JComboBox<>();
        for (String portName : comm.UartManager.getAvailablePorts()) {
            comPortCombo.addItem(portName);
        }
        comPortCombo.setPreferredSize(new Dimension(80, 25));

        JButton btnConnect = new JButton("Kết nối");
        btnConnect.addActionListener(e -> {
            if (btnConnect.getText().equals("Kết nối")) {
                String selectedPort = (String) comPortCombo.getSelectedItem();
                if (selectedPort != null) {
                    if (uartManager.connect(selectedPort, 115200)) {
                        btnConnect.setText("Ngắt kết nối");
                        setGotoStatus("Đã kết nối " + selectedPort, new Color(0, 140, 0));
                    } else {
                        setGotoStatus("Lỗi kết nối " + selectedPort, Color.RED);
                    }
                }
            } else {
                uartManager.disconnect();
                btnConnect.setText("Kết nối");
                setGotoStatus("Đã ngắt kết nối", Color.BLUE);
            }
        });

        JButton btnRefreshCom = new JButton("↻");
        btnRefreshCom.setToolTipText("Làm mới danh sách COM");
        btnRefreshCom.setMargin(new Insets(2, 5, 2, 5));
        btnRefreshCom.addActionListener(e -> {
            comPortCombo.removeAllItems();
            for (String portName : comm.UartManager.getAvailablePorts()) {
                comPortCombo.addItem(portName);
            }
        });

        topPanel.add(comPortCombo);
        topPanel.add(btnRefreshCom);
        topPanel.add(btnConnect);
    }

    private void buildMenuBar() {
        final JMenuBar menuBar = new JMenuBar();

        JMenu dieukhienMenu = new JMenu("Điều khiển");

        JMenuItem resetItem = new JMenuItem("Reset về gốc");
        resetItem.setMnemonic('R');
        resetItem.setActionCommand("Reset");

        JMenuItem trajItem = new JMenuItem("Quản lý Quỹ Đạo");
        trajItem.setMnemonic('Q');
        trajItem.setActionCommand("Trajectory");

        JMenuItem exitItem = new JMenuItem("Thoát");
        exitItem.setMnemonic('X');
        exitItem.setActionCommand("Exit");

        MenuItemListener menuItemListener = new MenuItemListener();
        resetItem.addActionListener(menuItemListener);
        trajItem.addActionListener(menuItemListener);
        exitItem.addActionListener(menuItemListener);

        dieukhienMenu.add(resetItem);
        dieukhienMenu.add(trajItem);
        dieukhienMenu.addSeparator();
        dieukhienMenu.add(exitItem);

        JMenu hienthiMenu = new JMenu("Hiển thị");

        JCheckBoxMenuItem gridItem = new JCheckBoxMenuItem("Hiện Lưới", showGridCb.isSelected());
        gridItem.addItemListener(e -> {
            showGridCb.setSelected(gridItem.isSelected());
            armPanel.repaint();
        });

        JCheckBoxMenuItem trailItem = new JCheckBoxMenuItem("Hiện Vết Quỹ Đạo", showTrailCb.isSelected());
        trailItem.addItemListener(e -> {
            showTrailCb.setSelected(trailItem.isSelected());
            armPanel.repaint();
        });

        JCheckBoxMenuItem workspaceItem = new JCheckBoxMenuItem("Hiện Vùng Làm Việc", showWorkspace);
        workspaceItem.addItemListener(e -> {
            showWorkspace = workspaceItem.isSelected();
            if (showWorkspace) {
                armPanel.workspacePoints.clear();
                armPanel.workspaceKeys.clear();
                armPanel.workspaceStatus = "";
                runWorkspaceExploration();
            } else {
                if (explorationThread != null)
                    explorationThread.interrupt();
                armPanel.workspaceStatus = "";
                armPanel.repaint();
            }
        });

        JMenuItem topViewItem = new JMenuItem("Hệ Trục (Top View)");
        topViewItem.setActionCommand("TopView");
        topViewItem.addActionListener(menuItemListener);

        JMenuItem perspItem = new JMenuItem("3D Perspective");
        perspItem.setActionCommand("Persp");
        perspItem.addActionListener(menuItemListener);

        hienthiMenu.add(gridItem);
        hienthiMenu.add(trailItem);
        hienthiMenu.add(workspaceItem);
        hienthiMenu.addSeparator();
        hienthiMenu.add(topViewItem);
        hienthiMenu.add(perspItem);

        // ---- Menu: Chế độ ----
        JMenu chedoMenu = new JMenu("Chế độ");

        JCheckBoxMenuItem clickModeItem = new JCheckBoxMenuItem("Click tới tọa độ (Z cố định)", fixedHeightMode);
        clickModeItem.addItemListener(e -> {
            fixedHeightMode = clickModeItem.isSelected();
            fixedHeightCb.setSelected(fixedHeightMode);
            armPanel.repaint();
        });
        fixedHeightCb.addActionListener(ev -> clickModeItem.setSelected(fixedHeightCb.isSelected()));

        JCheckBoxMenuItem manualModeItem = new JCheckBoxMenuItem("Chế độ thủ công (FK)", manualMode);
        manualModeItem.addItemListener(e -> {
            manualMode = manualModeItem.isSelected();
            if (manualMode)
                updateArm(); // Sync immediately
        });

        chedoMenu.add(clickModeItem);
        chedoMenu.add(manualModeItem);

        // Add menus to bar
        menuBar.add(dieukhienMenu);
        menuBar.add(hienthiMenu);
        menuBar.add(chedoMenu);

        setJMenuBar(menuBar);
    }

    /** Handles actions from menu items. */
    class MenuItemListener implements ActionListener {
        @Override
        public void actionPerformed(ActionEvent e) {
            switch (e.getActionCommand()) {
                case "Reset" -> resetAngles();
                case "Trajectory" -> mainTabs.setSelectedIndex(1);
                case "TopView" -> {
                    armPanel.camAz = 0;
                    armPanel.camEl = 89.9;
                    armPanel.repaint();
                }
                case "Persp" -> {
                    armPanel.camAz = -30;
                    armPanel.camEl = 25;
                    armPanel.repaint();
                }
                case "Exit" -> System.exit(0);
            }
        }
    }

    /** Starts the continuous motion interpolation timer (idempotent). */
    private void startMotionTimer() {
        if (motionTimer != null && motionTimer.isRunning())
            return;
        motionTimer = new Timer(MOTION_DT_MS, e -> tickMotion());
        motionTimer.start();
    }

    /**
     * Called every MOTION_DT_MS ms. Moves each joint angle toward its target
     * at the rate configured by speedSlider (degrees/second).
     */
    private void tickMotion() {
        // Avoid deadlock when speed slider is set to 0.
        double effectiveSpeed = Math.max(1.0, speedSlider.getValue());
        double maxStep = effectiveSpeed * (MOTION_DT_MS / 1000.0);
        boolean moving = false;

        // 1. Interpolate Right Arm
        for (int i = 0; i < NUM_JOINTS; i++) {
            double diff = targetAnglesRight[i] - anglesRight[i];

            // Shortest path interpolation (Angle Wrapping)
            while (diff > 180)
                diff -= 360;
            while (diff < -180)
                diff += 360;

            if (Math.abs(diff) < 0.05) {
                if (anglesRight[i] != targetAnglesRight[i]) {
                    anglesRight[i] = targetAnglesRight[i];
                    moving = true;
                }
            } else {
                anglesRight[i] += Math.signum(diff) * Math.min(Math.abs(diff), maxStep);
                moving = true;
            }

            while (anglesRight[i] > 180)
                anglesRight[i] -= 360;
            while (anglesRight[i] < -180)
                anglesRight[i] += 360;
        }

        // 2. Interpolate Left Arm
        for (int i = 0; i < NUM_JOINTS; i++) {
            double diff = targetAnglesLeft[i] - anglesLeft[i];

            // Shortest path interpolation (Angle Wrapping)
            while (diff > 180)
                diff -= 360;
            while (diff < -180)
                diff += 360;

            if (Math.abs(diff) < 0.05) {
                if (anglesLeft[i] != targetAnglesLeft[i]) {
                    anglesLeft[i] = targetAnglesLeft[i];
                    moving = true;
                }
            } else {
                anglesLeft[i] += Math.signum(diff) * Math.min(Math.abs(diff), maxStep);
                moving = true;
            }

            while (anglesLeft[i] > 180)
                anglesLeft[i] -= 360;
            while (anglesLeft[i] < -180)
                anglesLeft[i] += 360;
        }

        // Synchronize Joint 1 (Waist/Hip) for both arms
        anglesRight[0] = angles[0];
        anglesLeft[0] = angles[0];
        targetAnglesRight[0] = targetAngles[0];
        targetAnglesLeft[0] = targetAngles[0];

        // 3. Sync sliders and labels of the ACTIVE arm without triggering listener
        for (int i = 0; i < NUM_JOINTS; i++) {
            sliders[i].removeChangeListener(this);
            sliders[i].setValue((int) Math.round(angles[i]));
            sliders[i].addChangeListener(this);
            angleLbls[i].setText((int) Math.round(angles[i]) + "°");
        }

        updateArm();
        sendJointsToUart();

        // Stop timer if nothing is moving
        if (!moving) {
            motionTimer.stop();
        }
    }

    /**
     * Formats current joint angles and sends them to STM32 via UART.
     * Format: S:q1,q2,q3,q4,q5\n
     */
    private void sendJointsToUart() {
        if (uartManager != null) {
            // Check if angles have changed significantly since last send
            boolean changed = false;
            for (int i = 0; i < NUM_JOINTS; i++) {
                if (Math.abs(angles[i] - lastSentAngles[i]) > 0.01) {
                    changed = true;
                    break;
                }
            }

            if (!changed)
                return;

            StringBuilder sb = new StringBuilder(isRightArmSelected ? "R:" : "L:");
            for (int i = 0; i < NUM_JOINTS; i++) {
                // Send as integers to reduce packet size and STM32 CPU load
                sb.append(String.format("%d", (int) Math.round(angles[i])));
                if (i < NUM_JOINTS - 1) {
                    sb.append(",");
                }
                lastSentAngles[i] = angles[i];
            }
            sb.append("\n");

            String data = sb.toString();
            uartManager.sendData(data);
            if (uartManager.isConnected()) {
                System.out.print("Sent UART: " + data);
            }
        }
    }

    /**
     * Request the arm to move smoothly to the given joint angles (radians).
     * The motion timer will interpolate from the current position.
     */
    void setTargetAngles(double[] q_deg) {
        System.arraycopy(q_deg, 0, targetAngles, 0, NUM_JOINTS); // q_deg is already in degrees from solveIK
        // Synchronize Joint 1 (Waist/Hip) targets immediately
        targetAnglesRight[0] = targetAngles[0];
        targetAnglesLeft[0] = targetAngles[0];
        anglesRight[0] = angles[0];
        anglesLeft[0] = angles[0];
        startMotionTimer();
    }

    public void setGotoStatus(String text, Color color) {
        gotoStatus.setForeground(color);
        gotoStatus.setText(text);
    }

    private void gotoCoordinate() {
        try {
            double px = Double.parseDouble(txX.getText().trim());
            double py = Double.parseDouble(txY.getText().trim());
            double pz = Double.parseDouble(txZ.getText().trim());

            if (pz < 0) {
                setGotoStatus("Z không được âm!", Color.RED);
                return;
            }

            String prefCfg = configCombo.getSelectedIndex() == 0 ? "+" : "-";
            double[] result = solveIKSmart(px, py, pz, prefCfg);
            if (result != null) {
                setTargetAngles(result); // smooth motion
                setGotoStatus("OK", new Color(0, 140, 0));
            } else {
                setGotoStatus("Ngoài tầm/Giới hạn góc!", Color.RED);
            }
        } catch (NumberFormatException ex) {
            gotoStatus.setText("Sai định dạng");
        }
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        if (e.getSource() == showGridCb || e.getSource() == showTrailCb) {
            armPanel.repaint();
        } else if (e.getSource() == btnReset) {
            resetAngles();
        } else if (e.getSource() == btnTopView) {
            armPanel.camAz = 0;
            armPanel.camEl = 89.9;
            armPanel.repaint();
        } else if (e.getSource() == btnPersp) {
            armPanel.camAz = -30;
            armPanel.camEl = 25;
            armPanel.repaint();
        } else if (e.getSource() == btnGoto) {
            gotoCoordinate();
        } else if (e.getSource() == btnSyncCoords) {
            double[] ee = armPanel.getEndEffectorPosition();
            txX.setText(String.format("%.1f", ee[0]));
            txY.setText(String.format("%.1f", ee[1]));
            txZ.setText(String.format("%.1f", ee[2]));
        } else if (e.getSource() == fixedHeightCb) {
            fixedHeightMode = fixedHeightCb.isSelected();
            armPanel.repaint();
        }
    }

    @Override
    public void stateChanged(ChangeEvent e) {
        for (int i = 0; i < NUM_JOINTS; i++) {
            if (e.getSource() == sliders[i]) {
                double oldVal = angles[i];
                angles[i] = sliders[i].getValue();

                // Prevent gripper from going under floor
                double[] ee = armPanel.getEndEffectorPosition();
                if (ee[2] < 0) {
                    angles[i] = oldVal; // Revert
                    sliders[i].setValue((int) oldVal);
                }

                targetAngles[i] = angles[i]; // Prevent motion timer from snapping back
                
                if (i == 0) {
                    anglesRight[0] = angles[0];
                    anglesLeft[0] = angles[0];
                    targetAnglesRight[0] = angles[0];
                    targetAnglesLeft[0] = angles[0];
                }

                angleLbls[i].setText((int) angles[i] + "°");
                updateArm();
                break;
            }
        }
        for (int i = 0; i < 3; i++) {
            JSlider s = (i == 0) ? slX : (i == 1) ? slY : slZ;
            if (e.getSource() == s && !isUpdatingFromFK) {
                // UI -> IK
                // Only run IK if we are NOT updating these sliders from FK
                try {
                    double px = slX.getValue();
                    double py = slY.getValue();
                    double pz = slZ.getValue();
                    String prefCfg = configCombo.getSelectedIndex() == 0 ? "+" : "-";
                    double[] res = solveIKSmart(px, py, pz, prefCfg);
                    if (res != null) {
                        setTargetAngles(res);
                        txX.setText(String.valueOf((int) px));
                        txY.setText(String.valueOf((int) py));
                        txZ.setText(String.valueOf((int) pz));
                        setGotoStatus("OK", new Color(0, 140, 0));
                    }
                } catch (Exception ex) {
                }
                break;
            }
        }
    }

    void updateArm() {
        double[][] pts = armPanel.computeAllJoints3D();
        // Index 5 (NUM_JOINTS) is the wrist. Index 6 (NUM_JOINTS + 1) is the distal
        // Tool Tip.
        double[] ee = pts[NUM_JOINTS + 1];
        endEffectorLabel.setText(String.format("Tọa độ kẹp: (%.1f,  %.1f,  %.1f)", ee[0], ee[1], ee[2]));

        if (manualMode) {
            isUpdatingFromFK = true; // Lock IK
            // FK -> UI
            slX.removeChangeListener(this);
            slY.removeChangeListener(this);
            slZ.removeChangeListener(this);

            slX.setValue((int) Math.round(ee[0]));
            slY.setValue((int) Math.round(ee[1]));
            slZ.setValue((int) Math.round(ee[2]));

            txX.setText(String.format("%.1f", ee[0]));
            txY.setText(String.format("%.1f", ee[1]));
            txZ.setText(String.format("%.1f", ee[2]));

            slX.addChangeListener(this);
            slY.addChangeListener(this);
            slZ.addChangeListener(this);
            isUpdatingFromFK = false; // Unlock IK
        }

        armPanel.repaint();
    }

    void resetAngles() {
        armPanel.trail.clear();
        double[] defaultPose = isRightArmSelected ? new double[] { 0, 0, 0, -30.0, 0, 0 } : new double[] { 0, 0, 0, 30.0, 0, 0 };
        setTargetAngles(defaultPose);
    }

    // Trajectory logic methods follow ...

    void runLineTrajectory(double sx, double sy, double sz, double ex, double ey, double ez) {
        setTitle("Đang di chuyển tới điểm xuất phát...");
        setGotoStatus("Đang chuẩn bị quỹ đạo thẳng...", new Color(0, 90, 180));
        trajDebug("LINE_INIT", "Preparing line trajectory");
        if (speedSlider.getValue() <= 0) {
            speedSlider.setValue(30);
            speedLabel.setText("30 °/s");
            setGotoStatus("Tốc độ đang là 0, tự tăng lên 30 °/s để chạy quỹ đạo", new Color(180, 110, 0));
            trajDebug("LINE_SPEED_FIX", "Speed was 0, forced to 30");
        }

        // Tắt hiển thị quỹ đạo cũ và xóa cặn
        showTrailCb.setSelected(false);
        armPanel.trail.clear();

        String cfg = configCombo.getSelectedIndex() == 0 ? "+" : "-";
        trajectoryLockedCfg = cfg;
        trajectoryLastQ = null;
        trajectoryLastAlpha = alphaSlider.getValue();

        // Nhảy đến điểm xuất phát
        double[] startResult = solveIKForTrajectoryPoint(sx, sy, sz);
        if (startResult != null) {
            trajDebug("LINE_START_IK_OK", String.format("q=[%.1f, %.1f, %.1f, %.1f, %.1f]",
                    startResult[0], startResult[1], startResult[2], startResult[3], startResult[4]));
            setTargetAngles(startResult);
            trajectoryLastQ = startResult.clone();
            updateArm();
        } else {
            trajDebug("LINE_START_IK_FAIL", "No IK for start point");
            setGotoStatus("Điểm bắt đầu ngoài tầm!", Color.RED);
            setTitle("Mô Phỏng Cánh Tay Robot 6-DOF");
            return;
        }

        // Validate the whole line quickly to avoid "hold pose" for almost entire run.
        int failCount = 0;
        String firstFail = null;
        for (int i = 0; i <= 30; i++) {
            double r = i / 30.0;
            double tx = sx + (ex - sx) * r;
            double ty = sy + (ey - sy) * r;
            double tz = sz + (ez - sz) * r;
            if (!hasIKForPoint(tx, ty, tz)) {
                failCount++;
                if (firstFail == null) {
                    firstFail = String.format("(%.2f, %.2f, %.2f)", tx, ty, tz);
                }
            }
        }
        if (failCount > 8) {
            trajDebug("LINE_PATH_BLOCKED", "fails=" + failCount + " firstFail=" + firstFail);
            if (Math.abs(ez - sz) < 1e-6) {
                Double altZ = findAlternativeFlatLineZ(sx, sy, ex, ey, sz, failCount);
                if (altZ != null) {
                    trajDebug("LINE_AUTO_Z", String.format("Auto-adjust Z from %.2f to %.2f", sz, altZ));
                    txtLStartZ.setText(String.format("%.1f", altZ));
                    txtLEndZ.setText(String.format("%.1f", altZ));
                    setGotoStatus(String.format("Tự động đổi Z: %.1f -> %.1f để line khả thi", sz, altZ),
                            new Color(180, 110, 0));
                    runLineTrajectory(sx, sy, altZ, ex, ey, altZ);
                    return;
                }
            }
            setGotoStatus("Đường thẳng đi qua vùng không với tới, thử đổi điểm hoặc giảm Z", Color.RED);
            setTitle("Mô Phỏng Cánh Tay Robot 5-DOF");
            return;
        }

        // Chờ tay máy di chuyển tới điểm xuất phát, sau đó nghỉ 1 giây mới bật quỹ đạo
        // và vẽ
        final int[] waitMs = { 0 };
        Timer prepareTimer = new Timer(50, evt -> {
            waitMs[0] += 50;
            boolean arrived = true;
            for (int i = 0; i < NUM_JOINTS; i++) {
                if (Math.abs(angles[i] - targetAngles[i]) > 0.5)
                    arrived = false;
            }
            if (arrived || waitMs[0] >= 5000) {
                ((Timer) evt.getSource()).stop();
                trajDebug("LINE_PREPARE_DONE", "arrived=" + arrived + " waitMs=" + waitMs[0]);
                if (!arrived) {
                    setGotoStatus("Hết thời gian chờ điểm đầu, bắt đầu quỹ đạo từ vị trí hiện tại", new Color(180, 110, 0));
                }
                setTitle("Chờ 1 giây...");

                Timer delayTimer = new Timer(1000, evt2 -> {
                    ((Timer) evt2.getSource()).stop();

                    setTitle("Đường thẳng đang chạy...");
                    showTrailCb.setSelected(true); // Chỉ bật bắt đầu từ lúc này

                    final double L = Math.max(0.1,
                            Math.sqrt(Math.pow(ex - sx, 2) + Math.pow(ey - sy, 2) + Math.pow(ez - sz, 2)));
                    double[] ratio = { 0.0 };

                    trajectoryTimer = new Timer(MOTION_DT_MS, e -> {
                        double speed = Math.max(1.0, speedSlider.getValue() / 2.0); // units/sec Cartesian speed
                                                                                    // approximation
                        double dt = MOTION_DT_MS / 1000.0;
                        ratio[0] += (speed * dt) / L;

                        if (ratio[0] >= 1.0) {
                            ratio[0] = 1.0;
                            trajectoryTimer.stop();
                            trajDebug("LINE_DONE", "ratio reached 1.0");
                            setTitle("Simulation (5-DOF)");
                        }
                        double r = ratio[0];
                        double tx = sx + (ex - sx) * r;
                        double ty = sy + (ey - sy) * r;
                        double tz = sz + (ez - sz) * r;

                        double[] result = solveIKForTrajectoryPoint(tx, ty, tz);
                        if (result != null) {
                            trajectoryLastQ = result.clone();
                            for (int i = 0; i < NUM_JOINTS; i++) {
                                targetAngles[i] = angles[i] = result[i];
                                sliders[i].removeChangeListener(this);
                                sliders[i].setValue((int) Math.round(angles[i]));
                                sliders[i].addChangeListener(this);
                                angleLbls[i].setText((int) Math.round(angles[i]) + "°");
                            }
                            updateArm();
                        } else {
                            trajDebug("LINE_TRACK_HOLD",
                                    String.format("No IK at p=(%.2f, %.2f, %.2f), hold last pose", tx, ty, tz));
                            if (trajectoryLastQ != null) {
                                setTargetAngles(trajectoryLastQ);
                            }
                            setGotoStatus("~ Điểm khó đạt, đang giữ pose gần nhất", new Color(180, 110, 0));
                        }
                    });
                    trajDebug("LINE_TIMER_START", "Trajectory timer started");
                    trajectoryTimer.start();
                });
                delayTimer.start();
            }
        });
        prepareTimer.start();
    }

    void runSpiralTrajectoryParam(double cx, double cy, double cz, double R, double H, double K) {
        setTitle("Đang di chuyển tới điểm xuất phát...");
        setGotoStatus("Đang chuẩn bị quỹ đạo xoắn ốc...", new Color(0, 90, 180));
        trajDebug("SPIRAL_INIT", "Preparing spiral trajectory");
        if (speedSlider.getValue() <= 0) {
            speedSlider.setValue(30);
            speedLabel.setText("30 °/s");
            setGotoStatus("Tốc độ đang là 0, tự tăng lên 30 °/s để chạy quỹ đạo", new Color(180, 110, 0));
            trajDebug("SPIRAL_SPEED_FIX", "Speed was 0, forced to 30");
        }

        // Tắt hiển thị quỹ đạo cũ và xóa cặn
        showTrailCb.setSelected(false);
        armPanel.trail.clear();

        String cfg = configCombo.getSelectedIndex() == 0 ? "+" : "-";
        trajectoryLockedCfg = cfg;
        trajectoryLastQ = null;
        trajectoryLastAlpha = alphaSlider.getValue();

        // Tính toán điểm xuất phát của Xoắn ốc tại thời điểm t = 0
        double startX = cx + R * Math.cos(0);
        double startY = cy + R * Math.sin(0);
        double startZ = cz;

        double[] startResult = solveIKForTrajectoryPoint(startX, startY, startZ);
        if (startResult != null) {
            trajDebug("SPIRAL_START_IK_OK", String.format("q=[%.1f, %.1f, %.1f, %.1f, %.1f]",
                    startResult[0], startResult[1], startResult[2], startResult[3], startResult[4]));
            setTargetAngles(startResult);
            trajectoryLastQ = startResult.clone();
            updateArm();
        } else {
            trajDebug("SPIRAL_START_IK_FAIL", "No IK for spiral start point");
            setGotoStatus("Điểm bắt đầu xoắn ốc ngoài tầm!", Color.RED);
            setTitle("Mô Phỏng Cánh Tay Robot 6-DOF");
            return;
        }

        // Chờ tay máy di chuyển tới vị trí bắt đầu, ngủ 1s, rồi sau đó mới bắt đầu vẽ
        // quỹ đạo
        final int[] waitMs = { 0 };
        Timer prepareTimer = new Timer(50, evt -> {
            waitMs[0] += 50;
            boolean arrived = true;
            for (int i = 0; i < NUM_JOINTS; i++) {
                if (Math.abs(angles[i] - targetAngles[i]) > 0.5)
                    arrived = false;
            }
            if (arrived || waitMs[0] >= 5000) {
                ((Timer) evt.getSource()).stop();
                trajDebug("SPIRAL_PREPARE_DONE", "arrived=" + arrived + " waitMs=" + waitMs[0]);
                if (!arrived) {
                    setGotoStatus("Hết thời gian chờ điểm đầu, bắt đầu xoắn ốc từ vị trí hiện tại", new Color(180, 110, 0));
                }
                setTitle("Chờ 1 giây...");

                Timer delayTimer = new Timer(1000, evt2 -> {
                    ((Timer) evt2.getSource()).stop();

                    setTitle("Xoắn ốc đang chạy...");
                    showTrailCb.setSelected(true); // Lúc này mới bật hiển thị vết quỹ đạo

                    final double L = Math.max(0.1, Math.sqrt(Math.pow(2 * Math.PI * R * K, 2) + Math.pow(H, 2)));
                    double[] ratio = { 0.0 };

                    trajectoryTimer = new Timer(MOTION_DT_MS, e -> {
                        double speed = Math.max(1.0, speedSlider.getValue() / 2.0); // units/sec
                        double dt = MOTION_DT_MS / 1000.0;
                        ratio[0] += (speed * dt) / L;

                        if (ratio[0] >= 1.0) {
                            ratio[0] = 1.0;
                            trajectoryTimer.stop();
                            trajDebug("SPIRAL_DONE", "ratio reached 1.0");
                            setTitle("Simulation (6-DOF)");
                        }
                        double r = ratio[0];
                        double tx = cx + R * Math.cos(K * 2 * Math.PI * r);
                        double ty = cy + R * Math.sin(K * 2 * Math.PI * r);
                        double tz = cz + H * r;

                        double[] result = solveIKForTrajectoryPoint(tx, ty, tz);
                        if (result != null) {
                            trajectoryLastQ = result.clone();
                            for (int i = 0; i < NUM_JOINTS; i++) {
                                targetAngles[i] = angles[i] = result[i];
                                sliders[i].removeChangeListener(this);
                                sliders[i].setValue((int) Math.round(angles[i]));
                                sliders[i].addChangeListener(this);
                                angleLbls[i].setText((int) Math.round(angles[i]) + "°");
                            }
                            updateArm();
                        } else {
                            trajDebug("SPIRAL_TRACK_HOLD",
                                    String.format("No IK at p=(%.2f, %.2f, %.2f), hold last pose", tx, ty, tz));
                            if (trajectoryLastQ != null) {
                                setTargetAngles(trajectoryLastQ);
                            }
                            setGotoStatus("~ Điểm khó đạt, đang giữ pose gần nhất", new Color(180, 110, 0));
                        }
                    });
                    trajDebug("SPIRAL_TIMER_START", "Trajectory timer started");
                    trajectoryTimer.start();
                });
                delayTimer.start();
            }
        });
        prepareTimer.start();
    }

    private double wrappedDegDiff(double a, double b) {
        double d = a - b;
        while (d > 180)
            d -= 360;
        while (d < -180)
            d += 360;
        return d;
    }

    private double continuityCost(double[] q, double[] qPrev) {
        double s = 0.0;
        for (int i = 0; i < NUM_JOINTS; i++) {
            double d = wrappedDegDiff(q[i], qPrev[i]);
            s += d * d;
        }
        return s;
    }

    private double[] solveIKForTrajectoryPoint(double px, double py, double pz) {
        boolean isFirstWaypoint = (trajectoryLastQ == null);
        // First point anchors to current pose; next points anchor to previous trajectory IK.
        double[] qRef = isFirstWaypoint ? angles : trajectoryLastQ;
        String altCfg = trajectoryLockedCfg.equals("+") ? "-" : "+";
        String[] cfgCandidates = isFirstWaypoint ? new String[] { trajectoryLockedCfg, altCfg }
                : new String[] { trajectoryLockedCfg };

        double bestStrictCost = Double.MAX_VALUE;
        double[] bestStrictQ = null;
        double bestStrictAlpha = trajectoryLastAlpha;
        String bestStrictCfg = trajectoryLockedCfg;

        double bestRelaxedCost = Double.MAX_VALUE;
        double[] bestRelaxedQ = null;
        double bestRelaxedAlpha = trajectoryLastAlpha;
        String bestRelaxedCfg = trajectoryLockedCfg;

        // 1) Search near previous alpha to avoid branch jumping
        for (String cfgTry : cfgCandidates) {
            for (double a = trajectoryLastAlpha - 12; a <= trajectoryLastAlpha + 12; a += 1.0) {
                List<double[]> candidates = tryAlpha(px, py, pz, a, cfgTry);
                for (double[] q : candidates) {
                    double posErr = computePositionError(q, px, py, pz);
                    double c = posErr * 220.0 + continuityCost(q, qRef) * 0.04;
                    if (posErr <= MAX_IK_POSITION_ERROR && c < bestStrictCost) {
                        bestStrictCost = c;
                        bestStrictQ = q;
                        bestStrictAlpha = a;
                        bestStrictCfg = cfgTry;
                    }
                    if (posErr <= TRAJ_RELAXED_ERROR && c < bestRelaxedCost) {
                        bestRelaxedCost = c;
                        bestRelaxedQ = q;
                        bestRelaxedAlpha = a;
                        bestRelaxedCfg = cfgTry;
                    }
                }
            }
        }

        // 2) Fallback to global scan if local search fails
        if (bestStrictQ == null && bestRelaxedQ == null) {
            String[] cfgFallback = isFirstWaypoint ? cfgCandidates : new String[] { trajectoryLockedCfg, altCfg };
            for (String cfgTry : cfgFallback) {
                for (double a = -90; a <= 30; a += 1.5) {
                    List<double[]> candidates = tryAlpha(px, py, pz, a, cfgTry);
                    for (double[] q : candidates) {
                        double posErr = computePositionError(q, px, py, pz);
                        double c = posErr * 220.0 + continuityCost(q, qRef) * 0.04;
                        if (posErr <= MAX_IK_POSITION_ERROR && c < bestStrictCost) {
                            bestStrictCost = c;
                            bestStrictQ = q;
                            bestStrictAlpha = a;
                            bestStrictCfg = cfgTry;
                        }
                        if (posErr <= TRAJ_RELAXED_ERROR && c < bestRelaxedCost) {
                            bestRelaxedCost = c;
                            bestRelaxedQ = q;
                            bestRelaxedAlpha = a;
                            bestRelaxedCfg = cfgTry;
                        }
                    }
                }
            }
        }

        if (bestStrictQ != null) {
            trajectoryLastAlpha = bestStrictAlpha;
            trajectoryLockedCfg = bestStrictCfg;
            return bestStrictQ;
        }
        if (bestRelaxedQ != null) {
            trajectoryLastAlpha = bestRelaxedAlpha;
            trajectoryLockedCfg = bestRelaxedCfg;
            return bestRelaxedQ;
        }
        return null;
    }

    private boolean hasIKForPoint(double px, double py, double pz) {
        boolean oldLog = ikSelectionLogEnabled;
        ikSelectionLogEnabled = false;
        try {
            double[] q = solveIKSmart(px, py, pz, trajectoryLockedCfg);
            if (q != null) {
                return true;
            }
            String altCfg = trajectoryLockedCfg.equals("+") ? "-" : "+";
            q = solveIKSmart(px, py, pz, altCfg);
            return q != null;
        } finally {
            ikSelectionLogEnabled = oldLog;
        }
    }

    private int countLineIKFails(double sx, double sy, double sz, double ex, double ey, double ez) {
        int fails = 0;
        for (int i = 0; i <= 30; i++) {
            double r = i / 30.0;
            double tx = sx + (ex - sx) * r;
            double ty = sy + (ey - sy) * r;
            double tz = sz + (ez - sz) * r;
            if (!hasIKForPoint(tx, ty, tz)) {
                fails++;
            }
        }
        return fails;
    }

    private String buildLineFeasibilityReport(double sx, double sy, double sz, double ex, double ey, double ez) {
        int fails = 0;
        String firstFail = null;
        for (int i = 0; i <= 30; i++) {
            double r = i / 30.0;
            double tx = sx + (ex - sx) * r;
            double ty = sy + (ey - sy) * r;
            double tz = sz + (ez - sz) * r;
            if (!hasIKForPoint(tx, ty, tz)) {
                fails++;
                if (firstFail == null) {
                    firstFail = String.format("(%.2f, %.2f, %.2f)", tx, ty, tz);
                }
            }
        }

        StringBuilder sb = new StringBuilder();
        sb.append(String.format("Đoạn kiểm tra: S(%.1f, %.1f, %.1f) -> E(%.1f, %.1f, %.1f)\n",
                sx, sy, sz, ex, ey, ez));
        sb.append(String.format("Số điểm không thỏa IK: %d / 31\n", fails));
        if (fails == 0) {
            sb.append("Kết luận: Đoạn thẳng khả thi hoàn toàn.");
            return sb.toString();
        }

        sb.append("Điểm lỗi đầu tiên: ").append(firstFail).append('\n');
        if (Math.abs(ez - sz) < 1e-6) {
            Double altZ = findAlternativeFlatLineZ(sx, sy, ex, ey, sz, fails);
            if (altZ != null) {
                int altFails = countLineIKFails(sx, sy, altZ, ex, ey, altZ);
                sb.append(String.format("Gợi ý: đổi Z về %.1f (lỗi còn %d / 31).", altZ, altFails));
            } else {
                sb.append("Không tìm được Z phẳng tốt hơn trong tập thử.");
            }
        } else {
            sb.append("Đoạn không phẳng theo Z; thử giảm độ dốc hoặc đổi điểm cuối.");
        }
        return sb.toString();
    }

    private Double findAlternativeFlatLineZ(double sx, double sy, double ex, double ey, double currentZ, int baseFails) {
        double[] candidates = { 18, 16, 14, 12, 10, 8, 6, 4, 22, 24, 26 };
        double bestZ = Double.NaN;
        int bestFails = Integer.MAX_VALUE;
        for (double z : candidates) {
            if (Math.abs(z - currentZ) < 0.1) {
                continue;
            }
            int fails = countLineIKFails(sx, sy, z, ex, ey, z);
            if (fails < bestFails) {
                bestFails = fails;
                bestZ = z;
            }
        }
        if (!Double.isNaN(bestZ) && bestFails < baseFails) {
            trajDebug("LINE_AUTO_Z_CANDIDATE", String.format("bestZ=%.2f fails=%d (from %d)", bestZ, bestFails, baseFails));
            return bestZ;
        }
        return null;
    }

    void runWorkspaceExploration() {
        if (explorationThread != null && explorationThread.isAlive())
            return;

        explorationThread = new Thread(() -> {
            final double step = 8; // Degrees
            // Sample wrist orientations to get total reachable volume
            final double[] q4_samples = { -45, 0, 45 };
            final double[] q5_samples = { -60, 0, 60 };

            for (double q4 : q4_samples) {
                for (double q5 : q5_samples) {
                    for (double q3 = JOINT_MIN[2]; q3 <= JOINT_MAX[2]; q3 += step) {
                        for (double q2 = JOINT_MIN[1]; q2 <= JOINT_MAX[1]; q2 += step) {
                            // Compute FK for q1=0
                            double[] p0 = armPanel.computeFK(0, q2, q3, q4, q5, 0);

                            // Rotate around Z axis (symmetry)
                            for (double q1 = JOINT_MIN[0]; q1 <= JOINT_MAX[0]; q1 += 15) {
                                double rad = Math.toRadians(q1);
                                double x = p0[0] * Math.cos(rad) - p0[1] * Math.sin(rad);
                                double y = p0[0] * Math.sin(rad) + p0[1] * Math.cos(rad);
                                double z = p0[2];

                                if (z >= -5) { // Floor limit
                                    armPanel.addWorkspacePoint(new double[] { x, y, z });
                                }
                            }
                        }
                        if (Thread.interrupted())
                            return;

                        final int count = armPanel.workspacePoints.size();
                        SwingUtilities.invokeLater(() -> {
                            armPanel.workspaceStatus = String.format("ĐANG QUÉT VÙNG LÀM VIỆC... (%d điểm)", count);
                            armPanel.repaint();
                        });
                        try {
                            Thread.sleep(1);
                        } catch (InterruptedException e) {
                            return;
                        }
                    }
                }
            }

            SwingUtilities.invokeLater(() -> {
                armPanel.workspaceStatus = String.format("ĐÃ QUÉT XONG! (Tổng: %d điểm)",
                        armPanel.workspacePoints.size());
                armPanel.repaint();
            });
        });

        explorationThread.setPriority(Thread.MIN_PRIORITY);
        explorationThread.start();
    }

    public double[] solveIKSmart(double px, double py, double pz, String preferredConfig) {
        if (fixedAlphaCb.isSelected()) {
            double currentAlpha = alphaSlider.getValue();
            List<double[]> candidates = tryAlpha(px, py, pz, currentAlpha, preferredConfig);
            double[] best = selectBestByPositionError(candidates, px, py, pz);
            if (best != null && ikSelectionLogEnabled) {
                double err = computePositionError(best, px, py, pz);
                System.out.println(String.format("IK Selected | target=[%.2f, %.2f, %.2f] err=%.4f",
                        px, py, pz, err));
            }
            return best;
        }

        double minCost = Double.MAX_VALUE;
        double[] bestQ = null;
        double bestAlpha = alphaSlider.getValue();
        double[] q_pref = new double[NUM_JOINTS];
        // To bend both elbows forward naturally, both arms must have Joint 3 positive and Joint 4 negative
        q_pref[2] = 60.0;   // Joint 3 positive (strongly bent elbow forward)
        q_pref[3] = -35.0;  // Joint 4 negative (wrist pitch forward)

        // Optimization Loop: Scan likely alpha range [-90, 30] as per Matlab phi range
        // [-pi/2, pi/6]
        for (double a = -90; a <= 30; a += 1.5) {
            // Try both configurations, but prefer the one selected in the UI
            String[] configs = { "+", "-" };
            String userPref = configCombo.getSelectedIndex() == 0 ? "+" : "-";

            for (String cfg : configs) {
                List<double[]> candidates = tryAlpha(px, py, pz, a, cfg);
                for (double[] q : candidates) {
                    double posErr = computePositionError(q, px, py, pz);
                    if (posErr > MAX_IK_POSITION_ERROR) {
                        continue;
                    }
                    double cost = calculateIKCost(a, q, angles, q_pref, cfg, userPref);
                    // Keep Cartesian tracking as top priority; posture/smoothness tune after.
                    cost += posErr * 50.0;
                    if (cost < minCost) {
                        minCost = cost;
                        bestQ = q;
                        bestAlpha = a;
                    }
                }
            }
        }

        if (bestQ != null) {
            // Auto-update UI slider to reflect found optimal alpha
            final int optimalA = (int) Math.round(bestAlpha);
            SwingUtilities.invokeLater(() -> {
                if (!alphaSlider.getValueIsAdjusting()) {
                    alphaSlider.setValue(optimalA);
                }
            });
            if (ikSelectionLogEnabled) {
                double err = computePositionError(bestQ, px, py, pz);
                System.out.println(String.format("IK Selected | target=[%.2f, %.2f, %.2f] alpha=%.1f err=%.4f",
                        px, py, pz, bestAlpha, err));
            }
            return bestQ;
        }
        return null;
    }

    private double computePositionError(double[] qDeg, double tx, double ty, double tz) {
        double[] fk = armPanel.computeFK(qDeg[0], qDeg[1], qDeg[2], qDeg[3], qDeg[4], qDeg[5]);
        double dx = fk[0] - tx;
        double dy = fk[1] - ty;
        double dz = fk[2] - tz;
        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    private double[] selectBestByPositionError(List<double[]> candidates, double tx, double ty, double tz) {
        double[] best = null;
        double minErr = Double.MAX_VALUE;
        for (double[] q : candidates) {
            double err = computePositionError(q, tx, ty, tz);
            if (err < minErr) {
                minErr = err;
                best = q;
            }
        }
        if (best != null && minErr <= MAX_IK_POSITION_ERROR) {
            return best;
        }
        return null;
    }

    /**
     * Cost function for IK optimization (Translation of Matlab code provided by
     * user)
     */
    private double calculateIKCost(double alphaDeg, double[] q, double[] q_prev, double[] q_pref, String cfg,
            String prefCfg) {
        // 1. Smoothness (w=1.0): Sum of squared joint differences (in Radians)
        // Exclude Joint 2 (i==1) from smoothness penalty to allow shoulder pitch to move freely
        double jSmooth = 0;
        for (int i = 0; i < NUM_JOINTS; i++) {
            if (i == 1) continue; // Let Joint 2 change without smoothness cost
            double diffDeg = q[i] - q_prev[i];
            while (diffDeg > 180)
                diffDeg -= 360;
            while (diffDeg < -180)
                diffDeg += 360;
            jSmooth += Math.pow(Math.toRadians(diffDeg), 2);
        }

        // 2. Limits (w=0.5): Penalty for being near limits (10% margin, in Radians^2)
        double[] minLim = isRightArmSelected ? JOINT_MIN_RIGHT : JOINT_MIN_LEFT;
        double[] maxLim = isRightArmSelected ? JOINT_MAX_RIGHT : JOINT_MAX_LEFT;
        double jLimit = 0;
        for (int i = 0; i < NUM_JOINTS; i++) {
            double range = 360;
            if (maxLim[i] - minLim[i] < 5000) {
                range = maxLim[i] - minLim[i];
            }
            double margin = range * 0.10;
            double tooLow = Math.max(0, (minLim[i] + margin) - q[i]);
            double tooHigh = Math.max(0, q[i] - (maxLim[i] - margin));
            jLimit += Math.pow(Math.toRadians(tooLow), 2) + Math.pow(Math.toRadians(tooHigh), 2);
        }

        // 3. Posture (w=1.5): Deviation from preferred home (normalized by pi/2).
        // Increased weight for beauty.
        double jPosture = 0;
        // Only penalize posture for Joint 3 (index 2) and Joint 4 (index 3) to allow shoulder/waist to move freely
        for (int i = 2; i <= 3; i++) {
            jPosture += Math.pow(Math.toRadians(q[i] - q_pref[i]) / (Math.PI / 2.0), 2);
        }

        // 4. Alpha (w=1.0): Deviation from preferred alpha (-pi/6). Increased weight to
        // keep tool orientation stable.
        double alphaRad = Math.toRadians(alphaDeg);
        double alphaPrefRad = -Math.PI / 6.0;
        double jPhi = Math.pow(alphaRad - alphaPrefRad, 2);

        // 5. Configuration Penalty (w=0.3): Prefer the UI-selected configuration
        // (usually Elbow Up)
        double jConfig = cfg.equals(prefCfg) ? 0 : 0.4;

        return (0.15 * jSmooth) + (0.5 * jLimit) + (3.0 * jPosture) + (0.5 * jPhi) + jConfig;
    }

    private List<double[]> tryAlpha(double px, double py, double pz, double alphaDeg, String prefCfg) {
        List<double[]> validSolutions = new ArrayList<>();
        double alpha_rad = Math.toRadians(alphaDeg);
        double q1_min = isRightArmSelected ? JOINT_MIN_RIGHT[0] : JOINT_MIN_LEFT[0];
        double q1_max = isRightArmSelected ? JOINT_MAX_RIGHT[0] : JOINT_MAX_LEFT[0];
        double q1 = Math.max(Math.toRadians(q1_min), Math.min(Math.toRadians(q1_max), Math.atan2(py, px)));
        double c1 = Math.cos(q1), s1 = Math.sin(q1);

        double ca = Math.cos(Math.PI + alpha_rad), sa = Math.sin(Math.PI + alpha_rad);
        double[][] R_y = { { ca, 0, sa }, { 0, 1, 0 }, { -sa, 0, ca } };
        double[][] R_z = { { c1, -s1, 0 }, { s1, c1, 0 }, { 0, 0, 1 } };
        double[][] R_target = multiplyMatrices(R_z, R_y);

        // Convert current angles to radians for initial guess
        double[] qInit = new double[NUM_JOINTS];
        for (int i = 0; i < NUM_JOINTS; i++) {
            qInit[i] = Math.toRadians(angles[i]);
        }
        
        // 1. Try local neighborhood search
        double[] q = solveIK(px, py, pz, R_target, qInit, isRightArmSelected);
        if (q != null && isWithinLimits(q)) {
            validSolutions.add(q);
        }

        // 2. Try standard home postures with multiple Joint 2 initial guesses to avoid local minima
        double[] q2_guesses = { 1.2, 0.8, 0.4, 0.0, -0.4, -0.8, -1.2 };
        for (double q2_val : q2_guesses) {
            double[] qHome = new double[NUM_JOINTS];
            qHome[0] = qInit[0];
            qHome[1] = q2_val;
            // Joint 3: Right Arm prefers positive (1.0), Left Arm prefers negative (-1.0)
            qHome[2] = isRightArmSelected ? 1.0 : -1.0;
            // Joint 4: Right Arm prefers negative (-35 deg), Left Arm prefers positive (35 deg)
            qHome[3] = isRightArmSelected ? Math.toRadians(-35.0) : Math.toRadians(35.0);
            
            double[] q2 = solveIK(px, py, pz, R_target, qHome, isRightArmSelected);
            if (q2 != null && isWithinLimits(q2)) {
                // Avoid duplicates
                boolean duplicate = false;
                for (double[] existing : validSolutions) {
                    double diff = 0;
                    for (int j = 0; j < NUM_JOINTS; j++) {
                        diff += Math.abs(existing[j] - q2[j]);
                    }
                    if (diff < 0.1) {
                        duplicate = true;
                        break;
                    }
                }
                if (!duplicate) {
                    validSolutions.add(q2);
                }
            }
        }
        return validSolutions;
    }

    public double[] solveIKSmart(double px, double py, double pz, double[][] R_target) {
        double[] qInit = new double[NUM_JOINTS];
        for (int i = 0; i < NUM_JOINTS; i++) {
            qInit[i] = Math.toRadians(angles[i]);
        }
        double[] q = solveIK(px, py, pz, R_target, qInit, isRightArmSelected);
        if (q != null && isWithinLimits(q)) {
            return q;
        }
        
        double[] q2_guesses = { 1.2, 0.8, 0.4, 0.0, -0.4, -0.8, -1.2 };
        for (double q2_val : q2_guesses) {
            double[] qHome = new double[NUM_JOINTS];
            double q1_min = isRightArmSelected ? JOINT_MIN_RIGHT[0] : JOINT_MIN_LEFT[0];
            double q1_max = isRightArmSelected ? JOINT_MAX_RIGHT[0] : JOINT_MAX_LEFT[0];
            qHome[0] = Math.max(Math.toRadians(q1_min), Math.min(Math.toRadians(q1_max), Math.atan2(py, px)));
            qHome[1] = q2_val;
            qHome[2] = isRightArmSelected ? 1.0 : -1.0;
            qHome[3] = isRightArmSelected ? Math.toRadians(-35.0) : Math.toRadians(35.0);
            
            q = solveIK(px, py, pz, R_target, qHome, isRightArmSelected);
            if (q != null && isWithinLimits(q)) {
                return q;
            }
        }
        return null;
    }

    public double[] solveIKSmart(double px, double py, double pz, double alpha_deg) {
        double alpha_rad = Math.toRadians(alpha_deg);
        double q1_min = isRightArmSelected ? JOINT_MIN_RIGHT[0] : JOINT_MIN_LEFT[0];
        double q1_max = isRightArmSelected ? JOINT_MAX_RIGHT[0] : JOINT_MAX_LEFT[0];
        double q1 = Math.max(Math.toRadians(q1_min), Math.min(Math.toRadians(q1_max), Math.atan2(py, px)));
        double c1 = Math.cos(q1), s1 = Math.sin(q1);
        double ca = Math.cos(-alpha_rad), sa = Math.sin(-alpha_rad);
        double[][] R_y = { { ca, 0, sa }, { 0, 1, 0 }, { -sa, 0, ca } };
        double[][] R_z = { { c1, -s1, 0 }, { s1, c1, 0 }, { 0, 0, 1 } };
        double[][] R_target = multiplyMatrices(R_z, R_y);
        return solveIKSmart(px, py, pz, R_target);
    }

    private double[] findBestSolution(List<double[]> solutions) {
        if (solutions.isEmpty())
            return null;
        double minScore = Double.MAX_VALUE;
        double[] best = null;
        double[] minLim = isRightArmSelected ? JOINT_MIN_RIGHT : JOINT_MIN_LEFT;
        double[] maxLim = isRightArmSelected ? JOINT_MAX_RIGHT : JOINT_MAX_LEFT;
        for (double[] q : solutions) {
            double score = 0;
            for (int i = 0; i < NUM_JOINTS; i++) {
                double qDeg = q[i]; // q is already in Degrees
                double diffCurrent = Math.abs(qDeg - angles[i]);
                if (diffCurrent > 180)
                    diffCurrent = 360 - diffCurrent;
                score += diffCurrent * ((i == 1) ? 0.3 : 2.0);
                double center = (minLim[i] + maxLim[i]) / 2.0;
                double wCenter = (i == 1) ? 0.05 : 0.5; // Let Joint 2 swing freely from center to expand reach
                score += Math.abs(qDeg - center) * wCenter;
                if (i == 2)
                    score += Math.abs(qDeg) * 1.5;
            }
            if (score < minScore) {
                minScore = score;
                best = q;
            }
        }
        return best;
    }

    private boolean isWithinLimits(double[] q) {
        double[] minLim = isRightArmSelected ? JOINT_MIN_RIGHT : JOINT_MIN_LEFT;
        double[] maxLim = isRightArmSelected ? JOINT_MAX_RIGHT : JOINT_MAX_LEFT;
        for (int i = 0; i < q.length; i++) {
            // q is in Degrees, limits are in Degrees
            if (q[i] < (minLim[i] - 0.1) || q[i] > (maxLim[i] + 0.1))
                return false;
        }
        return true;
    }

}
