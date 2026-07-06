package gui;

import javax.swing.*;
import java.util.ArrayList;
import java.util.List;
import java.util.Arrays;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import static kinematics.Kinematics.*;
import comm.ControllerReceiver;
import utils.WorkspaceLogger;
import utils.WorkspaceMap;

public final class MainFrame extends JFrame implements ActionListener, ChangeListener {
    /** Set to false for demos to suppress debug output. Set to true during development. */
    public static final boolean DEBUG = false;
    public static final boolean FAST_RENDER =
            !"full".equalsIgnoreCase(System.getenv().getOrDefault("AGV_RENDER_QUALITY", "fast"));

    private static final double MAX_IK_POSITION_ERROR = 0.20; // General IK threshold
    private static final double TRAJ_RELAXED_ERROR = 0.25; // Trajectory fallback threshold
    private static final double TRAJ_STRICT_ERROR = 0.10; // Trajectory strict tracking threshold
    private static final double WORKSPACE_FALLBACK_MAX_DISTANCE = 2.0;
    private static final double WORKSPACE_FALLBACK_MAX_JOINT_JUMP = 25.0;
    private static final double WORKSPACE_SEED_MAX_DISTANCE = 1.0;
    // θ-space: θ₃=q₃=20, θ₄=q₄-q₃=-15-20=-35 (Right)
    double[] anglesRight = { 0, 0, 20, -35, 0, 0 };
    double[] targetAnglesRight = { 0, 0, 20, -35, 0, 0 };
    double[] lastSentAnglesRight = { -999, -999, -999, -999, -999, -999 };

    // θ-space: θ₃=q₃=-20, θ₄=q₄-q₃=15-(-20)=35 (Left)
    double[] anglesLeft = { 0, 0, -20, 35, 0, 0 };
    double[] targetAnglesLeft = { 0, 0, -20, 35, 0, 0 };
    double[] lastSentAnglesLeft = { -999, -999, -999, -999, -999, -999 };

    public double[] angles = anglesRight;
    public double[] targetAngles = targetAnglesRight;
    public double[] lastSentAngles = lastSentAnglesRight;

    public boolean isRightArmSelected = true;

    public double[] getAnglesRight() { return anglesRight; }
    public double[] getAnglesLeft() { return anglesLeft; }

    JPanel controlPanel = new JPanel();
    JPanel topPanel = new JPanel();

    JSlider[] slidersRight = new JSlider[NUM_JOINTS];
    JLabel[] angleLblsRight = new JLabel[NUM_JOINTS];
    JSlider[] slidersLeft = new JSlider[NUM_JOINTS];
    JLabel[] angleLblsLeft = new JLabel[NUM_JOINTS];

    ArmPanel armPanel;
    JLabel endEffectorLabelRight = new JLabel("Tọa độ kẹp (R): 0, 0, 0");
    JLabel endEffectorLabelLeft = new JLabel("Tọa độ kẹp (L): 0, 0, 0");

    JCheckBox showGridCb = new JCheckBox("Hiện Lưới", true);
    JCheckBox showTrailCb = new JCheckBox("Hiện Vết Quỹ Đạo", false);
    JCheckBox useJniIKCb = new JCheckBox("Dùng bộ giải C++ (JNI)", false);
    JCheckBox useIkFastCb = new JCheckBox("Dùng bộ giải C++ (IKFast)", false);

    JComboBox<String> configComboRight = new JComboBox<>(new String[] { "Up (+)", "Down (-)" });
    JComboBox<String> configComboLeft = new JComboBox<>(new String[] { "Up (+)", "Down (-)" });
    JComboBox<String> gripperModeComboRight = new JComboBox<>(new String[] { "Bàn tay song song mặt đất", "Tự do" });
    JComboBox<String> gripperModeComboLeft = new JComboBox<>(new String[] { "Bàn tay song song mặt đất", "Tự do" });
    double activeAlphaRight = 0.0;
    double activeAlphaLeft = 0.0;

    JButton btnReset = new JButton("Reset");
    JButton btnDemo = new JButton("Quỹ đạo Xoắn ốc");
    JButton btnTopView = new JButton("Hệ Trục (Top)");
    JButton btnPersp = new JButton("3D Perspective");

    JTextField txXRight = new JTextField("0", 5);
    JTextField txYRight = new JTextField("0", 5);
    JTextField txZRight = new JTextField("0", 5);
    JSlider slXRight = new JSlider(-50, 50, 0);
    JSlider slYRight = new JSlider(-50, 50, 0);
    JSlider slZRight = new JSlider(-20, 80, 0);
    JButton btnGotoRight = new JButton("Đến");
    JButton btnSyncCoordsRight = new JButton("Lấy");
    JLabel gotoStatusRight = new JLabel(" ");

    JTextField txXLeft = new JTextField("0", 5);
    JTextField txYLeft = new JTextField("0", 5);
    JTextField txZLeft = new JTextField("0", 5);
    JSlider slXLeft = new JSlider(-50, 50, 0);
    JSlider slYLeft = new JSlider(-50, 50, 0);
    JSlider slZLeft = new JSlider(-20, 80, 0);
    JButton btnGotoLeft = new JButton("Đến");
    JButton btnSyncCoordsLeft = new JButton("Lấy");
    JLabel gotoStatusLeft = new JLabel(" ");

    JComboBox<String> trajArmCombo = new JComboBox<>(new String[] { "Cánh Tay Phải (Right)", "Cánh Tay Trái (Left)" });
    boolean isGrippedRight = false;
    boolean isGrippedLeft = false;

    boolean manualMode = false;
    boolean isUpdatingFromFK = false;

    JComboBox<String> trajTypeCombo = new JComboBox<>(new String[] { "Vẽ bằng chuột" });

    // Mouse Draw UI components
    public JCheckBox cbEnableDrawing = new JCheckBox("Bật chế độ vẽ (Kéo chuột trái)", true);
    public JTextField txtMouseZ = new JTextField("20", 4);
    public JButton btnClearMouseDraw = new JButton("Xóa hình vẽ");

    public boolean isDrawingActive() {
        return trajTypeCombo.getSelectedIndex() == 0 && cbEnableDrawing != null && cbEnableDrawing.isSelected();
    }
    JCheckBox fixedHeightCb = new JCheckBox("Click cố định Z (chuột phải)", false);
    JSpinner fixedHeightSpinner = new JSpinner(new SpinnerNumberModel(100.0, -200.0, 500.0, 1.0));
    boolean fixedHeightMode = false;
    JCheckBoxMenuItem clickModeItem;
    JSlider speedSlider = new JSlider(0, 120, 60);
    JLabel speedLabel = new JLabel("60 °/s");
    private static final int MOTION_DT_MS = 30;
    Timer motionTimer;

    boolean showWorkspace = false;
    Thread explorationThread;
    boolean showWorkspaceSlice = false;
    Thread sliceExplorationThread;
    String lastLimitInfo = "";

    boolean isGripped = false;
    Timer trajectoryTimer;
    double[] trajectoryLastQ = null;
    double trajectoryLastAlpha = 0.0;
    double trajectoryLastYawOffset = 0.0;
    String trajectoryLockedCfg = "+";
    boolean ikSelectionLogEnabled = false;
    boolean useGlobalC2 = true;
    boolean useDescartesIK = true;
    private java.util.List<double[]> lastJointTrajectory = null;
    private java.util.List<double[]> lastCartesianPath = null;
    private java.util.List<java.util.List<double[]>> lastCandidatesPerStep = null;
    private int[] lastSelectedIndices = null;
    private boolean trajectoryNeedsRecalculation = true;
    private WorkspaceMap activeWorkspaceMapForTrajectory = null;

    public void notifyPathChanged() {
        trajectoryNeedsRecalculation = true;
    }

    public void setFixedHeight(double z) {
        fixedHeightSpinner.setValue(z);
        trajectoryNeedsRecalculation = true;
    }


    comm.UartManager uartManager = new comm.UartManager();
    private final comm.Ros2BridgeClient ros2BridgeClient = new comm.Ros2BridgeClient();
    private ControllerReceiver controllerReceiver;
    private Timer controllerTimer;

    private void trajDebug(String phase, String msg) {
        if (DEBUG) System.out.println("[TRAJ][" + phase + "] " + msg);
    }

    public MainFrame() {
        armPanel = new ArmPanel(this);

        setLayout(new BorderLayout());
        add(armPanel, BorderLayout.CENTER);

        buildControlPanel();
        buildTopPanel();
        buildMenuBar();

        setSize(1400, 780);
        setTitle("Mô Phỏng cánh tay Robot 6-DOF");
        setLocationRelativeTo(null);
        setDefaultCloseOperation(EXIT_ON_CLOSE);

        // Emergency Stop: ESC key stops all motion immediately
        getRootPane().getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW)
            .put(KeyStroke.getKeyStroke("ESCAPE"), "emergencyStop");
        getRootPane().getActionMap().put("emergencyStop", new AbstractAction() {
            @Override
            public void actionPerformed(ActionEvent e) {
                emergencyStop();
            }
        });

        startMotionTimer();
        
        // Start PS5 Controller Receiver & Timer
        controllerReceiver = new ControllerReceiver(5005);
        controllerReceiver.setMainFrame(this);
        controllerReceiver.start();
        startControllerTimer();

        updateArm();
        syncGuiCoordsFromFK();
    }

    /**
     * Emergency Stop: Immediately halts all motion and freezes the robot in its current position.
     * Triggered by pressing ESC or clicking the E-STOP button.
     */
    public void emergencyStop() {
        // 1. Stop all timers
        if (motionTimer != null) motionTimer.stop();
        if (trajectoryTimer != null) trajectoryTimer.stop();

        // 2. Freeze targets at current position (prevent further movement)
        System.arraycopy(anglesRight, 0, targetAnglesRight, 0, NUM_JOINTS);
        System.arraycopy(anglesLeft, 0, targetAnglesLeft, 0, NUM_JOINTS);

        // 3. Send STOP command to STM32
        if (uartManager != null && uartManager.isConnected()) {
            uartManager.sendData("ESTOP\n");
        }

        // 4. Update UI
        setTitle("⛔ EMERGENCY STOP — Nhấn Reset để tiếp tục");
        setGotoStatusRight("E-STOP!", Color.RED);
        setGotoStatusLeft("E-STOP!", Color.RED);
        armPanel.repaint();
        System.out.println("[E-STOP] Emergency stop activated!");
    }

    public double getFixedHeight() {
        return ((Number) fixedHeightSpinner.getValue()).doubleValue();
    }

    private void buildControlPanel() {
        add(BorderLayout.EAST, controlPanel);
        controlPanel.setLayout(new BorderLayout());
        controlPanel.setPreferredSize(new Dimension(680, 0));

        JPanel manualPanel = new JPanel(new GridLayout(1, 2, 8, 0));
        manualPanel.setBorder(BorderFactory.createEmptyBorder(5, 5, 5, 5));

        JPanel leftCol = buildArmControlPanel(false);
        leftCol.setBorder(BorderFactory.createTitledBorder(BorderFactory.createLineBorder(new Color(200, 80, 80), 2), "Cánh Tay TRÁI (Left Arm)"));

        JPanel rightCol = buildArmControlPanel(true);
        rightCol.setBorder(BorderFactory.createTitledBorder(BorderFactory.createLineBorder(new Color(50, 120, 200), 2), "Cánh Tay PHẢI (Right Arm)"));

        manualPanel.add(leftCol);
        manualPanel.add(rightCol);

        JScrollPane scrollManual = new JScrollPane(manualPanel);
        scrollManual.setBorder(null);
        scrollManual.setHorizontalScrollBarPolicy(ScrollPaneConstants.HORIZONTAL_SCROLLBAR_NEVER);
        scrollManual.getHorizontalScrollBar().setUnitIncrement(16);
        scrollManual.getVerticalScrollBar().setUnitIncrement(16);

        controlPanel.add(scrollManual, BorderLayout.CENTER);
    }

    private JPanel buildArmControlPanel(final boolean isRight) {
        JPanel panel = new JPanel();
        panel.setLayout(new BoxLayout(panel, BoxLayout.Y_AXIS));
        panel.setBorder(BorderFactory.createEmptyBorder(5, 5, 5, 5));

        // 1. Joint Sliders
        JPanel jointContainer = new JPanel();
        jointContainer.setLayout(new BoxLayout(jointContainer, BoxLayout.Y_AXIS));
        jointContainer.setBorder(BorderFactory.createTitledBorder("Góc các khớp (Degrees)"));

        JSlider[] armSliders = isRight ? slidersRight : slidersLeft;
        JLabel[] armLbls = isRight ? angleLblsRight : angleLblsLeft;
        double[] armAngles = isRight ? anglesRight : anglesLeft;
        double[] minLimits = isRight ? JOINT_MIN_RIGHT : JOINT_MIN_LEFT;
        double[] maxLimits = isRight ? JOINT_MAX_RIGHT : JOINT_MAX_LEFT;

        for (int i = 0; i < NUM_JOINTS; i++) {
            JPanel jointRow = new JPanel(new BorderLayout());
            jointRow.setBorder(BorderFactory.createEmptyBorder(2, 2, 2, 2));

            JLabel nameLabel = new JLabel(JOINT_NAMES[i]);
            jointRow.add(nameLabel, BorderLayout.NORTH);

            int minVal = (int) Math.min(minLimits[i], maxLimits[i]);
            int maxVal = (int) Math.max(minLimits[i], maxLimits[i]);
            armSliders[i] = new JSlider(minVal, maxVal, (int) Math.round(armAngles[i]));
            armSliders[i].setMajorTickSpacing(60);
            armSliders[i].setPaintTicks(true);
            armSliders[i].setPreferredSize(new Dimension(120, 25));
            armSliders[i].addChangeListener(this);

            armLbls[i] = new JLabel(armSliders[i].getValue() + "°");
            armLbls[i].setPreferredSize(new Dimension(45, 20));
            armLbls[i].setHorizontalAlignment(SwingConstants.RIGHT);

            jointRow.add(armSliders[i], BorderLayout.CENTER);
            jointRow.add(armLbls[i], BorderLayout.EAST);
            jointContainer.add(jointRow);
        }
        panel.add(jointContainer);

        // 2. Cartesian Coordinates Panel
        JPanel gotoPanel = new JPanel();
        gotoPanel.setLayout(new BoxLayout(gotoPanel, BoxLayout.Y_AXIS));
        gotoPanel.setBorder(BorderFactory.createTitledBorder("Tọa độ kẹp (X, Y, Z)"));

        JLabel eeLabel = isRight ? endEffectorLabelRight : endEffectorLabelLeft;
        eeLabel.setFont(new Font("Arial", Font.BOLD, 12));
        eeLabel.setBorder(BorderFactory.createEmptyBorder(5, 5, 5, 5));
        gotoPanel.add(eeLabel);

        JPanel coordRow = new JPanel(new GridLayout(3, 1, 4, 2));

        JSlider sX = isRight ? slXRight : slXLeft;
        JSlider sY = isRight ? slYRight : slYLeft;
        JSlider sZ = isRight ? slZRight : slZLeft;
        JTextField tX = isRight ? txXRight : txXLeft;
        JTextField tY = isRight ? txYRight : txYLeft;
        JTextField tZ = isRight ? txZRight : txZLeft;

        sX.setPreferredSize(new Dimension(120, 22));
        sY.setPreferredSize(new Dimension(120, 22));
        sZ.setPreferredSize(new Dimension(120, 22));

        JPanel rowX = new JPanel(new BorderLayout(5, 0));
        rowX.add(new JLabel("X:"), BorderLayout.WEST);
        rowX.add(sX, BorderLayout.CENTER);
        tX.setPreferredSize(new Dimension(50, 20));
        rowX.add(tX, BorderLayout.EAST);

        JPanel rowY = new JPanel(new BorderLayout(5, 0));
        rowY.add(new JLabel("Y:"), BorderLayout.WEST);
        rowY.add(sY, BorderLayout.CENTER);
        tY.setPreferredSize(new Dimension(50, 20));
        rowY.add(tY, BorderLayout.EAST);

        JPanel rowZ = new JPanel(new BorderLayout(5, 0));
        rowZ.add(new JLabel("Z:"), BorderLayout.WEST);
        rowZ.add(sZ, BorderLayout.CENTER);
        tZ.setPreferredSize(new Dimension(50, 20));
        rowZ.add(tZ, BorderLayout.EAST);

        coordRow.add(rowX);
        coordRow.add(rowY);
        coordRow.add(rowZ);
        gotoPanel.add(coordRow);

        sX.addChangeListener(this);
        sY.addChangeListener(this);
        sZ.addChangeListener(this);

        JButton btnG = isRight ? btnGotoRight : btnGotoLeft;
        JButton btnS = isRight ? btnSyncCoordsRight : btnSyncCoordsLeft;
        JLabel lblStatus = isRight ? gotoStatusRight : gotoStatusLeft;

        JPanel btnRow = new JPanel(new FlowLayout(FlowLayout.LEFT, 4, 2));
        btnRow.add(btnG);
        btnRow.add(btnS);
        btnRow.add(lblStatus);
        gotoPanel.add(btnRow);

        btnG.addActionListener(this);
        btnS.addActionListener(this);
        ActionListener gotoAction = e -> gotoCoordinate(isRight);
        tX.addActionListener(gotoAction);
        tY.addActionListener(gotoAction);
        tZ.addActionListener(gotoAction);

        panel.add(gotoPanel);

        // 3. Config (IK Pose) & Alpha Orientation
        JPanel configPanel = new JPanel();
        configPanel.setLayout(new BoxLayout(configPanel, BoxLayout.Y_AXIS));
        configPanel.setBorder(BorderFactory.createTitledBorder("Cấu hình & Hướng kẹp"));

        JComboBox<String> comb = isRight ? configComboRight : configComboLeft;
        configPanel.add(comb);

        JComboBox<String> gCombo = isRight ? gripperModeComboRight : gripperModeComboLeft;
        JPanel alphaRow = new JPanel(new BorderLayout());
        alphaRow.add(new JLabel("Chế độ hướng kẹp:"), BorderLayout.NORTH);
        alphaRow.add(gCombo, BorderLayout.CENTER);

        gCombo.addActionListener(e -> {
            gotoCoordinate(isRight);
        });
        comb.addActionListener(e -> {
            gotoCoordinate(isRight);
        });

        configPanel.add(alphaRow);

        panel.add(configPanel);

        return panel;
    }

    private void buildTopPanel() {
        add(BorderLayout.NORTH, topPanel);

        // Emergency Stop Button (prominent red button)
        JButton btnEStop = new JButton("⛔ E-STOP");
        btnEStop.setBackground(new Color(220, 30, 30));
        btnEStop.setForeground(Color.WHITE);
        btnEStop.setFont(new Font("Arial", Font.BOLD, 13));
        btnEStop.setFocusPainted(false);
        btnEStop.setToolTipText("Emergency Stop — Dừng khẩn cấp (ESC)");
        btnEStop.addActionListener(e -> emergencyStop());
        topPanel.add(btnEStop);
        topPanel.add(new JSeparator(SwingConstants.VERTICAL));

        JButton btnGripper = new JButton("Đóng / Mở Kẹp");
        btnGripper.addActionListener(e -> {
            if (isRightArmSelected) {
                isGrippedRight = !isGrippedRight;
                setGotoStatusRight(isGrippedRight ? "Đã ĐÓNG kẹp (R)" : "Đã MỞ kẹp (R)", Color.BLUE);
                if (uartManager != null && uartManager.isConnected()) {
                    String cmd = isGrippedRight ? "R:GRIP\n" : "R:RELEASE\n";
                    uartManager.sendData(cmd);
                }
            } else {
                isGrippedLeft = !isGrippedLeft;
                setGotoStatusLeft(isGrippedLeft ? "Đã ĐÓNG kẹp (L)" : "Đã MỞ kẹp (L)", Color.BLUE);
                if (uartManager != null && uartManager.isConnected()) {
                    String cmd = isGrippedLeft ? "L:GRIP\n" : "L:RELEASE\n";
                    uartManager.sendData(cmd);
                }
            }
            armPanel.repaint();
        });
        topPanel.add(btnGripper);

        topPanel.add(new JSeparator(SwingConstants.VERTICAL));

        trajArmCombo.addActionListener(e -> {
            boolean right = (trajArmCombo.getSelectedIndex() == 0);
            if (isRightArmSelected != right) {
                isRightArmSelected = right;
                updateArm();
                if (showWorkspaceSlice) {
                    updateWorkspaceSlice();
                }
                trajectoryNeedsRecalculation = true;
            }
        });
        topPanel.add(new JLabel(" Tay Vẽ:"));
        topPanel.add(trajArmCombo);

        cbEnableDrawing.setText("Bật Vẽ");
        topPanel.add(cbEnableDrawing);

        JButton btnClearMouseDraw = new JButton("Xóa Hình");
        btnClearMouseDraw.addActionListener(evt -> {
            armPanel.referencePath.clear();
            armPanel.repaint();
            trajectoryNeedsRecalculation = true;
        });
        topPanel.add(btnClearMouseDraw);

        JButton btnStartTraj = new JButton("Chạy Quỹ Đạo");
        btnStartTraj.addActionListener(e -> {
            trajDebug("START_CLICK", "Start trajectory button clicked");
            if (trajectoryTimer != null) trajectoryTimer.stop();
            isRightArmSelected = (trajArmCombo.getSelectedIndex() == 0);
            try {
                trajDebug("INPUT_MOUSE_DRAW", "Running custom mouse drawn trajectory");
                runCustomPathTrajectory(armPanel.referencePath);
            } catch (Exception ex) {
                trajDebug("INPUT_ERROR", ex.toString());
                JOptionPane.showMessageDialog(this, "Có lỗi xảy ra", "Lỗi", JOptionPane.ERROR_MESSAGE);
            }
        });
        topPanel.add(btnStartTraj);

        JButton btnStopTraj = new JButton("Dừng");
        btnStopTraj.addActionListener(e -> {
            if (trajectoryTimer != null) trajectoryTimer.stop();
            setTitle("Mô Phỏng robot song song");
        });
        topPanel.add(btnStopTraj);

        topPanel.add(new JSeparator(SwingConstants.VERTICAL));

        topPanel.add(new JLabel("  Tốc độ:"));
        speedSlider.setPreferredSize(new Dimension(100, 25));
        speedSlider.setMajorTickSpacing(30);
        speedSlider.setPaintTicks(true);
        speedSlider.addChangeListener(ev -> {
            speedLabel.setText(speedSlider.getValue() + " °/s");
            trajectoryNeedsRecalculation = true;
        });
        topPanel.add(speedSlider);
        topPanel.add(speedLabel);

        JButton btnRos2Plan = new JButton("ROS2 Plan");
        btnRos2Plan.setToolTipText("Gui toa do hien tai sang ROS 2 bridge");
        btnRos2Plan.addActionListener(e -> requestRos2Plan());
        topPanel.add(new JSeparator(SwingConstants.VERTICAL));
        topPanel.add(btnRos2Plan);

        topPanel.add(new JSeparator(SwingConstants.VERTICAL));
        topPanel.add(fixedHeightCb);
        topPanel.add(new JLabel("Z:"));
        fixedHeightSpinner.setPreferredSize(new Dimension(55, 22));
        topPanel.add(fixedHeightSpinner);
        fixedHeightSpinner.addChangeListener(ev -> {
            if (showWorkspaceSlice) {
                updateWorkspaceSlice();
            }
            armPanel.repaint();
        });
        
        fixedHeightCb.addActionListener(ev -> {
            fixedHeightMode = fixedHeightCb.isSelected();
            if (clickModeItem != null) {
                clickModeItem.setSelected(fixedHeightMode);
            }
            armPanel.repaint();
        });

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

    private java.util.List<double[]> parseTrajectory(String json) {
        java.util.List<double[]> trajectory = new java.util.ArrayList<>();
        int trajIndex = json.indexOf("\"trajectory\"");
        if (trajIndex == -1) return trajectory;

        int start = json.indexOf("[[", trajIndex);
        int end = json.indexOf("]]", trajIndex);
        if (start == -1 || end == -1 || start >= end) return trajectory;

        String content = json.substring(start + 2, end);
        String[] steps = content.split("\\]\\s*,\\s*\\[");
        for (String step : steps) {
            String[] parts = step.split(",");
            if (parts.length >= 6) {
                double[] q = new double[6];
                for (int i = 0; i < 6; i++) {
                    q[i] = Double.parseDouble(parts[i].trim());
                }
                trajectory.add(q);
            }
        }
        return trajectory;
    }

    private void requestRos2Plan() {
        final boolean isRight = isRightArmSelected;
        final String armName = isRight ? "right" : "left";
        
        final boolean hasDrawnPath = cbEnableDrawing.isSelected() && !armPanel.referencePath.isEmpty();
        final java.util.List<double[]> finalPath;
        if (hasDrawnPath) {
            double speed = Math.max(1.0, speedSlider.getValue() / 2.0); // units per sec
            java.util.List<double[]> resampled = resamplePath(armPanel.referencePath, speed);
            finalPath = resampled.isEmpty() ? armPanel.referencePath : resampled;
            setGotoStatus("ROS2: dang gui yeu cau quy dao (" + finalPath.size() + " diem)...", new Color(0, 90, 180));
        } else {
            finalPath = null;
            setGotoStatus("ROS2: dang gui yeu cau...", new Color(0, 90, 180));
        }

        final JTextField tX = isRight ? txXRight : txXLeft;
        final JTextField tY = isRight ? txYRight : txYLeft;
        final JTextField tZ = isRight ? txZRight : txZLeft;

        final double x;
        final double y;
        final double z;
        if (!hasDrawnPath) {
            try {
                x = Double.parseDouble(tX.getText().trim());
                y = Double.parseDouble(tY.getText().trim());
                z = Double.parseDouble(tZ.getText().trim());
            } catch (NumberFormatException ex) {
                setGotoStatus("ROS2: toa do khong hop le", Color.RED);
                return;
            }
        } else {
            x = 0; y = 0; z = 0;
        }

        final JComboBox<String> armConfigCombo = isRight ? configComboRight : configComboLeft;
        final String preferredConfig = armConfigCombo.getSelectedIndex() == 0 ? "+" : "-";

        new SwingWorker<String, Void>() {
            @Override
            protected String doInBackground() throws Exception {
                double[] currentAngles = isRight ? anglesRight : anglesLeft;
                if (hasDrawnPath) {
                    return ros2BridgeClient.requestPlanPath(armName, finalPath, currentAngles, preferredConfig);
                } else {
                    return ros2BridgeClient.requestPlanPose(armName, x, y, z, currentAngles, preferredConfig);
                }
            }

            @Override
            protected void done() {
                try {
                    String response = get();
                    if (response.contains("\"ok\":true") || response.contains("\"ok\": true")) {
                        java.util.List<double[]> traj = parseTrajectory(response);
                        if (!traj.isEmpty()) {
                            setGotoStatus("ROS2: Lap quy dao thanh cong!", new Color(0, 140, 0));
                            double[] armAngles = isRight ? anglesRight : anglesLeft;
                            double[] armTargetAngles = isRight ? targetAnglesRight : targetAnglesLeft;
                            JSlider[] armSliders = isRight ? slidersRight : slidersLeft;
                            JLabel[] armAngleLbls = isRight ? angleLblsRight : angleLblsLeft;
                            runPlayback(traj, "ROS2 Path", isRight, armAngles, armTargetAngles, armSliders, armAngleLbls);
                        } else {
                            setGotoStatus("ROS2: da nhan phan hoi nhung rong", new Color(180, 110, 0));
                        }
                    } else {
                        // Extract error
                        int errStart = response.indexOf("\"error\":\"");
                        String errMsg = "Thất bại";
                        if (errStart != -1) {
                            int errEnd = response.indexOf("\"", errStart + 9);
                            if (errEnd != -1) {
                                errMsg = response.substring(errStart + 9, errEnd);
                            }
                        }
                        setGotoStatus("ROS2: " + errMsg, Color.RED);
                    }
                } catch (Exception ex) {
                    setGotoStatus("ROS2: khong ket noi bridge", Color.RED);
                    if (DEBUG) {
                        ex.printStackTrace();
                    }
                }
            }
        }.execute();
    }

    private void buildMenuBar() {
        final JMenuBar menuBar = new JMenuBar();

        JMenu dieukhienMenu = new JMenu("Điều khiển");

        JMenuItem resetItem = new JMenuItem("Reset về gốc");
        resetItem.setMnemonic('R');
        resetItem.setActionCommand("Reset");


        JMenuItem exitItem = new JMenuItem("Thoát");
        exitItem.setMnemonic('X');
        exitItem.setActionCommand("Exit");

        MenuItemListener menuItemListener = new MenuItemListener();
        resetItem.addActionListener(menuItemListener);

        exitItem.addActionListener(menuItemListener);

        dieukhienMenu.add(resetItem);

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
        JCheckBoxMenuItem workspaceSliceItem = new JCheckBoxMenuItem("Hiện Lát Cắt Vùng Làm Việc (Fixed Z)", showWorkspaceSlice);

        workspaceItem.addItemListener(e -> {
            showWorkspace = workspaceItem.isSelected();
            if (showWorkspace) {
                workspaceSliceItem.setSelected(false);
                showWorkspaceSlice = false;
                stopWorkspaceSliceExploration();
                armPanel.clearWorkspaceSlice();
                
                armPanel.clearWorkspace();
                armPanel.workspaceStatus = "";
                runWorkspaceExploration();
            } else {
                stopWorkspaceExploration();
                armPanel.workspaceStatus = "";
                armPanel.repaint();
            }
        });

        final boolean[] isInternalChanging = { false };
        workspaceSliceItem.addItemListener(e -> {
            if (isInternalChanging[0]) return;
            showWorkspaceSlice = workspaceSliceItem.isSelected();
            if (showWorkspaceSlice) {
                String input = JOptionPane.showInputDialog(MainFrame.this, 
                        "Nhập chiều cao Z mong muốn quét (mm):", 
                        String.valueOf(getFixedHeight()));
                if (input == null) {
                    isInternalChanging[0] = true;
                    workspaceSliceItem.setSelected(false);
                    isInternalChanging[0] = false;
                    
                    showWorkspaceSlice = false;
                    stopWorkspaceSliceExploration();
                    armPanel.clearWorkspaceSlice();
                    armPanel.workspaceStatus = "";
                    armPanel.repaint();
                    return;
                }
                
                double enteredZ;
                try {
                    enteredZ = Double.parseDouble(input.trim());
                } catch (NumberFormatException ex) {
                    JOptionPane.showMessageDialog(MainFrame.this, 
                            "Vui lòng nhập một số thực hợp lệ!", 
                            "Lỗi định dạng", 
                            JOptionPane.ERROR_MESSAGE);
                    isInternalChanging[0] = true;
                    workspaceSliceItem.setSelected(false);
                    isInternalChanging[0] = false;
                    
                    showWorkspaceSlice = false;
                    stopWorkspaceSliceExploration();
                    armPanel.clearWorkspaceSlice();
                    armPanel.workspaceStatus = "";
                    armPanel.repaint();
                    return;
                }

                fixedHeightSpinner.setValue(enteredZ);

                workspaceItem.setSelected(false);
                showWorkspace = false;
                stopWorkspaceExploration();
                armPanel.clearWorkspace();
                
                updateWorkspaceSlice();
            } else {
                stopWorkspaceSliceExploration();
                armPanel.clearWorkspaceSlice();
                armPanel.workspaceStatus = "";
                armPanel.repaint();
            }
        });

        JMenuItem analyticsItem = new JMenuItem("Phân Tích Đồ Thị Quỹ Đạo");
        analyticsItem.setActionCommand("ShowAnalytics");
        analyticsItem.addActionListener(menuItemListener);

        JMenuItem topViewItem = new JMenuItem("Hệ Trục (Top View)");
        topViewItem.setActionCommand("TopView");
        topViewItem.addActionListener(menuItemListener);

        JMenuItem perspItem = new JMenuItem("3D Perspective");
        perspItem.setActionCommand("Persp");
        perspItem.addActionListener(menuItemListener);

        hienthiMenu.add(gridItem);
        hienthiMenu.add(trailItem);
        hienthiMenu.add(workspaceItem);
        hienthiMenu.add(workspaceSliceItem);
        hienthiMenu.addSeparator();
        hienthiMenu.add(analyticsItem);
        hienthiMenu.addSeparator();
        hienthiMenu.add(topViewItem);
        hienthiMenu.add(perspItem);

        // ---- Menu: Chế độ ----
        JMenu chedoMenu = new JMenu("Chế độ");

        clickModeItem = new JCheckBoxMenuItem("Click tới tọa độ (Z cố định)", fixedHeightMode);
        clickModeItem.addItemListener(e -> {
            fixedHeightMode = clickModeItem.isSelected();
            fixedHeightCb.setSelected(fixedHeightMode);
            armPanel.repaint();
        });

        JCheckBoxMenuItem manualModeItem = new JCheckBoxMenuItem("Chế độ thủ công (FK)", manualMode);
        manualModeItem.addItemListener(e -> {
            manualMode = manualModeItem.isSelected();
            if (manualMode)
                updateArm(); // Sync immediately
        });

        JCheckBoxMenuItem c2SplineItem = new JCheckBoxMenuItem("Mượt C2 toàn cục (Quintic Spline)", useGlobalC2);
        c2SplineItem.addItemListener(e -> {
            useGlobalC2 = c2SplineItem.isSelected();
            trajectoryNeedsRecalculation = true;
        });

        JCheckBoxMenuItem descartesItem = new JCheckBoxMenuItem("Tối ưu quỹ đạo Descartes (Graph Search)", useDescartesIK);
        descartesItem.addItemListener(e -> {
            useDescartesIK = descartesItem.isSelected();
            trajectoryNeedsRecalculation = true;
        });

        JCheckBoxMenuItem jniItem = new JCheckBoxMenuItem("Dùng bộ giải C++ (JNI Numerical)", kinematics.Kinematics.solverMode == 1);
        JCheckBoxMenuItem ikFastItem = new JCheckBoxMenuItem("Dùng bộ giải C++ (IKFast)", kinematics.Kinematics.solverMode == 2);

        jniItem.addItemListener(e -> {
            if (jniItem.isSelected()) {
                if (kinematics.JniKinematics.isLoaded()) {
                    ikFastItem.setSelected(false);
                    useJniIKCb.setSelected(true);
                    useIkFastCb.setSelected(false);
                    kinematics.Kinematics.solverMode = 1;
                } else {
                    JOptionPane.showMessageDialog(MainFrame.this,
                            "Không tìm thấy thư viện kinematics_jni!\nVui lòng biên dịch thư viện C++ bằng file build_jni.py.",
                            "Lỗi tải thư viện",
                            JOptionPane.ERROR_MESSAGE);
                    jniItem.setSelected(false);
                }
            } else {
                if (kinematics.Kinematics.solverMode == 1) {
                    kinematics.Kinematics.solverMode = 0;
                    useJniIKCb.setSelected(false);
                }
            }
            trajectoryNeedsRecalculation = true;
        });

        ikFastItem.addItemListener(e -> {
            if (ikFastItem.isSelected()) {
                if (kinematics.JniKinematics.isLoaded()) {
                    jniItem.setSelected(false);
                    useJniIKCb.setSelected(false);
                    useIkFastCb.setSelected(true);
                    kinematics.Kinematics.solverMode = 2;
                } else {
                    JOptionPane.showMessageDialog(MainFrame.this,
                            "Không tìm thấy thư viện kinematics_jni!\nVui lòng biên dịch thư viện C++ bằng file build_jni.py.",
                            "Lỗi tải thư viện",
                            JOptionPane.ERROR_MESSAGE);
                    ikFastItem.setSelected(false);
                }
            } else {
                if (kinematics.Kinematics.solverMode == 2) {
                    kinematics.Kinematics.solverMode = 0;
                    useIkFastCb.setSelected(false);
                }
            }
            trajectoryNeedsRecalculation = true;
        });

        chedoMenu.add(clickModeItem);
        chedoMenu.add(manualModeItem);
        chedoMenu.add(c2SplineItem);
        chedoMenu.add(descartesItem);
        chedoMenu.add(jniItem);
        chedoMenu.add(ikFastItem);


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

                case "ShowAnalytics" -> {
                    if (lastJointTrajectory == null) {
                        JOptionPane.showMessageDialog(MainFrame.this,
                            "Chưa có quỹ đạo nào được chạy để phân tích!",
                            "Cảnh báo", JOptionPane.WARNING_MESSAGE);
                    } else {
                        TrajectoryAnalyticsWindow analytics = new TrajectoryAnalyticsWindow(
                            lastJointTrajectory,
                            lastCartesianPath,
                            lastCandidatesPerStep,
                            lastSelectedIndices,
                            isRightArmSelected
                        );
                        analytics.setVisible(true);
                    }
                }

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
        anglesLeft[0] = anglesRight[0];
        targetAnglesLeft[0] = targetAnglesRight[0];

        // 3. Sync sliders and labels of Right arm without triggering listener
        for (int i = 0; i < NUM_JOINTS; i++) {
            int rounded = (int) Math.round(anglesRight[i]);
            if (slidersRight[i] != null && slidersRight[i].getValue() != rounded) {
                slidersRight[i].removeChangeListener(this);
                slidersRight[i].setValue(rounded);
                slidersRight[i].addChangeListener(this);
            }
            if (angleLblsRight[i] != null) {
                String text = rounded + "°";
                if (!text.equals(angleLblsRight[i].getText())) {
                    angleLblsRight[i].setText(text);
                }
            }
        }

        // 4. Sync sliders and labels of Left arm without triggering listener
        for (int i = 0; i < NUM_JOINTS; i++) {
            int rounded = (int) Math.round(anglesLeft[i]);
            if (slidersLeft[i] != null && slidersLeft[i].getValue() != rounded) {
                slidersLeft[i].removeChangeListener(this);
                slidersLeft[i].setValue(rounded);
                slidersLeft[i].addChangeListener(this);
            }
            if (angleLblsLeft[i] != null) {
                String text = rounded + "°";
                if (!text.equals(angleLblsLeft[i].getText())) {
                    angleLblsLeft[i].setText(text);
                }
            }
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
     * Format: R:q1,q2,... and L:q1,q2,...
     */
    private void sendJointsToUart() {
        if (uartManager != null && uartManager.isConnected()) {
            // Check Right Arm
            boolean changedRight = false;
            for (int i = 0; i < NUM_JOINTS; i++) {
                if (Math.abs(anglesRight[i] - lastSentAnglesRight[i]) > 0.01) {
                    changedRight = true;
                    break;
                }
            }

            if (changedRight) {
                StringBuilder sb = new StringBuilder("R:");
                for (int i = 0; i < NUM_JOINTS; i++) {
                    double val = anglesRight[i];
                    if (i == 3) {
                        // q₄ = θ₄ + θ₃ (parallelogram: θ→q)
                        val = anglesRight[3] + anglesRight[2];
                    }
                    sb.append(String.format("%d", (int) Math.round(val)));
                    if (i < NUM_JOINTS - 1) {
                        sb.append(",");
                    }
                    lastSentAnglesRight[i] = anglesRight[i];
                }
                sb.append("\n");
                String data = sb.toString();
                uartManager.sendData(data);
                if (DEBUG && uartManager.isConnected()) {
                    System.out.print("Sent UART: " + data);
                }
            }

            // Check Left Arm
            boolean changedLeft = false;
            for (int i = 0; i < NUM_JOINTS; i++) {
                if (Math.abs(anglesLeft[i] - lastSentAnglesLeft[i]) > 0.01) {
                    changedLeft = true;
                    break;
                }
            }

            if (changedLeft) {
                StringBuilder sb = new StringBuilder("L:");
                for (int i = 0; i < NUM_JOINTS; i++) {
                    double val = anglesLeft[i];
                    if (i == 3) {
                        // q₄ = θ₄ + θ₃ (parallelogram coupling: θ→q)
                        val = anglesLeft[3] + anglesLeft[2];
                    }
                    sb.append(String.format("%d", (int) Math.round(val)));
                    if (i < NUM_JOINTS - 1) {
                        sb.append(",");
                    }
                    lastSentAnglesLeft[i] = anglesLeft[i];
                }
                sb.append("\n");
                String data = sb.toString();
                uartManager.sendData(data);
                if (DEBUG && uartManager.isConnected()) {
                    System.out.print("Sent UART: " + data);
                }
            }
        }
    }

    /**
     * Request the arm to move smoothly to the given joint angles (radians).
     * The motion timer will interpolate from the current position.
     */
    void setTargetAngles(double[] q_deg) {
        if (isRightArmSelected) {
            setTargetAnglesRight(q_deg);
        } else {
            setTargetAnglesLeft(q_deg);
        }
    }

    void setTargetAnglesRight(double[] q_deg) {
        System.arraycopy(q_deg, 0, targetAnglesRight, 0, NUM_JOINTS);
        // Synchronize Joint 1 (Waist/Hip) targets immediately
        targetAnglesLeft[0] = targetAnglesRight[0];
        anglesLeft[0] = anglesRight[0];
        startMotionTimer();
    }

    void setTargetAnglesLeft(double[] q_deg) {
        System.arraycopy(q_deg, 0, targetAnglesLeft, 0, NUM_JOINTS);
        // Synchronize Joint 1 (Waist/Hip) targets immediately
        targetAnglesRight[0] = targetAnglesLeft[0];
        anglesRight[0] = anglesLeft[0];
        startMotionTimer();
    }

    public void setGotoStatus(String text, Color color) {
        if (isRightArmSelected) {
            setGotoStatusRight(text, color);
        } else {
            setGotoStatusLeft(text, color);
        }
    }

    public void setGotoStatusRight(String text, Color color) {
        gotoStatusRight.setForeground(color);
        gotoStatusRight.setText(text);
    }

    public void setGotoStatusLeft(String text, Color color) {
        gotoStatusLeft.setForeground(color);
        gotoStatusLeft.setText(text);
    }

    public boolean isArmInterpolating() {
        boolean isRight = isRightArmSelected;
        double[] armAngles = isRight ? anglesRight : anglesLeft;
        double[] armTargetAngles = isRight ? targetAnglesRight : targetAnglesLeft;
        for (int i = 0; i < NUM_JOINTS; i++) {
            if (Math.abs(armTargetAngles[i] - armAngles[i]) > 0.05) {
                return true;
            }
        }
        return false;
    }

    private void gotoCoordinate() {
        gotoCoordinate(isRightArmSelected);
    }

    private void gotoCoordinate(boolean isRight) {
        try {
            JTextField tX = isRight ? txXRight : txXLeft;
            JTextField tY = isRight ? txYRight : txYLeft;
            JTextField tZ = isRight ? txZRight : txZLeft;

            double px = Double.parseDouble(tX.getText().trim());
            double py = Double.parseDouble(tY.getText().trim());
            double pz = Double.parseDouble(tZ.getText().trim());

            if (pz < 0) {
                if (isRight) setGotoStatusRight("Z không âm!", Color.RED);
                else setGotoStatusLeft("Z không âm!", Color.RED);
                return;
            }

            String prefCfg = (isRight ? configComboRight : configComboLeft).getSelectedIndex() == 0 ? "+" : "-";
            double[] result = isRight ? solveIKSmartRight(px, py, pz, prefCfg) : solveIKSmartLeft(px, py, pz, prefCfg);
            if (result != null) {
                if (isRight) {
                    setTargetAnglesRight(result);
                    
                    // Retract Left arm to home, preserving shared Joint 1
                    double[] leftHome = { result[0], 0, -10, 30, 0, 0 };
                    setTargetAnglesLeft(leftHome);
                    
                    setGotoStatusRight("OK", new Color(0, 140, 0));
                    setGotoStatusLeft("Về Home", Color.BLUE);
                } else {
                    setTargetAnglesLeft(result);
                    
                    // Retract Right arm to home, preserving shared Joint 1
                    double[] rightHome = { result[0], 0, 10, -30, 0, 0 };
                    setTargetAnglesRight(rightHome);
                    
                    setGotoStatusLeft("OK", new Color(0, 140, 0));
                    setGotoStatusRight("Về Home", Color.BLUE);
                }
            } else {
                if (isRight) setGotoStatusRight("Ngoài tầm/Góc!", Color.RED);
                else setGotoStatusLeft("Ngoài tầm/Góc!", Color.RED);
            }

            // Sync Z slider anyway
            JSlider sZ = isRight ? slZRight : slZLeft;
            isUpdatingFromFK = true;
            sZ.setValue((int) Math.round(pz));
            isUpdatingFromFK = false;

            if (showWorkspaceSlice) {
                updateWorkspaceSlice();
            }
        } catch (NumberFormatException ex) {
            if (isRight) gotoStatusRight.setText("Sai định dạng");
            else gotoStatusLeft.setText("Sai định dạng");
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
        } else if (e.getSource() == btnGotoRight) {
            gotoCoordinate(true);
        } else if (e.getSource() == btnGotoLeft) {
            gotoCoordinate(false);
        } else if (e.getSource() == btnSyncCoordsRight) {
            double[] ee = armPanel.getRightEndEffectorPosition();
            txXRight.setText(String.format("%.1f", ee[0]));
            txYRight.setText(String.format("%.1f", ee[1]));
            txZRight.setText(String.format("%.1f", ee[2]));
        } else if (e.getSource() == btnSyncCoordsLeft) {
            double[] ee = armPanel.getLeftEndEffectorPosition();
            txXLeft.setText(String.format("%.1f", ee[0]));
            txYLeft.setText(String.format("%.1f", ee[1]));
            txZLeft.setText(String.format("%.1f", ee[2]));
        } else if (e.getSource() == fixedHeightCb) {
            fixedHeightMode = fixedHeightCb.isSelected();
            armPanel.repaint();
        }
    }

    @Override
    public void stateChanged(ChangeEvent e) {
        // 1. Right joint sliders
        for (int i = 0; i < NUM_JOINTS; i++) {
            if (e.getSource() == slidersRight[i]) {
                double oldVal = anglesRight[i];
                anglesRight[i] = slidersRight[i].getValue();

                // Prevent gripper from going under floor
                double[][] pts3d = armPanel.computeAllJoints3DRight();
                double[] ee = pts3d[NUM_JOINTS + 1];
                // Parallelogram constraint: 5° ≤ |q₃+q₄| ≤ 90° where q₃=θ₃, q₄=θ₄+θ₃
                double q3 = anglesRight[2];
                double q4 = anglesRight[3] + anglesRight[2];
                double absSum = Math.abs(q3 + q4);
                if (ee[2] < 0 || absSum < 5.0 - 0.1 || absSum > 90.0 + 0.1) {
                    anglesRight[i] = oldVal;
                    slidersRight[i].setValue((int) Math.round(oldVal));
                }

                targetAnglesRight[i] = anglesRight[i];
                
                if (i == 0) {
                    // Synchronize shared waist joint 1
                    anglesLeft[0] = anglesRight[0];
                    targetAnglesLeft[0] = anglesRight[0];
                    if (slidersLeft[0] != null && slidersLeft[0].getValue() != (int)Math.round(anglesRight[0])) {
                        slidersLeft[0].setValue((int)Math.round(anglesRight[0]));
                    }
                    if (angleLblsLeft[0] != null) {
                        angleLblsLeft[0].setText((int)Math.round(anglesRight[0]) + "°");
                    }
                }

                angleLblsRight[i].setText((int) anglesRight[i] + "°");
                updateArm();
                return;
            }
        }

        // 2. Left joint sliders
        for (int i = 0; i < NUM_JOINTS; i++) {
            if (e.getSource() == slidersLeft[i]) {
                double oldVal = anglesLeft[i];
                anglesLeft[i] = slidersLeft[i].getValue();

                // Prevent gripper from going under floor
                double[][] pts3d = armPanel.computeAllJoints3DLeft();
                double[] ee = pts3d[NUM_JOINTS + 1];
                // Parallelogram constraint: 5° ≤ |q₃+q₄| ≤ 90° where q₃=θ₃, q₄=θ₄+θ₃
                double q3 = anglesLeft[2];
                double q4 = anglesLeft[3] + anglesLeft[2];
                double absSum = Math.abs(q3 + q4);
                if (ee[2] < 0 || absSum < 5.0 - 0.1 || absSum > 90.0 + 0.1) {
                    anglesLeft[i] = oldVal;
                    slidersLeft[i].setValue((int) Math.round(oldVal));
                }

                targetAnglesLeft[i] = anglesLeft[i];
                
                if (i == 0) {
                    // Synchronize shared waist joint 1
                    anglesRight[0] = anglesLeft[0];
                    targetAnglesRight[0] = anglesLeft[0];
                    if (slidersRight[0] != null && slidersRight[0].getValue() != (int)Math.round(anglesLeft[0])) {
                        slidersRight[0].setValue((int)Math.round(anglesLeft[0]));
                    }
                    if (angleLblsRight[0] != null) {
                        angleLblsRight[0].setText((int)Math.round(anglesLeft[0]) + "°");
                    }
                }

                angleLblsLeft[i].setText((int) anglesLeft[i] + "°");
                updateArm();
                return;
            }
        }

        // 3. Right Coordinate Slider drag
        for (int i = 0; i < 3; i++) {
            JSlider s = (i == 0) ? slXRight : (i == 1) ? slYRight : slZRight;
            if (e.getSource() == s && !isUpdatingFromFK) {
                try {
                    double px = slXRight.getValue();
                    double py = slYRight.getValue();
                    double pz = slZRight.getValue();
                    
                    if (i == 2) {
                        txZRight.setText(String.valueOf((int) pz));
                        if (showWorkspaceSlice) {
                            updateWorkspaceSlice();
                        }
                    }

                    String prefCfg = configComboRight.getSelectedIndex() == 0 ? "+" : "-";
                    double[] res = solveIKSmartRight(px, py, pz, prefCfg);
                    if (res != null) {
                        setTargetAnglesRight(res);
                        txXRight.setText(String.valueOf((int) px));
                        txYRight.setText(String.valueOf((int) py));
                        txZRight.setText(String.valueOf((int) pz));
                        setGotoStatusRight("OK", new Color(0, 140, 0));
                    } else {
                        setGotoStatusRight("Ngoài tầm/Góc", Color.RED);
                    }
                } catch (Exception ex) {
                }
                return;
            }
        }

        // 4. Left Coordinate Slider drag
        for (int i = 0; i < 3; i++) {
            JSlider s = (i == 0) ? slXLeft : (i == 1) ? slYLeft : slZLeft;
            if (e.getSource() == s && !isUpdatingFromFK) {
                try {
                    double px = slXLeft.getValue();
                    double py = slYLeft.getValue();
                    double pz = slZLeft.getValue();
                    
                    if (i == 2) {
                        txZLeft.setText(String.valueOf((int) pz));
                        if (showWorkspaceSlice) {
                            updateWorkspaceSlice();
                        }
                    }

                    String prefCfg = configComboLeft.getSelectedIndex() == 0 ? "+" : "-";
                    double[] res = solveIKSmartLeft(px, py, pz, prefCfg);
                    if (res != null) {
                        setTargetAnglesLeft(res);
                        txXLeft.setText(String.valueOf((int) px));
                        txYLeft.setText(String.valueOf((int) py));
                        txZLeft.setText(String.valueOf((int) pz));
                        setGotoStatusLeft("OK", new Color(0, 140, 0));
                    } else {
                        setGotoStatusLeft("Ngoài tầm/Góc", Color.RED);
                    }
                } catch (Exception ex) {
                }
                return;
            }
        }
    }

    void updateArm() {
        double[][] ptsRight = armPanel.computeAllJoints3DRight();
        double[] eeRight = ptsRight[NUM_JOINTS + 1];
        setLabelIfChanged(endEffectorLabelRight, String.format("Tọa độ kẹp (R): (%.1f,  %.1f,  %.1f)", eeRight[0], eeRight[1], eeRight[2]));

        double[][] ptsLeft = armPanel.computeAllJoints3DLeft();
        double[] eeLeft = ptsLeft[NUM_JOINTS + 1];
        setLabelIfChanged(endEffectorLabelLeft, String.format("Tọa độ kẹp (L): (%.1f,  %.1f,  %.1f)", eeLeft[0], eeLeft[1], eeLeft[2]));

        if (manualMode) {
            isUpdatingFromFK = true; // Lock IK

            // FK -> UI Right
            slXRight.removeChangeListener(this);
            slYRight.removeChangeListener(this);
            slZRight.removeChangeListener(this);

            slXRight.setValue((int) Math.round(eeRight[0]));
            slYRight.setValue((int) Math.round(eeRight[1]));
            slZRight.setValue((int) Math.round(eeRight[2]));

            txXRight.setText(String.format("%.1f", eeRight[0]));
            txYRight.setText(String.format("%.1f", eeRight[1]));
            txZRight.setText(String.format("%.1f", eeRight[2]));

            slXRight.addChangeListener(this);
            slYRight.addChangeListener(this);
            slZRight.addChangeListener(this);

            // FK -> UI Left
            slXLeft.removeChangeListener(this);
            slYLeft.removeChangeListener(this);
            slZLeft.removeChangeListener(this);

            slXLeft.setValue((int) Math.round(eeLeft[0]));
            slYLeft.setValue((int) Math.round(eeLeft[1]));
            slZLeft.setValue((int) Math.round(eeLeft[2]));

            txXLeft.setText(String.format("%.1f", eeLeft[0]));
            txYLeft.setText(String.format("%.1f", eeLeft[1]));
            txZLeft.setText(String.format("%.1f", eeLeft[2]));

            slXLeft.addChangeListener(this);
            slYLeft.addChangeListener(this);
            slZLeft.addChangeListener(this);

            isUpdatingFromFK = false; // Unlock IK
        }

        armPanel.repaint();
    }

    private void setLabelIfChanged(JLabel label, String text) {
        if (!text.equals(label.getText())) {
            label.setText(text);
        }
    }

    void resetAngles() {
        armPanel.trail.clear();
        double[] defaultPoseRight = { 0, 0, 10.0, -30.0, 0, 0 };
        double[] defaultPoseLeft = { 0, 0, -10.0, 30.0, 0, 0 };
        setTargetAnglesRight(defaultPoseRight);
        setTargetAnglesLeft(defaultPoseLeft);
        syncGuiCoordsFromFK();
        if (showWorkspaceSlice) {
            updateWorkspaceSlice();
        }
    }

    public void syncGuiCoordsFromFK() {
        double[] eeR = armPanel.getRightEndEffectorPosition();
        txXRight.setText(String.format("%.1f", eeR[0]));
        txYRight.setText(String.format("%.1f", eeR[1]));
        txZRight.setText(String.format("%.1f", eeR[2]));
        
        slXRight.removeChangeListener(this);
        slYRight.removeChangeListener(this);
        slZRight.removeChangeListener(this);
        slXRight.setValue((int) Math.round(eeR[0]));
        slYRight.setValue((int) Math.round(eeR[1]));
        slZRight.setValue((int) Math.round(eeR[2]));
        slXRight.addChangeListener(this);
        slYRight.addChangeListener(this);
        slZRight.addChangeListener(this);

        double[] eeL = armPanel.getLeftEndEffectorPosition();
        txXLeft.setText(String.format("%.1f", eeL[0]));
        txYLeft.setText(String.format("%.1f", eeL[1]));
        txZLeft.setText(String.format("%.1f", eeL[2]));
        
        slXLeft.removeChangeListener(this);
        slYLeft.removeChangeListener(this);
        slZLeft.removeChangeListener(this);
        slXLeft.setValue((int) Math.round(eeL[0]));
        slYLeft.setValue((int) Math.round(eeL[1]));
        slZLeft.setValue((int) Math.round(eeL[2]));
        slXLeft.addChangeListener(this);
        slYLeft.addChangeListener(this);
        slZLeft.addChangeListener(this);
    }

    // Trajectory logic methods follow ...



    private void executeTrajectory(java.util.List<double[]> path, String statusTitle) {
        if (path == null || path.isEmpty()) {
            JOptionPane.showMessageDialog(this, "Không có tọa độ quỹ đạo để chạy!", "Lỗi", JOptionPane.ERROR_MESSAGE);
            return;
        }

        final boolean isRight = isRightArmSelected;
        final double[] armAngles = isRight ? anglesRight : anglesLeft;
        final double[] armTargetAngles = isRight ? targetAnglesRight : targetAnglesLeft;
        final JSlider[] armSliders = isRight ? slidersRight : slidersLeft;
        final JLabel[] armAngleLbls = isRight ? angleLblsRight : angleLblsLeft;

        if (!trajectoryNeedsRecalculation && lastJointTrajectory != null && !lastJointTrajectory.isEmpty()) {
            setGotoStatus("Chạy trực tiếp quỹ đạo đã lưu...", new Color(0, 140, 0));
            runPlayback(lastJointTrajectory, statusTitle, isRight, armAngles, armTargetAngles, armSliders, armAngleLbls);
            return;
        }

        setTitle("Đang chuẩn bị dữ liệu...");
        setGotoStatus("Đang tính toán động học ngược (IK) cho quỹ đạo...", new Color(0, 90, 180));
        
        if (speedSlider.getValue() <= 0) {
            speedSlider.setValue(30);
            speedLabel.setText("30 °/s");
            setGotoStatus("Tốc độ đang là 0, tự tăng lên 30 °/s để chạy quỹ đạo", new Color(180, 110, 0));
        }

        armPanel.trail.clear();

        final JComboBox<String> armConfigCombo = isRight ? configComboRight : configComboLeft;
        String cfg = armConfigCombo.getSelectedIndex() == 0 ? "+" : "-";
        trajectoryLockedCfg = cfg;
        trajectoryLastQ = null;
        trajectoryLastAlpha = getInitialTrajectoryAlpha(isRight);
        trajectoryLastYawOffset = 0.0;

        // Tạm dừng motionTimer để tránh xung đột
        if (motionTimer != null) motionTimer.stop();

        // Resample the path
        double speed = Math.max(1.0, speedSlider.getValue() / 2.0); // units per sec
        java.util.List<double[]> resampled = resamplePath(path, speed);
        if (resampled.isEmpty()) {
            resampled = path;
        }
        final java.util.List<double[]> finalPath = resampled;

        // Dùng SwingWorker để tính toán trước (Precompute Trajectory)
        SwingWorker<java.util.List<double[]>, String> precomputeWorker = new SwingWorker<java.util.List<double[]>, String>() {
            @Override
            protected java.util.List<double[]> doInBackground() throws Exception {
                java.util.List<double[]> rawJointTrajectory;
                activeWorkspaceMapForTrajectory = loadWorkspaceFallbackMap();
                
                if (useDescartesIK) {
                    publish("Đang phân tích đồ thị toàn cục (Descartes ROS-Industrial)...");
                    rawJointTrajectory = planDescartesTrajectory(finalPath, isRight);
                } else {
                    // BƯỚC 1: FORWARD PASS (QUÉT GIẢI VÀ ĐÁNH DẤU LỖ HỔNG) TUẦN TỰ (Greedy Warm-Start)
                    rawJointTrajectory = new java.util.ArrayList<>();
                    lastCartesianPath = finalPath;
                    lastCandidatesPerStep = new java.util.ArrayList<>();
                    lastSelectedIndices = new int[finalPath.size()];
                    java.util.Arrays.fill(lastSelectedIndices, -1);

                    for (int i = 0; i < finalPath.size(); i++) {
                        double[] pt = finalPath.get(i);
                        double[] result = solveIKForTrajectoryPoint(pt[0], pt[1], pt[2], true);
                        
                        java.util.List<double[]> stepCandidates = new java.util.ArrayList<>();
                        
                        if (result != null) {
                            stepCandidates.add(result.clone());
                            lastSelectedIndices[i] = 0;
                            lastCandidatesPerStep.add(stepCandidates);

                            // Cập nhật Warm Start liên tục bằng kết quả giải IK (bao gồm cả relaxed/rescue)
                            // để tránh sụp đổ dây chuyền (Cascade Null Failure)
                            trajectoryLastQ = result.clone();
                            
                            double posErr = computePositionError(result, pt[0], pt[1], pt[2], isRight);
                            if (posErr <= TRAJ_STRICT_ERROR) {
                                rawJointTrajectory.add(result.clone());
                                
                                if (DEBUG) {
                                    System.out.printf("[DEBUG_TRAJ] Point %d Target=[%.2f, %.2f, %.2f] SolvedQ=[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f] posErr=%.4f mm\n",
                                        i + 1, pt[0], pt[1], pt[2], result[0], result[1], result[2], result[3], result[4], result[5], posErr * 10.0);
                                }
                                if (i % 5 == 0 || i == finalPath.size() - 1) {
                                    publish(String.format("Đang giải IK: %d / %d điểm (OK)", i + 1, finalPath.size()));
                                }
                            } else {
                                // Nghiệm vượt quá sai số cho phép, đánh dấu lỗ hổng để vá bằng Cubic Spline
                                rawJointTrajectory.add(null);
                                lastSelectedIndices[i] = -1; // reset to invalid if error is too large
                                if (DEBUG) {
                                    System.out.printf("[DEBUG_TRAJ] Point %d Target=[%.2f, %.2f, %.2f] posErr=%.4f mm vượt quá TRAJ_STRICT_ERROR, đánh dấu lỗ hổng!\n",
                                        i + 1, pt[0], pt[1], pt[2], posErr * 10.0);
                                }
                                publish(String.format("Cảnh báo: Điểm %d vượt biên (%.1f mm), đánh dấu lỗ hổng!", i + 1, posErr * 10.0));
                            }
                        } else {
                            // Thử nghiệm nhánh khác (KHÔNG áp hasHugeJump ở bước tìm kiếm này)
                            double savedAlpha = trajectoryLastAlpha;
                            String savedCfg = trajectoryLockedCfg;
                            double savedYawOffset = trajectoryLastYawOffset;

                            double[] jumpResult = solveIKForTrajectoryPoint(pt[0], pt[1], pt[2], false);
                            if (jumpResult != null && trajectoryLastQ != null) {
                                // Tính toán bước nhảy lớn nhất giữa các khớp
                                double maxJointJump = 0.0;
                                for (int j = 0; j < NUM_JOINTS; j++) {
                                    double diff = Math.abs(wrappedDegDiff(jumpResult[j], trajectoryLastQ[j]));
                                    if (diff > maxJointJump) {
                                        maxJointJump = diff;
                                    }
                                }

                                java.util.List<double[]> transitionPoints = generateBranchTransition(trajectoryLastQ, jumpResult, isRight, i + 1);

                                if (transitionPoints != null) {
                                    publish(String.format("Chuyển nhánh tại điểm %d: chèn %d điểm thuần khớp (Cảnh báo: Lệch đường vẽ, khuyên dùng Pen-Up)", i + 1, transitionPoints.size()));
                                    
                                    stepCandidates.add(jumpResult.clone());
                                    lastSelectedIndices[i] = 0;
                                    lastCandidatesPerStep.add(stepCandidates);

                                    // Chèn tất cả các điểm quá độ vào quỹ đạo
                                    rawJointTrajectory.addAll(transitionPoints);
                                    trajectoryLastQ = jumpResult.clone();
                                    continue; // Bỏ qua phần ghi nhận lỗi, tiếp tục vòng lặp sang điểm tiếp theo
                                }
                            }

                            // Nếu không thể chuyển nhánh hoặc chuyển nhánh không hợp lệ, phục hồi trạng thái cũ và coi như FAILED
                            trajectoryLastAlpha = savedAlpha;
                            trajectoryLockedCfg = savedCfg;
                            trajectoryLastYawOffset = savedYawOffset;

                            // IK thất bại hoàn toàn, đánh dấu lỗ hổng, giữ nguyên trajectoryLastQ cũ làm mốc gần nhất
                            lastCandidatesPerStep.add(stepCandidates); // empty list
                            rawJointTrajectory.add(null);
                            if (DEBUG) {
                                System.out.printf("[DEBUG_TRAJ] Point %d FAILED! Target=[%.2f, %.2f, %.2f], đánh dấu lỗ hổng!\n", i + 1, pt[0], pt[1], pt[2]);
                            }
                            publish(String.format("Cảnh báo: Điểm %d bị kẹt (Ngoài vùng), đánh dấu lỗ hổng!", i + 1));
                        }
                    }
                } // End of else block (Sequential Warm-Start)

                int[] workspaceReplacementCount = { 0 };
                rawJointTrajectory = applyWorkspaceMapFallback(rawJointTrajectory, finalPath, isRight, activeWorkspaceMapForTrajectory, workspaceReplacementCount);
                if (workspaceReplacementCount[0] > 0) {
                    publish(String.format("Da thay %d diem loi bang workspace CSV.", workspaceReplacementCount[0]));
                }

                // THÊM BỘ KHỐNG CHẾ KHOẢNG CÁCH VÁ (GAP LIMITER)
                int consecutiveNulls = 0;
                int maxConsecutiveNulls = 0;
                for (int i = 0; i < rawJointTrajectory.size(); i++) {
                    if (rawJointTrajectory.get(i) == null) {
                        consecutiveNulls++;
                        if (consecutiveNulls > maxConsecutiveNulls) {
                            maxConsecutiveNulls = consecutiveNulls;
                        }
                        if (consecutiveNulls > 15) {
                            publish("Lỗi: Quỹ đạo vượt ngoài tầm với (Trajectory Out of Reach)!");
                            if (DEBUG) {
                                System.out.println("[DEBUG_TRAJ] FAILED: Consecutive nulls > 15 at point " + (i + 1));
                            }
                            return new java.util.ArrayList<>();
                        }
                    } else {
                        consecutiveNulls = 0;
                    }
                }
                if (maxConsecutiveNulls > 3 && DEBUG) {
                    System.out.printf("[DEBUG_TRAJ] WARNING: Max gap = %d points (will be interpolated by cubic spline)\n", maxConsecutiveNulls);
                }

                // BƯỚC 2a: VÁ LỖ HỔNG BẰNG IK CARTESIAN (bám sát đường vẽ hơn joint-space interpolation)
                publish("Đang vá lỗ hổng bằng IK Cartesian...");
                rawJointTrajectory = refillGapsCartesian(rawJointTrajectory, finalPath);

                // BƯỚC 2b: VÁ LỖ HỔNG CÒN LẠI VÀ LẬP KẾ HOẠCH QUỸ ĐẠO BẰNG QUINTIC SPLINE
                publish("Đang tối ưu quỹ đạo mượt mà C2 bằng Quintic Spline...");
                double maxVelocity = 180.0;
                double maxAcceleration = 360.0;
                double dtSec = (double) MOTION_DT_MS / 1000.0;

                // Điền đầy các điểm neo trống còn lại bằng các điểm neo gần nhất hoặc nội suy cục bộ tương thích
                java.util.List<double[]> filledJoints = kinematics.TrajectoryPlanner.fillGaps(rawJointTrajectory, armAngles);

                // Lập kế hoạch quỹ đạo không gian khớp qua Quintic Spline & co giãn thời gian (retiming loop)
                java.util.List<double[]> jointTrajectory = kinematics.TrajectoryPlanner.planTrajectory(
                        filledJoints,
                        maxVelocity,
                        maxAcceleration,
                        useGlobalC2,
                        dtSec
                );

                return jointTrajectory;
            }

            @Override
            protected void process(java.util.List<String> chunks) {
                if (!chunks.isEmpty()) {
                    String latest = chunks.get(chunks.size() - 1);
                    Color color = new Color(0, 90, 180);
                    if (latest.startsWith("Cảnh báo")) {
                        color = new Color(180, 110, 0);
                    } else if (latest.startsWith("Lỗi")) {
                        color = Color.RED;
                    }
                    setGotoStatus(latest, color);
                }
            }

            @Override
            protected void done() {
                try {
                    java.util.List<double[]> jointTrajectory = get();
                    if (jointTrajectory.isEmpty()) {
                        String currentTxt = isRight ? gotoStatusRight.getText() : gotoStatusLeft.getText();
                        if (!currentTxt.startsWith("Lỗi")) {
                            setGotoStatus("Không thể tính toán quỹ đạo (Ngoài vùng làm việc)!", Color.RED);
                        }
                        setTitle("Mô Phỏng Cánh Tay Robot 6-DOF");
                        startMotionTimer();
                        return;
                    }

                    lastJointTrajectory = jointTrajectory;
                    trajectoryNeedsRecalculation = false;

                    SwingUtilities.invokeLater(() -> {
                        TrajectoryAnalyticsWindow analytics = new TrajectoryAnalyticsWindow(
                            lastJointTrajectory,
                            lastCartesianPath,
                            lastCandidatesPerStep,
                            lastSelectedIndices,
                            isRight
                        );
                        analytics.setVisible(true);
                    });

                    runPlayback(jointTrajectory, statusTitle, isRight, armAngles, armTargetAngles, armSliders, armAngleLbls);

                } catch (Exception ex) {
                    ex.printStackTrace();
                    setGotoStatus("Lỗi tính toán quỹ đạo!", Color.RED);
                    startMotionTimer();
                }
            }
        };

        precomputeWorker.execute();
    }

    private void runPlayback(final java.util.List<double[]> jointTrajectory, final String statusTitle, final boolean isRight, final double[] armAngles, final double[] armTargetAngles, final JSlider[] armSliders, final JLabel[] armAngleLbls) {
        armPanel.trail.clear();
        showTrailCb.setSelected(true);
        armPanel.repaint();

        if (motionTimer != null) motionTimer.stop();
        if (trajectoryTimer != null) trajectoryTimer.stop();

        // Nhảy đến điểm xuất phát
        double[] startResult = jointTrajectory.get(0);
        if (isRight) {
            setTargetAnglesRight(startResult);
        } else {
            setTargetAnglesLeft(startResult);
        }
        updateArm();

        setGotoStatus("Tính toán xong. Đang di chuyển đến điểm bắt đầu...", new Color(0, 90, 180));

        final int[] waitMs = { 0 };
        Timer prepareTimer = new Timer(50, evt -> {
            waitMs[0] += 50;
            boolean arrived = true;
            for (int i = 0; i < NUM_JOINTS; i++) {
                if (Math.abs(armAngles[i] - armTargetAngles[i]) > 0.5) {
                    arrived = false;
                }
            }
            if (arrived || waitMs[0] >= 5000) {
                ((Timer) evt.getSource()).stop();
                if (!arrived) {
                    setGotoStatus("Hết thời gian chờ điểm đầu, chạy quỹ đạo từ vị trí hiện tại", new Color(180, 110, 0));
                }
                setTitle("Chờ 1 giây...");

                Timer delayTimer = new Timer(1000, evt2 -> {
                    ((Timer) evt2.getSource()).stop();
                    setTitle(statusTitle + " đang chạy...");
                    // showTrailCb state is preserved

                    final int[] currentIndex = { 0 };
                    trajectoryTimer = new Timer(MOTION_DT_MS, e -> {
                        if (currentIndex[0] >= jointTrajectory.size()) {
                            trajectoryTimer.stop();
                            setTitle("Mô Phỏng Cánh Tay Robot 6-DOF");
                            setGotoStatus("Quỹ đạo hoàn thành!", new Color(0, 140, 0));
                            startMotionTimer();
                            return;
                        }

                        double[] q_target = jointTrajectory.get(currentIndex[0]);
                        currentIndex[0]++;

                        for (int i = 0; i < NUM_JOINTS; i++) {
                            armTargetAngles[i] = armAngles[i] = q_target[i];
                            armSliders[i].removeChangeListener(MainFrame.this);
                            armSliders[i].setValue((int) Math.round(armAngles[i]));
                            armSliders[i].addChangeListener(MainFrame.this);
                            armAngleLbls[i].setText((int) Math.round(armAngles[i]) + "°");
                        }
                        updateArm();
                        sendJointsToUart();
                    });
                    trajectoryTimer.start();
                });
                delayTimer.start();
            }
        });
        prepareTimer.start();
    }

    void runCustomPathTrajectory(java.util.List<double[]> path) {
        executeTrajectory(path, "Quỹ đạo tùy chỉnh");
    }

    private java.util.List<double[]> resamplePath(java.util.List<double[]> path, double speedUnitsPerSec) {
        java.util.List<double[]> result = new java.util.ArrayList<>();
        if (path.size() < 2) return path;

        double dt = MOTION_DT_MS / 1000.0;
        double stepLen = speedUnitsPerSec * dt;
        if (stepLen < 1.2) stepLen = 1.2; // Force minimum 1.2 cm (12mm) spacing between anchors to prevent drift and heavy solver load

        double[] lastPt = path.get(0);
        result.add(lastPt.clone());

        double remainingDist = 0;

        for (int i = 1; i < path.size(); i++) {
            double[] pA = path.get(i - 1);
            double[] pB = path.get(i);

            double segmentLen = Math.sqrt(Math.pow(pB[0] - pA[0], 2) + Math.pow(pB[1] - pA[1], 2) + Math.pow(pB[2] - pA[2], 2));
            if (segmentLen < 1e-4) continue;

            double currentDist = remainingDist;
            while (currentDist <= segmentLen) {
                double t = currentDist / segmentLen;
                double[] p = {
                    pA[0] + (pB[0] - pA[0]) * t,
                    pA[1] + (pB[1] - pA[1]) * t,
                    pA[2] + (pB[2] - pA[2]) * t
                };
                result.add(p);
                currentDist += stepLen;
            }
            remainingDist = currentDist - segmentLen;
        }

        // Ensure last point is added if not already
        double[] lastTarget = path.get(path.size() - 1);
        double[] lastAdded = result.get(result.size() - 1);
        double finalDist = Math.sqrt(Math.pow(lastTarget[0]-lastAdded[0],2) + Math.pow(lastTarget[1]-lastAdded[1],2) + Math.pow(lastTarget[2]-lastAdded[2],2));
        if (finalDist > 0.01) {
            result.add(lastTarget.clone());
        }

        // --- Lọc là phẳng quỹ đạo (Moving Average Smoothing) ---
        // Giúp các điểm vẽ bằng chuột không bị gấp khúc, loại bỏ hiện tượng giật cục
        int windowSize = 15;
        if (result.size() > windowSize) {
            java.util.List<double[]> smoothed = new java.util.ArrayList<>();
            for (int i = 0; i < result.size(); i++) {
                int start = Math.max(0, i - windowSize / 2);
                int end = Math.min(result.size() - 1, i + windowSize / 2);
                double sumX = 0, sumY = 0, sumZ = 0;
                int count = end - start + 1;
                for (int j = start; j <= end; j++) {
                    sumX += result.get(j)[0];
                    sumY += result.get(j)[1];
                    sumZ += result.get(j)[2];
                }
                smoothed.add(new double[] { sumX / count, sumY / count, sumZ / count });
            }
            // Giữ nguyên điểm đầu và cuối để không bị lệch đích
            smoothed.set(0, result.get(0));
            smoothed.set(smoothed.size() - 1, result.get(result.size() - 1));
            return smoothed;
        }

        return result;
    }

    void runSpiralTrajectoryParam(double cx, double cy, double cz, double R, double H, double K) {
        java.util.List<double[]> path = new java.util.ArrayList<>();
        int numPoints = (int) (100 * K);
        if (numPoints < 100) numPoints = 100;
        
        for (int i = 0; i <= numPoints; i++) {
            double r = i / (double) numPoints;
            double tx = cx + R * Math.cos(K * 2 * Math.PI * r);
            double ty = cy + R * Math.sin(K * 2 * Math.PI * r);
            double tz = cz + H * r;
            path.add(new double[]{tx, ty, tz});
        }
        executeTrajectory(path, "Quỹ đạo xoắn ốc");
    }

    private double wrappedDegDiff(double a, double b) {
        double d = a - b;
        while (d > 180)
            d -= 360;
        while (d < -180)
            d += 360;
        return d;
    }

    private double continuityCost(double[] q, double[] qPrev, boolean enforceVelocityLimit) {
        double s = 0.0;
        double maxDeltaDeg = 15.0; // ~0.26 rad per step safety limit
        for (int i = 0; i < NUM_JOINTS; i++) {
            double d = wrappedDegDiff(q[i], qPrev[i]);
            if (enforceVelocityLimit && Math.abs(d) > maxDeltaDeg) {
                s += 100000.0; // Heavy penalty for velocity violation (Configuration Flipping)
            }
            s += d * d;
        }
        return s;
    }

    private double getInitialTrajectoryAlpha(boolean isRight) {
        JComboBox<String> gCombo = isRight ? gripperModeComboRight : gripperModeComboLeft;
        if (gCombo.getSelectedIndex() == 0) {
            return 0.0;
        } else {
            return isRight ? activeAlphaRight : activeAlphaLeft;
        }
    }

    private double calculateEdgeCost(double[] qA, double[] qB) {
        double cost = 0.0;
        double maxJump = 0.0;
        for (int i = 0; i < 6; i++) {
            double d = Math.abs(wrappedDegDiff(qA[i], qB[i]));
            cost += d * d; // Penalize large individual joint moves
            if (d > maxJump) maxJump = d;
        }
        if (maxJump > 30.0) {
            cost += 100000.0; // Heavy penalty for jumping branches, but allows it if no other path exists
        }
        return cost;
    }

    private static class DescartesNode {
        double[] q;
        double alphaDeg;
        double yawOffsetDeg;
        public DescartesNode(double[] q, double alpha, double yaw) {
            this.q = q;
            this.alphaDeg = alpha;
            this.yawOffsetDeg = yaw;
        }
    }

    private static class WorkspaceScanSolution {
        double[] q;
        double alphaDeg;
        double yawOffsetDeg;
        int reachClass;
        double posError;

        WorkspaceScanSolution(double[] q, double alphaDeg, double yawOffsetDeg, int reachClass, double posError) {
            this.q = q;
            this.alphaDeg = alphaDeg;
            this.yawOffsetDeg = yawOffsetDeg;
            this.reachClass = reachClass;
            this.posError = posError;
        }
    }

    private java.util.List<DescartesNode> generateAllValidIK(double px, double py, double pz, boolean isRight) {
        java.util.List<DescartesNode> allSolutions = new java.util.ArrayList<>();
        double activeAlpha = isRight ? activeAlphaRight : activeAlphaLeft;
        
        JComboBox<String> gCombo = isRight ? gripperModeComboRight : gripperModeComboLeft;
        boolean fixedGround = (gCombo.getSelectedIndex() == 0);
        
        // Coarse grid for alpha and yawOffset to keep graph size manageable
        double[] alphaGrid = fixedGround ? new double[]{-90, -75, -60, -45, -30, -15, 0, 15, 30} 
                                         : new double[]{activeAlpha - 15, activeAlpha, activeAlpha + 15};
        
        double[] activeA = isRight ? anglesRight : anglesLeft;
        double prefY = getYawOffsetFromQ(activeA, px, py, isRight);

        WorkspaceMap.Entry csvSeed = findWorkspaceSeed(px, py, pz, isRight, activeA);
        if (csvSeed != null) {
            double seedErr = computePositionError(csvSeed.q, px, py, pz, isRight);
            if (seedErr <= TRAJ_STRICT_ERROR) {
                allSolutions.add(new DescartesNode(csvSeed.q.clone(), csvSeed.alphaDeg, csvSeed.yawOffsetDeg));
                return allSolutions;
            }
        }

        for (double alphaDeg : alphaGrid) {
            java.util.List<double[]> sols = tryAlpha(px, py, pz, alphaDeg, isRight, activeA, prefY, false);
            for (double[] q : sols) {
                // Ensure unique solutions roughly
                boolean isDuplicate = false;
                for (DescartesNode existing : allSolutions) {
                    double diff = 0;
                    for (int j = 0; j < 6; j++) {
                        diff += Math.abs(wrappedDegDiff(q[j], existing.q[j]));
                    }
                    if (diff < 1.0) {
                        isDuplicate = true;
                        break;
                    }
                }
                if (!isDuplicate && isWithinLimits(q, isRight)) {
                    double posErr = computePositionError(q, px, py, pz, isRight);
                    if (posErr <= TRAJ_STRICT_ERROR) {
                        // Estimate yaw offset
                        double q1_base = isRight ? Math.atan2(py, px) : -Math.atan2(py, -px);
                        double yawOffset = Math.toDegrees(q[0] - q1_base);
                        while(yawOffset > 180) yawOffset -= 360;
                        while(yawOffset < -180) yawOffset += 360;
                        
                        allSolutions.add(new DescartesNode(q, alphaDeg, yawOffset));
                    }
                }
            }
        }
        return allSolutions;
    }

    private java.util.List<double[]> planDescartesTrajectory(java.util.List<double[]> path, boolean isRight) throws Exception {
        int N = path.size();
        java.util.List<java.util.List<DescartesNode>> layers = new java.util.ArrayList<>();
        java.util.List<Integer> validIndices = new java.util.ArrayList<>();
        
        java.util.List<java.util.List<double[]>> candidatesPerStep = new java.util.ArrayList<>();
        int[] selectedIndices = new int[N];
        java.util.Arrays.fill(selectedIndices, -1);

        // Step 1: Generate graph layers (only for reachable points)
        for (int i = 0; i < N; i++) {
            double[] pt = path.get(i);
            java.util.List<DescartesNode> sols = generateAllValidIK(pt[0], pt[1], pt[2], isRight);
            java.util.List<double[]> stepCandidates = new java.util.ArrayList<>();
            for (DescartesNode n : sols) {
                stepCandidates.add(n.q.clone());
            }
            candidatesPerStep.add(stepCandidates);

            if (!sols.isEmpty()) {
                layers.add(sols);
                validIndices.add(i);
            }
        }
        
        if (layers.isEmpty()) {
            this.lastCartesianPath = path;
            this.lastCandidatesPerStep = candidatesPerStep;
            this.lastSelectedIndices = selectedIndices;
            // No reachable points in the entire path
            java.util.List<double[]> finalPath = new java.util.ArrayList<>();
            for(int i = 0; i < N; i++) finalPath.add(null);
            return finalPath;
        }
        
        int numLayers = layers.size();
        
        // Step 2: Dynamic Programming (Viterbi) shortest path
        double[][] dp = new double[numLayers][];
        int[][] parent = new int[numLayers][];
        
        dp[0] = new double[layers.get(0).size()];
        parent[0] = new int[layers.get(0).size()];
        
        double[] currentAngles = isRight ? anglesRight : anglesLeft;
        for (int k = 0; k < layers.get(0).size(); k++) {
            dp[0][k] = calculateEdgeCost(currentAngles, layers.get(0).get(k).q);
            parent[0][k] = -1;
        }
        
        for (int i = 1; i < numLayers; i++) {
            java.util.List<DescartesNode> prevLayer = layers.get(i-1);
            java.util.List<DescartesNode> currLayer = layers.get(i);
            dp[i] = new double[currLayer.size()];
            parent[i] = new int[currLayer.size()];
            
            for (int currNode = 0; currNode < currLayer.size(); currNode++) {
                double minCost = Double.POSITIVE_INFINITY;
                int bestPrevNode = -1;
                double[] qCurr = currLayer.get(currNode).q;
                
                for (int prevNode = 0; prevNode < prevLayer.size(); prevNode++) {
                    double[] qPrev = prevLayer.get(prevNode).q;
                    double edgeCost = calculateEdgeCost(qPrev, qCurr);
                    double totalCost = dp[i-1][prevNode] + edgeCost;
                    if (totalCost < minCost) {
                        minCost = totalCost;
                        bestPrevNode = prevNode;
                    }
                }
                dp[i][currNode] = minCost;
                parent[i][currNode] = bestPrevNode;
            }
        }
        
        // Step 3: Backtrack
        int bestLastNode = 0;
        double minFinalCost = Double.POSITIVE_INFINITY;
        for (int k = 0; k < layers.get(numLayers-1).size(); k++) {
            if (dp[numLayers-1][k] < minFinalCost) {
                minFinalCost = dp[numLayers-1][k];
                bestLastNode = k;
            }
        }
        
        java.util.List<DescartesNode> optimalValidNodes = new java.util.ArrayList<>();
        int currNode = bestLastNode;
        for (int i = numLayers - 1; i >= 0; i--) {
            DescartesNode selectedNodeForLayer = layers.get(i).get(currNode);
            optimalValidNodes.add(0, selectedNodeForLayer);
            
            int originalStepIdx = validIndices.get(i);
            java.util.List<double[]> stepCandidates = candidatesPerStep.get(originalStepIdx);
            for (int k = 0; k < stepCandidates.size(); k++) {
                boolean match = true;
                for (int joint = 0; joint < 6; joint++) {
                    if (Math.abs(stepCandidates.get(k)[joint] - selectedNodeForLayer.q[joint]) > 1e-5) {
                        match = false;
                        break;
                    }
                }
                if (match) {
                    selectedIndices[originalStepIdx] = k;
                    break;
                }
            }

            currNode = parent[i][currNode];
        }
        
        // Save terminal state to global UI variables
        if (!optimalValidNodes.isEmpty()) {
            DescartesNode lastNode = optimalValidNodes.get(optimalValidNodes.size() - 1);
            trajectoryLastQ = lastNode.q.clone();
            trajectoryLastAlpha = lastNode.alphaDeg;
            trajectoryLastYawOffset = lastNode.yawOffsetDeg;
            trajectoryLockedCfg = lastNode.q[3] > 0 ? "+" : "-"; 
        }
        
        this.lastCartesianPath = path;
        this.lastCandidatesPerStep = candidatesPerStep;
        this.lastSelectedIndices = selectedIndices;
        
        // Step 4: Construct final path with missing 'null' gaps and branch transition nodes
        java.util.List<double[]> finalPath = new java.util.ArrayList<>();
        int validPtr = 0;
        double[] qRef = null;
        
        for (int i = 0; i < N; i++) {
            if (validPtr < validIndices.size() && validIndices.get(validPtr) == i) {
                double[] qNext = optimalValidNodes.get(validPtr).q;
                
                if (qRef != null) {
                    java.util.List<double[]> transitionPoints = generateBranchTransition(qRef, qNext, isRight, i);
                    if (transitionPoints != null && !transitionPoints.isEmpty()) {
                        finalPath.addAll(transitionPoints);
                    }
                }
                
                finalPath.add(qNext);
                qRef = qNext;
                validPtr++;
            } else {
                finalPath.add(null);
            }
        }
        return finalPath;
    }

    private WorkspaceMap loadWorkspaceFallbackMap() {
        try {
            WorkspaceMap map = WorkspaceMap.loadDefault();
            return map.size() == 0 ? null : map;
        } catch (java.io.IOException ex) {
            if (DEBUG) {
                System.err.println("[WORKSPACE_FALLBACK] Could not load workspace_points.csv: " + ex.getMessage());
            }
            return null;
        }
    }

    private String getGripperModeName(boolean isRight) {
        return (isRight ? gripperModeComboRight : gripperModeComboLeft).getSelectedIndex() == 0
                ? "FIXED_GROUND" : "FREE";
    }

    private WorkspaceMap.Entry findWorkspaceSeed(double px, double py, double pz, boolean isRight, double[] qRef) {
        if (activeWorkspaceMapForTrajectory == null) {
            return null;
        }
        return activeWorkspaceMapForTrajectory.findBestReplacement(
                px,
                py,
                pz,
                isRight,
                getGripperModeName(isRight),
                qRef,
                WORKSPACE_SEED_MAX_DISTANCE,
                WORKSPACE_FALLBACK_MAX_JOINT_JUMP);
    }

    private java.util.List<double[]> applyWorkspaceMapFallback(
            java.util.List<double[]> rawJointTraj,
            java.util.List<double[]> cartesianPath,
            boolean isRight,
            WorkspaceMap workspaceMap,
            int[] replacementCount) {
        if (workspaceMap == null || rawJointTraj == null || cartesianPath == null) {
            return rawJointTraj;
        }

        java.util.List<double[]> result = new java.util.ArrayList<>(rawJointTraj);
        String gripperMode = getGripperModeName(isRight);
        double[] defaultQ = isRight ? anglesRight : anglesLeft;

        for (int i = 0; i < result.size() && i < cartesianPath.size(); i++) {
            if (result.get(i) != null) {
                continue;
            }

            double[] qRef = defaultQ;
            for (int k = i - 1; k >= 0; k--) {
                if (result.get(k) != null) {
                    qRef = result.get(k);
                    break;
                }
            }

            double[] pt = cartesianPath.get(i);
            WorkspaceMap.Entry entry = workspaceMap.findBestReplacement(
                    pt[0],
                    pt[1],
                    pt[2],
                    isRight,
                    gripperMode,
                    qRef,
                    WORKSPACE_FALLBACK_MAX_DISTANCE,
                    WORKSPACE_FALLBACK_MAX_JOINT_JUMP);

            if (entry != null) {
                double[] qReplacement = entry.q.clone();
                result.set(i, qReplacement);
                if (replacementCount != null && replacementCount.length > 0) {
                    replacementCount[0]++;
                }

                if (lastCandidatesPerStep != null && i < lastCandidatesPerStep.size()) {
                    java.util.List<double[]> candidates = lastCandidatesPerStep.get(i);
                    if (candidates != null) {
                        candidates.add(qReplacement.clone());
                        if (lastSelectedIndices != null && i < lastSelectedIndices.length) {
                            lastSelectedIndices[i] = candidates.size() - 1;
                        }
                    }
                }

                if (DEBUG) {
                    double dist = entry.distanceTo(pt[0], pt[1], pt[2]);
                    System.out.printf("[WORKSPACE_FALLBACK] Replaced waypoint %d using CSV point dist=%.3f class=%d%n",
                            i + 1, dist, entry.reachClass);
                }
            }
        }

        return result;
    }

    /**
     * Vá null gaps bằng cách re-solve IK cho các điểm Cartesian nội suy tuyến tính.
     * Giúp EE bám sát đường vẽ gốc thay vì đi đường cong theo joint-space interpolation.
     */
    private java.util.List<double[]> refillGapsCartesian(
            java.util.List<double[]> rawJointTraj,
            java.util.List<double[]> cartesianPath) {

        boolean isRight = isRightArmSelected;
        int n = rawJointTraj.size();
        java.util.List<double[]> result = new java.util.ArrayList<>(rawJointTraj);

        int i = 0;
        while (i < n) {
            if (result.get(i) != null) { i++; continue; }

            // Tìm điểm hợp lệ liền trước (prevIdx) và liền sau (nextIdx)
            int prevIdx = -1, nextIdx = -1;
            for (int k = i - 1; k >= 0; k--) {
                if (result.get(k) != null) { prevIdx = k; break; }
            }
            for (int k = i + 1; k < n; k++) {
                if (result.get(k) != null) { nextIdx = k; break; }
            }

            // Không có cả 2 điểm neo → skip
            if (prevIdx < 0 || nextIdx < 0 || nextIdx >= cartesianPath.size()) {
                i++; continue;
            }

            double[] qPrev = result.get(prevIdx);
            double[] ptPrev = cartesianPath.get(Math.min(prevIdx, cartesianPath.size() - 1));
            double[] ptNext = cartesianPath.get(Math.min(nextIdx, cartesianPath.size() - 1));
            int gapLen = nextIdx - prevIdx;

            // Re-solve IK dọc đường thẳng Cartesian từ ptPrev -> ptNext
            double[] qCur = qPrev.clone();
            boolean allOk = true;
            for (int g = prevIdx + 1; g < nextIdx; g++) {
                if (g >= cartesianPath.size()) break;
                double t = (double)(g - prevIdx) / gapLen;
                double[] pt = cartesianPath.get(g);

                // Hướng đích: giữ nguyên alpha/yaw từ điểm gần nhất
                double alpha_rad = Math.toRadians(trajectoryLastAlpha);
                double q1_base = isRight ? Math.atan2(pt[1], pt[0]) : -Math.atan2(pt[1], -pt[0]);
                double yaw = q1_base + Math.toRadians(trajectoryLastYawOffset);
                double ca = Math.cos(Math.PI + alpha_rad), sa = Math.sin(Math.PI + alpha_rad);
                double[][] R_y = { { ca, 0, sa }, { 0, 1, 0 }, { -sa, 0, ca } };
                double cy2 = Math.cos(yaw), sy2 = Math.sin(yaw);
                double[][] R_z, R_target;
                if (isRight) {
                    R_z = new double[][] { { cy2, -sy2, 0 }, { sy2, cy2, 0 }, { 0, 0, 1 } };
                    R_target = multiplyMatrices(R_z, R_y);
                } else {
                    double yawR = -yaw;
                    double cyR = Math.cos(yawR), syR = Math.sin(yawR);
                    double[][] Rz_r = { { cyR, -syR, 0 }, { syR, cyR, 0 }, { 0, 0, 1 } };
                    double[][] Rt_r = multiplyMatrices(Rz_r, R_y);
                    R_target = new double[][] {
                        {  Rt_r[0][0], -Rt_r[0][1], -Rt_r[0][2] },
                        { -Rt_r[1][0],  Rt_r[1][1],  Rt_r[1][2] },
                        { -Rt_r[2][0],  Rt_r[2][1],  Rt_r[2][2] }
                    };
                }

                double[] qInitRad = new double[NUM_JOINTS];
                for (int j = 0; j < NUM_JOINTS; j++) {
                    qInitRad[j] = Math.toRadians(qCur[j]);
                }
                double[] qSol = kinematics.Kinematics.solveIK(pt[0], pt[1], pt[2], R_target, qInitRad, isRight);

                if (qSol != null && isWithinLimits(qSol, isRight)) {
                    double posErr = computePositionError(qSol, pt[0], pt[1], pt[2], isRight);
                    if (posErr <= TRAJ_RELAXED_ERROR) {
                        result.set(g, qSol);
                        qCur = qSol;
                        continue;
                    }
                }
                allOk = false;
                // Giữ null nếu IK thất bại, fillGaps sẽ xử lý sau
            }

            // Bước qua hết gap đã xử lý
            i = nextIdx;
        }
        return result;
    }

    private java.util.List<double[]> generateBranchTransition(double[] qRef, double[] qNext, boolean isRight, int index) {
        double maxJump = 0.0;
        for (int j = 0; j < 6; j++) {
            double diff = Math.abs(wrappedDegDiff(qNext[j], qRef[j]));
            if (diff > maxJump) maxJump = diff;
        }
        
        if (maxJump > 30.0) {
            int M = (int) Math.ceil(maxJump / 15.0);
            if (M < 1) M = 1;
            if (DEBUG) {
                System.out.printf("[DEBUG_TRAJ] Điểm %d: Phát hiện lật cấu hình (Jump=%.1f) -> Chèn %d điểm quá độ thuần khớp\n", index, maxJump, M);
            }
            java.util.List<double[]> transitionPoints = new java.util.ArrayList<>();
            for (int step = 1; step <= M; step++) {
                double factor = (double) step / M;
                double[] interp = new double[6];
                for (int j = 0; j < 6; j++) {
                    double diff = wrappedDegDiff(qNext[j], qRef[j]);
                    interp[j] = qRef[j] + diff * factor;
                    while (interp[j] > 180) interp[j] -= 360;
                    while (interp[j] < -180) interp[j] += 360;
                }
                // Check if interpolated points hit joint limits. For DP, it's safer to clamp if they do, but let's just test limits.
                if (!isWithinLimits(interp, isRight)) {
                    if (DEBUG) System.out.println("[DEBUG_TRAJ] Interpolated point hits limit, returning null.");
                    return null;
                }
                transitionPoints.add(interp);
            }
            return transitionPoints;
        }
        return new java.util.ArrayList<>();
    }

    private double[] solveIKForTrajectoryPoint(double px, double py, double pz, boolean enforceJumpLimit) {
        boolean isFirstWaypoint = (trajectoryLastQ == null);
        double[] currentAngles = isRightArmSelected ? anglesRight : anglesLeft;
        double[] qRef = isFirstWaypoint ? currentAngles : trajectoryLastQ;
        String altCfg = trajectoryLockedCfg.equals("+") ? "-" : "+";
        String[] cfgCandidates = new String[] { trajectoryLockedCfg, altCfg }; // Luôn thử cả 2 cấu hình để bám được quỹ đạo

        double preferredYaw = isFirstWaypoint ? Double.NaN : trajectoryLastYawOffset;
        double[] qSearchRef = qRef;
        WorkspaceMap.Entry csvSeed = findWorkspaceSeed(px, py, pz, isRightArmSelected, qRef);
        if (csvSeed != null) {
            double seedErr = computePositionError(csvSeed.q, px, py, pz, isRightArmSelected);
            boolean seedJumpOk = true;
            if (enforceJumpLimit && !isFirstWaypoint) {
                for (int i = 0; i < NUM_JOINTS; i++) {
                    if (Math.abs(wrappedDegDiff(csvSeed.q[i], qRef[i])) > WORKSPACE_FALLBACK_MAX_JOINT_JUMP) {
                        seedJumpOk = false;
                        break;
                    }
                }
            }
            if (seedErr <= TRAJ_STRICT_ERROR && seedJumpOk) {
                trajectoryLastAlpha = Double.isFinite(csvSeed.alphaDeg) ? csvSeed.alphaDeg : trajectoryLastAlpha;
                trajectoryLockedCfg = getActualConfig(csvSeed.q, isRightArmSelected);
                trajectoryLastYawOffset = Double.isFinite(csvSeed.yawOffsetDeg)
                        ? csvSeed.yawOffsetDeg
                        : getYawOffsetFromQ(csvSeed.q, px, py, isRightArmSelected);
                if (DEBUG) {
                    System.out.printf("[WORKSPACE_SEED] Direct CSV hit target=(%.2f, %.2f, %.2f) err=%.4f%n",
                            px, py, pz, seedErr);
                }
                return csvSeed.q.clone();
            }
            if (seedJumpOk) {
                qSearchRef = csvSeed.q;
                if (Double.isFinite(csvSeed.yawOffsetDeg)) {
                    preferredYaw = csvSeed.yawOffsetDeg;
                }
            }
        }

        if (DEBUG) {
            System.out.printf("[DEBUG_TRAJ_IK] Target: (X=%.2f, Y=%.2f, Z=%.2f) | Arm=%s | ConfigRef=%s | isFirst=%b\n",
                px, py, pz, isRightArmSelected ? "RIGHT" : "LEFT", trajectoryLockedCfg, isFirstWaypoint);
        }

        double bestStrictCost = Double.MAX_VALUE;
        double[] bestStrictQ = null;
        double bestStrictAlpha = trajectoryLastAlpha;
        String bestStrictCfg = trajectoryLockedCfg;

        double bestRelaxedCost = Double.MAX_VALUE;
        double[] bestRelaxedQ = null;
        double bestRelaxedAlpha = trajectoryLastAlpha;
        String bestRelaxedCfg = trajectoryLockedCfg;

        double bestRescueCost = Double.MAX_VALUE;
        double[] bestRescueQ = null;
        double bestRescueAlpha = trajectoryLastAlpha;
        String bestRescueCfg = trajectoryLockedCfg;

        JComboBox<String> gCombo = isRightArmSelected ? gripperModeComboRight : gripperModeComboLeft;
        boolean fixedGround = (gCombo.getSelectedIndex() == 0);

        // 1) Search near previous alpha to avoid branch jumping
        // For fixedGround: scan full range but add alpha² penalty to prefer alpha=0
        double aStart = fixedGround ? -90 : (trajectoryLastAlpha - 18);
        double aEnd = fixedGround ? 30 : (trajectoryLastAlpha + 18);
        double aStep = fixedGround ? 3.0 : 1.0;
        for (double a = aStart; a <= aEnd; a += aStep) {
            List<double[]> candidates = tryAlpha(px, py, pz, a, isRightArmSelected, qSearchRef, preferredYaw, false);
            for (double[] q : candidates) {
                // Reject candidates with a huge joint jump (> 30 degrees) to prevent wild/erratic movements
                boolean hasHugeJump = false;
                if (enforceJumpLimit && !isFirstWaypoint) {
                    for (int i = 0; i < NUM_JOINTS; i++) {
                        if (Math.abs(wrappedDegDiff(q[i], qRef[i])) > 30.0) {
                            hasHugeJump = true;
                            break;
                        }
                    }
                }
                if (hasHugeJump) {
                    continue;
                }
                
                double posErr = computePositionError(q, px, py, pz, isRightArmSelected);
                for (String cfgTry : cfgCandidates) {
                    String actualCfg = getActualConfig(q, isRightArmSelected);
                    if (!actualCfg.equals(cfgTry)) continue;
                    double c = posErr * 220.0 + continuityCost(q, qRef, !isFirstWaypoint) * 0.04;
                    // Heavy alpha penalty for fixedGround mode
                    if (fixedGround) {
                        c += a * a * 50.0;
                    }
                    if (posErr <= TRAJ_STRICT_ERROR && c < bestStrictCost) {
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
                    
                    double rescueC = posErr * 150.0 + continuityCost(q, qRef, !isFirstWaypoint) * 1.5;
                    if (rescueC < bestRescueCost) {
                        bestRescueCost = rescueC;
                        bestRescueQ = q;
                        bestRescueAlpha = a;
                        bestRescueCfg = cfgTry;
                    }
                }
            }
        }

        // 2) Fallback to global scan if local search fails or is inaccurate
        // For fixedGround, step 1 already scanned full range, so only fallback for free mode
        if (bestStrictQ == null && !fixedGround) {
            String[] cfgFallback = cfgCandidates;
            for (double a = -90; a <= 30; a += 1.5) {
                List<double[]> candidates = tryAlpha(px, py, pz, a, isRightArmSelected, qSearchRef, preferredYaw, false);
                for (double[] q : candidates) {
                    // Reject candidates with a huge joint jump (> 30 degrees) to prevent wild/erratic movements
                    boolean hasHugeJump = false;
                    if (enforceJumpLimit && !isFirstWaypoint) {
                        for (int i = 0; i < NUM_JOINTS; i++) {
                            if (Math.abs(wrappedDegDiff(q[i], qRef[i])) > 30.0) {
                                hasHugeJump = true;
                                break;
                            }
                        }
                    }
                    if (hasHugeJump) {
                        continue;
                    }
                    
                    double posErr = computePositionError(q, px, py, pz, isRightArmSelected);
                    for (String cfgTry : cfgFallback) {
                        String actualCfg = getActualConfig(q, isRightArmSelected);
                        if (!actualCfg.equals(cfgTry)) continue;
                        double c = posErr * 220.0 + continuityCost(q, qRef, !isFirstWaypoint) * 0.04;
                        if (posErr <= TRAJ_STRICT_ERROR && c < bestStrictCost) {
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
                        
                        double rescueC = posErr * 150.0 + continuityCost(q, qRef, !isFirstWaypoint) * 1.5;
                        if (rescueC < bestRescueCost) {
                            bestRescueCost = rescueC;
                            bestRescueQ = q;
                            bestRescueAlpha = a;
                            bestRescueCfg = cfgTry;
                        }
                    }
                }
            }
        }

        if (bestStrictQ != null) {
            trajectoryLastAlpha = bestStrictAlpha;
            trajectoryLockedCfg = bestStrictCfg;
            trajectoryLastYawOffset = getYawOffsetFromQ(bestStrictQ, px, py, isRightArmSelected);
            if (DEBUG) {
                System.out.printf("[DEBUG_TRAJ_IK] SUCCESS (Strict) | Config=%s | Alpha=%.1f | YawOffset=%.1f\n", bestStrictCfg, bestStrictAlpha, trajectoryLastYawOffset);
            }
            return bestStrictQ;
        }
        if (bestRelaxedQ != null) {
            trajectoryLastAlpha = bestRelaxedAlpha;
            trajectoryLockedCfg = bestRelaxedCfg;
            trajectoryLastYawOffset = getYawOffsetFromQ(bestRelaxedQ, px, py, isRightArmSelected);
            if (DEBUG) {
                System.out.printf("[DEBUG_TRAJ_IK] SUCCESS (Relaxed) | Config=%s | Alpha=%.1f | YawOffset=%.1f\n", bestRelaxedCfg, bestRelaxedAlpha, trajectoryLastYawOffset);
            }
            return bestRelaxedQ;
        }
        if (bestRescueQ != null) {
            trajectoryLastAlpha = bestRescueAlpha;
            trajectoryLockedCfg = bestRescueCfg;
            trajectoryLastYawOffset = getYawOffsetFromQ(bestRescueQ, px, py, isRightArmSelected);
            if (DEBUG) {
                System.out.printf("[DEBUG_TRAJ_IK] RESCUE (Relaxed boundary) | Config=%s | Alpha=%.1f | YawOffset=%.1f | Err=%.2f\n",
                    bestRescueCfg, bestRescueAlpha, trajectoryLastYawOffset, bestRescueCost / 1000.0);
            }
            return bestRescueQ;
        }
        if (DEBUG) {
            System.out.println("[DEBUG_TRAJ_IK] FAILED - No valid IK found!");
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


    void stopWorkspaceExploration() {
        if (explorationThread != null && explorationThread.isAlive()) {
            explorationThread.interrupt();
            try {
                explorationThread.join(500);
            } catch (InterruptedException ignored) {}
        }
    }

    void runWorkspaceExploration() {
        stopWorkspaceExploration();

        explorationThread = new Thread(() -> {
            WorkspaceLogger workspaceLogger = new WorkspaceLogger();
            workspaceLogger.init();
            int[] loggedCount = { 0 };
            try {
            final double step = 8; // Degrees
            final boolean[] arms = { true, false }; // Scan Right (true) first, then Left (false)

            for (boolean isRight : arms) {
                final double[] minLim = isRight ? kinematics.Kinematics.JOINT_MIN_RIGHT : kinematics.Kinematics.JOINT_MIN_LEFT;
                final double[] maxLim = isRight ? kinematics.Kinematics.JOINT_MAX_RIGHT : kinematics.Kinematics.JOINT_MAX_LEFT;

                final double q4_min = minLim[3];
                final double q4_max = maxLim[3];
                final double[] q4_samples = { q4_min, (q4_min + q4_max) / 2.0, q4_max };

                final double q5_min = minLim[4];
                final double q5_max = maxLim[4];
                final double[] q5_samples = { q5_min + 30.0, (q5_min + q5_max) / 2.0, q5_max - 30.0 };

                for (double q4 : q4_samples) {
                    for (double q5 : q5_samples) {
                        for (double q3 = minLim[2]; q3 <= maxLim[2]; q3 += step) {
                            for (double q2 = minLim[1]; q2 <= maxLim[1]; q2 += step) {
                                // Compute FK for q1=0
                                double[] p0 = armPanel.computeFK(0, q2, q3, q4, q5, 0, isRight);

                                // Rotate around Z axis (symmetry)
                                for (double q1 = minLim[0]; q1 <= maxLim[0]; q1 += 15) {
                                    double rad = Math.toRadians(q1);
                                    double x = p0[0] * Math.cos(rad) - p0[1] * Math.sin(rad);
                                    double y = p0[0] * Math.sin(rad) + p0[1] * Math.cos(rad);
                                    double z = p0[2];

                                    if (z >= -5) { // Floor limit
                                        double[] qSample = new double[] { q1, q2, q3, q4, q5, 0 };
                                        armPanel.addWorkspacePoint(new double[] { x, y, z }, isRight);
                                        workspaceLogger.logRecord(
                                                isRight ? "R" : "L",
                                                x,
                                                y,
                                                z,
                                                qSample,
                                                Double.NaN,
                                                getYawOffsetFromQ(qSample, x, y, isRight),
                                                "ANY",
                                                getActualConfig(qSample, isRight),
                                                1,
                                                0.0,
                                                computeJointMargin(qSample, isRight),
                                                computeManipulability(qSample, isRight));
                                        loggedCount[0]++;
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
            }

            SwingUtilities.invokeLater(() -> {
                armPanel.workspaceStatus = String.format("ĐÃ QUÉT XONG! (Tổng: %d điểm)",
                        armPanel.workspacePoints.size());
                armPanel.workspaceStatus = String.format("DA QUET XONG! (%d diem, CSV: %d dong)",
                        armPanel.workspacePoints.size(), loggedCount[0]);
                armPanel.repaint();
            });
            } finally {
                workspaceLogger.close();
            }
        });

        explorationThread.setPriority(Thread.MIN_PRIORITY);
        explorationThread.start();
    }

    public double getActiveArmZ() {
        try {
            JTextField tZ = isRightArmSelected ? txZRight : txZLeft;
            return Double.parseDouble(tZ.getText().trim());
        } catch (Exception e) {
            return 20.0;
        }
    }

    private boolean checkPointReachableFast(double x, double y, double z, boolean isRight, double[] qInitGuess) {
        double dx = x;
        double dy = y;
        double dz = z - (kinematics.Kinematics.L0 + kinematics.Kinematics.L1);
        double distSq = dx * dx + dy * dy + dz * dz;
        double maxReach = kinematics.Kinematics.L2 + kinematics.Kinematics.L3 + kinematics.Kinematics.L4 
                + kinematics.Kinematics.L5 + kinematics.Kinematics.L6 + kinematics.Kinematics.L7;
        if (distSq > maxReach * maxReach) return false;

        double q1_deg = isRight ? Math.toDegrees(Math.atan2(y, x)) : Math.toDegrees(-Math.atan2(y, -x));
        double q1_min = isRight ? JOINT_MIN_RIGHT[0] : JOINT_MIN_LEFT[0];
        double q1_max = isRight ? JOINT_MAX_RIGHT[0] : JOINT_MAX_LEFT[0];
        if (q1_deg < q1_min - 2.0 || q1_deg > q1_max + 2.0) return false;

        double alpha_rad = 0.0;
        double ca = Math.cos(Math.PI + alpha_rad), sa = Math.sin(Math.PI + alpha_rad);
        double[][] R_y = { { ca, 0, sa }, { 0, 1, 0 }, { -sa, 0, ca } };
        double yaw = isRight ? Math.atan2(y, x) : -Math.atan2(y, -x);
        double[][] R_target;
        if (isRight) {
            double cy = Math.cos(yaw), sy = Math.sin(yaw);
            double[][] R_z = { { cy, -sy, 0 }, { sy, cy, 0 }, { 0, 0, 1 } };
            R_target = kinematics.Kinematics.multiplyMatrices(R_z, R_y);
        } else {
            double yawR = -yaw;
            double cyR = Math.cos(yawR), syR = Math.sin(yawR);
            double[][] R_z_right = { { cyR, -syR, 0 }, { syR, cyR, 0 }, { 0, 0, 1 } };
            double[][] R_target_right = kinematics.Kinematics.multiplyMatrices(R_z_right, R_y);
            R_target = new double[][] {
                {  R_target_right[0][0], -R_target_right[0][1], -R_target_right[0][2] },
                { -R_target_right[1][0],  R_target_right[1][1],  R_target_right[1][2] },
                { -R_target_right[2][0],  R_target_right[2][1],  R_target_right[2][2] }
            };
        }

        double[] qInitRad = new double[NUM_JOINTS];
        for (int i = 0; i < NUM_JOINTS; i++) {
            qInitRad[i] = Math.toRadians(qInitGuess[i]);
        }

        double[] qSol = kinematics.Kinematics.solveIK(x, y, z, R_target, qInitRad, isRight);
        if (qSol != null && isWithinLimits(qSol, isRight)) {
            System.arraycopy(qSol, 0, qInitGuess, 0, NUM_JOINTS);
            return true;
        }

        alpha_rad = Math.toRadians(-30.0);
        ca = Math.cos(Math.PI + alpha_rad); sa = Math.sin(Math.PI + alpha_rad);
        R_y[0][0] = ca; R_y[0][2] = sa; R_y[2][0] = -sa; R_y[2][2] = ca;
        if (isRight) {
            double cy = Math.cos(yaw), sy = Math.sin(yaw);
            double[][] R_z = { { cy, -sy, 0 }, { sy, cy, 0 }, { 0, 0, 1 } };
            R_target = kinematics.Kinematics.multiplyMatrices(R_z, R_y);
        } else {
            double yawR = -yaw;
            double cyR = Math.cos(yawR), syR = Math.sin(yawR);
            double[][] R_z_right = { { cyR, -syR, 0 }, { syR, cyR, 0 }, { 0, 0, 1 } };
            double[][] R_target_right = kinematics.Kinematics.multiplyMatrices(R_z_right, R_y);
            R_target = new double[][] {
                {  R_target_right[0][0], -R_target_right[0][1], -R_target_right[0][2] },
                { -R_target_right[1][0],  R_target_right[1][1],  R_target_right[1][2] },
                { -R_target_right[2][0],  R_target_right[2][1],  R_target_right[2][2] }
            };
        }
        qSol = kinematics.Kinematics.solveIK(x, y, z, R_target, qInitRad, isRight);
        if (qSol != null && isWithinLimits(qSol, isRight)) {
            System.arraycopy(qSol, 0, qInitGuess, 0, NUM_JOINTS);
            return true;
        }

        return false;
    }

    private int checkIKForWorkspaceScan(double px, double py, double pz, boolean isRight, boolean fixedGround, String prefCfg, double[] activeAngles) {
        WorkspaceScanSolution solution = findWorkspaceScanSolution(px, py, pz, isRight, fixedGround, prefCfg, activeAngles);
        return solution == null ? 0 : solution.reachClass;
    }

    private WorkspaceScanSolution findWorkspaceScanSolution(double px, double py, double pz, boolean isRight, boolean fixedGround, String prefCfg, double[] activeAngles) {
        double activeAlpha = isRight ? activeAlphaRight : activeAlphaLeft;
        double prefYaw = getYawOffsetFromQ(activeAngles, px, py, isRight);

        // 1. Check if direct/strict path solution is available (class 2)
        double aStart = fixedGround ? -90 : (activeAlpha - 18);
        double aEnd = fixedGround ? 30 : (activeAlpha + 18);
        double aStep = fixedGround ? 3.0 : 1.0;
        for (double a = aStart; a <= aEnd; a += aStep) {
            List<double[]> candidates = tryAlpha(px, py, pz, a, isRight, activeAngles, prefYaw, true);
            for (double[] q : candidates) {
                double posErr = computePositionError(q, px, py, pz, isRight);
                if (posErr <= TRAJ_STRICT_ERROR) {
                    boolean directPath = true;
                    for (int i = 0; i < NUM_JOINTS; i++) {
                        if (Math.abs(wrappedDegDiff(q[i], activeAngles[i])) > 45.0) {
                            directPath = false;
                            break;
                        }
                    }
                    if (directPath) {
                        return new WorkspaceScanSolution(q.clone(), a, getYawOffsetFromQ(q, px, py, isRight), 2, posErr);
                    }
                }
            }
        }

        // 2. Fallback to basic geometric IK with 15mm tolerance (class 1)
        if (fixedGround) {
            for (double a = -90; a <= 30; a += 5.0) {
                List<double[]> candidates = tryAlpha(px, py, pz, a, isRight, activeAngles, prefYaw, true);
                for (double[] q : candidates) {
                    double posErr = computePositionError(q, px, py, pz, isRight);
                    if (posErr <= MAX_IK_POSITION_ERROR) {
                        return new WorkspaceScanSolution(q.clone(), a, getYawOffsetFromQ(q, px, py, isRight), 1, posErr);
                    }
                }
            }
            return null;
        }

        // Chế độ tự do: tìm xung quanh alpha hiện tại trước, sau đó tìm toàn cục
        for (double a = activeAlpha - 15; a <= activeAlpha + 15; a += 5.0) {
            List<double[]> candidates = tryAlpha(px, py, pz, a, isRight, activeAngles, prefYaw, true);
            for (double[] q : candidates) {
                double posErr = computePositionError(q, px, py, pz, isRight);
                if (posErr <= MAX_IK_POSITION_ERROR) {
                    return new WorkspaceScanSolution(q.clone(), a, getYawOffsetFromQ(q, px, py, isRight), 1, posErr);
                }
            }
        }
        for (double a = -90; a <= 30; a += 10.0) {
            List<double[]> candidates = tryAlpha(px, py, pz, a, isRight, activeAngles, prefYaw, true);
            for (double[] q : candidates) {
                double posErr = computePositionError(q, px, py, pz, isRight);
                if (posErr <= MAX_IK_POSITION_ERROR) {
                    return new WorkspaceScanSolution(q.clone(), a, getYawOffsetFromQ(q, px, py, isRight), 1, posErr);
                }
            }
        }
        return null;
    }

    private void logWorkspaceScanSolution(WorkspaceLogger logger, double[] pt, boolean isRight, boolean fixedGround, WorkspaceScanSolution solution) {
        if (logger == null || solution == null) {
            return;
        }
        logger.logRecord(
                isRight ? "R" : "L",
                pt[0],
                pt[1],
                pt[2],
                solution.q,
                solution.alphaDeg,
                solution.yawOffsetDeg,
                fixedGround ? "FIXED_GROUND" : "FREE",
                getActualConfig(solution.q, isRight),
                solution.reachClass,
                solution.posError,
                computeJointMargin(solution.q, isRight),
                computeManipulability(solution.q, isRight));
    }

    public void updateWorkspaceSlice() {
        if (!showWorkspaceSlice) return;

        stopWorkspaceSliceExploration();
        armPanel.clearWorkspaceSlice();
        armPanel.repaint();

        final double fixedZ = getFixedHeight();

        final boolean fixedGroundRight = (gripperModeComboRight.getSelectedIndex() == 0);
        final boolean fixedGroundLeft = (gripperModeComboLeft.getSelectedIndex() == 0);
        final String prefCfgRight = configComboRight.getSelectedIndex() == 0 ? "+" : "-";
        final String prefCfgLeft = configComboLeft.getSelectedIndex() == 0 ? "+" : "-";
        final double[] activeAnglesRight = anglesRight.clone();
        final double[] activeAnglesLeft = anglesLeft.clone();

        sliceExplorationThread = new Thread(() -> {
            WorkspaceLogger workspaceLogger = new WorkspaceLogger();
            workspaceLogger.init();
            try {
            boolean[] arms = { true, false }; // Scan both Right (true) and Left (false) arms
            for (boolean isRight : arms) {
                java.util.List<double[]> directDots = new java.util.ArrayList<>();
                java.util.List<double[]> transitionDots = new java.util.ArrayList<>();
                java.util.List<double[]> outer = new java.util.ArrayList<>();
                java.util.List<double[]> inner = new java.util.ArrayList<>();

                double[] minLim = isRight ? JOINT_MIN_RIGHT : JOINT_MIN_LEFT;
                double[] maxLim = isRight ? JOINT_MAX_RIGHT : JOINT_MAX_LEFT;

                double q1_min = minLim[0];
                double q1_max = maxLim[0];

                boolean fixedGround = isRight ? fixedGroundRight : fixedGroundLeft;
                String prefCfg = isRight ? prefCfgRight : prefCfgLeft;
                double[] activeAngles = isRight ? activeAnglesRight : activeAnglesLeft;

                java.util.TreeSet<Double> reachableRadii = new java.util.TreeSet<>();

                // Quét trực tiếp bán kính r và kiểm tra bằng bộ giải IK thực tế.
                for (double r = 15.0; r <= 70.0; r += 0.2) {
                    if (Thread.interrupted()) return;
                    double px = isRight ? r : -r;
                    int code = checkIKForWorkspaceScan(px, 0, fixedZ, isRight, fixedGround, prefCfg, activeAngles);
                    if (code > 0) {
                        reachableRadii.add(r);
                    }
                }

                if (!reachableRadii.isEmpty()) {
                    double rMin = reachableRadii.first();
                    double rMax = reachableRadii.last();

                    // Tạo các đa giác biên
                    for (double theta = q1_min; theta <= q1_max; theta += 2.0) {
                        double rad = Math.toRadians(theta);
                        double cos = Math.cos(rad);
                        double sin = Math.sin(rad);

                        double pxMin, pyMin, pxMax, pyMax;
                        if (isRight) {
                            pxMin = rMin * cos;
                            pyMin = rMin * sin;
                            pxMax = rMax * cos;
                            pyMax = rMax * sin;
                        } else {
                            pxMin = -rMin * cos;
                            pyMin = -rMin * sin;
                            pxMax = -rMax * cos;
                            pyMax = -rMax * sin;
                        }

                        inner.add(new double[] { pxMin, pyMin, fixedZ });
                        outer.add(new double[] { pxMax, pyMax, fixedZ });
                    }

                    // Tạo các điểm chi tiết bên trong lát cắt
                    double lastRAdded = -100;
                    for (double r : reachableRadii) {
                        if (r - lastRAdded >= 1.5) { // Lọc không gian
                            lastRAdded = r;
                            for (double theta = q1_min; theta <= q1_max; theta += 4.0) {
                                double rad = Math.toRadians(theta);
                                double px, py;
                                if (isRight) {
                                    px = r * Math.cos(rad);
                                    py = r * Math.sin(rad);
                                } else {
                                    px = -r * Math.cos(rad);
                                    py = -r * Math.sin(rad);
                                }
                                double[] pt = new double[] { px, py, fixedZ };
                                
                                WorkspaceScanSolution solution = findWorkspaceScanSolution(px, py, fixedZ, isRight, fixedGround, prefCfg, activeAngles);
                                int pointClass = solution == null ? 0 : solution.reachClass;
                                if (pointClass == 2) {
                                    directDots.add(pt);
                                    logWorkspaceScanSolution(workspaceLogger, pt, isRight, fixedGround, solution);
                                } else if (pointClass == 1) {
                                    transitionDots.add(pt);
                                    logWorkspaceScanSolution(workspaceLogger, pt, isRight, fixedGround, solution);
                                }
                            }
                        }
                    }
                }

                armPanel.setWorkspaceSliceData(directDots, transitionDots, outer, inner, isRight);

                SwingUtilities.invokeLater(() -> {
                    armPanel.workspaceStatus = String.format("LÁT CẮT Z = %.1f", fixedZ);
                    armPanel.repaint();
                });
            }
            } finally {
                workspaceLogger.close();
            }
        });

        sliceExplorationThread.setPriority(Thread.MIN_PRIORITY);
        sliceExplorationThread.start();
    }

    public void stopWorkspaceSliceExploration() {
        if (sliceExplorationThread != null && sliceExplorationThread.isAlive()) {
            sliceExplorationThread.interrupt();
            try {
                sliceExplorationThread.join(500);
            } catch (InterruptedException ignored) {}
        }
    }

    public double[] solveIKSmart(double px, double py, double pz, String preferredConfig) {
        return solveIKSmartInternal(px, py, pz, preferredConfig, isRightArmSelected);
    }

    public double[] solveIKSmartRight(double px, double py, double pz, String preferredConfig) {
        long startTime = System.currentTimeMillis();
        double[] result = solveIKSmartInternal(px, py, pz, preferredConfig, true);
        long endTime = System.currentTimeMillis();
        if (DEBUG) {
            System.out.println("--- solveIKSmartRight total time: " + (endTime - startTime) + " ms ---");
        }
        return result;
    }

    public double[] solveIKSmartLeft(double px, double py, double pz, String preferredConfig) {
        long startTime = System.currentTimeMillis();
        double[] result = solveIKSmartInternal(px, py, pz, preferredConfig, false);
        long endTime = System.currentTimeMillis();
        if (DEBUG) {
            System.out.println("--- solveIKSmartLeft total time: " + (endTime - startTime) + " ms ---");
        }
        return result;
    }

    private double[] solveIKSmartInternal(double px, double py, double pz, String preferredConfig, boolean isRight) {
        JComboBox<String> gCombo = isRight ? gripperModeComboRight : gripperModeComboLeft;
        JComboBox<String> cCombo = isRight ? configComboRight : configComboLeft;
        double[] activeAngles = isRight ? anglesRight : anglesLeft;
        boolean fixedGround = (gCombo.getSelectedIndex() == 0);
        double prefYaw = getYawOffsetFromQ(activeAngles, px, py, isRight);

        if (fixedGround) {
            // Scan alphas from 0 outward, heavily penalizing deviation from 0
            // This ensures the gripper is as vertical as possible while still being reachable
            String userPref = cCombo.getSelectedIndex() == 0 ? "+" : "-";
            double minCost = Double.MAX_VALUE;
            double[] best = null;
            double bestAlphaGround = 0.0;
            
            // Try alpha=0 first. Because alphaPenalty = a^2 * 50, a valid
            // vertical-gripper solution dominates nonzero-alpha candidates.
            double[] alphaScan = { 0, -15, 15, -30, 30, -45, -60, -75, -90 };
            for (double a : alphaScan) {
                List<double[]> candidates = tryAlpha(px, py, pz, a, isRight, activeAngles, prefYaw, true);
                for (double[] q : candidates) {
                    double posErr = computePositionError(q, px, py, pz, isRight);
                    if (posErr > MAX_IK_POSITION_ERROR) {
                        continue;
                    }
                    String actualCfg = getActualConfig(q, isRight);
                    // Very heavy penalty for alpha deviation from 0 (vertical gripper)
                    double alphaPenalty = a * a * 50.0;
                    double cost = posErr * 200.0 + alphaPenalty;
                    if (!actualCfg.equals(userPref)) {
                        cost += 10000.0;
                    }
                    cost += continuityCost(q, activeAngles, false) * 0.02;
                    if (cost < minCost) {
                        minCost = cost;
                        best = q;
                        bestAlphaGround = a;
                    }
                }
                if (best != null && a == 0.0) {
                    if (isRight) activeAlphaRight = bestAlphaGround;
                    else activeAlphaLeft = bestAlphaGround;
                    return best;
                }
            }
            if (best != null) {
                if (isRight) activeAlphaRight = bestAlphaGround;
                else activeAlphaLeft = bestAlphaGround;
                if (ikSelectionLogEnabled) {
                    double err = computePositionError(best, px, py, pz, isRight);
                    System.out.println(String.format("IK Selected (Fixed Ground) | target=[%.2f, %.2f, %.2f] alpha=%.1f err=%.4f",
                            px, py, pz, bestAlphaGround, err));
                }
            }
            return best;
        }

        double minCost = Double.MAX_VALUE;
        double[] bestQ = null;
        double currentAlpha = isRight ? activeAlphaRight : activeAlphaLeft;
        double bestAlpha = currentAlpha;
        double[] q_pref = new double[NUM_JOINTS];
        q_pref[2] = 60.0;
        q_pref[3] = -35.0;

        // 1. Try local search around current alpha first (very fast, covers small movements)
        for (double a = currentAlpha - 15; a <= currentAlpha + 15; a += 5.0) {
            String userPref = cCombo.getSelectedIndex() == 0 ? "+" : "-";

            List<double[]> candidates = tryAlpha(px, py, pz, a, isRight, activeAngles, prefYaw, true);
            for (double[] q : candidates) {
                double posErr = computePositionError(q, px, py, pz, isRight);
                if (posErr > MAX_IK_POSITION_ERROR) {
                    continue;
                }
                String actualCfg = getActualConfig(q, isRight);
                double cost = calculateIKCost(a, q, activeAngles, q_pref, actualCfg, userPref, isRight);
                cost += posErr * 50.0;
                if (cost < minCost) {
                    minCost = cost;
                    bestQ = q;
                    bestAlpha = a;
                }
            }
        }

        // If local search found a highly accurate solution, return it immediately
        if (bestQ != null && computePositionError(bestQ, px, py, pz, isRight) < 0.2) {
            if (isRight) {
                activeAlphaRight = bestAlpha;
            } else {
                activeAlphaLeft = bestAlpha;
            }
            if (ikSelectionLogEnabled) {
                double err = computePositionError(bestQ, px, py, pz, isRight);
                System.out.println(String.format("IK Selected (Local) | target=[%.2f, %.2f, %.2f] alpha=%.1f err=%.4f",
                        px, py, pz, bestAlpha, err));
            }
            return bestQ;
        }

        // 2. Fallback to global scan if local search fails or is inaccurate
        for (double a = -90; a <= 30; a += 15.0) {
            String userPref = cCombo.getSelectedIndex() == 0 ? "+" : "-";

            List<double[]> candidates = tryAlpha(px, py, pz, a, isRight, activeAngles, prefYaw, true);
            for (double[] q : candidates) {
                double posErr = computePositionError(q, px, py, pz, isRight);
                if (posErr > MAX_IK_POSITION_ERROR) {
                    continue;
                }
                String actualCfg = getActualConfig(q, isRight);
                double cost = calculateIKCost(a, q, activeAngles, q_pref, actualCfg, userPref, isRight);
                cost += posErr * 50.0;
                if (cost < minCost) {
                    minCost = cost;
                    bestQ = q;
                    bestAlpha = a;
                }
            }
        }

        if (bestQ != null) {
            if (isRight) {
                activeAlphaRight = bestAlpha;
            } else {
                activeAlphaLeft = bestAlpha;
            }
            if (ikSelectionLogEnabled) {
                double err = computePositionError(bestQ, px, py, pz, isRight);
                System.out.println(String.format("IK Selected (Global) | target=[%.2f, %.2f, %.2f] alpha=%.1f err=%.4f",
                        px, py, pz, bestAlpha, err));
            }
            return bestQ;
        }
        return null;
    }

    private double computePositionError(double[] qDeg, double tx, double ty, double tz, boolean isRight) {
        double[] fk = armPanel.computeFK(qDeg[0], qDeg[1], qDeg[2], qDeg[3], qDeg[4], qDeg[5], isRight);
        double dx = fk[0] - tx;
        double dy = fk[1] - ty;
        double dz = fk[2] - tz;
        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    private double computeJointMargin(double[] qDeg, boolean isRight) {
        double[] minLim = isRight ? JOINT_MIN_RIGHT : JOINT_MIN_LEFT;
        double[] maxLim = isRight ? JOINT_MAX_RIGHT : JOINT_MAX_LEFT;
        double margin = Double.MAX_VALUE;
        for (int i = 0; i < NUM_JOINTS; i++) {
            margin = Math.min(margin, qDeg[i] - minLim[i]);
            margin = Math.min(margin, maxLim[i] - qDeg[i]);
        }
        return margin;
    }

    private double computeManipulability(double[] qDeg, boolean isRight) {
        double[] qRad = new double[NUM_JOINTS];
        for (int i = 0; i < NUM_JOINTS; i++) {
            qRad[i] = Math.toRadians(qDeg[i]);
        }
        double[][] jacobian = kinematics.Kinematics.computeJacobianEE(qRad, isRight);
        return Math.abs(kinematics.Kinematics.compute6x6Determinant(jacobian));
    }

    private double[] selectBestByPositionError(List<double[]> candidates, double tx, double ty, double tz, boolean isRight) {
        double[] best = null;
        double minErr = Double.MAX_VALUE;
        for (double[] q : candidates) {
            double err = computePositionError(q, tx, ty, tz, isRight);
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
            String prefCfg, boolean isRight) {
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
        double[] minLim = isRight ? JOINT_MIN_RIGHT : JOINT_MIN_LEFT;
        double[] maxLim = isRight ? JOINT_MAX_RIGHT : JOINT_MAX_LEFT;
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
        double jPosture = 0;
        double q3 = q[2];
        if (isRight) {
            if (q3 < 0) {
                jPosture += 5.0 * Math.pow(Math.toRadians(q3), 2); // Heavy penalty for wrong sign
            } else {
                jPosture += 0.5 * Math.pow(Math.toRadians(q3), 2); // Small pull towards 0 to encourage Joint 2
            }
        } else {
            if (q3 > 0) {
                jPosture += 5.0 * Math.pow(Math.toRadians(q3), 2); // Heavy penalty for wrong sign
            } else {
                jPosture += 0.5 * Math.pow(Math.toRadians(q3), 2); // Small pull towards 0 to encourage Joint 2
            }
        }

        double q4 = q[3];
        double q4_pref = isRight ? -45.0 : 45.0;
        jPosture += 0.2 * Math.pow(Math.toRadians(q4 - q4_pref), 2);

        // 3b. Wrist Pitch (Joint 5): Prevent backward bending (prefer positive for both arms)
        double q5 = q[4];
        if (q5 < 0) {
            jPosture += 4.0 * Math.pow(Math.toRadians(q5), 2); // Heavy penalty for wrong sign
        } else {
            jPosture += 0.2 * Math.pow(Math.toRadians(q5), 2); // Small pull to keep close to 0
        }

        // 3c. Wrist Roll (Joint 6): Small pull to neutral (0) to stabilize orientation
        double q6 = q[5];
        jPosture += 0.15 * Math.pow(Math.toRadians(q6), 2);

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

    public String getActualConfig(double[] q, boolean isRight) {
        if (isRight) {
            return q[2] >= 0 ? "+" : "-";
        } else {
            return q[2] <= 0 ? "+" : "-";
        }
    }

    private double getYawOffsetFromQ(double[] q, double px, double py, boolean isRight) {
        double q1_min = isRight ? JOINT_MIN_RIGHT[0] : JOINT_MIN_LEFT[0];
        double q1_max = isRight ? JOINT_MAX_RIGHT[0] : JOINT_MAX_LEFT[0];
        double q1_base = isRight ? Math.atan2(py, px) : -Math.atan2(py, -px);
        q1_base = Math.max(Math.toRadians(q1_min), Math.min(Math.toRadians(q1_max), q1_base));
        double diff = Math.toDegrees(Math.toRadians(q[0]) - q1_base);
        while (diff > 180) diff -= 360;
        while (diff < -180) diff += 360;
        return diff;
    }

    private List<double[]> tryAlpha(double px, double py, double pz, double alphaDeg, boolean isRight, double[] qRef, double preferredYaw, boolean findOneOnly) {
        List<double[]> validSolutions = new ArrayList<>();
        double alpha_rad = Math.toRadians(alphaDeg);
        double q1_min = isRight ? JOINT_MIN_RIGHT[0] : JOINT_MIN_LEFT[0];
        double q1_max = isRight ? JOINT_MAX_RIGHT[0] : JOINT_MAX_LEFT[0];
        double q1_base = isRight ? Math.atan2(py, px) : -Math.atan2(py, -px);
        q1_base = Math.max(Math.toRadians(q1_min), Math.min(Math.toRadians(q1_max), q1_base));
        
        double ca = Math.cos(Math.PI + alpha_rad), sa = Math.sin(Math.PI + alpha_rad);
        double[][] R_y = { { ca, 0, sa }, { 0, 1, 0 }, { -sa, 0, ca } };

        Double[] yawOffsets = { 0.0, -15.0, 15.0, -30.0, 30.0, -45.0, 45.0, -60.0, 60.0, -75.0, 75.0, -90.0, 90.0 };
        if (!Double.isNaN(preferredYaw)) {
            java.util.Arrays.sort(yawOffsets, (a, b) -> Double.compare(Math.abs(a - preferredYaw), Math.abs(b - preferredYaw)));
        }

        double[] activeAngles = qRef;

        // --- Loop 1: Strategy 1 (Warm Start) for ALL yawOffsets first ---
        for (double offsetDeg : yawOffsets) {
            double yaw = q1_base + Math.toRadians(offsetDeg);
            double[][] R_target;
            if (isRight) {
                double cy = Math.cos(yaw), sy = Math.sin(yaw);
                double[][] R_z = { { cy, -sy, 0 }, { sy, cy, 0 }, { 0, 0, 1 } };
                R_target = multiplyMatrices(R_z, R_y);
            } else {
                double yawR = -yaw;
                double cyR = Math.cos(yawR), syR = Math.sin(yawR);
                double[][] R_z_right = { { cyR, -syR, 0 }, { syR, cyR, 0 }, { 0, 0, 1 } };
                double[][] R_target_right = multiplyMatrices(R_z_right, R_y);
                R_target = new double[][] {
                    {  R_target_right[0][0], -R_target_right[0][1], -R_target_right[0][2] },
                    { -R_target_right[1][0],  R_target_right[1][1],  R_target_right[1][2] },
                    { -R_target_right[2][0],  R_target_right[2][1],  R_target_right[2][2] }
                };
            }

            double[] qInit = new double[NUM_JOINTS];
            for (int i = 0; i < NUM_JOINTS; i++) {
                qInit[i] = Math.toRadians(activeAngles[i]);
            }
            
            double[] q = solveIK(px, py, pz, R_target, qInit, isRight);
            if (q != null && isWithinLimits(q, isRight)) {
                addUniqueSolution(validSolutions, q);
                if (findOneOnly) return validSolutions;
            }
        }

        // --- Loop 2: Strategy 2 & 3 (Cold Starts) ONLY if no Warm Start was found ---
        for (double offsetDeg : yawOffsets) {
            double yaw = q1_base + Math.toRadians(offsetDeg);
            double[][] R_target;
            if (isRight) {
                double cy = Math.cos(yaw), sy = Math.sin(yaw);
                double[][] R_z = { { cy, -sy, 0 }, { sy, cy, 0 }, { 0, 0, 1 } };
                R_target = multiplyMatrices(R_z, R_y);
            } else {
                double yawR = -yaw;
                double cyR = Math.cos(yawR), syR = Math.sin(yawR);
                double[][] R_z_right = { { cyR, -syR, 0 }, { syR, cyR, 0 }, { 0, 0, 1 } };
                double[][] R_target_right = multiplyMatrices(R_z_right, R_y);
                R_target = new double[][] {
                    {  R_target_right[0][0], -R_target_right[0][1], -R_target_right[0][2] },
                    { -R_target_right[1][0],  R_target_right[1][1],  R_target_right[1][2] },
                    { -R_target_right[2][0],  R_target_right[2][1],  R_target_right[2][2] }
                };
            }

            // Strategy 2: Cold start guesses
            double[] q3_options = isRight ? new double[] { 0.3, -0.3 } : new double[] { -0.3, 0.3 };
            double[] q4_options = isRight ? new double[] { -35.0, 35.0 } : new double[] { 35.0, -35.0 };
            
            double[] q2_guesses = { 1.2, 0.6, 0.0, -0.6, -1.2 };
            for (int cfgIdx = 0; cfgIdx < 2; cfgIdx++) {
                double q3_val = q3_options[cfgIdx];
                double q4_val = Math.toRadians(q4_options[cfgIdx]);
                for (double q2_val : q2_guesses) {
                    double[] qHome = new double[NUM_JOINTS];
                    qHome[0] = yaw;
                    qHome[1] = q2_val;
                    qHome[2] = q3_val;
                    qHome[3] = q4_val;
                    
                    double[] q2 = solveIK(px, py, pz, R_target, qHome, isRight);
                    if (q2 != null && isWithinLimits(q2, isRight)) {
                        addUniqueSolution(validSolutions, q2);
                        if (findOneOnly) return validSolutions;
                    }
                }
            }
            
            // Strategy 3: Alternative cold start guesses
            double[] qInit = new double[NUM_JOINTS];
            for (int i = 0; i < NUM_JOINTS; i++) {
                qInit[i] = Math.toRadians(activeAngles[i]);
            }
            if (Math.abs(qInit[0] - yaw) > 0.15) {
                for (int cfgIdx = 0; cfgIdx < 2; cfgIdx++) {
                    double q3_val = q3_options[cfgIdx];
                    double q4_val = Math.toRadians(q4_options[cfgIdx]);
                    for (double q2_val : new double[]{ 0.6, 0.0, -0.6 }) {
                        double[] qAlt = new double[NUM_JOINTS];
                        qAlt[0] = qInit[0];
                        qAlt[1] = q2_val;
                        qAlt[2] = q3_val;
                        qAlt[3] = q4_val;
                        
                        double[] q3 = solveIK(px, py, pz, R_target, qAlt, isRight);
                        if (q3 != null && isWithinLimits(q3, isRight)) {
                            addUniqueSolution(validSolutions, q3);
                            if (findOneOnly) return validSolutions;
                        }
                    }
                }
            }
        }
        return validSolutions;
    }

    private void addUniqueSolution(List<double[]> list, double[] sol) {
        for (double[] existing : list) {
            double diff = 0;
            for (int j = 0; j < NUM_JOINTS; j++) {
                diff += Math.abs(existing[j] - sol[j]);
            }
            if (diff < 0.1) {
                return;
            }
        }
        list.add(sol);
    }

    public double[] solveIKSmart(double px, double py, double pz, double[][] R_target) {
        double[] qInit = new double[NUM_JOINTS];
        double[] activeAngles = isRightArmSelected ? anglesRight : anglesLeft;
        for (int i = 0; i < NUM_JOINTS; i++) {
            qInit[i] = Math.toRadians(activeAngles[i]);
        }
        double[] q = solveIK(px, py, pz, R_target, qInit, isRightArmSelected);
        if (q != null && isWithinLimits(q, isRightArmSelected)) {
            return q;
        }
        
        double[] q3_options = isRightArmSelected ? new double[] { 0.3, -0.3 } : new double[] { -0.3, 0.3 };
        double[] q4_options = isRightArmSelected ? new double[] { -35.0, 35.0 } : new double[] { 35.0, -35.0 };
        double[] q2_guesses = { 1.2, 0.8, 0.4, 0.0, -0.4, -0.8, -1.2 };
        
        for (int cfgIdx = 0; cfgIdx < 2; cfgIdx++) {
            double q3_val = q3_options[cfgIdx];
            double q4_val = Math.toRadians(q4_options[cfgIdx]);
            for (double q2_val : q2_guesses) {
                double[] qHome = new double[NUM_JOINTS];
                double q1_min = isRightArmSelected ? JOINT_MIN_RIGHT[0] : JOINT_MIN_LEFT[0];
                double q1_max = isRightArmSelected ? JOINT_MAX_RIGHT[0] : JOINT_MAX_LEFT[0];
                double q1_base = isRightArmSelected ? Math.atan2(py, px) : -Math.atan2(py, -px);
                qHome[0] = Math.max(Math.toRadians(q1_min), Math.min(Math.toRadians(q1_max), q1_base));
                qHome[1] = q2_val;
                qHome[2] = q3_val;
                qHome[3] = q4_val;
                
                q = solveIK(px, py, pz, R_target, qHome, isRightArmSelected);
                if (q != null && isWithinLimits(q, isRightArmSelected)) {
                    return q;
                }
            }
        }
        return null;
    }

    public double[] solveIKSmart(double px, double py, double pz, double alpha_deg) {
        double alpha_rad = Math.toRadians(alpha_deg);
        double q1_min = isRightArmSelected ? JOINT_MIN_RIGHT[0] : JOINT_MIN_LEFT[0];
        double q1_max = isRightArmSelected ? JOINT_MAX_RIGHT[0] : JOINT_MAX_LEFT[0];
        double q1_base = isRightArmSelected ? Math.atan2(py, px) : -Math.atan2(py, -px);
        double q1 = Math.max(Math.toRadians(q1_min), Math.min(Math.toRadians(q1_max), q1_base));
        double[][] R_target;
        if (isRightArmSelected) {
            double c1 = Math.cos(q1), s1 = Math.sin(q1);
            double ca = Math.cos(-alpha_rad), sa = Math.sin(-alpha_rad);
            double[][] R_y = { { ca, 0, sa }, { 0, 1, 0 }, { -sa, 0, ca } };
            double[][] R_z = { { c1, -s1, 0 }, { s1, c1, 0 }, { 0, 0, 1 } };
            R_target = multiplyMatrices(R_z, R_y);
        } else {
            double q1R = -q1;
            double c1R = Math.cos(q1R), s1R = Math.sin(q1R);
            double ca = Math.cos(-alpha_rad), sa = Math.sin(-alpha_rad);
            double[][] R_y = { { ca, 0, sa }, { 0, 1, 0 }, { -sa, 0, ca } };
            double[][] R_z_right = { { c1R, -s1R, 0 }, { s1R, c1R, 0 }, { 0, 0, 1 } };
            double[][] R_target_right = multiplyMatrices(R_z_right, R_y);
            R_target = new double[][] {
                {  R_target_right[0][0], -R_target_right[0][1], -R_target_right[0][2] },
                { -R_target_right[1][0],  R_target_right[1][1],  R_target_right[1][2] },
                { -R_target_right[2][0],  R_target_right[2][1],  R_target_right[2][2] }
            };
        }
        return solveIKSmart(px, py, pz, R_target);
    }

    private double[] findBestSolution(List<double[]> solutions) {
        if (solutions.isEmpty())
            return null;
        double minScore = Double.MAX_VALUE;
        double[] best = null;
        double[] minLim = isRightArmSelected ? JOINT_MIN_RIGHT : JOINT_MIN_LEFT;
        double[] maxLim = isRightArmSelected ? JOINT_MAX_RIGHT : JOINT_MAX_LEFT;
        double[] activeAngles = isRightArmSelected ? anglesRight : anglesLeft;
        for (double[] q : solutions) {
            double score = 0;
            for (int i = 0; i < NUM_JOINTS; i++) {
                double qDeg = q[i]; // q is already in Degrees
                double diffCurrent = Math.abs(qDeg - activeAngles[i]);
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
        return isWithinLimits(q, isRightArmSelected);
    }    private boolean isWithinLimits(double[] q, boolean isRight) {
        double[] minLim = isRight ? JOINT_MIN_RIGHT : JOINT_MIN_LEFT;
        double[] maxLim = isRight ? JOINT_MAX_RIGHT : JOINT_MAX_LEFT;
        for (int i = 0; i < q.length; i++) {
            if (q[i] < (minLim[i] - 0.1) || q[i] > (maxLim[i] + 0.1))
                return false;
        }
        return true;
    }

    private void startControllerTimer() {
        controllerTimer = new Timer(40, new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                if (controllerReceiver != null && controllerReceiver.isConnected()) {
                    processControllerInput();
                }
            }
        });
        controllerTimer.start();
    }

    private void processControllerInput() {
        if (controllerReceiver == null || !controllerReceiver.isConnected()) return;

        double[] axes = controllerReceiver.getAxes();
        int[] buttons = controllerReceiver.getButtons();

        if (axes == null || axes.length < 4) return;

        // Print debug info to Java console when active
        boolean hasActiveAxis = false;
        for (double a : axes) {
            // Treat axes that idle at -1.0 (triggers) as active only when pressed (value > -0.85)
            if (Math.abs(a) > 0.15 && Math.abs(a + 1.0) > 0.15) { 
                hasActiveAxis = true; 
                break; 
            }
        }
        boolean hasActiveBtn = false;
        for (int b : buttons) {
            if (b == 1) { hasActiveBtn = true; break; }
        }
        if (hasActiveAxis || hasActiveBtn) {
            System.out.println("[DEBUG_JAVA] Axes: " + Arrays.toString(axes) + " | Buttons: " + Arrays.toString(buttons));
        }

        // 1. Detect mapping & extract axes
        double lx = axes[0];
        double ly = axes[1];
        double rx = 0.0;
        double ry = 0.0;
        double l2_val = 0.0;
        double r2_val = 0.0;

        if (axes.length >= 6) {
            // Check DirectInput triggers: axes[4] and axes[5] idle at -1.0
            if (Math.abs(axes[4] + 1.0) < 0.25 && Math.abs(axes[5] + 1.0) < 0.25) {
                rx = axes[2];
                ry = axes[3];
                l2_val = (axes[4] + 1.0) / 2.0; // scale to [0, 1]
                r2_val = (axes[5] + 1.0) / 2.0; // scale to [0, 1]
            } else {
                // Xbox emulation (triggers combined or separate)
                rx = axes[3];
                ry = axes[4];
                l2_val = (axes[2] + 1.0) / 2.0;
                r2_val = axes.length > 5 ? (axes[5] + 1.0) / 2.0 : 0.0;
            }
        } else {
            rx = axes[2];
            ry = axes[3];
        }

        // Apply deadzone of 0.15
        double deadzone = 0.15;
        if (Math.abs(lx) < deadzone) lx = 0;
        if (Math.abs(ly) < deadzone) ly = 0;
        if (Math.abs(rx) < deadzone) rx = 0;
        if (Math.abs(ry) < deadzone) ry = 0;
        if (l2_val < 0.05) l2_val = 0;
        if (r2_val < 0.05) r2_val = 0;

        // Bumpers (L1: index 4 or 9, R1: index 5 or 10)
        boolean l1 = (buttons.length > 9 && buttons[9] == 1) || (buttons.length > 4 && buttons[4] == 1);
        boolean r1 = (buttons.length > 10 && buttons[10] == 1) || (buttons.length > 5 && buttons[5] == 1);

        // Reset buttons: 
        // Reset Left: Triangle/Y (button 3)
        boolean resetLeft = buttons.length > 3 && buttons[3] == 1;
        // Reset Right: Circle/B (button 1 or 2)
        boolean resetRight = buttons.length > 2 && (buttons[1] == 1 || buttons[2] == 1);
        // Reset Both: Options/Start (button 6, 7 or 8)
        boolean resetBoth = (buttons.length > 8 && buttons[8] == 1) || 
                            (buttons.length > 7 && buttons[7] == 1) || 
                            (buttons.length > 6 && buttons[6] == 1);

        if (resetBoth) {
            double[] rightHome = { 0, 0, 10, -30, 0, 0 };
            double[] leftHome = { 0, 0, -10, 30, 0, 0 };
            setTargetAnglesRight(rightHome);
            setTargetAnglesLeft(leftHome);
            setGotoStatusRight("Home (PS5)", Color.BLUE);
            setGotoStatusLeft("Home (PS5)", Color.BLUE);
            return;
        }
        if (resetLeft) {
            double[] leftHome = { targetAnglesRight[0], 0, -10, 30, 0, 0 };
            setTargetAnglesLeft(leftHome);
            setGotoStatusLeft("Home (PS5)", Color.BLUE);
        }
        if (resetRight) {
            double[] rightHome = { targetAnglesLeft[0], 0, 10, -30, 0, 0 };
            setTargetAnglesRight(rightHome);
            setGotoStatusRight("Home (PS5)", Color.BLUE);
        }

        // 2. Process displacements
        double speed = 0.5; // 0.5 cm per step
        boolean rightMoved = (rx != 0 || ry != 0 || r1 || r2_val > 0);
        boolean leftMoved = (lx != 0 || ly != 0 || l1 || l2_val > 0);

        if (rightMoved || leftMoved) {
            // Determine active master for joint 1
            boolean rightIsMaster = isRightArmSelected;
            if (rightMoved && !leftMoved) rightIsMaster = true;
            else if (leftMoved && !rightMoved) rightIsMaster = false;

            double sharedJoint1 = targetAngles[0]; // default to current target Joint 1

            if (rightIsMaster) {
                // 1. Solve Right Arm (Master)
                if (rightMoved) {
                    double[] eeRight = armPanel.computeFK(targetAnglesRight[0], targetAnglesRight[1], targetAnglesRight[2], targetAnglesRight[3], targetAnglesRight[4], targetAnglesRight[5], true);
                    double dx = rx * speed;
                    double dy = -ry * speed;
                    double dz = 0;
                    if (r1) dz += speed;
                    if (r2_val > 0) dz -= r2_val * speed;

                    double nx = eeRight[0] + dx;
                    double ny = eeRight[1] + dy;
                    double nz = eeRight[2] + dz;

                    String prefCfg = configComboRight.getSelectedIndex() == 0 ? "+" : "-";
                    double[] sol = solveIKSmartRight(nx, ny, nz, prefCfg);
                    if (sol != null) {
                        setTargetAnglesRight(sol);
                        sharedJoint1 = sol[0];
                        setGotoStatusRight(String.format("OK (%.1f, %.1f, %.1f)", nx, ny, nz), new Color(0, 140, 0));
                    } else {
                        setGotoStatusRight("Ngoài tầm (PS5)", Color.RED);
                    }
                }

                // 2. Solve Left Arm (Slave, with Joint 1 synchronized)
                if (leftMoved) {
                    double[] eeLeft = armPanel.computeFK(targetAnglesLeft[0], targetAnglesLeft[1], targetAnglesLeft[2], targetAnglesLeft[3], targetAnglesLeft[4], targetAnglesLeft[5], false);
                    double dx = lx * speed;
                    double dy = -ly * speed;
                    double dz = 0;
                    if (l1) dz += speed;
                    if (l2_val > 0) dz -= l2_val * speed;

                    double nx = eeLeft[0] + dx;
                    double ny = eeLeft[1] + dy;
                    double nz = eeLeft[2] + dz;

                    String prefCfg = configComboLeft.getSelectedIndex() == 0 ? "+" : "-";
                    double[] sol = solveIKSmartLeft(nx, ny, nz, prefCfg);
                    if (sol != null) {
                        // Force Joint 1 to match the master
                        sol[0] = sharedJoint1;
                        setTargetAnglesLeft(sol);
                        setGotoStatusLeft(String.format("OK (%.1f, %.1f, %.1f)", nx, ny, nz), new Color(0, 140, 0));
                    } else {
                        setGotoStatusLeft("Ngoài tầm (PS5)", Color.RED);
                    }
                } else {
                    // Update Joint 1 for Left Arm
                    double[] leftHome = targetAnglesLeft.clone();
                    leftHome[0] = sharedJoint1;
                    setTargetAnglesLeft(leftHome);
                }
            } else {
                // 1. Solve Left Arm (Master)
                if (leftMoved) {
                    double[] eeLeft = armPanel.computeFK(targetAnglesLeft[0], targetAnglesLeft[1], targetAnglesLeft[2], targetAnglesLeft[3], targetAnglesLeft[4], targetAnglesLeft[5], false);
                    double dx = lx * speed;
                    double dy = -ly * speed;
                    double dz = 0;
                    if (l1) dz += speed;
                    if (l2_val > 0) dz -= l2_val * speed;

                    double nx = eeLeft[0] + dx;
                    double ny = eeLeft[1] + dy;
                    double nz = eeLeft[2] + dz;

                    String prefCfg = configComboLeft.getSelectedIndex() == 0 ? "+" : "-";
                    double[] sol = solveIKSmartLeft(nx, ny, nz, prefCfg);
                    if (sol != null) {
                        setTargetAnglesLeft(sol);
                        sharedJoint1 = sol[0];
                        setGotoStatusLeft(String.format("OK (%.1f, %.1f, %.1f)", nx, ny, nz), new Color(0, 140, 0));
                    } else {
                        setGotoStatusLeft("Ngoài tầm (PS5)", Color.RED);
                    }
                }

                // 2. Solve Right Arm (Slave, with Joint 1 synchronized)
                if (rightMoved) {
                    double[] eeRight = armPanel.computeFK(targetAnglesRight[0], targetAnglesRight[1], targetAnglesRight[2], targetAnglesRight[3], targetAnglesRight[4], targetAnglesRight[5], true);
                    double dx = rx * speed;
                    double dy = -ry * speed;
                    double dz = 0;
                    if (r1) dz += speed;
                    if (r2_val > 0) dz -= r2_val * speed;

                    double nx = eeRight[0] + dx;
                    double ny = eeRight[1] + dy;
                    double nz = eeRight[2] + dz;

                    String prefCfg = configComboRight.getSelectedIndex() == 0 ? "+" : "-";
                    double[] sol = solveIKSmartRight(nx, ny, nz, prefCfg);
                    if (sol != null) {
                        // Force Joint 1 to match the master
                        sol[0] = sharedJoint1;
                        setTargetAnglesRight(sol);
                        setGotoStatusRight(String.format("OK (%.1f, %.1f, %.1f)", nx, ny, nz), new Color(0, 140, 0));
                    } else {
                        setGotoStatusRight("Ngoài tầm (PS5)", Color.RED);
                    }
                } else {
                    // Update Joint 1 for Right Arm
                    double[] rightHome = targetAnglesRight.clone();
                    rightHome[0] = sharedJoint1;
                    setTargetAnglesRight(rightHome);
                }
            }

            armPanel.repaint();
        }
    }

    private boolean isArmAtTarget(boolean isRight) {
        double[] current = isRight ? anglesRight : anglesLeft;
        double[] target = isRight ? targetAnglesRight : targetAnglesLeft;
        for (int i = 0; i < NUM_JOINTS; i++) {
            if (Math.abs(current[i] - target[i]) > 0.5) { // Within 0.5 degrees
                return false;
            }
        }
        return true;
    }

    public void triggerPickAndPlace(double px, double py, double pz) {
        new Thread(() -> {
            try {
                boolean isRight = isRightArmSelected;
                String armLabel = isRight ? "RIGHT" : "LEFT";
                System.out.printf("[PICK-AND-PLACE] Starting sequence for %s arm to target (%.1f, %.1f, %.1f)\n", armLabel, px, py, pz);
                
                // Solve IK for pre-grasp hover (z + 40.0)
                double hoverZ = pz + 40.0;
                double[] hoverQ = solveIKSmart(px, py, hoverZ, (String) null);
                if (hoverQ == null) {
                    System.err.println("[PICK-AND-PLACE] Pre-grasp hover position is out of reach!");
                    SwingUtilities.invokeLater(() -> setGotoStatus("Vượt giới hạn!", Color.RED));
                    return;
                }
                
                // Solve IK for grasp (z)
                double[] graspQ = solveIKSmart(px, py, pz, (String) null);
                if (graspQ == null) {
                    System.err.println("[PICK-AND-PLACE] Grasp position is out of reach!");
                    SwingUtilities.invokeLater(() -> setGotoStatus("Vượt giới hạn!", Color.RED));
                    return;
                }
                
                SwingUtilities.invokeLater(() -> setGotoStatus("Gắp: Bắt đầu...", Color.BLUE));
                
                // Step 1: Open Gripper
                SwingUtilities.invokeLater(() -> {
                    setGotoStatus("Gắp: Mở kẹp...", Color.BLUE);
                    if (isRight) {
                        isGrippedRight = false;
                    } else {
                        isGrippedLeft = false;
                    }
                    armPanel.repaint();
                });
                if (uartManager != null && uartManager.isConnected()) {
                    uartManager.sendData(isRight ? "R:RELEASE\n" : "L:RELEASE\n");
                }
                Thread.sleep(800);
                
                // Step 2: Move to hover
                SwingUtilities.invokeLater(() -> {
                    setGotoStatus("Gắp: Di chuyển tới vị trí trên vật...", Color.BLUE);
                    setTargetAngles(hoverQ);
                });
                while (!isArmAtTarget(isRight)) {
                    Thread.sleep(50);
                }
                Thread.sleep(300);
                
                // Step 3: Descend to grasp
                SwingUtilities.invokeLater(() -> {
                    setGotoStatus("Gắp: Hạ xuống vật...", Color.BLUE);
                    setTargetAngles(graspQ);
                });
                while (!isArmAtTarget(isRight)) {
                    Thread.sleep(50);
                }
                Thread.sleep(500);
                
                // Step 4: Close Gripper
                SwingUtilities.invokeLater(() -> {
                    setGotoStatus("Gắp: Đóng kẹp...", Color.BLUE);
                    if (isRight) {
                        isGrippedRight = true;
                    } else {
                        isGrippedLeft = true;
                    }
                    armPanel.repaint();
                });
                if (uartManager != null && uartManager.isConnected()) {
                    uartManager.sendData(isRight ? "R:GRIP\n" : "L:GRIP\n");
                }
                Thread.sleep(1000);
                
                // Step 5: Ascend back to hover
                SwingUtilities.invokeLater(() -> {
                    setGotoStatus("Gắp: Nhấc vật lên...", Color.BLUE);
                    setTargetAngles(hoverQ);
                });
                while (!isArmAtTarget(isRight)) {
                    Thread.sleep(50);
                }
                Thread.sleep(300);
                
                // Step 6: Move to drop deposit position
                double dropX = isRight ? 120.0 : -120.0;
                double dropY = 0.0;
                double dropZ = 80.0;
                double[] dropQ = solveIKSmart(dropX, dropY, dropZ, (String) null);
                if (dropQ != null) {
                    SwingUtilities.invokeLater(() -> {
                        setGotoStatus("Gắp: Di chuyển tới chỗ thả...", Color.BLUE);
                        setTargetAngles(dropQ);
                    });
                    while (!isArmAtTarget(isRight)) {
                        Thread.sleep(50);
                    }
                    Thread.sleep(500);
                    
                    // Step 7: Open Gripper
                    SwingUtilities.invokeLater(() -> {
                        setGotoStatus("Gắp: Thả vật...", Color.BLUE);
                        if (isRight) {
                            isGrippedRight = false;
                        } else {
                            isGrippedLeft = false;
                        }
                        armPanel.repaint();
                    });
                    if (uartManager != null && uartManager.isConnected()) {
                        uartManager.sendData(isRight ? "R:RELEASE\n" : "L:RELEASE\n");
                    }
                    Thread.sleep(800);
                }
                
                // Step 8: Return to Home
                SwingUtilities.invokeLater(() -> {
                    setGotoStatus("Gắp: Trở về vị trí nghỉ...", Color.BLUE);
                    double[] homeQ = isRight ? new double[]{ 0, 0, 10, -30, 0, 0 } : new double[]{ 0, 0, 10, 30, 0, 0 };
                    setTargetAngles(homeQ);
                });
                while (!isArmAtTarget(isRight)) {
                    Thread.sleep(50);
                }
                
                SwingUtilities.invokeLater(() -> setGotoStatus("Hoàn thành!", new Color(0, 140, 0)));
                System.out.println("[PICK-AND-PLACE] Sequence completed successfully.");
            } catch (InterruptedException e) {
                System.err.println("[PICK-AND-PLACE] Sequence interrupted.");
            }
        }, "PickAndPlaceThread").start();
    }
}
