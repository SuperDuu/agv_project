import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtWidgets import (
    QApplication,
    QGridLayout,
    QGroupBox,
    QLabel,
    QMainWindow,
    QSlider,
    QVBoxLayout,
    QWidget,
)


Q3_RIGHT_MIN = 0
Q3_RIGHT_MAX = 145
Q4_RIGHT_MIN_OFFSET = -60
Q4_RIGHT_MAX_OFFSET = 20

Q3_LEFT_MIN = -145
Q3_LEFT_MAX = 0
Q4_LEFT_MIN_OFFSET = -20
Q4_LEFT_MAX_OFFSET = 60


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def deg_to_rad(value):
    return math.radians(float(value))


def right_actuator_to_joint(q3, q4):
    theta3 = q3 + 20
    theta4 = q4 - q3 - 35
    return theta3, theta4


def left_actuator_to_joint(q3, q4):
    theta3 = q3 - 20
    theta4 = q4 - q3 + 35
    return theta3, theta4


def actuator_to_ros_joint(q3, q4):
    return q3, q4 - q3


class JointStatePublisher(Node):
    def __init__(self):
        super().__init__("java_style_joint_gui")
        self.publisher = self.create_publisher(JointState, "joint_states", 10)
        self.joint_names = [
            "right_joint_1",
            "right_joint_2",
            "right_joint_3",
            "right_joint_4",
            "right_joint_5",
            "right_joint_6",
            "left_joint_1",
            "left_joint_2",
            "left_joint_3",
            "left_joint_4",
            "left_joint_5",
            "left_joint_6",
        ]
        self.values_deg = {
            "shared_q1": 0,
            "right_q2": 0,
            "right_q3": 0,
            "right_q4": 0,
            "right_q5": 0,
            "right_q6": 0,
            "left_q2": 0,
            "left_q3": 0,
            "left_q4": 0,
            "left_q5": 0,
            "left_q6": 0,
        }

    def publish_state(self):
        right_theta3, right_theta4 = actuator_to_ros_joint(
            self.values_deg["right_q3"],
            self.values_deg["right_q4"],
        )
        left_theta3, left_theta4 = actuator_to_ros_joint(
            self.values_deg["left_q3"],
            self.values_deg["left_q4"],
        )

        positions_deg = [
            self.values_deg["shared_q1"],
            self.values_deg["right_q2"],
            right_theta3,
            right_theta4,
            self.values_deg["right_q5"],
            self.values_deg["right_q6"],
            self.values_deg["shared_q1"],
            self.values_deg["left_q2"],
            left_theta3,
            left_theta4,
            self.values_deg["left_q5"],
            self.values_deg["left_q6"],
        ]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [deg_to_rad(value) for value in positions_deg]
        self.publisher.publish(msg)


class JointSlider:
    def __init__(self, parent, layout, row, label, minimum, maximum, initial, on_change):
        self.name_label = QLabel(label)
        self.value_label = QLabel()
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(minimum, maximum)
        self.slider.setValue(initial)
        self.slider.valueChanged.connect(on_change)
        layout.addWidget(self.name_label, row, 0)
        layout.addWidget(self.slider, row, 1)
        layout.addWidget(self.value_label, row, 2)
        self.update_label()

    def value(self):
        return self.slider.value()

    def set_range(self, minimum, maximum):
        self.slider.setRange(minimum, maximum)
        self.update_label()

    def set_value(self, value):
        self.slider.setValue(value)
        self.update_label()

    def update_label(self, text=None):
        if text is None:
            text = f"{self.slider.value()} deg"
        self.value_label.setText(text)


class JavaStyleJointWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.updating = False
        self.sliders = {}

        self.setWindowTitle("AGV Java-Style Joint State Publisher")
        root = QWidget()
        root_layout = QVBoxLayout(root)

        shared_group = QGroupBox("Shared base yaw")
        shared_layout = QGridLayout(shared_group)
        self.sliders["shared_q1"] = JointSlider(
            self,
            shared_layout,
            0,
            "Joint 1",
            -45,
            45,
            0,
            lambda value: self.set_value("shared_q1", value),
        )
        root_layout.addWidget(shared_group)

        root_layout.addWidget(self.create_arm_group("right", "Right arm"))
        root_layout.addWidget(self.create_arm_group("left", "Left arm"))
        self.setCentralWidget(root)

        self.refresh_dynamic_ranges("right")
        self.refresh_dynamic_ranges("left")
        self.refresh_labels()
        self.node.publish_state()

    def create_arm_group(self, arm, title):
        group = QGroupBox(title)
        layout = QGridLayout(group)
        specs = [
            ("q2", "Joint 2", -90, 90, 0),
            ("q3", "Actuator q3", Q3_RIGHT_MIN if arm == "right" else Q3_LEFT_MIN, Q3_RIGHT_MAX if arm == "right" else Q3_LEFT_MAX, 0),
            ("q4", "Actuator q4", -60 if arm == "right" else -20, 20 if arm == "right" else 60, 0),
            ("q5", "Joint 5", -90, 90, 0),
            ("q6", "Joint 6", 0, 90, 0),
        ]
        for row, (joint, label, minimum, maximum, initial) in enumerate(specs):
            key = f"{arm}_{joint}"
            self.sliders[key] = JointSlider(
                self,
                layout,
                row,
                label,
                minimum,
                maximum,
                initial,
                lambda value, slider_key=key: self.set_value(slider_key, value),
            )
        return group

    def set_value(self, key, value):
        if self.updating:
            return
        self.node.values_deg[key] = value
        if key in ("right_q3", "right_q4"):
            self.refresh_dynamic_ranges("right")
        elif key in ("left_q3", "left_q4"):
            self.refresh_dynamic_ranges("left")
        self.refresh_labels()
        self.node.publish_state()

    def refresh_dynamic_ranges(self, arm):
        self.updating = True
        try:
            if arm == "right":
                q3_key, q4_key = "right_q3", "right_q4"
                q3_min_base, q3_max_base = Q3_RIGHT_MIN, Q3_RIGHT_MAX
                q4_min_offset, q4_max_offset = Q4_RIGHT_MIN_OFFSET, Q4_RIGHT_MAX_OFFSET
            else:
                q3_key, q4_key = "left_q3", "left_q4"
                q3_min_base, q3_max_base = Q3_LEFT_MIN, Q3_LEFT_MAX
                q4_min_offset, q4_max_offset = Q4_LEFT_MIN_OFFSET, Q4_LEFT_MAX_OFFSET

            q3 = self.node.values_deg[q3_key]
            q4 = self.node.values_deg[q4_key]

            q4_min = q3 + q4_min_offset
            q4_max = q3 + q4_max_offset
            q4 = clamp(q4, q4_min, q4_max)
            self.node.values_deg[q4_key] = q4
            self.sliders[q4_key].set_range(q4_min, q4_max)
            self.sliders[q4_key].set_value(q4)

            q3_min = max(q3_min_base, q4 - q4_max_offset)
            q3_max = min(q3_max_base, q4 - q4_min_offset)
            q3 = clamp(q3, q3_min, q3_max)
            self.node.values_deg[q3_key] = q3
            self.sliders[q3_key].set_range(q3_min, q3_max)
            self.sliders[q3_key].set_value(q3)
        finally:
            self.updating = False

    def refresh_labels(self):
        for key, slider in self.sliders.items():
            if key in ("right_q3", "right_q4"):
                theta3, theta4 = right_actuator_to_joint(
                    self.node.values_deg["right_q3"],
                    self.node.values_deg["right_q4"],
                )
                theta = theta3 if key == "right_q3" else theta4
                slider.update_label(f"{slider.value()} deg -> theta {theta:.0f} deg")
            elif key in ("left_q3", "left_q4"):
                theta3, theta4 = left_actuator_to_joint(
                    self.node.values_deg["left_q3"],
                    self.node.values_deg["left_q4"],
                )
                theta = theta3 if key == "left_q3" else theta4
                slider.update_label(f"{slider.value()} deg -> theta {theta:.0f} deg")
            else:
                slider.update_label()


def main():
    rclpy.init(args=None)
    app = QApplication(sys.argv)
    node = JointStatePublisher()
    window = JavaStyleJointWindow(node)
    window.resize(720, 420)
    window.show()

    spin_timer = QTimer()
    spin_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    spin_timer.start(20)

    publish_timer = QTimer()
    publish_timer.timeout.connect(node.publish_state)
    publish_timer.start(100)

    exit_code = app.exec_()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
