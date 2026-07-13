import argparse
import math
from pathlib import Path
from xml.dom import minidom
from xml.etree import ElementTree as ET

import yaml


def mm(value):
    return float(value) / 1000.0


def deg(value):
    return math.radians(float(value))


def fmt(value):
    return f"{value:.17g}"


def xyz(values):
    return " ".join(fmt(v) for v in values)


def mat_mult(A, B):
    C = [[0.0]*4 for _ in range(4)]
    for i in range(4):
        for j in range(4):
            C[i][j] = sum(A[i][k] * B[k][j] for k in range(4))
    return C


def Rx(a):
    ca, sa = math.cos(a), math.sin(a)
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, ca,  -sa, 0.0],
        [0.0, sa,  ca,  0.0],
        [0.0, 0.0, 0.0, 1.0]
    ]


def Rz(a):
    ca, sa = math.cos(a), math.sin(a)
    return [
        [ca,  -sa, 0.0, 0.0],
        [sa,  ca,  0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ]


def Tx(d):
    M = [[0.0]*4 for _ in range(4)]
    for i in range(4): M[i][i] = 1.0
    M[0][3] = d
    return M


def Tz(d):
    M = [[0.0]*4 for _ in range(4)]
    for i in range(4): M[i][i] = 1.0
    M[2][3] = d
    return M


def extract_xyz_rpy(T):
    x, y, z = T[0][3], T[1][3], T[2][3]
    sy = math.sqrt(T[2][1]**2 + T[2][2]**2)
    singular = sy < 1e-6
    if not singular:
        r = math.atan2(T[2][1], T[2][2])
        p = math.atan2(-T[2][0], sy)
        yaw = math.atan2(T[1][0], T[0][0])
    else:
        r = math.atan2(-T[1][2], T[1][1])
        p = math.atan2(-T[2][0], sy)
        yaw = 0.0
    return [x, y, z], [r, p, yaw]


def add_material(robot, name, rgba):
    material = ET.SubElement(robot, "material", {"name": name})
    ET.SubElement(material, "color", {"rgba": rgba})


def add_link_custom(robot, name, size, origin_xyz, color="dark_gray"):
    link = ET.SubElement(robot, "link", {"name": name})

    visual = ET.SubElement(link, "visual")
    ET.SubElement(visual, "origin", {"xyz": xyz(origin_xyz), "rpy": "0 0 0"})
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(geometry, "box", {"size": xyz(size)})
    ET.SubElement(visual, "material", {"name": color})

    collision = ET.SubElement(link, "collision")
    ET.SubElement(collision, "origin", {"xyz": xyz(origin_xyz), "rpy": "0 0 0"})
    collision_geometry = ET.SubElement(collision, "geometry")
    ET.SubElement(collision_geometry, "box", {"size": xyz(size)})

    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "mass", {"value": "0.1"})
    ET.SubElement(inertial, "origin", {"xyz": xyz(origin_xyz), "rpy": "0 0 0"})
    ET.SubElement(inertial, "inertia", {
        "ixx": "0.0001",
        "ixy": "0.0",
        "ixz": "0.0",
        "iyy": "0.0001",
        "iyz": "0.0",
        "izz": "0.0001",
    })
    return link


def add_link(robot, name, length=0.02, radius=0.006, color="dark_gray"):
    return add_link_custom(robot, name, [length, radius * 2.0, radius * 2.0], [length / 2.0, 0.0, 0.0], color=color)


def add_simple_link(robot, name, color="dark_gray"):
    return add_link(robot, name, length=0.005, radius=0.006, color=color)


def add_joint_rpy(robot, name, joint_type, parent, child, origin_xyz, origin_rpy, axis=None, limit=None):
    joint = ET.SubElement(robot, "joint", {"name": name, "type": joint_type})
    ET.SubElement(joint, "parent", {"link": parent})
    ET.SubElement(joint, "child", {"link": child})
    ET.SubElement(joint, "origin", {"xyz": xyz(origin_xyz), "rpy": xyz(origin_rpy)})
    if axis is not None:
        ET.SubElement(joint, "axis", {"xyz": xyz(axis)})
    if limit is not None:
        ET.SubElement(joint, "limit", {
            "lower": fmt(deg(limit["lower"] - 0.01)),
            "upper": fmt(deg(limit["upper"] + 0.01)),
            "velocity": fmt(limit.get("velocity", 1.0)),
            "effort": fmt(limit.get("effort", 5.0)),
        })
    return joint


def add_joint(robot, name, joint_type, parent, child, origin_xyz, axis=None, limit=None):
    return add_joint_rpy(robot, name, joint_type, parent, child, origin_xyz, [0.0, 0.0, 0.0], axis, limit)


def arm_lengths(config):
    links = config["links"]
    return [
        mm(links["L2"] + links["L3"]),
        mm(links["L4"]),
        mm(links["L5"]),
        mm(links["L6"]),
        mm(links["L7"]),
        mm(config["visual"].get("gripper_length", 20.0)),
    ]


def add_arm(robot, config, side):
    is_right = side == "right"
    sign = -1.0 if is_right else 1.0
    color = "right_blue" if is_right else "left_red"
    prefix = "right" if is_right else "left"
    limits = config["joint_limits"][prefix]
    base = config["base"]
    visual = config["visual"]
    radius = mm(visual.get("link_radius", 6.0))

    links = config["links"]
    L0 = mm(links["L0"])
    L1 = mm(links["L1"])
    L2 = mm(links["L2"])
    L3 = mm(links["L3"])
    L4 = mm(links["L4"])
    L5 = mm(links["L5"])
    L6 = mm(links["L6"])
    L7 = mm(links["L7"])

    d2 = (L2 + L3) if is_right else -(L2 + L3)

    shoulder_y = sign * mm(base["shoulder_y_offset"])
    shoulder_z = mm(base["shoulder_z"])

    shoulder_link = f"{prefix}_shoulder_link"
    add_simple_link(robot, shoulder_link, color=color)
    add_joint_rpy(robot, f"{prefix}_shoulder_mount", "fixed", "torso_link", shoulder_link, [0.0, shoulder_y, shoulder_z], [0.0, 0.0, 0.0])

    T_orig_1 = mat_mult(Rz(-math.pi / 2), Tz(L1 + L0))
    T_orig_2 = mat_mult(mat_mult(Rx(-math.pi / 2), Rz(-math.pi / 2)), Tz(d2))
    T_orig_3 = mat_mult(Rx(-math.pi / 2), Rz(-math.pi))
    T_orig_4 = mat_mult(Tx(L4), Rz(-math.pi / 2))
    T_orig_5 = mat_mult(Rx(-math.pi / 2), Tz(L5 + L6))
    T_orig_6 = Rx(-math.pi / 2)
    T_tool = [
        [0.0, -1.0,  0.0, 0.0],
        [0.0,  0.0, -1.0, -L7],
        [1.0,  0.0,  0.0, 0.0],
        [0.0,  0.0,  0.0, 1.0]
    ]

    origins = [T_orig_1, T_orig_2, T_orig_3, T_orig_4, T_orig_5, T_orig_6]

    parent = shoulder_link
    for index in range(6):
        link_name = f"{prefix}_link_{index + 1}"
        r2 = radius * 2.0
        if index == 0:
            l_val = L1 + L0
            size = [r2, r2, max(l_val, 0.005)]
            origin_xyz = [0.0, 0.0, l_val / 2.0]
        elif index == 1:
            l_val = abs(d2)
            size = [r2, r2, max(l_val, 0.005)]
            origin_xyz = [0.0, 0.0, d2 / 2.0]
        elif index == 2:
            size = [r2, r2, r2]
            origin_xyz = [0.0, 0.0, 0.0]
        elif index == 3:
            l_val = L4
            size = [max(l_val, 0.005), r2, r2]
            origin_xyz = [l_val / 2.0, 0.0, 0.0]
        elif index == 4:
            l_val = L5 + L6
            size = [r2, r2, max(l_val, 0.005)]
            origin_xyz = [0.0, 0.0, l_val / 2.0]
        elif index == 5:
            size = [r2, r2, r2]
            origin_xyz = [0.0, 0.0, 0.0]

        add_link_custom(robot, link_name, size, origin_xyz, color=color)

        T = origins[index]
        o_xyz, o_rpy = extract_xyz_rpy(T)

        if (not is_right) and (index == 5):
            axis = [0.0, 0.0, -1.0]
        else:
            axis = [0.0, 0.0, 1.0]

        add_joint_rpy(
            robot,
            f"{prefix}_joint_{index + 1}",
            "revolute",
            parent,
            link_name,
            o_xyz,
            o_rpy,
            axis=axis,
            limit=limits[f"joint_{index + 1}"],
        )
        parent = link_name

    tool_link = f"{prefix}_tool0"
    add_simple_link(robot, tool_link, color=color)
    o_xyz, o_rpy = extract_xyz_rpy(T_tool)
    add_joint_rpy(robot, f"{prefix}_tool0_fixed", "fixed", parent, tool_link, o_xyz, o_rpy)


def generate(config):
    robot = ET.Element("robot", {"name": config["robot"]["name"]})
    add_material(robot, "dark_gray", "0.2 0.22 0.24 1.0")
    add_material(robot, "right_blue", "0.1 0.35 0.8 1.0")
    add_material(robot, "left_red", "0.8 0.2 0.2 1.0")

    add_link(robot, "base_link", length=0.08, radius=0.04, color="dark_gray")
    add_link(robot, "torso_link", length=0.04, radius=0.03, color="dark_gray")
    add_joint(robot, "base_to_torso", "fixed", "base_link", "torso_link", [0.0, 0.0, mm(config["base"]["torso_height"]) / 2.0])

    add_arm(robot, config, "right")
    add_arm(robot, config, "left")
    return robot


def pretty_xml(root):
    rough = ET.tostring(root, encoding="utf-8")
    parsed = minidom.parseString(rough)
    return parsed.toprettyxml(indent="  ")


def main():
    parser = argparse.ArgumentParser(description="Generate AGV dual-arm URDF from YAML parameters.")
    parser.add_argument("--config", required=True, help="Path to agv_arm_geometry.yaml")
    parser.add_argument("--output", help="Optional output URDF path. Prints to stdout when omitted.")
    args = parser.parse_args()

    with open(args.config, "r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)

    xml_text = pretty_xml(generate(config))
    if args.output:
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(xml_text, encoding="utf-8")
    else:
        print(xml_text)


if __name__ == "__main__":
    main()
