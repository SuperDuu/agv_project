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
    return f"{value:.6f}"


def xyz(values):
    return " ".join(fmt(v) for v in values)


def add_material(robot, name, rgba):
    material = ET.SubElement(robot, "material", {"name": name})
    ET.SubElement(material, "color", {"rgba": rgba})


def add_link(robot, name, length=0.02, radius=0.006, color="dark_gray"):
    link = ET.SubElement(robot, "link", {"name": name})

    visual = ET.SubElement(link, "visual")
    ET.SubElement(visual, "origin", {"xyz": xyz([length / 2.0, 0.0, 0.0]), "rpy": "0 0 0"})
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(geometry, "box", {"size": f"{fmt(max(length, 0.005))} {fmt(radius * 2.0)} {fmt(radius * 2.0)}"})
    ET.SubElement(visual, "material", {"name": color})

    collision = ET.SubElement(link, "collision")
    ET.SubElement(collision, "origin", {"xyz": xyz([length / 2.0, 0.0, 0.0]), "rpy": "0 0 0"})
    collision_geometry = ET.SubElement(collision, "geometry")
    ET.SubElement(collision_geometry, "box", {"size": f"{fmt(max(length, 0.005))} {fmt(radius * 2.0)} {fmt(radius * 2.0)}"})

    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "mass", {"value": "0.1"})
    ET.SubElement(inertial, "origin", {"xyz": xyz([length / 2.0, 0.0, 0.0]), "rpy": "0 0 0"})
    ET.SubElement(inertial, "inertia", {
        "ixx": "0.0001",
        "ixy": "0.0",
        "ixz": "0.0",
        "iyy": "0.0001",
        "iyz": "0.0",
        "izz": "0.0001",
    })
    return link


def add_simple_link(robot, name, color="dark_gray"):
    return add_link(robot, name, length=0.01, radius=0.006, color=color)


def add_joint(robot, name, joint_type, parent, child, origin_xyz, axis=None, limit=None):
    joint = ET.SubElement(robot, "joint", {"name": name, "type": joint_type})
    ET.SubElement(joint, "parent", {"link": parent})
    ET.SubElement(joint, "child", {"link": child})
    ET.SubElement(joint, "origin", {"xyz": xyz(origin_xyz), "rpy": "0 0 0"})
    if axis is not None:
        ET.SubElement(joint, "axis", {"xyz": xyz(axis)})
    if limit is not None:
        ET.SubElement(joint, "limit", {
            "lower": fmt(deg(limit["lower"])),
            "upper": fmt(deg(limit["upper"])),
            "velocity": fmt(limit.get("velocity", 1.0)),
            "effort": fmt(limit.get("effort", 5.0)),
        })
    return joint


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
    shoulder_y = sign * mm(base["shoulder_y_offset"])
    shoulder_z = mm(base["shoulder_z"])
    lengths = arm_lengths(config)

    shoulder_link = f"{prefix}_shoulder_link"
    add_simple_link(robot, shoulder_link, color=color)
    add_joint(robot, f"{prefix}_shoulder_mount", "fixed", "torso_link", shoulder_link, [0.0, shoulder_y, shoulder_z])

    parent = shoulder_link
    axes = [
        [0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0],
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
    ]

    for index in range(6):
        link_name = f"{prefix}_link_{index + 1}"
        length = lengths[index]
        add_link(robot, link_name, length=length, radius=radius, color=color)
        origin = [0.0, 0.0, 0.0] if index == 0 else [lengths[index - 1], 0.0, 0.0]
        add_joint(
            robot,
            f"{prefix}_joint_{index + 1}",
            "revolute",
            parent,
            link_name,
            origin,
            axis=axes[index],
            limit=limits[f"joint_{index + 1}"],
        )
        parent = link_name

    tool_link = f"{prefix}_tool0"
    add_simple_link(robot, tool_link, color=color)
    add_joint(robot, f"{prefix}_tool0_fixed", "fixed", parent, tool_link, [lengths[-1], 0.0, 0.0])


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
