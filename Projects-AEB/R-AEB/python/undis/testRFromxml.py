import math
import xml.etree.ElementTree as ET

import numpy as np


FRONT_XML_PATH = "/home/gq/guoqian/Projects-AEB/M310/front_3d_map.xml"
REAR_XML_PATH = "/home/gq/guoqian/Projects-AEB/M310/rear_3d_map.xml"


def euler_to_rotation_matrix(roll, pitch, yaw):
    """Convert roll, pitch, and yaw angles in radians to a 3x3 rotation matrix."""
    rotation_x = np.array(
        [
            [1, 0, 0],
            [0, math.cos(roll), -math.sin(roll)],
            [0, math.sin(roll), math.cos(roll)],
        ]
    )
    rotation_y = np.array(
        [
            [math.cos(pitch), 0, math.sin(pitch)],
            [0, 1, 0],
            [-math.sin(pitch), 0, math.cos(pitch)],
        ]
    )
    rotation_z = np.array(
        [
            [math.cos(yaw), -math.sin(yaw), 0],
            [math.sin(yaw), math.cos(yaw), 0],
            [0, 0, 1],
        ]
    )
    return (rotation_z @ rotation_y @ rotation_x).T


def _read_camera_values(xml_path):
    root = ET.parse(xml_path).getroot()
    values = {}
    for tag in ("yawangle", "pitchangle", "headangle", "fpt"):
        element = root.find(tag)
        if element is None or element.text is None:
            raise ValueError(f"Missing <{tag}> in {xml_path}")
        try:
            values[tag] = [float(value) for value in element.text.split()]
        except ValueError as error:
            raise ValueError(f"Invalid numeric value in <{tag}> in {xml_path}") from error

    if len(values["yawangle"]) != 1 or len(values["pitchangle"]) != 1 or len(values["headangle"]) != 1:
        raise ValueError(f"Expected one angle value per angle tag in {xml_path}")
    if len(values["fpt"]) < 3:
        raise ValueError(f"Expected at least three values in <fpt> in {xml_path}")
    return values


def testRFromxml(
    front_xml_path=FRONT_XML_PATH,
    rear_xml_path=REAR_XML_PATH,
):
    """Return front/rear rotation matrices and translations in meters."""
    front = _read_camera_values(front_xml_path)
    rear = _read_camera_values(rear_xml_path)

    def rotation(values):
        roll_deg = values["yawangle"][0]
        pitch_deg = values["pitchangle"][0]
        yaw_deg = values["headangle"][0]
        return euler_to_rotation_matrix(
            math.radians(roll_deg),
            math.radians(pitch_deg),
            math.radians(yaw_deg),
        )

    front_fpt = front["fpt"]
    rear_fpt = rear["fpt"]

    front_translation = np.array(
        [
            front_fpt[1] + abs(rear_fpt[1]) - 100,
            front_fpt[0],
            front_fpt[2],
        ],
        dtype=float,
    ) * 0.01
    rear_translation = np.array(
        [-100, rear_fpt[0], rear_fpt[2]],
        dtype=float,
    ) * 0.01

    return rotation(front), rotation(rear), front_translation, rear_translation, front,rear


if __name__ == "__main__":
    front_rotation, rear_rotation, front_translation, rear_translation, front, rear = testRFromxml()
    print("前视旋转矩阵:")
    print(front_rotation)

    print("后视旋转矩阵:")
    print(rear_rotation)

    v_rotated_front = (-front_rotation) @ front_translation
    print("前视平移向量 (m):")
    print(v_rotated_front)
    
    v_rotated_rear = (-rear_rotation) @ rear_translation
    print("后视平移向量 (m):")
    print(v_rotated_rear)

    print("原始数值:x,y,z,yaw,pitch,roll")
    print(front_translation,front["headangle"][0],front["pitchangle"][0],front["yawangle"][0])
    print(rear_translation,rear["headangle"][0],rear["pitchangle"][0],rear["yawangle"][0])
