#!/usr/bin/env python3
"""Generate front/rear JSONC configs, remap tables, meshes, and rectified images."""

from __future__ import annotations

import argparse
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import cv2
import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_FRONT_XML = SCRIPT_DIR.parents[1] / "M83" /"front_3d_map.xml"
DEFAULT_REAR_XML = SCRIPT_DIR.parents[1] / "M83" / "rear_3d_map.xml"
DEFAULT_INTRINSICS_XML = SCRIPT_DIR.parent / "cameraInterParams.xml"
DEFAULT_FRONT_IMAGES = SCRIPT_DIR.parents[1] / "M83" / "front"
DEFAULT_REAR_IMAGES = SCRIPT_DIR.parents[1] / "M83" / "rear"
SCALE_1 = 1.0
SCALE_2 = 0.75 * SCALE_1
OUT_WIDTH = 960
OUT_HEIGHT = 480
CYL_CENTER_U_RATIO = 0.5
CYL_CENTER_V_RATIO = 0.5

# Fixed model/runtime settings shared by front and rear configurations.
FIXED_CONFIG = {
    "model": {
        "name": "lyl",
        "target_width": 960,
        "target_height": 480,
        "target_type": "NV12",
        "num_cls": 8,
        "num_mask": 32,
        "conf_thresh": 0.5,
        "iou_thresh": 0.3,
        "mask_thresh": 0.5,
        "batch_size": 1,
        "mask_up": 4,
        "id2name": {
            "0": "person",
            "1": "bicycle",
            "2": "car",
            "3": "motorcycle",
            "4": "fake person",
            "5": "bus",
            "6": "fake bicycle",
            "7": "truck",
        },
        "whitelist": {
            "0": {"name": "person", "size": [0.40, 0.40, 1.70]},
            "1": {"name": "bicycle", "size": [1.80, 0.60, 1.10]},
            "2": {"name": "car", "size": [4.50, 1.80, 1.50]},
            "3": {"name": "motorcycle", "size": [2.10, 0.80, 1.20]},
            "4": {"name": "fake person", "size": [0.40, 0.40, 1.70]},
            "5": {"name": "bus", "size": [12.00, 2.50, 3.20]},
            "6": {"name": "fake bicycle", "size": [1.80, 0.60, 1.10]},
            "7": {"name": "truck", "size": [8.00, 2.50, 3.50]},
        },
    },
    "other": {"velocity_mode": "ego"},
    "3d_model": {
        "model_spec": {
            "input_w": 512,
            "input_h": 256,
            "bbox_rows": 4,
            "keypoint_count": 8,
            "classes": [
                {"model_id": 0, "output_id": 2, "name": "car", "color_bgr": [107, 107, 255]},
                {"model_id": 1, "output_id": 0, "name": "person", "color_bgr": [43, 146, 255]},
                {"model_id": 2, "output_id": 1, "name": "bike", "color_bgr": [59, 212, 255]},
            ],
            "ground_first": [0, 1, 2, 3],
            "ground_second": [4, 5, 6, 7],
        },
        "postprocess": {"conf": 0.30, "iou": 0.45, "max_det": 10, "ground": "first"},
    },
    "angle_postproc": {
        "enabled": True,
        "column": 6,
        "input_unit": "rad",
        "scale": 1.0,
        "offset": 0.0,
        "invert": False,
        "wrap_mode": "half_pi",
        "decode_mode": "bins_residual",
        "angle_bins": 8,
        "raw_layout": "auto",
        "raw_logits_first": True,
    },
}


def _read_values(path: Path, tag: str, expected: int | None = None) -> list[float]:
    root = ET.parse(path).getroot()
    element = root.find(tag)
    if element is None or element.text is None:
        raise ValueError(f"Missing <{tag}> in {path}")
    try:
        values = [float(value) for value in element.text.split()]
    except ValueError as error:
        raise ValueError(f"Invalid numeric value in <{tag}> in {path}") from error
    if expected is not None and len(values) != expected:
        raise ValueError(f"Expected {expected} values in <{tag}> in {path}, got {len(values)}")
    return values


def _matmul(left: list[list[float]], right: list[list[float]]) -> list[list[float]]:
    return [
        [sum(left[row][k] * right[k][column] for k in range(len(right))) for column in range(len(right[0]))]
        for row in range(len(left))
    ]


def _matvec(matrix: list[list[float]], vector: list[float]) -> list[float]:
    return [sum(row[index] * vector[index] for index in range(len(vector))) for row in matrix]


def _rotation_matrix(roll: float, pitch: float, yaw: float) -> list[list[float]]:
    """Match testRFromxml's transposed Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation_x = [[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]]
    rotation_y = [[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]]
    rotation_z = [[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]]
    combined = _matmul(_matmul(rotation_z, rotation_y), rotation_x)
    return [[combined[column][row] for column in range(3)] for row in range(3)]


def _camera_values(path: Path) -> dict[str, Any]:
    root = ET.parse(path).getroot()
    angles = {}
    for tag in ("yawangle", "pitchangle", "headangle"):
        element = root.find(tag)
        if element is None or element.text is None:
            raise ValueError(f"Missing <{tag}> in {path}")
        try:
            values = [float(value) for value in element.text.split()]
        except ValueError as error:
            raise ValueError(f"Invalid numeric value in <{tag}> in {path}") from error
        if len(values) != 1:
            raise ValueError(f"Expected one value in <{tag}> in {path}, got {len(values)}")
        angles[tag] = values[0]
    return {
        "fpt": _read_values(path, "fpt", expected=3),
        "yawangle": angles["yawangle"],
        "pitchangle": angles["pitchangle"],
        "headangle": angles["headangle"],
        "camera_matrix": _read_values(path, "CameraMatrix/data", expected=9),
    }


def _camera_calib(
    camera: dict[str, Any],
    translation: list[float],
    scale_1: float,
    fisheye_distort: list[float],
    intrinsics: list[float] | None = None,
) -> dict[str, Any]:
    rotation = _rotation_matrix(
        math.radians(camera["yawangle"]),
        math.radians(camera["pitchangle"]),
        math.radians(camera["headangle"]),
    )
    camera_matrix = camera["camera_matrix"] if intrinsics is None else intrinsics
    center_u = camera_matrix[2] / scale_1
    center_v = camera_matrix[5] / scale_1
    return {
        "world_x": translation[0],
        "world_y": translation[1],
        "world_z": translation[2],
        "yaw": camera["headangle"],
        "pitch": camera["pitchangle"],
        "roll": camera["yawangle"],
        "angle_type": "degree",
        "image_width": 2 * center_u,
        "image_height": 2 * center_v,
        "focal_u": camera_matrix[0] / scale_1,
        "focal_v": camera_matrix[4] / scale_1,
        "center_u": center_u,
        "center_v": center_v,
        "fov": 204,
        "image_type": "NV12",
        "type": "fisheye",
        "pinhole_distort": [],
        "fisheye_distort": list(fisheye_distort),
        "R_act": rotation,
        "t_act": [-value for value in _matvec(rotation, translation)],
    }


def _rot_x(angle: float) -> list[list[float]]:
    cosine, sine = math.cos(angle), math.sin(angle)
    return [[1.0, 0.0, 0.0], [0.0, cosine, -sine], [0.0, sine, cosine]]


def _rot_y(angle: float) -> list[list[float]]:
    cosine, sine = math.cos(angle), math.sin(angle)
    return [[cosine, 0.0, sine], [0.0, 1.0, 0.0], [-sine, 0.0, cosine]]


def _cyl_rotation(calib: dict[str, Any]) -> list[list[float]]:
    return _matmul(
        _rot_y(math.radians(float(calib["pitch"]))),
        _rot_x(math.radians(float(calib["roll"]))),
    )


def _cyl_calib(calib: dict[str, Any]) -> dict[str, Any]:
    camera_to_cyl = _cyl_rotation(calib)
    focal = float(calib["focal_u"]) * SCALE_2
    canvas_u = OUT_WIDTH * CYL_CENTER_U_RATIO
    canvas_v = OUT_HEIGHT * CYL_CENTER_V_RATIO
    ray = _matvec(camera_to_cyl, [1.0, 0.0, 0.0])
    kxy = math.hypot(ray[0], ray[1])
    if kxy <= 1e-12:
        raise ValueError("original camera optical axis is degenerate in cylinder coordinates")
    center_u = canvas_u + focal * math.atan2(-ray[1], ray[0])
    center_v = canvas_v - focal * ray[2] / kxy
    rotation = calib["R_act"]
    translation = calib["t_act"]
    return {
        "type": "cyl",
        "image_type": "NV12",
        "image_width": OUT_WIDTH,
        "image_height": OUT_HEIGHT,
        "focal_u": focal,
        "focal_v": focal,
        "center_u": center_u,
        "center_v": center_v,
        "hfov_deg": math.degrees(OUT_WIDTH / focal),
        "vfov_deg": math.degrees(2.0 * math.atan(OUT_HEIGHT / (2.0 * focal))),
        "R_cam_to_cyl": camera_to_cyl,
        "R_cyl_to_cam": [[camera_to_cyl[column][row] for column in range(3)] for row in range(3)],
        "R_act": _matmul(camera_to_cyl, rotation),
        "t_act": _matvec(camera_to_cyl, translation),
    }


def _make_config(calib: dict[str, Any]) -> dict[str, Any]:
    return {
        "fisheye": calib,
        "cyl": _cyl_calib(calib),
        **FIXED_CONFIG,
    }


def generate_configs(
    front_xml: Path,
    rear_xml: Path,
    intrinsics_xml: Path,
    output_dir: Path,
) -> tuple[dict[str, Any], dict[str, Any]]:
    front = _camera_values(front_xml)
    rear = _camera_values(rear_xml)
    fisheye_distort = _read_values(
        intrinsics_xml,
        "distcoef_angle_to_realheight/data",
        expected=4,
    )
    front_translation = [
        (front["fpt"][1] + abs(rear["fpt"][1]) - 100.0) * 0.01,
        front["fpt"][0] * 0.01,
        front["fpt"][2] * 0.01,
    ]
    rear_translation = [-1.0, rear["fpt"][0] * 0.01, rear["fpt"][2] * 0.01]
    front_calib = _camera_calib(front, front_translation, SCALE_1, fisheye_distort)
    rear_calib = _camera_calib(
        rear,
        rear_translation,
        SCALE_1,
        fisheye_distort,
        front["camera_matrix"],
    )

    output_dir.mkdir(parents=True, exist_ok=True)
    for filename, calib in (
        ("configFront.jsonc", front_calib),
        ("configRear.jsonc", rear_calib),
    ):
        config = _make_config(calib)
        (output_dir / filename).write_text(
            "// R-AEB 任务配置文件\n"
            + json.dumps(config, indent=4, ensure_ascii=False)
            + "\n",
            encoding="utf-8",
        )
    return front_calib, rear_calib


def _build_map_uv_float(
    u_src: np.ndarray,
    v_src: np.ndarray,
    width_src: int,
    height_src: int,
    invalid: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    out_of_bounds = (
        (u_src < 0.0)
        | (u_src > width_src - 1)
        | (v_src < 0.0)
        | (v_src > height_src - 1)
    )
    map_uv = np.stack((u_src.astype(np.float32), v_src.astype(np.float32)), axis=-1)
    return map_uv, (~(invalid | out_of_bounds)).astype(np.float32)


def _build_map_fish_to_cyl(calib: dict[str, Any]) -> tuple[np.ndarray, tuple[int, int]]:
    width_src, height_src = int(calib["image_width"]), int(calib["image_height"])
    fu0, fv0 = float(calib["focal_u"]), float(calib["focal_v"])
    cu0, cv0 = float(calib["center_u"]), float(calib["center_v"])
    k1, k2, k3, k4 = [float(value) for value in calib["fisheye_distort"]]
    fu, fv, cu, cv = fu0 * SCALE_2, fv0 * SCALE_2, cu0 * SCALE_2, cv0 * SCALE_2
    focal = fu
    center_u, center_v = OUT_WIDTH * CYL_CENTER_U_RATIO, OUT_HEIGHT * CYL_CENTER_V_RATIO
    uu, vv = np.meshgrid(
        np.arange(OUT_WIDTH, dtype=np.float32),
        np.arange(OUT_HEIGHT, dtype=np.float32),
    )
    phi = (uu - center_u) / focal
    vertical = (center_v - vv) / focal
    cyl_directions = np.stack(
        (np.cos(phi), -np.sin(phi), vertical), axis=-1
    ).astype(np.float64)
    camera_directions = cyl_directions @ np.asarray(_cyl_rotation(calib))
    x, y, z = (camera_directions[..., axis] for axis in range(3))
    perpendicular = np.sqrt(y * y + z * z)
    theta = np.arctan2(perpendicular, x)
    theta2 = theta * theta
    theta4 = theta2 * theta2
    theta6 = theta4 * theta2
    theta8 = theta4 * theta4
    distance = theta * (
        1.0 + k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8
    )
    inverse_perpendicular = np.where(perpendicular < 1e-6, 0.0, 1.0 / perpendicular)
    u_scaled = cu - fu * distance * y * inverse_perpendicular
    v_scaled = cv - fv * distance * z * inverse_perpendicular
    u_full, v_full = u_scaled / SCALE_2, v_scaled / SCALE_2
    map_uv, _ = _build_map_uv_float(
        u_full, v_full, width_src, height_src, x <= 0.0
    )
    return map_uv, (OUT_WIDTH, OUT_HEIGHT)


def _save_map_uv(
    map_uv: np.ndarray,
    dst_wh: tuple[int, int],
    prefix: str,
    output_dir: Path,
) -> Path:
    width, height = dst_wh
    if map_uv.shape != (height, width, 2):
        raise ValueError(f"map size mismatch: {map_uv.shape} vs {(height, width, 2)}")
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / f"{prefix}_{width}x{height}_uv.bin"
    np.ascontiguousarray(map_uv[:, :, 0], dtype=np.float32).tofile(path)
    with path.open("ab") as stream:
        np.ascontiguousarray(map_uv[:, :, 1], dtype=np.float32).tofile(stream)
    return path


def _load_map_uv(path: Path, width: int, height: int) -> tuple[np.ndarray, np.ndarray]:
    buffer = np.fromfile(path, dtype=np.float32)
    expected = 2 * width * height
    if buffer.size != expected:
        raise ValueError(f"bin size mismatch: got {buffer.size}, expected {expected}")
    count = width * height
    return buffer[:count].reshape(height, width), buffer[count:].reshape(height, width)


def _apply_map(
    image: np.ndarray,
    u: np.ndarray,
    v: np.ndarray,
) -> np.ndarray:
    height, width = image.shape[:2]
    valid = (
        np.isfinite(u)
        & np.isfinite(v)
        & (u >= 0.0)
        & (u <= width - 1)
        & (v >= 0.0)
        & (v <= height - 1)
    )
    result = cv2.remap(
        image,
        np.nan_to_num(u, nan=-1.0).astype(np.float32),
        np.nan_to_num(v, nan=-1.0).astype(np.float32),
        cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=(0, 0, 0),
    )
    result[~valid] = 0
    return result


def _matlab_round_to_int(values: np.ndarray) -> np.ndarray:
    return (np.sign(values) * np.floor(np.abs(values) + 0.5)).astype(np.int32)


def _save_mesh_txt(
    calib: dict[str, Any],
    dst_wh: tuple[int, int],
    output_path: Path,
    m: int = 4,
) -> Path:
    width, height = dst_wh
    step = 1 << m
    width_src, height_src = int(calib["image_width"]), int(calib["image_height"])
    fu0, fv0 = float(calib["focal_u"]), float(calib["focal_v"])
    cu0, cv0 = float(calib["center_u"]), float(calib["center_v"])
    k1, k2, k3, k4 = [float(value) for value in calib["fisheye_distort"]]
    fu, fv, cu, cv = fu0 * SCALE_2, fv0 * SCALE_2, cu0 * SCALE_2, cv0 * SCALE_2
    focal = fu
    center_u, center_v = width * CYL_CENTER_U_RATIO, height * CYL_CENTER_V_RATIO
    xs = np.arange(0, width + 1, step, dtype=np.float64)
    ys = np.arange(0, height + 1, step, dtype=np.float64)
    horizontal, vertical = np.meshgrid(xs, ys)
    phi = (horizontal - center_u) / focal
    height_angle = (center_v - vertical) / focal
    directions = np.stack(
        (np.cos(phi), -np.sin(phi), height_angle), axis=-1
    ) @ np.asarray(_cyl_rotation(calib))
    x, y, z = (directions[..., axis] for axis in range(3))
    perpendicular = np.sqrt(y * y + z * z)
    theta = np.arctan2(perpendicular, x)
    theta2 = theta * theta
    theta4 = theta2 * theta2
    theta6 = theta4 * theta2
    theta8 = theta4 * theta4
    distance = theta * (
        1.0 + k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8
    )
    inverse_perpendicular = np.where(perpendicular < 1e-6, 0.0, 1.0 / perpendicular)
    u_scaled = cu - fu * distance * y * inverse_perpendicular
    v_scaled = cv - fv * distance * z * inverse_perpendicular
    u_full, v_full = u_scaled / SCALE_2, v_scaled / SCALE_2
    horizontal_delta = _matlab_round_to_int((u_full - horizontal) * 8.0).T
    vertical_delta = _matlab_round_to_int((v_full - vertical) * 8.0).T
    mesh = np.column_stack(
        (horizontal_delta.reshape(-1, order="F"), vertical_delta.reshape(-1, order="F"))
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(output_path, mesh, fmt="%d", delimiter=" ")
    print(f"Saved mesh: {output_path} ({mesh.shape[0]} rows)")
    return output_path


def _rectify_images(
    image_dir: Path,
    output_dir: Path,
    calib: dict[str, Any],
    u: np.ndarray,
    v: np.ndarray,
) -> None:
    image_paths = sorted(image_dir.glob("*.jpg"))
    output_dir.mkdir(parents=True, exist_ok=True)
    for image_path in image_paths:
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError(f"Failed to read image: {image_path}")
        image = cv2.resize(
            image,
            (int(calib["image_width"]), int(calib["image_height"])),
            interpolation=cv2.INTER_LINEAR,
        )
        rectified = _apply_map(image, u, v)
        output_path = output_dir / image_path.name
        if not cv2.imwrite(str(output_path), rectified):
            raise OSError(f"Failed to write rectified image: {output_path}")
        print(f"Saved rectified image: {output_path}")


def generate_maps_and_rectify(
    calib: dict[str, Any],
    camera_name: str,
    maps_dir: Path,
    image_dir: Path,
    rectified_dir: Path,
) -> None:
    camera_maps_dir = maps_dir / camera_name
    map_uv, dst_wh = _build_map_fish_to_cyl(calib)
    bin_path = _save_map_uv(
        map_uv, dst_wh, prefix=f"{camera_name}_visper", output_dir=camera_maps_dir
    )
    _save_mesh_txt(
        calib, dst_wh, camera_maps_dir / "mesh4.txt", m=4
    )
    u, v = _load_map_uv(bin_path, *dst_wh)
    _rectify_images(image_dir, rectified_dir, calib, u, v)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--front-xml", type=Path, default=DEFAULT_FRONT_XML)
    parser.add_argument("--rear-xml", type=Path, default=DEFAULT_REAR_XML)
    parser.add_argument("--intrinsics-xml", type=Path, default=DEFAULT_INTRINSICS_XML)
    parser.add_argument("--output-dir", type=Path, default=SCRIPT_DIR)
    parser.add_argument("--maps-dir", type=Path, default=SCRIPT_DIR / "maps")
    parser.add_argument("--front-images", type=Path, default=DEFAULT_FRONT_IMAGES)
    parser.add_argument("--rear-images", type=Path, default=DEFAULT_REAR_IMAGES)
    parser.add_argument("--front-rectified-dir", type=Path, default=SCRIPT_DIR / "outF")
    parser.add_argument("--rear-rectified-dir", type=Path, default=SCRIPT_DIR / "outR")
    args = parser.parse_args()

    front_calib, rear_calib = generate_configs(
        args.front_xml, args.rear_xml, args.intrinsics_xml, args.output_dir
    )
    generate_maps_and_rectify(
        front_calib, "front", args.maps_dir, args.front_images, args.front_rectified_dir
    )
    generate_maps_and_rectify(
        rear_calib, "rear", args.maps_dir, args.rear_images, args.rear_rectified_dir
    )
    print(f"Generated {args.output_dir / 'configFront.jsonc'}")
    print(f"Generated {args.output_dir / 'configRear.jsonc'}")


if __name__ == "__main__":
    main()
