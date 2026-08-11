# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Any
from pathlib import Path
import math
import numpy as np
import cv2

SCALE_1 = 1  # 控制输入鱼眼图尺寸

# -------------------- 常量 --------------------
# CALIB: dict[str, Any] = {
#     "pitch": 27.829443983736383,
#     "yaw": 177.76353058382546,
#     "roll": -0.9230713233313438,
#     "world_x": -1.0632968236,
#     "world_y": 0.0209697906,
#     "world_z": 0.7299492368,
#
#     "image_width": 1392,
#     "image_height": 976,
#     "focal_u": 364.21 / SCALE_1,
#     "focal_v": 364.08 / SCALE_1,
#     "center_u": 696.0 / SCALE_1,
#     "center_v": 486.0 / SCALE_1,
#     "fisheye_distort": [0.105764, -0.021087, -0.000328, 0.000021],
#
#     "R_act": [
#         [-0.88366759, 0.034510393, -0.46684116],
#         [-0.031503726, -0.99940211, -0.014246681],
#         [-0.46705368, 0.0021179055, 0.88422644],
#     ],
#     "t_act": [-59.955425, -0.21412054, -114.21015],
# }


# # 2026/6/2 Rt
# # 平移字段统一存米：原始 Rt 的平移是厘米，这里的 world_x/world_y/world_z
# # 和 t_act 都是原始厘米值 / 100，后续圆柱外参生成逻辑不再额外换算。
# CALIB: dict[str, Any] = {
#     "world_x": -1.049774427407,
#     "world_y": 0.025542403707,
#     "world_z": 0.679575930453,
#
#     "yaw": 175.8271940901,
#     "pitch": 40.7239339859,
#     "roll": 1.3463462335,
#     "angle_type": "degree",
#
#     "image_width": 1392,
#     "image_height": 976,
#     "focal_u": 360.94 / SCALE_1,
#     "focal_v": 361.08 / SCALE_1,
#     "center_u": 696.0 / SCALE_1,
#     "center_v": 486.0 / SCALE_1,
#     "fisheye_distort": [0.108399, -0.019862, -0.001218, 0.000171],
#
#     "R_act": [
#         [-0.75585288,   0.055145696, -0.65241504],
#         [-0.088033266, -0.99595839,   0.017806733],
#         [-0.6487962,    0.070893496,  0.7576527],
#     ],
#     "t_act": [
#         -0.3515181,
#         -0.079076929,
#         -1.197783,
#     ],
# }

# # 2026/6/30 蓝色哪吒 Rt
# 平移字段统一存米：原始 Rt 的平移是厘米，这里的 world_x/world_y/world_z
# 和 t_act 都是原始厘米值 / 100，后续圆柱外参生成逻辑不再额外换算。
CALIB: dict[str, Any] = {
    "world_x": -1.0042536704,
    "world_y": 0.1303077147,
    "world_z": 1.2070025239,

    "yaw": 176.0649693732,
    "pitch": 31.0428831960,
    "roll": -2.3165611410,
    "angle_type": "degree",

    "image_width": 1280,
    "image_height": 960,
    "focal_u": 316 / SCALE_1,
    "focal_v": 316 / SCALE_1,
    "center_u": 640.0 / SCALE_1,
    "center_v": 480.0 / SCALE_1,
    "fisheye_distort": [1.29134998e-01, -4.47760001e-02, 1.54010002e-02, -2.65199994e-03],

    "R_act": [
    [-0.85476172 , 0.05879686, -0.51567948],
    [-0.04777424, -0.99825764, -0.034631629],
    [-0.51681721, -0.0049655926, 0.85608137],
],

    "t_act" : [
        -0.24363283 ,
        0.12390367,
        -1.5516609,
    ]

}

SCALE_2: float = 0.75 * SCALE_1  # 控制输出圆柱图尺寸

OUT_WIDTH: int = 960
OUT_HEIGHT: int = 480
CYL_CENTER_U_RATIO: float = 0.5
CYL_CENTER_V_RATIO: float = 0.5


# -------------------- 工具 --------------------
def _rot_y(rad: float) -> np.ndarray:
    c, s = math.cos(rad), math.sin(rad)
    return np.array(
        [
            [c, 0.0, s],
            [0.0, 1.0, 0.0],
            [-s, 0.0, c],
        ],
        dtype=np.float64,
    )


def _rot_x(rad: float) -> np.ndarray:
    c, s = math.cos(rad), math.sin(rad)
    return np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, c, -s],
            [0.0, s, c],
        ],
        dtype=np.float64,
    )


def _rot_z(rad: float) -> np.ndarray:
    c, s = math.cos(rad), math.sin(rad)
    return np.array(
        [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def _calib_angle_rad(name: str) -> float:
    value = float(CALIB[name])
    angle_type = str(CALIB.get("angle_type", "degree")).lower()
    if angle_type in {"degree", "deg", "degrees"}:
        return math.radians(value)
    if angle_type in {"rad", "radian", "radians"}:
        return value
    raise ValueError(f"unsupported angle_type: {CALIB.get('angle_type')!r}")


def _camera_to_base_rz_ry_rx() -> np.ndarray:
    """
    CameraExt convention used by temporary video calibration:
      R = Rz(yaw) * Ry(pitch) * Rx(roll)

    The matrix is treated as camera(OpenCV axes) -> base_link.
    OpenCV camera axes are X right, Y down, Z forward.
    """
    roll = _calib_angle_rad("roll")
    pitch = _calib_angle_rad("pitch")
    yaw = _calib_angle_rad("yaw")
    return _rot_z(yaw) @ _rot_y(pitch) @ _rot_x(roll)


def _build_map_uv_float(
        u_src: np.ndarray,
        v_src: np.ndarray,
        W_src: int,
        H_src: int,
        invalid: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, tuple[int, int]]:
    """
    输出浮点 remap（不做 round/clip，只存坐标场）：
      map_uv: (H_dst, W_dst, 2) float32, [:,:,0]=u_src, [:,:,1]=v_src
      mask:   (H_dst, W_dst) float32, 有效=1，无效=0
      out_wh: (W_dst, H_dst)
    """
    assert u_src.shape == v_src.shape == invalid.shape
    H_dst, W_dst = u_src.shape

    oob = (u_src < 0.0) | (u_src > (W_src - 1)) | (v_src < 0.0) | (v_src > (H_src - 1))
    invalid_all = invalid | oob

    map_uv = np.stack([u_src.astype(np.float32), v_src.astype(np.float32)], axis=-1)
    mask = (~invalid_all).astype(np.float32)
    return map_uv, mask, (W_dst, H_dst)


def _project_original_center_to_cyl(
        R_cam_to_cyl: np.ndarray,
        f_cyl: float,
        canvas_u: float,
        canvas_v: float,
) -> tuple[float, float]:
    ray_cyl = R_cam_to_cyl @ np.array([1.0, 0.0, 0.0], dtype=np.float64)
    kxy = math.hypot(float(ray_cyl[0]), float(ray_cyl[1]))
    if kxy <= 1e-12:
        raise ValueError("original camera optical axis is degenerate in cylinder coordinates")

    phi = math.atan2(-float(ray_cyl[1]), float(ray_cyl[0]))
    y_cyl = float(ray_cyl[2]) / kxy
    return canvas_u + f_cyl * phi, canvas_v - f_cyl * y_cyl


def _get_cyl_canvas(f_cyl: float) -> tuple[int, int, float, float]:
    Wc = int(OUT_WIDTH)
    Hc = int(OUT_HEIGHT)
    if Wc <= 0 or Hc <= 0:
        raise ValueError(f"invalid output size: {Wc}x{Hc}")

    hfov = Wc / f_cyl
    vfov = 2.0 * math.atan(Hc / (2.0 * f_cyl))
    return Wc, Hc, hfov, vfov


def _get_cyl_center(Wc: int, Hc: int) -> tuple[float, float]:
    return Wc * float(CYL_CENTER_U_RATIO), Hc * float(CYL_CENTER_V_RATIO)


def get_cyl_calib() -> dict[str, Any]:
    pitch = math.radians(float(CALIB["pitch"]))
    roll = math.radians(float(CALIB["roll"]))

    fu0 = float(CALIB["focal_u"])
    fv0 = float(CALIB["focal_v"])

    fu = fu0 * SCALE_2
    fv = fv0 * SCALE_2
    f_cyl = fu

    Wc, Hc, hfov, vfov = _get_cyl_canvas(f_cyl)
    uc_canvas, vc_canvas = _get_cyl_center(Wc, Hc)

    R_cam_to_cyl = _rot_y(pitch) @ _rot_x(roll)
    R_cyl_to_cam = R_cam_to_cyl.T
    center_u, center_v = _project_original_center_to_cyl(
        R_cam_to_cyl,
        f_cyl,
        uc_canvas,
        vc_canvas,
    )

    R_act = np.array(CALIB["R_act"], dtype=np.float64)
    t_act = np.array(CALIB["t_act"], dtype=np.float64).reshape(3)

    R_act_cyl = R_cam_to_cyl @ R_act
    t_act_cyl = R_cam_to_cyl @ t_act

    return {
        "image_width": Wc,
        "image_height": Hc,
        "focal_u": float(f_cyl),
        "focal_v": float(f_cyl),
        "center_u": float(center_u),
        "center_v": float(center_v),
        "hfov_deg": math.degrees(hfov),
        "vfov_deg": math.degrees(vfov),
        "R_cam_to_cyl": R_cam_to_cyl,
        "R_cyl_to_cam": R_cyl_to_cam,
        "R_act": R_act_cyl,
        "t_act": t_act_cyl,
    }


def print_cyl_calib() -> None:
    cyl_calib = get_cyl_calib()

    print("CYL_CALIB = {")
    print(f'    "image_width": {cyl_calib["image_width"]},')
    print(f'    "image_height": {cyl_calib["image_height"]},')
    print(f'    "focal_u": {cyl_calib["focal_u"]:.10f},')
    print(f'    "focal_v": {cyl_calib["focal_v"]:.10f},')
    print(f'    "center_u": {cyl_calib["center_u"]:.10f},')
    print(f'    "center_v": {cyl_calib["center_v"]:.10f},')
    print(f'    "hfov_deg": {cyl_calib["hfov_deg"]:.10f},')
    print(f'    "vfov_deg": {cyl_calib["vfov_deg"]:.10f},')
    print(f'    "R_cam_to_cyl": {cyl_calib["R_cam_to_cyl"].tolist()},')
    print(f'    "R_cyl_to_cam": {cyl_calib["R_cyl_to_cam"].tolist()},')
    print(f'    "R_act": {cyl_calib["R_act"].tolist()},')
    print(f'    "t_act": {cyl_calib["t_act"].tolist()},')
    print("}")


# -------------------- 鱼眼 -> 圆柱（map 为 float uv，源图是 full 鱼眼） --------------------
def build_map_fish_to_cyl() -> tuple[np.ndarray, np.ndarray, tuple[int, int], tuple[int, int]]:
    """
    输出：
      map_uv: (Hc, Wc, 2)，存 full 鱼眼源图坐标 (u_src,v_src) float
      mask:   (Hc, Wc)
      src_wh: (Wf_full, Hf_full)
      dst_wh: (Wc, Hc)
    """
    Wf_full, Hf_full = int(CALIB["image_width"]), int(CALIB["image_height"])
    fu0, fv0 = float(CALIB["focal_u"]), float(CALIB["focal_v"])
    cu0, cv0 = float(CALIB["center_u"]), float(CALIB["center_v"])
    k1, k2, k3, k4 = [float(x) for x in CALIB["fisheye_distort"]]
    pitch = math.radians(float(CALIB["pitch"]))
    roll = math.radians(float(CALIB["roll"]))

    fu, fv = fu0 * SCALE_2, fv0 * SCALE_2
    cu, cv = cu0 * SCALE_2, cv0 * SCALE_2

    f_cyl = fu
    Wc, Hc, hfov, vfov = _get_cyl_canvas(f_cyl)
    uc, vc = _get_cyl_center(Wc, Hc)

    uu, vv = np.meshgrid(
        np.arange(Wc, dtype=np.float32),
        np.arange(Hc, dtype=np.float32),
    )
    phi = ((uu - uc) / f_cyl).astype(np.float64)
    Y = ((vc - vv) / f_cyl).astype(np.float64)

    x_cyl = np.cos(phi)
    y_cyl = -np.sin(phi)
    z_cyl = Y

    R_cam_to_cyl = _rot_y(pitch) @ _rot_x(roll)
    R_cyl_to_cam = R_cam_to_cyl.T
    dir_cam = np.stack([x_cyl, y_cyl, z_cyl], axis=-1) @ R_cyl_to_cam.T
    x, y, z = dir_cam[..., 0], dir_cam[..., 1], dir_cam[..., 2]

    r_perp = np.sqrt(y * y + z * z)
    theta = np.arctan2(r_perp, x)

    t2 = theta * theta
    t4 = t2 * t2
    t6 = t4 * t2
    t8 = t4 * t4
    poly = 1.0 + k1 * t2 + k2 * t4 + k3 * t6 + k4 * t8
    d = theta * poly

    inv_r = np.where(r_perp < 1e-6, 0.0, 1.0 / r_perp)
    u_scaled = cu - fu * d * (y * inv_r)
    v_scaled = cv - fv * d * (z * inv_r)

    if SCALE_2 != 0.0:
        u_full = u_scaled / SCALE_2
        v_full = v_scaled / SCALE_2
    else:
        u_full = u_scaled
        v_full = v_scaled

    invalid = x <= 0.0
    map_uv, mask, dst_wh = _build_map_uv_float(
        u_full,
        v_full,
        W_src=Wf_full,
        H_src=Hf_full,
        invalid=invalid,
    )
    return map_uv, mask, (Wf_full, Hf_full), dst_wh


def build_map_fish_to_base_cyl() -> tuple[np.ndarray, np.ndarray, tuple[int, int], tuple[int, int]]:
    """
    Fish-eye image -> base_link cylindrical projection.

    This is for calibrations that provide CameraExt as camera -> base_link with
    Euler convention R = Rz(yaw) * Ry(pitch) * Rx(roll), and a standard OpenCV
    fisheye camera frame:
      camera X: right, Y: down, Z: forward

    The output cylinder frame is base_link:
      base X: forward, Y: left, Z: up
    """
    Wf_full, Hf_full = int(CALIB["image_width"]), int(CALIB["image_height"])
    fu0, fv0 = float(CALIB["focal_u"]), float(CALIB["focal_v"])
    cu0, cv0 = float(CALIB["center_u"]), float(CALIB["center_v"])
    k1, k2, k3, k4 = [float(x) for x in CALIB["fisheye_distort"]]

    f_cyl = fu0 * SCALE_2
    Wc, Hc, _, _ = _get_cyl_canvas(f_cyl)
    uc, vc = _get_cyl_center(Wc, Hc)

    uu, vv = np.meshgrid(
        np.arange(Wc, dtype=np.float32),
        np.arange(Hc, dtype=np.float32),
    )
    phi = ((uu - uc) / f_cyl).astype(np.float64)
    z_rel = ((vc - vv) / f_cyl).astype(np.float64)

    # u grows to the right; base_link +Y is left, so the ray's Y component is -sin(phi).
    dir_base = np.stack(
        [
            np.cos(phi),
            -np.sin(phi),
            z_rel,
        ],
        axis=-1,
    )

    R_cam_to_base = _camera_to_base_rz_ry_rx()
    # Row-vector form: dir_cam = dir_base @ R_base_to_cam.
    dir_cam = dir_base @ R_cam_to_base
    x, y, z = dir_cam[..., 0], dir_cam[..., 1], dir_cam[..., 2]

    r_perp = np.sqrt(x * x + y * y)
    theta = np.arctan2(r_perp, z)

    t2 = theta * theta
    t4 = t2 * t2
    t6 = t4 * t2
    t8 = t4 * t4
    poly = 1.0 + k1 * t2 + k2 * t4 + k3 * t6 + k4 * t8
    d = theta * poly

    inv_r = np.where(r_perp < 1e-6, 0.0, 1.0 / r_perp)
    u_full = cu0 + fu0 * d * (x * inv_r)
    v_full = cv0 + fv0 * d * (y * inv_r)

    invalid = z <= 0.0
    map_uv, mask, dst_wh = _build_map_uv_float(
        u_full,
        v_full,
        W_src=Wf_full,
        H_src=Hf_full,
        invalid=invalid,
    )
    return map_uv, mask, (Wf_full, Hf_full), dst_wh


# -------------------- 圆柱 -> 鱼眼（先小图映射，再缩放 map 到 full） --------------------
def build_map_cyl_to_fish() -> tuple[np.ndarray, np.ndarray, tuple[int, int], tuple[int, int]]:
    """
    输出：
      map_uv: (Hf_full, Wf_full, 2)，存圆柱源图坐标 (u_src,v_src) float
      mask:   (Hf_full, Wf_full)
      src_wh: (Wc, Hc)
      dst_wh: (Wf_full, Hf_full)
    """
    Wf_full, Hf_full = int(CALIB["image_width"]), int(CALIB["image_height"])
    Wf_s = int(round(Wf_full * SCALE_2))
    Hf_s = int(round(Hf_full * SCALE_2))

    fu0, fv0 = float(CALIB["focal_u"]), float(CALIB["focal_v"])
    cu0, cv0 = float(CALIB["center_u"]), float(CALIB["center_v"])
    k1, k2, k3, k4 = [float(x) for x in CALIB["fisheye_distort"]]
    pitch = math.radians(float(CALIB["pitch"]))
    roll = math.radians(float(CALIB["roll"]))

    fu, fv = fu0 * SCALE_2, fv0 * SCALE_2
    cu, cv = cu0 * SCALE_2, cv0 * SCALE_2

    f_cyl = fu
    Wc, Hc, hfov, vfov = _get_cyl_canvas(f_cyl)
    uc, vc = _get_cyl_center(Wc, Hc)

    uu, vv = np.meshgrid(
        np.arange(Wf_s, dtype=np.float32),
        np.arange(Hf_s, dtype=np.float32),
    )

    du = (cu - uu) / fu
    dv = (cv - vv) / fv
    d = np.sqrt(du * du + dv * dv)

    theta_max = 0.5 * math.pi
    t2m = theta_max ** 2
    t4m = t2m ** 2
    t6m = t4m * t2m
    t8m = t4m ** 2
    d_lim = theta_max * (1.0 + k1 * t2m + k2 * t4m + k3 * t6m + k4 * t8m)
    base_valid = d <= d_lim

    theta = np.minimum(d.astype(np.float64), theta_max)
    for _ in range(8):
        t2 = theta * theta
        t4 = t2 * t2
        t6 = t4 * t2
        t8 = t4 * t4
        poly = 1.0 + k1 * t2 + k2 * t4 + k3 * t6 + k4 * t8
        f_val = theta * poly - d
        dpoly = (
                2.0 * k1 * theta
                + 4.0 * k2 * t2 * theta
                + 6.0 * k3 * t4 * theta
                + 8.0 * k4 * t6 * theta
        )
        f_deriv = poly + theta * dpoly
        theta -= np.where(f_deriv != 0.0, f_val / f_deriv, 0.0)
    theta = np.where(base_valid, np.clip(theta, 0.0, theta_max), theta)

    inv_d = np.where(d > 1e-6, 1.0 / d, 0.0)
    y_ratio = du * inv_d
    z_ratio = dv * inv_d
    sin_t = np.sin(theta)
    cos_t = np.cos(theta)

    x_cam = cos_t
    y_cam = y_ratio * sin_t
    z_cam = z_ratio * sin_t

    R_cam_to_cyl = _rot_y(pitch) @ _rot_x(roll)
    R_cyl_to_cam = R_cam_to_cyl.T
    dir_cyl = np.stack([x_cam, y_cam, z_cam], axis=-1) @ R_cyl_to_cam
    x_c, y_c, z_c = dir_cyl[..., 0], dir_cyl[..., 1], dir_cyl[..., 2]

    kxy = np.sqrt(x_c * x_c + y_c * y_c)
    phi = np.arctan2(-y_c, x_c)
    Y = np.where(kxy > 1e-6, z_c / kxy, 0.0)

    u_cyl = uc + f_cyl * phi
    v_cyl = vc - f_cyl * Y

    invalid_small = (~base_valid) | (x_cam <= 0.0)

    map_uv_s, mask_s, _ = _build_map_uv_float(
        u_cyl,
        v_cyl,
        W_src=Wc,
        H_src=Hc,
        invalid=invalid_small,
    )

    map_uv_full = cv2.resize(
        map_uv_s,
        (Wf_full, Hf_full),
        interpolation=cv2.INTER_LINEAR,
    ).astype(np.float32)

    mask_full = cv2.resize(
        mask_s,
        (Wf_full, Hf_full),
        interpolation=cv2.INTER_NEAREST,
    ).astype(np.float32)

    return map_uv_full, mask_full, (Wc, Hc), (Wf_full, Hf_full)


def save_map_uv(
        map_uv: np.ndarray,
        dst_wh: tuple[int, int],
        prefix: str,
        out_dir: str | Path = ".",
) -> Path:
    """
    map_uv: (H, W, 2)  map_uv[...,0]=u_src, map_uv[...,1]=v_src
    dst_wh: (W, H)

    保存格式（float32, row-major）：
      [u(0,0), u(0,1), ... u(0,W-1), u(1,0), ... u(H-1,W-1),
       v(0,0), v(0,1), ... v(H-1,W-1)]
    总长度：2 * H * W 个 float32
    """
    W_dst, H_dst = dst_wh
    if map_uv.ndim != 3 or map_uv.shape[2] != 2:
        raise ValueError(f"map_uv must be (H,W,2), got {map_uv.shape}")
    if map_uv.shape[0] != H_dst or map_uv.shape[1] != W_dst:
        raise ValueError(
            f"dst_wh mismatch: map_uv is (H={map_uv.shape[0]}, W={map_uv.shape[1]}), "
            f"dst_wh is (W={W_dst}, H={H_dst})"
        )

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    bin_path = out_dir / f"{prefix}_{W_dst}x{H_dst}_uv.bin"

    u = np.ascontiguousarray(map_uv[:, :, 0], dtype=np.float32)
    v = np.ascontiguousarray(map_uv[:, :, 1], dtype=np.float32)

    with open(bin_path, "wb") as f:
        u.tofile(f)
        v.tofile(f)

    print(bin_path)
    return bin_path


def load_map_uv(bin_path: Path, w: int, h: int) -> tuple[np.ndarray, np.ndarray]:
    buf = np.fromfile(str(bin_path), dtype=np.float32)
    expected = 2 * w * h
    if buf.size != expected:
        raise ValueError(f"bin size mismatch: got {buf.size}, expected {expected} (2*{w}*{h})")
    n = w * h
    u = buf[:n].reshape(h, w).astype(np.float32, copy=False)
    v = buf[n:].reshape(h, w).astype(np.float32, copy=False)
    return u, v


def apply_map(
        src_bgr: np.ndarray,
        u: np.ndarray,
        v: np.ndarray,
        mask: np.ndarray | None = None,
        interpolation: int = cv2.INTER_LINEAR,
        border_value: tuple[int, int, int] = (0, 0, 0),
) -> tuple[np.ndarray, np.ndarray]:
    """
    使用映射表 u/v 对 src_bgr 做 remap。

    参数:
      src_bgr: (H_src, W_src, 3)
      u, v:    (H_dst, W_dst) float32/float64
      mask:    (H_dst, W_dst) float32/uint8/bool, 可选；>0 视为有效
              若提供，则 valid = valid & (mask>0)
    返回:
      remap_img: (H_dst, W_dst, 3)
      valid:     (H_dst, W_dst) bool，有效像素位置
    """
    if u.shape != v.shape:
        raise ValueError(f"u/v shape mismatch: u={u.shape}, v={v.shape}")

    h_dst, w_dst = u.shape
    h_src, w_src = src_bgr.shape[:2]

    valid = (
            np.isfinite(u)
            & np.isfinite(v)
            & (u >= 0.0)
            & (u <= (w_src - 1))
            & (v >= 0.0)
            & (v <= (h_src - 1))
    )

    if mask is not None:
        if mask.shape != u.shape:
            raise ValueError(f"mask shape mismatch: mask={mask.shape}, uv={u.shape}")
        valid = valid & (mask > 0)

    map_x = np.nan_to_num(u, nan=-1.0).astype(np.float32, copy=False)
    map_y = np.nan_to_num(v, nan=-1.0).astype(np.float32, copy=False)

    remap_img = cv2.remap(
        src_bgr,
        map_x,
        map_y,
        interpolation=interpolation,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=border_value,
    )

    remap_img[~valid] = 0
    return remap_img, valid

# Ti相关代码
def _matlab_round_to_int(x: np.ndarray) -> np.ndarray:
    """
    模拟 MATLAB round：0.5 远离 0 取整
    """
    return (np.sign(x) * np.floor(np.abs(x) + 0.5)).astype(np.int32)


def save_mesh_txt_fish_to_cyl_matlab(
        dst_wh: tuple[int, int],
        m: int = 4,
        out_dir: str | Path = ".",
        filename: str = "mesh.txt",
) -> Path:
    """
    严格按照 MATLAB 版本生成 mesh.txt：

      [h_p, v_p] = meshgrid(0:W, 0:H)
      h_delta = round((h_d - h_p) * 8)
      v_delta = round((v_d - v_p) * 8)

      mh = h_delta(1:2^m:end, 1:2^m:end)'
      mv = v_delta(1:2^m:end, 1:2^m:end)'
      dlmwrite('mesh.txt', [mh(:), mv(:)], 'delimiter', ' ')

    因此会包含边界点：
      x = 0, 16, ..., W
      y = 0, 16, ..., H

    对于 960x480、m=4：
      x 点数 = 61
      y 点数 = 31
      总行数 = 61 * 31 = 1891
    """
    W_dst, H_dst = dst_wh
    step = 1 << int(m)

    Wf_full, Hf_full = int(CALIB["image_width"]), int(CALIB["image_height"])
    fu0, fv0 = float(CALIB["focal_u"]), float(CALIB["focal_v"])
    cu0, cv0 = float(CALIB["center_u"]), float(CALIB["center_v"])
    k1, k2, k3, k4 = [float(x) for x in CALIB["fisheye_distort"]]

    pitch = math.radians(float(CALIB["pitch"]))
    roll = math.radians(float(CALIB["roll"]))

    fu, fv = fu0 * SCALE_2, fv0 * SCALE_2
    cu, cv = cu0 * SCALE_2, cv0 * SCALE_2

    f_cyl = fu
    uc, vc = _get_cyl_center(W_dst, H_dst)

    xs = np.arange(0, W_dst + 1, step, dtype=np.float64)
    ys = np.arange(0, H_dst + 1, step, dtype=np.float64)
    h_p, v_p = np.meshgrid(xs, ys)

    phi = (h_p - uc) / f_cyl
    Y = (vc - v_p) / f_cyl

    x_cyl = np.cos(phi)
    y_cyl = -np.sin(phi)
    z_cyl = Y

    R_cam_to_cyl = _rot_y(pitch) @ _rot_x(roll)
    R_cyl_to_cam = R_cam_to_cyl.T

    dir_cam = np.stack([x_cyl, y_cyl, z_cyl], axis=-1) @ R_cyl_to_cam.T

    x = dir_cam[..., 0]
    y = dir_cam[..., 1]
    z = dir_cam[..., 2]

    r_perp = np.sqrt(y * y + z * z)
    theta = np.arctan2(r_perp, x)

    t2 = theta * theta
    t4 = t2 * t2
    t6 = t4 * t2
    t8 = t4 * t4

    poly = 1.0 + k1 * t2 + k2 * t4 + k3 * t6 + k4 * t8
    d = theta * poly

    inv_r = np.where(r_perp < 1e-6, 0.0, 1.0 / r_perp)

    u_scaled = cu - fu * d * (y * inv_r)
    v_scaled = cv - fv * d * (z * inv_r)

    if SCALE_2 != 0.0:
        h_d = u_scaled / SCALE_2
        v_d = v_scaled / SCALE_2
    else:
        h_d = u_scaled
        v_d = v_scaled

    h_delta = _matlab_round_to_int((h_d - h_p) * 8.0)
    v_delta = _matlab_round_to_int((v_d - v_p) * 8.0)

    # 严格模拟 MATLAB:
    # mh = h_delta(...)'
    # mv = v_delta(...)'
    # [mh(:), mv(:)]
    mh = h_delta.T
    mv = v_delta.T

    mesh = np.column_stack([
        mh.reshape(-1, order="F"),
        mv.reshape(-1, order="F"),
    ])

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    txt_path = out_dir / filename
    np.savetxt(txt_path, mesh, fmt="%d", delimiter=" ")

    print(
        f"save mesh txt: {txt_path}, "
        f"shape={mesh.shape}, "
        f"grid={len(xs)}x{len(ys)}, "
        f"step={step}"
    )

    return txt_path


if __name__ == "__main__":
    print_cyl_calib()

    map_f2c, mask_f2c, fish_size, cyl_size = build_map_fish_to_cyl()
    map_c2f, mask_c2f, _, _ = build_map_cyl_to_fish()

    bin_path = save_map_uv(map_f2c, cyl_size, prefix="visper", out_dir="./maps")

    # ti畸变表
    mesh_path = save_mesh_txt_fish_to_cyl_matlab(cyl_size, m=4, out_dir="./maps", filename="mesh4.txt")

    u, v = load_map_uv(bin_path, cyl_size[0], cyl_size[1])

    # import glob
    # import os
    # # 方法1：使用 glob 匹配
    # image_folder = "/home/gq/guoqian/Projects-AEB/testImage/images/"
    # image_paths = glob.glob(os.path.join(image_folder, "*.jpg"))

    # # 读取所有图片
    # images = []
    # for path in image_paths:
    #     fish = cv2.imread(path, cv2.IMREAD_COLOR)
    #     fish = cv2.resize(
    #         fish,
    #         (int(1280 // SCALE_1), int(960 // SCALE_1)),
    #         interpolation=cv2.INTER_LINEAR,
    #         )

    #     remap, valid = apply_map(fish, u, v)
    #     cv2.imwrite(f"./out/{os.path.basename(path)}", remap)


    img_path = "11.jpg"
    fish = cv2.imread(f"/home/gq/guoqian/Projects-AEB/{img_path}", cv2.IMREAD_COLOR)
    fish = cv2.resize(
        fish,
        (int(1280 // SCALE_1), int(960 // SCALE_1)),
        interpolation=cv2.INTER_LINEAR,
    )

    remap, valid = apply_map(fish, u, v)
    cv2.imwrite(f"./out/{img_path}", remap)

    print(f"fish_size:{fish_size}, cyl_size:{cyl_size}")
