import numpy as np

from dataclasses import dataclass, asdict, replace
from pathlib import Path
import json
import math

import cv2
from typing import Literal
from dataclasses import dataclass, asdict, field


# ----------------------------------------------------------------------
# 1. 图像 + 相机标定
# ----------------------------------------------------------------------

@dataclass
class CalibratedImage:
    # ---------------- 图像数据 (BGR) ----------------
    image: np.ndarray | None = None  # shape=(H, W, 3), dtype=uint8

    # ---------------- 标定参数（与 JSON calib 对应） ----------------
    type: Literal["pinhole", "fisheye"] = "pinhole"

    # --- extrinsics 外参（世界->相机）--------------
    world_x: float = 0.0
    world_y: float = 0.0
    world_z: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    roll: float = 0.0
    angle_type: Literal["degree", "rad"] = "degree"

    # --- intrinsics 内参（像素坐标系） --------------
    image_type: Literal["RGB", "BGR"] = "BGR"
    image_width: int = 0
    image_height: int = 0
    focal_u: float = 0.0
    focal_v: float = 0.0
    center_u: float = 0.0
    center_v: float = 0.0

    # 视场角
    fov: float = 0.0

    # --- “世界坐标系 → 相机坐标系”的主动外参矩阵 [R_act | t_act] (3×4)。--------------
    # 3x3 旋转矩阵（row-major）
    R_act: list[list[float]] = field(default_factory=list)
    # 3x1 平移向量（米）
    t_act: list[float] = field(default_factory=list)

    # --- distortion 畸变参数 ---------------------
    pinhole_distort: list[float] = field(default_factory=list)  # [k1,k2,p1,p2]
    fisheye_distort: list[float] = field(default_factory=list)  # [k1,k2,k3,k4]

    # ---------------- I/O ----------------
    @classmethod
    def load(cls, img_src: np.ndarray, calib_params: dict) -> "CalibratedImage":
        """
        calib_params 直接是 calib 字典（不是包着 'calib' 的上层 dict）。
        加载时将角度（pitch/yaw/roll）统一转换为弧度，angle_type 固定为 'rad'。
        """
        params = dict(calib_params)  # 浅拷贝，避免修改外部字典

        # 读取并标准化 angle_type
        angle_type_raw = str(params.get("angle_type", "rad")).lower()

        pitch = float(params.get("pitch", 0.0))
        yaw = float(params.get("yaw", 0.0))
        roll = float(params.get("roll", 0.0))

        # 若为度，转为弧度
        if angle_type_raw == "degree":
            pitch = math.radians(pitch)
            yaw = math.radians(yaw)
            roll = math.radians(roll)

        params["pitch"] = pitch
        params["yaw"] = yaw
        params["roll"] = roll
        params["angle_type"] = "rad"  # 统一为弧度标记
        params["image"] = img_src

        obj = cls(**params)
        return obj

    def save(self, img_path: str | Path, calib_path: str | Path | None = None) -> None:
        cv2.imwrite(str(img_path), self.image)
        if calib_path is not None:
            meta = asdict(self).copy()
            meta.pop("image")
            Path(calib_path).write_text(json.dumps(meta, ensure_ascii=False, indent=4), encoding="utf-8")


# ----------------------------------------------------------------------
# 2. 图片处理器
# ----------------------------------------------------------------------
class ImageProcessor:
    @staticmethod
    def _Rx(roll: float) -> np.ndarray:
        """绕 x 轴的旋转矩阵（右手定则正方向），弧度制"""
        return np.array([
            [1, 0, 0],
            [0, math.cos(roll), -math.sin(roll)],
            [0, math.sin(roll), math.cos(roll)]
        ], dtype=np.float64)

    @staticmethod
    def _Ry(pitch: float) -> np.ndarray:
        """绕 y 轴的旋转矩阵（右手定则正方向），弧度制"""
        return np.array([
            [math.cos(pitch), 0, math.sin(pitch)],
            [0, 1, 0],
            [-math.sin(pitch), 0, math.cos(pitch)]
        ], dtype=np.float64)

    @staticmethod
    def _Rz(yaw: float) -> np.ndarray:
        """绕 z 轴的旋转矩阵（右手定则正方向），弧度制"""
        return np.array([
            [math.cos(yaw), -math.sin(yaw), 0],
            [math.sin(yaw), math.cos(yaw), 0],
            [0, 0, 1]
        ], dtype=np.float64)

    @staticmethod
    def _Rt_active(cimg) -> np.ndarray:
        """
        生成“世界坐标系 → 相机坐标系”的主动外参矩阵 [R_act | t_act] (3×4)。

        说明
        ----
        对世界点 P_w (3×1) 做主动变换得到相机系点 P_c：
            P_c = R_act @ P_w + t_act

        矩阵结构
        ----
            M = [ R_act | t_act ]
        其中
            R_act：3×3 主动旋转矩阵（R_passive 的转置）
            t_act：3×1 主动平移向量，计算为 –R_act @ C
                    其中 C = [world_x, world_y, world_z]^T

        参数
        ----
        cimg : CalibratedImage
            已标定图像，包含：
              - yaw|pitch|roll: 相机欧拉角（弧度），Z→Y→X 顺序
              - world_x|world_y|world_z: 相机中心在世界系下的位置（米）

        返回
        ----
        M : np.ndarray, shape=(3,4)
            主动外参矩阵，可直接用于齐次坐标变换：
                [X_c, Y_c, Z_c]^T = M @ [X_w, Y_w, Z_w, 1]^T
        """
        # 1. 先算出被动外参 R_passive
        yaw, pitch, roll = cimg.yaw, cimg.pitch, cimg.roll
        R_passive = (
                ImageProcessor._Rz(yaw)
                @ ImageProcessor._Ry(pitch)
                @ ImageProcessor._Rx(roll)
        )

        # 2. 转置得到主动旋转
        R_act = R_passive.T
        #    等价于：R_act = _Rx(-roll) @ _Ry(-pitch) @ _Rz(-yaw)
        #    但直接转置更简洁且少算三次三角函数

        # 3. 计算主动平移 t_act = -R_act @ C
        C = np.array(
            [cimg.world_x, cimg.world_y, cimg.world_z],
            dtype=np.float64
        ).reshape(3, 1)
        t_act = -R_act @ C

        # 4. 合并成 3×4 矩阵
        M = np.hstack((R_act, t_act))  # shape=(3, 4)
        return M

    @staticmethod
    def _P_cam2pixel(cimg: CalibratedImage) -> np.ndarray:
        """正确的相机→像素投影矩阵 (3×4)。"""
        fu, fv = cimg.focal_u, cimg.focal_v
        u0, v0 = cimg.center_u, cimg.center_v
        return np.array([
            [u0, -fu, 0.0, 0.0],
            [v0, 0.0, -fv, 0.0],
            [1.0, 0.0, 0.0, 0.0]
        ], dtype=np.float64)

    @staticmethod
    def _K_intrinsic(
            f_u: float,
            f_v: float,
            c_u: float,
            c_v: float
    ) -> np.ndarray:
        """
        构造相机内参矩阵 K（3×3）：
            [f_u,   0,  c_u]
            [  0, f_v,  c_v]
            [  0,   0,    1]
        参数
        ----
        f_u, f_v : float
            焦距（像素）沿 u、v 方向
        c_u, c_v : float
            主点坐标 u0、v0
        返回
        ----
        K : np.ndarray, shape=(3,3)
        """
        return np.array([
            [f_u, 0.0, c_u],
            [0.0, f_v, c_v],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)

    @staticmethod
    def check(
            cimg: CalibratedImage,
            *,
            tol_img_px: float = 0.5,
            tol_pp_px: float = 2.0,
            tol_det: float = 1e-3,
            tol_orth: float = 1e-3,
            tol_R: float = 1e-2,
            tol_t: float = 1e-3,
            tol_fov_deg: float = 2.0
    ) -> None:
        """
        只打印、不返回。遵从 ImageProcessor 约定：
          - 欧拉角顺序 Z(yaw)→Y(pitch)→X(roll)
          - 外参主动定义：R_act = R_passive^T, t_act = -R_act @ C
        横向 FOV：
          - pinhole：2*atan((2*r_pix_max)/(2*fu))
          - fisheye：OpenCV 模型 r = fu * theta_d(theta)，扫样找零点区间 + 二分求最大根 theta_max
        若给定的 R 或 t 与 _Rt_active 期望不一致，直接报错。
        """
        import math, json
        import numpy as np

        issues: list[str] = []
        details: dict = {}

        # ----- 内部小函数（仅在本函数内可见） -----
        def _euler_to_R_passive_zyx(yaw: float, pitch: float, roll: float) -> np.ndarray:
            # 遵从 ImageProcessor：R_passive = Rz(yaw) @ Ry(pitch) @ Rx(roll)
            return (ImageProcessor._Rz(yaw) @ ImageProcessor._Ry(pitch) @ ImageProcessor._Rx(roll)).astype(np.float64)

        def _theta_distorted(theta: float, k: list[float]) -> float:
            # OpenCV fisheye: theta_d = theta * (1 + k1 θ^2 + k2 θ^4 + k3 θ^6 + k4 θ^8)
            k1 = k[0] if len(k) > 0 else 0.0
            k2 = k[1] if len(k) > 1 else 0.0
            k3 = k[2] if len(k) > 2 else 0.0
            k4 = k[3] if len(k) > 3 else 0.0
            t2 = theta * theta
            return theta * (1.0 + k1 * t2 + k2 * (t2 ** 2) + k3 * (t2 ** 3) + k4 * (t2 ** 4))

        def _largest_root_theta_for_radius(r_pix: float, fu: float, k: list[float]) -> float | None:
            """
            解 g(θ)=fu*θd(θ)-r=0 在 θ∈(0, π-ε) 的“最大根”（非单调也可）。
            步骤：均匀扫样→找符号变化区间→逐区间二分→取最大的 θ。
            """
            if fu <= 0 or r_pix <= 0:
                return None

            theta_lo, theta_hi = 0.0, math.pi - 1e-6  # 接近 180°
            r = float(r_pix)

            def g(th: float) -> float:
                return fu * _theta_distorted(th, k) - r

            # 扫样（较密，覆盖先增后减）
            N = 2048
            thetas = np.linspace(theta_lo, theta_hi, N, dtype=np.float64)
            vals = np.array([g(th) for th in thetas], dtype=np.float64)

            roots: list[float] = []
            # 精确零点（数值容忍）
            zero_hits = thetas[np.isclose(vals, 0.0, atol=1e-10)]
            roots.extend(zero_hits.tolist())

            # 符号变化区间二分
            for i in range(N - 1):
                a, b = thetas[i], thetas[i + 1]
                ga, gb = vals[i], vals[i + 1]
                if ga == 0.0 or gb == 0.0:
                    continue
                if ga * gb > 0:
                    continue
                lo, hi = a, b
                for _ in range(64):
                    mid = 0.5 * (lo + hi)
                    gm = g(mid)
                    if gm == 0.0:
                        lo = hi = mid
                        break
                    if ga * gm <= 0:
                        hi = mid
                        gb = gm
                    else:
                        lo = mid
                        ga = gm
                roots.append(0.5 * (lo + hi))

            if not roots:
                return None
            return max(roots)  # 最大根对应最靠水平边界的半视角

        def _check_list(arr, name: str):
            if not isinstance(arr, list):
                issues.append(f"{name} 应为 list。")
                return
            for i, v in enumerate(arr):
                if not isinstance(v, (int, float)):
                    issues.append(f"{name}[{i}] 不是数值：{type(v)}。")

        # ----- 1) 图像尺寸/类型 -----
        H_decl, W_decl = cimg.image_height, cimg.image_width
        if cimg.image is None:
            issues.append("image 为空。")
        else:
            if cimg.image.ndim != 3 or cimg.image.shape[2] != 3:
                issues.append(f"image 形状异常，期望 (H,W,3)，实际 {cimg.image.shape}。")
            H_img, W_img = cimg.image.shape[:2]
            if abs(H_img - H_decl) > tol_img_px or abs(W_img - W_decl) > tol_img_px:
                issues.append(f"图像尺寸与声明不一致：实际 ({W_img}x{H_img})，声明 ({W_decl}x{H_decl})。")
            if cimg.image.dtype != np.uint8:
                issues.append(f"image dtype 不是 uint8，而是 {cimg.image.dtype}。")

        # ----- 2) 内参基本检查 -----
        if cimg.focal_u <= 0 or cimg.focal_v <= 0:
            issues.append(f"焦距应为正数：f_u={cimg.focal_u}, f_v={cimg.focal_v}。")
        if not (-tol_pp_px <= cimg.center_u <= W_decl + tol_pp_px and
                -tol_pp_px <= cimg.center_v <= H_decl + tol_pp_px):
            issues.append(f"主点 (u0={cimg.center_u}, v0={cimg.center_v}) 超出图像范围（含容忍 {tol_pp_px}px）。")
        if cimg.image_type not in ("RGB", "BGR"):
            issues.append(f"image_type 非法：{cimg.image_type}（应为 'RGB' 或 'BGR'）。")

        # ----- 3) 给定 R 的正交性 -----
        R_given = None
        if hasattr(cimg, "R_act") and isinstance(cimg.R_act, list) and cimg.R_act:
            try:
                R_given = np.array(cimg.R_act, dtype=np.float64).reshape(3, 3)
                RtR = R_given.T @ R_given
                I = np.eye(3)
                orth_err = float(np.linalg.norm(RtR - I, ord="fro"))
                detR = float(np.linalg.det(R_given))
                details["R_orth_err"] = orth_err
                details["detR"] = detR
                if orth_err > tol_orth:
                    issues.append(f"R 非正交，||R^T R - I||_F = {orth_err:.3e} > {tol_orth}。")
                if abs(detR - 1.0) > tol_det:
                    issues.append(f"det(R)={detR:.6f}，应接近 1（容忍 {tol_det}）。")
            except Exception as e:
                issues.append(f"R 无法解析为 3×3：{e}")

        # ----- 4) 严格遵从 _Rt_active 的 R/t 一致性 -----
        R_passive = _euler_to_R_passive_zyx(cimg.yaw, cimg.pitch, cimg.roll)
        R_act_expected = R_passive.T
        C = np.array([cimg.world_x, cimg.world_y, cimg.world_z], dtype=np.float64).reshape(3, 1)
        t_act_expected = (-R_act_expected @ C).reshape(3)

        details["R_act_expected"] = R_act_expected.tolist()
        details["t_act_expected"] = t_act_expected.tolist()

        if R_given is not None:
            diff_R_act = float(np.linalg.norm(R_given - R_act_expected, ord="fro"))
            details["R_diff_act_fro"] = diff_R_act
            if diff_R_act > tol_R:
                issues.append(
                    f"R 与 _Rt_active 期望的 R_act 不一致：||R_given - R_act||_F = {diff_R_act:.3e} > {tol_R}。")

        if hasattr(cimg, "t_act") and isinstance(cimg.t_act, list) and cimg.t_act:
            try:
                t_given = np.array(cimg.t_act, dtype=np.float64).reshape(3)
                diff_t_act = float(np.linalg.norm(t_given - t_act_expected))
                details["t_diff_act_l2"] = diff_t_act
                if diff_t_act > tol_t:
                    issues.append(
                        f"t 与 _Rt_active 期望的 t_act 不一致：||t_given - t_act||_2 = {diff_t_act:.3e} m > {tol_t}。")
            except Exception as e:
                issues.append(f"t 无法解析为 3×1：{e}")

        # ----- 5) 角度单位 -----
        if cimg.angle_type != "rad":
            issues.append(f"angle_type 应为 'rad'，当前为 '{cimg.angle_type}'。")

        # ----- 6) 畸变参数类型 -----
        _check_list(cimg.pinhole_distort, "pinhole_distort")
        _check_list(cimg.fisheye_distort, "fisheye_distort")

        # ===== 7) 横向 FOV（仅横向） =====
        fu = cimg.focal_u
        W = W_decl
        r_left = cimg.center_u
        r_right = max(0.0, (W - 1) - cimg.center_u)  # 以像素索引边界为准
        r_pix_max = min(r_left, r_right)

        if fu > 0 and r_pix_max > 0:
            if cimg.type == "pinhole":
                fov_h_rad = 2.0 * math.atan((2.0 * r_pix_max) / (2.0 * fu))
                fov_h_deg = math.degrees(fov_h_rad)
                details["fov_h_deg_calc"] = fov_h_deg
                # 新逻辑：fov<=0 也报
                if not isinstance(cimg.fov, (int, float)) or not math.isfinite(cimg.fov) or cimg.fov <= 0:
                    issues.append(f"[pinhole] fov 未设置或为 0，建议填写为≈{fov_h_deg:.3f}°（水平）。")
                elif abs(fov_h_deg - cimg.fov) > tol_fov_deg:
                    issues.append(
                        f"[pinhole] 计算的水平 FOV≈{fov_h_deg:.3f}° 与给定 fov={cimg.fov:.3f}° 相差超过 {tol_fov_deg}°。")

            elif cimg.type == "fisheye":
                theta_max = _largest_root_theta_for_radius(r_pix=r_pix_max, fu=fu, k=cimg.fisheye_distort)
                if theta_max is None:
                    issues.append(
                        "[fisheye] 依据 fu 与 fisheye_distort 未在 θ∈(0, π) 找到达到水平边界的解；"
                        "若标定可信，建议提高扫样密度或检查 r_pix_max/fu/系数是否匹配。")
                else:
                    fov_h_deg = math.degrees(2.0 * theta_max)
                    details["theta_max_rad"] = theta_max
                    details["fov_h_deg_calc"] = fov_h_deg
                    # 宽松 sanity（允许 >180°）
                    if not (0.0 < fov_h_deg < 300.0):
                        issues.append(
                            f"[fisheye] 由系数计算的水平 FOV={fov_h_deg:.3f}° 看起来异常（应在 0–300° 之间，允许>180°）。")
                    # 新逻辑：fov<=0 也报
                    if not isinstance(cimg.fov, (int, float)) or not math.isfinite(cimg.fov) or cimg.fov <= 0:
                        issues.append(f"[fisheye] fov 未设置或为 0，建议填写为≈{fov_h_deg:.3f}°（水平）。")
                    elif abs(fov_h_deg - cimg.fov) > tol_fov_deg:
                        issues.append(
                            f"[fisheye] 计算的水平 FOV≈{fov_h_deg:.3f}° 与给定 fov={cimg.fov:.3f}° 相差超过 {tol_fov_deg}°。")
        else:
            issues.append("无法计算横向 FOV：f_u 或 r_pix_max 非法（可能主点超出画面或焦距无效）。")

        # ----- 打印结果 -----
        ok = len(issues) == 0
        summary = "All good." if ok else f"发现 {len(issues)} 个问题。"

        print("=== CalibratedImage Check (严格遵从 _Rt_active / ZYX) ===")
        print(summary)
        if issues:
            print("- Issues:")
            for s in issues:
                print("  - " + s)
        if details:
            print("- Details:")
            print(json.dumps(details, ensure_ascii=False, indent=2))
        return None

    @staticmethod
    def reorient(
            cimg: CalibratedImage,
            new_yaw: float = 0,
            new_pitch: float = 0,
            new_roll: float = 0,
            out_size: tuple[int, int] | None = None,
    ) -> CalibratedImage:
        """
        表示画面转动角度
        """
        # 1. 确定输出大小
        if out_size is None:
            W_new, H_new = cimg.image_width, cimg.image_height
        else:
            W_new, H_new = out_size

        # 2. 构造 OpenCV 标准内参矩阵 K
        K = ImageProcessor._K_intrinsic(
            cimg.focal_u, cimg.focal_v,
            cimg.center_u, cimg.center_v
        )

        # 3. 计算主动旋转矩阵R
        R_delta = (
                ImageProcessor._Rx(new_roll)
                @ ImageProcessor._Ry(new_pitch)
                @ ImageProcessor._Rz(new_yaw)
        )

        # 5. 将 R_rel_orig 转到 OpenCV 坐标系（光轴=Z_o，X_o→右，Y_o→下）
        Q = np.array([
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0],
            [1.0, 0.0, 0.0]
        ], dtype=np.float64)
        R_rel_cv = Q @ R_delta @ Q.T

        # 6. 构造单应矩阵 H = K * R_rel_cv * K^{-1}
        H = K @ R_rel_cv @ np.linalg.inv(K)

        # 7. 用 cv2.warpPerspective 做重映射
        img_rot = cv2.warpPerspective(
            cimg.image,
            H,
            (W_new, H_new),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(0, 0, 0)
        )

        # 8. 如果要更新主点 (center_u, center_v)，可以在这里计算新的 cu_out, cv_out
        #    若主点保持不变，则直接用原值：
        cu_out = cimg.center_u
        cv_out = cimg.center_v
        # -- 如果你确实需要基于输出尺寸居中，可改为：
        # cu_out = W_new / 2.0
        # cv_out = H_new / 2.0

        # 9. 用 dataclasses.replace 生成新的 CalibratedImage
        return replace(
            cimg,
            image=img_rot,
            image_width=W_new,
            image_height=H_new,
            center_u=cu_out,
            center_v=cv_out,
            yaw=cimg.yaw - new_yaw,  # TODO 这里没有仔细验证写的对不对
            pitch=cimg.pitch - new_pitch,
            roll=cimg.roll - new_roll,
        )

    @staticmethod
    def translate(
            cimg: CalibratedImage,
            depth: np.ndarray,
            t_x: float = 0.0,
            t_y: float = 0.0,
            t_z: float = 0.0,
            out_size: tuple[int, int] | None = None,
    ) -> "CalibratedImage":
        """
        参数
        ----
        cimg      : 输入 CalibratedImage（含 RGB、内参、外参）
        depth     : 与 cimg 对齐的深度图，X 轴正向为“前”
        t_x/y/z   : 相机在自身坐标系的平移量
        out_size  : 输出图像尺寸 (W, H)，默认与输入相同
        """
        # ---------- 0. 基础 ----------
        H, W = cimg.image_height, cimg.image_width
        W_out, H_out = out_size if out_size else (W, H)
        u0, v0 = cimg.center_u, cimg.center_v
        f_u, f_v = cimg.focal_u, cimg.focal_v
        eps = 1e-6

        # ---------- 1. 展开像素 & 过滤无效深度 ----------
        uu, vv = np.meshgrid(np.arange(W, dtype=np.float32),
                             np.arange(H, dtype=np.float32))
        uu, vv = uu.ravel(), vv.ravel()
        X_old = depth.ravel().astype(np.float32)

        valid = X_old > eps
        if not np.any(valid):
            blank = np.zeros((H_out, W_out, 3), np.uint8)
            return replace(cimg, image=blank,
                           image_width=W_out, image_height=H_out)

        uu, vv, X_old = uu[valid], vv[valid], X_old[valid]
        Y_old = (u0 - uu) / f_u * X_old
        Z_old = (v0 - vv) / f_v * X_old

        # ---------- 2. 平移 ----------
        Xn = X_old + t_x
        Yn = Y_old + t_y
        Zn = Z_old + t_z
        in_front = Xn > eps
        if not np.any(in_front):
            blank = np.zeros((H_out, W_out, 3), np.uint8)
            return replace(cimg, image=blank,
                           image_width=W_out, image_height=H_out)

        Xn, Yn, Zn = Xn[in_front], Yn[in_front], Zn[in_front]
        uu, vv = uu[in_front], vv[in_front]

        # ---------- 3. 投影 ----------
        j = np.round(u0 - f_u * (Yn / Xn)).astype(np.int32)
        i = np.round(v0 - f_v * (Zn / Xn)).astype(np.int32)
        in_img = (0 <= i) & (i < H_out) & (0 <= j) & (j < W_out)
        if not np.any(in_img):
            blank = np.zeros((H_out, W_out, 3), np.uint8)
            return replace(cimg, image=blank,
                           image_width=W_out, image_height=H_out)

        i, j, Xn = i[in_img], j[in_img], Xn[in_img]
        uu = uu[in_img].astype(np.int32)
        vv = vv[in_img].astype(np.int32)
        idx_dst = i * W_out + j  # 线性下标

        # ---------- 4. 向量化 Z-Buffer ----------
        zbuf = np.full(H_out * W_out, np.inf, np.float32)
        np.minimum.at(zbuf, idx_dst, Xn)  # 最近深度
        keep = Xn == zbuf[idx_dst]

        new_img = np.zeros((H_out * W_out, 3), np.uint8)
        new_img[idx_dst[keep]] = cimg.image[vv[keep], uu[keep]]
        new_img = new_img.reshape(H_out, W_out, 3)

        # ---------- 5. 统一 inpaint 填洞 ----------
        mask = (new_img[..., 0] | new_img[..., 1] | new_img[..., 2]) == 0
        if np.any(mask):
            new_img = cv2.inpaint(
                new_img,
                (mask.astype(np.uint8) * 255),
                3,  # 半径，可按需调整
                cv2.INPAINT_NS  # 或 cv2.INPAINT_TELEA
            )

        # ---------- 6. 打包 ----------
        return replace(
            cimg,
            image=new_img,
            image_width=W_out,
            image_height=H_out,
            world_x=cimg.world_x - t_x,
            world_y=cimg.world_y - t_y,
            world_z=cimg.world_z - t_z,
        )

    # ---------- 裁剪（按比例） ----------
    @staticmethod
    def crop(
            cimg: CalibratedImage,
            t: float = 0.0,
            b: float = 0.0,
            l: float = 0.0,
            r: float = 0.0,
    ) -> CalibratedImage:
        """
        按比例裁剪图像四周，并同步更新主点。

        参数（0.0–1.0）：
          t: 上方裁剪比例
          b: 下方裁剪比例
          l: 左侧裁剪比例
          r: 右侧裁剪比例
        """

        # 合法性检查
        for name, v in (("top", t), ("bottom", b), ("left", l), ("right", r)):
            if not (0.0 <= v <= 1.0):
                raise ValueError(f"{name} 必须在 [0,1] 之间")
        if t + b >= 1.0 or l + r >= 1.0:
            raise ValueError("上下或左右裁剪比例之和须 < 1")

        # 转为像素
        w, h = cimg.image_width, cimg.image_height
        tp = int(round(t * h))
        bp = int(round(b * h))
        lp = int(round(l * w))
        rp = int(round(r * w))

        # 计算裁剪区域
        y0, y1 = tp, h - bp
        x0, x1 = lp, w - rp
        img2 = cimg.image[y0:y1, x0:x1].copy()
        nw, nh = x1 - x0, y1 - y0

        # 更新主点并返回
        return replace(
            cimg,
            image=img2,
            center_u=cimg.center_u - x0,
            center_v=cimg.center_v - y0,
            image_width=nw,
            image_height=nh,
        )

    @staticmethod
    def crop_by_point(
            cimg: CalibratedImage,
            target_u: float,
            target_v: float
    ) -> CalibratedImage:
        """
        自动裁剪，使得裁剪后图像的主点移动到 (target_u, target_v)。

        参数
        ----
        cimg : CalibratedImage
            输入的已标定图像
        target_u, target_v : float
            裁剪后希望的主点坐标（像素）

        返回
        ----
        CalibratedImage
            裁剪并更新主点后的新图像
        """
        W, H = cimg.image_width, cimg.image_height
        # 计算原主点到目标主点的偏移量（像素）
        delta_u = cimg.center_u - target_u
        delta_v = cimg.center_v - target_v

        # 左右裁剪比例（>0 表示裁左，<0 表示裁右）
        l = max(delta_u, 0.0) / W
        r = max(-delta_u, 0.0) / W
        # 上下裁剪比例（>0 表示裁上，<0 表示裁下）
        t = max(delta_v, 0.0) / H
        b = max(-delta_v, 0.0) / H

        # 合法性检查
        if l + r >= 1.0 or t + b >= 1.0:
            raise ValueError(
                f"目标主点 (u={target_u}, v={target_v}) 距离图像边界过近，无法裁剪"
            )

        return ImageProcessor.crop(
            cimg,
            t=t, b=b,
            l=l, r=r,
        )

    # ---------- 缩放 ----------
    @staticmethod
    def scale(cimg: CalibratedImage, new_h: int, new_w: int) -> CalibratedImage:
        """
        将图像与标定参数一起缩放至指定尺寸，并自动选择最佳插值。

        参数
        -------
        new_h: 目标高度（像素）
        new_w: 目标宽度（像素）
        """
        old_w, old_h = cimg.image_width, cimg.image_height
        if new_w <= 0 or new_h <= 0:
            raise ValueError("new_w/new_h 必须为正整数")
        if new_w == old_w and new_h == old_h:
            return cimg  # 尺寸未变，直接返回

        # 计算缩放比例
        scale_w = new_w / old_w
        scale_h = new_h / old_h

        # 自动选择插值
        up_h = new_h > old_h
        up_w = new_w > old_w
        if up_h and up_w:  # 双向放大
            sf = max(scale_w, scale_h)
            interp = cv2.INTER_LANCZOS4 if sf >= 1.5 else cv2.INTER_CUBIC
        elif (not up_h) and (not up_w):  # 双向缩小
            interp = cv2.INTER_AREA
        else:  # 非等比：一边放大一边缩小
            interp = cv2.INTER_LINEAR

        # 重采样图像
        img_resized = cv2.resize(
            cimg.image,
            (int(new_w), int(new_h)),
            interpolation=interp
        )

        # 更新相机内参（像素坐标系下）
        return replace(
            cimg,
            image=img_resized,
            focal_u=cimg.focal_u * scale_w,
            focal_v=cimg.focal_v * scale_h,
            center_u=cimg.center_u * scale_w,
            center_v=cimg.center_v * scale_h,
            image_width=int(new_w),
            image_height=int(new_h),
        )

    @staticmethod
    def cam_points_to_uv(
            cimg: CalibratedImage,
            cam_points: np.ndarray
    ) -> np.ndarray:
        """
        计算相机坐标系下三维点在图像上的像素坐标，不进行绘制，仅返回 uv_pix。

        参数
        ----
        cam_points | np.ndarray, shape=(…,3)
            若干相机坐标系下的三维点，最后一维为 [X_c, Y_c, Z_c]。
        cimg | CalibratedImage
            已去畸变的图像及内外参，包含：
              - image: BGR 图像数据（此处未使用，仅保留以兼容）
              - f_u, f_v: 焦距
              - u0, v0: 主点

        返回
        ----
        uv_pix | np.ndarray, shape=(…,2), dtype=int
            每个点的像素坐标，深度 ≤ 0 或投影外的点为 (-1, -1)。
        """

        # 只检查最后一维
        if cam_points.ndim < 1 or cam_points.shape[-1] != 3:
            raise ValueError(f"cam_points 最后一维需为 3，当前形状为 {cam_points.shape}")

        # 原始前置维度
        orig_shape = cam_points.shape[:-1]
        total_pts = int(np.prod(orig_shape))

        # 展平到 (total_pts, 3)
        pts_flat = cam_points.reshape(-1, 3)

        # 构造齐次坐标并投影
        ones = np.ones((total_pts, 1), dtype=np.float64)
        pts_homo = np.hstack((pts_flat, ones))  # (total_pts,4)
        P = ImageProcessor._P_cam2pixel(cimg)  # (3,4)
        proj = (P @ pts_homo.T).T  # (total_pts,3)
        z = proj[:, 2]

        # 计算像素坐标
        valid = np.isfinite(z) & (z > 0)
        u = np.full_like(z, -1, dtype=np.int32)
        v = np.full_like(z, -1, dtype=np.int32)

        u[valid] = np.round(proj[valid, 0] / z[valid]).astype(np.int32)
        v[valid] = np.round(proj[valid, 1] / z[valid]).astype(np.int32)

        # 合并并恢复原始形状
        uv_flat = np.stack((u, v), axis=1)  # (total_pts,2)
        uv_pix = uv_flat.reshape(*orig_shape, 2)

        return uv_pix

    @staticmethod
    def world_points_to_uv(
            cimg: CalibratedImage,
            world_points: np.ndarray,
    ) -> np.ndarray:
        """
        将世界坐标系下的三维点投影到图像上，仅返回像素坐标 uv_pix，不做绘制。

        参数
        ----
        world_points | np.ndarray, shape=(…,3)
            任意维度的世界坐标点，最后一维为 [X_w, Y_w, Z_w]。
        cimg | CalibratedImage
            已去畸变的图像及其内外参，包含：
              - image: BGR 图像数据（此处未使用，仅为兼容）
              - 外参由 ImageProcessor._Rt_active(cimg) 提供，shape=(3,4)

        返回
        ----
        uv_pix | np.ndarray, shape=(…,2), dtype=int
            每个点在图像上的像素坐标，深度 ≤ 0 或超出图像范围的点为 (-1, -1)。
        """

        # 1. 世界坐标 -> 相机坐标
        Rt = ImageProcessor._Rt_active(cimg)  # (3,4)

        # 检查最后一维
        if world_points.ndim < 1 or world_points.shape[-1] != 3:
            raise ValueError(f"world_points 最后一维需为 3，当前形状为 {world_points.shape}")

        # 展平除最后一维外的所有维度
        orig_shape = world_points.shape[:-1]
        total_pts = int(np.prod(orig_shape))
        pts_w_flat = world_points.reshape(-1, 3)

        # 构造齐次坐标并转换到相机系
        ones = np.ones((total_pts, 1), dtype=np.float64)
        pts_w_homo = np.hstack((pts_w_flat, ones))  # (total_pts,4)
        pts_cam_flat = (Rt @ pts_w_homo.T).T  # (total_pts,3)

        # 恢复到原始前置维度
        cam_points = pts_cam_flat.reshape(*orig_shape, 3)

        # 2. 调用 cam_points_to_uv，仅返回 uv_pix
        uv_pix = ImageProcessor.cam_points_to_uv(
            cimg=cimg,
            cam_points=cam_points
        )
        return uv_pix
