from __future__ import annotations
import math
import numpy as np
import cv2
from typing import Any


# ===================== 世界接地点与 3D 盒（单图版）=====================
class Cyl3DBoxEstimator:
    """
    圆柱相机下：由 2D 框 → 接地像素 / 射线 → 3D 立方体 (cx,cy,cz,l,w,h,theta_abs)

    - cyl_cam：新结构（center_* 为光轴像素；R_act/t_act 为 世界→圆柱 主动外参；R_cyl_to_cam = Ry(-pitch)）
    - whitelist：{name: {"id": cls_id, "size": [l,w,h]}, ...}
    - objs: (N,7) = [x1,y1,x2,y2,conf,cls,theta_rel]，theta_rel 为**相对视线**的角
    - z: 标量或 (N,)；世界系平面 Z=z（现在只作为先验 / fallback）
    - masks: (N,H0,W0) uint8 掩膜，坐标与 objs 在同一尺度

    重要说明（本次修改点）：
    - 行人（cid==0）在 XY 平面用“圆柱体底面圆”建模：
        * 底面中心：由两条射线与圆相切求得
        * z_base：由 mask 底点射线与“底面圆（不是矩形边）”求交，反推 z_base
      这样保证建模一致性，并让 z_base 对 theta_abs 不敏感（更符合圆柱体）。
    """

    def __init__(self, cyl_cam: dict[str, Any], whitelist: dict[str, Any]) -> None:
        # —— 内参与外参 —— #
        self.f: float = float(cyl_cam["focal_u"])
        self.cu: float = float(cyl_cam["center_u"])  # 光轴像素（可能不在画布内）
        self.cv: float = float(cyl_cam["center_v"])
        R_act = np.asarray(cyl_cam["R_act"], dtype=np.float64)  # 世界→圆柱（主动）
        t_act = np.asarray(cyl_cam["t_act"], dtype=np.float64).reshape(3)
        self.R_w2c: np.ndarray = R_act
        self.R_c2w: np.ndarray = R_act.T
        self.t: np.ndarray = t_act
        self.C_w: np.ndarray = -self.R_c2w @ self.t  # 世界系相机中心

        # R_cyl_to_cam = Ry(-pitch)，这里用它把 pitch 以 tan(pitch) 的形式塞进圆柱纵向项
        R_c2c = np.asarray(cyl_cam["R_cyl_to_cam"], dtype=np.float64)
        self.tan_pitch: float = float(-R_c2c[0, 2] / (R_c2c[0, 0] + 1e-12))

        # —— 白名单 id -> (l,w,h) —— #
        # 注意：这里假设 whitelist 的 key 可以直接 int(k) 得到 cls_id
        # 如果你的 whitelist 真实结构是 name->{"id":...,"size":...}，需要在外部先转换成 id->meta
        self.id2size: dict[int, tuple[float, float, float]] = {}
        for k, v in whitelist.items():
            self.id2size[int(k)] = tuple(v["size"])

        # 轨迹级“深度-像素高”稳定量：k ~= depth * h_pix
        self._track_depth_k: dict[int, float] = {}
        self._solver_mode: str = "ground0_dynamic_width"
        self._use_mask_bbox_refine_in_cuboids: bool = False

    # ========= 内部工具：角度/扇形/矩形解算 =========

    def _normalize_angle(self, angle: float) -> float:
        """归一化角度到 [0, 2π)."""
        two_pi = 2.0 * math.pi
        angle = math.fmod(angle, two_pi)
        if angle < 0.0:
            angle += two_pi
        return angle

    def _canonicalize_rays(self, alpha: float, beta: float, eps: float = 1e-9) -> tuple[float, float]:
        """
        任意给定两条射线角度 alpha, beta（弧度，正负都行，顺序也随意），
        返回规范化后的 (alpha_c, beta_c)，满足：
          - 都在 [0, 2π) 内
          - 从 alpha_c 逆时针转到 beta_c 的夹角在 (0, π) 内（较小的那个夹角）
        """
        two_pi = 2.0 * math.pi
        a = self._normalize_angle(alpha)
        b = self._normalize_angle(beta)

        diff = (b - a) % two_pi  # 从 a 逆时针到 b

        # 平行或反向：没有真正的楔形区域
        if diff < eps or abs(diff - math.pi) < eps:
            raise ValueError("Rays are (almost) parallel or opposite; acute wedge not well-defined.")

        # 如果从 a 到 b 的角度 > π，则说明小扇形在反方向，交换一下
        if diff > math.pi:
            a, b = b, a

        return a, b

    def _cross(self, ax: float, ay: float, bx: float, by: float) -> float:
        """2D 叉积 a × b 的 z 分量。"""
        return ax * by - ay * bx

    def _find_rectangle_centers_between_rays(
            self,
            length: float,
            width: float,
            theta: float,
            alpha: float,
            beta: float,
            *,
            check_inside_wedge: bool = True,
            eps: float = 1e-9,
    ) -> list[tuple[float, float]]:
        """
        已知：
          - 矩形长宽 length, width
          - 朝向角 theta（相对 x 轴，弧度）
          - 两条射线方向角 alpha, beta（弧度）

        坐标系：
          - 原点在相机地面投影
          - x/y 轴与世界系 XY 平行
          - 射线起点均在原点

        返回所有可能的矩形中心 (cx, cy)，使得：
          - 矩形有两个顶点分别落在射线 1 / 射线 2 上；
          - 矩形整体位于两射线夹的锐角扇形内（含边界）。
        """
        alpha_c, beta_c = self._canonicalize_rays(alpha, beta, eps=eps)

        # 射线单位方向
        u1x = math.cos(alpha_c)
        u1y = math.sin(alpha_c)
        u2x = math.cos(beta_c)
        u2y = math.sin(beta_c)

        half_l = 0.5 * length
        half_w = 0.5 * width
        local_vertices: list[tuple[float, float]] = [
            (half_l, half_w),    # v0
            (-half_l, half_w),   # v1
            (-half_l, -half_w),  # v2
            (half_l, -half_w),   # v3
        ]

        c = math.cos(theta)
        s = math.sin(theta)

        def rotate(vx: float, vy: float) -> tuple[float, float]:
            return c * vx - s * vy, s * vx + c * vy

        centers_raw: list[tuple[float, float]] = []

        for i1 in range(4):
            for i2 in range(4):
                if i1 == i2:
                    continue

                v1x, v1y = local_vertices[i1]
                v2x, v2y = local_vertices[i2]

                dvx, dvy = v1x - v2x, v1y - v2y
                kx, ky = rotate(dvx, dvy)

                # 解 t1, t2：t1 * u1 - t2 * u2 = R(v1 - v2)
                det = u1x * (-u2y) - u1y * (-u2x)  # = -(u1 × u2)
                if abs(det) < eps:
                    continue

                inv11 = -u2y / det
                inv12 = u2x / det
                inv21 = -u1y / det
                inv22 = u1x / det

                t1 = inv11 * kx + inv12 * ky
                t2 = inv21 * kx + inv22 * ky

                # 必须在射线正方向
                if t1 < -eps or t2 < -eps:
                    continue

                rv1x, rv1y = rotate(v1x, v1y)
                cx = t1 * u1x - rv1x
                cy = t1 * u1y - rv1y

                if not check_inside_wedge:
                    centers_raw.append((cx, cy))
                    continue

                # 还原四个世界顶点，检查是否在楔形内部
                world_vertices: list[tuple[float, float]] = []
                for vx, vy in local_vertices:
                    rx, ry = rotate(vx, vy)
                    px = cx + rx
                    py = cy + ry
                    world_vertices.append((px, py))

                p1x, p1y = world_vertices[i1]
                p2x, p2y = world_vertices[i2]

                # 顶点在射线上：u × p ≈ 0
                if abs(self._cross(u1x, u1y, p1x, p1y)) > 1e-6:
                    continue
                if abs(self._cross(u2x, u2y, p2x, p2y)) > 1e-6:
                    continue

                # 落在射线正向：点与射线方向点积 >= 0
                if p1x * u1x + p1y * u1y < -eps:
                    continue
                if p2x * u2x + p2y * u2y < -eps:
                    continue

                # 其他顶点必须在楔形内部（含边界）
                valid = True
                for idx, (px, py) in enumerate(world_vertices):
                    if idx == i1 or idx == i2:
                        continue
                    c1 = self._cross(u1x, u1y, px, py)  # >= 0: 在 ray1 左侧或线上
                    c2 = self._cross(u2x, u2y, px, py)  # <= 0: 在 ray2 右侧或线上
                    if c1 < -1e-6 or c2 > 1e-6:
                        valid = False
                        break

                if not valid:
                    continue

                centers_raw.append((cx, cy))

        # 去重
        centers: list[tuple[float, float]] = []
        for cx, cy in centers_raw:
            duplicated = False
            for ux, uy in centers:
                if math.hypot(cx - ux, cy - uy) < 1e-6:
                    duplicated = True
                    break
            if not duplicated:
                centers.append((cx, cy))

        return centers

    def _find_circle_center_tangent_to_rays(
            self,
            radius: float,
            alpha: float,
            beta: float,
            *,
            eps: float = 1e-9,
    ) -> tuple[float, float] | None:
        """
        行人特例：用底面矩形外接圆作为轮廓。求一个半径为 radius 的圆，
        与原点出发的两条射线（取锐角扇形）都相切时的圆心 (cx, cy)。

        若不可解返回 None。
        """
        if radius <= 0.0 or not math.isfinite(radius):
            return None

        try:
            a, b = self._canonicalize_rays(alpha, beta, eps=eps)
        except ValueError:
            return None

        two_pi = 2.0 * math.pi
        delta = (b - a) % two_pi  # (0, pi)
        if delta <= eps or delta >= math.pi - eps:
            return None

        s = math.sin(0.5 * delta)
        if abs(s) < eps:
            return None

        # 圆心位于角平分线上，距离 d = r / sin(delta/2)
        d = radius / s
        theta_b = a + 0.5 * delta
        cx = d * math.cos(theta_b)
        cy = d * math.sin(theta_b)
        return cx, cy

    # ---------- 工具：由像素求世界系射线方向（未归一也可） ----------
    def _world_ray_dir(self, u: float, v: float) -> np.ndarray:
        """
        圆柱模型下，由像素 (u,v) 构造一个世界系射线方向（长度不要求归一化）。
        - 水平方向：phi = (u - cu) / f
        - 垂直方向：Y_rel = (cv - v) / f，并减 pitch 的 tan 修正得到 Y_abs
        """
        phi = (u - self.cu) / self.f
        Y_rel = (self.cv - v) / self.f
        Y_abs = Y_rel - self.tan_pitch
        cphi, sphi = math.cos(phi), math.sin(phi)
        dir_cyl = np.array([cphi, -sphi, Y_abs], dtype=np.float64)  # 圆柱系
        d_w = self.R_c2w @ dir_cyl  # 世界系
        return d_w

    def _intersect_ray_with_plane_z(
            self,
            *,
            cam_cx: float,
            cam_cy: float,
            cam_cz: float,
            ray_dir: np.ndarray,
            z_plane: float,
            eps: float = 1e-9,
    ) -> tuple[tuple[float, float, float] | None, float | None]:
        dx = float(ray_dir[0])
        dy = float(ray_dir[1])
        dz = float(ray_dir[2])

        if not (math.isfinite(dx) and math.isfinite(dy) and math.isfinite(dz)):
            return None, None
        if abs(dz) < eps:
            return None, None

        lam = (float(z_plane) - float(cam_cz)) / dz
        if (not math.isfinite(lam)) or lam <= eps:
            return None, None

        px = float(cam_cx) + lam * dx
        py = float(cam_cy) + lam * dy
        if not (math.isfinite(px) and math.isfinite(py)):
            return None, None

        return (px, py, float(z_plane)), float(lam)

    def _estimate_lr_points_from_mask_levels(
            self,
            mask_i: np.ndarray,
            x1: float,
            x2: float,
            y2: float,
            *,
            sample_levels: int = 9,
            top_ratio: float = 0.1,
            bottom_ratio: float = 0.9,
            min_pixels_per_row: int = 2,
    ) -> tuple[float, float, float, bool]:
        """
        由 mask 按不同高度采样左右点，并独立做平均：
        - 在 mask 高度方向按多个层级采样
        - 每层提取最左前景点 left_x、最右前景点 right_x
        - 对 left_x 集合与 right_x 集合分别独立求平均
        返回:
            (u_left, u_right, v_ref, ok)
        """
        if mask_i.ndim != 2:
            return float(x1), float(x2), float(y2), False

        ys, xs = np.where(mask_i > 0)
        if ys.size == 0:
            return float(x1), float(x2), float(y2), False

        y_min = int(ys.min())
        y_max = int(ys.max())
        if y_max <= y_min:
            return float(x1), float(x2), float(y2), False

        h = float(y_max - y_min)
        v_start = int(round(y_min + top_ratio * h))
        v_end = int(round(y_min + bottom_ratio * h))
        v_start = max(y_min, min(y_max, v_start))
        v_end = max(y_min, min(y_max, v_end))
        if v_end < v_start:
            v_start, v_end = v_end, v_start

        lefts: list[float] = []
        rights: list[float] = []
        rows: list[float] = []

        if sample_levels <= 1:
            sample_vs = [int(round(0.5 * (v_start + v_end)))]
        else:
            sample_vs = [
                int(round(v_start + (v_end - v_start) * k / (sample_levels - 1)))
                for k in range(sample_levels)
            ]

        for v in sample_vs:
            xs_row = np.where(mask_i[v] > 0)[0]
            if xs_row.size < min_pixels_per_row:
                continue
            lefts.append(float(xs_row.min()))
            rights.append(float(xs_row.max()))
            rows.append(float(v))

        if not lefts or not rights:
            return float(x1), float(x2), float(y2), False

        # 左右边界独立平均（不是宽度平均）
        u_left = float(np.mean(lefts))
        u_right = float(np.mean(rights))
        v_ref = float(np.mean(rows))

        if not (math.isfinite(u_left) and math.isfinite(u_right) and math.isfinite(v_ref)):
            return float(x1), float(x2), float(y2), False
        if u_right <= u_left:
            return float(x1), float(x2), float(y2), False

        return u_left, u_right, v_ref, True

    def _stabilize_depth_with_top_boundary(
            self,
            *,
            track_id: int | None,
            cx: float,
            cy: float,
            cam_cx: float,
            cam_cy: float,
            y_top: float,
            y_bottom: float,
            ema_alpha: float = 0.06,
            blend_bottom: float = 0.7,
    ) -> tuple[float, float]:
        """
        用 2D box 上边界（像素高）稳定深度，不显式拟合物理高度：
        - 定义每条轨迹隐变量 k ~= depth * h_pix
        - depth_top = k / h_pix
        - depth_fused = blend_bottom*depth_bottom + (1-blend_bottom)*depth_top
        - 沿“相机->目标”的方向缩放 (cx, cy)
        """
        if track_id is None or track_id < 0:
            return cx, cy

        h_pix = float(y_bottom - y_top)
        if not math.isfinite(h_pix) or h_pix < 2.0:
            return cx, cy

        dx = float(cx - cam_cx)
        dy = float(cy - cam_cy)
        depth_bottom = math.hypot(dx, dy)
        if not math.isfinite(depth_bottom) or depth_bottom < 1e-6:
            return cx, cy

        k_prev = self._track_depth_k.get(track_id)
        if k_prev is None or not math.isfinite(k_prev) or k_prev <= 0.0:
            self._track_depth_k[track_id] = depth_bottom * h_pix
            return cx, cy

        depth_top = k_prev / h_pix
        if not math.isfinite(depth_top) or depth_top <= 0.0:
            self._track_depth_k[track_id] = depth_bottom * h_pix
            return cx, cy

        # 防止单帧异常框导致深度跳变
        depth_top = float(np.clip(depth_top, 0.5 * depth_bottom, 1.5 * depth_bottom))

        w = float(np.clip(blend_bottom, 0.0, 1.0))
        depth_fused = w * depth_bottom + (1.0 - w) * depth_top
        if not math.isfinite(depth_fused) or depth_fused <= 0.0:
            self._track_depth_k[track_id] = depth_bottom * h_pix
            return cx, cy

        scale = depth_fused / depth_bottom
        cx_new = cam_cx + dx * scale
        cy_new = cam_cy + dy * scale

        # 更新轨迹隐变量（慢更新，抑制漂移）
        meas = depth_fused * h_pix
        if not math.isfinite(meas) or meas <= 0.0:
            meas = depth_bottom * h_pix
        a = float(np.clip(ema_alpha, 0.0, 1.0))
        self._track_depth_k[track_id] = (1.0 - a) * k_prev + a * meas

        return float(cx_new), float(cy_new)

    # ---------- 工具：射线与底边交点 -> 反推 z_base（矩形底面） ----------
    def _intersect_ray_with_bottom_edges_z(
            self,
            cam_cx: float,
            cam_cy: float,
            cam_cz: float,
            ray_dir: np.ndarray,
            bottom_corners_xy: np.ndarray,
            *,
            eps: float = 1e-9,
    ) -> tuple[float | None, float | None]:
        """
        已知：
          - 相机中心 C = (cam_cx, cam_cy, cam_cz)
          - 射线方向 ray_dir (世界系，长度任意) -> 只用到 d_xy, d_z
          - 物体底面四个角的 XY 坐标 bottom_corners_xy: (4,2)，顺序需与 edges 定义一致
        要做：
          - 求这条射线与“底面矩形四条边”的交点（如果有的话）；
          - 交点必须：
              * 在射线前方 (λ > 0)
              * 在线段内部 (0 <= μ <= 1)
          - 若有多个交点，取 λ 最小的那个；
          - 返回 (z_base, best_lambda)，其中 z_base 是底面 Z 坐标。

        若无交点则返回 (None, None)。
        """
        dx = float(ray_dir[0])
        dy = float(ray_dir[1])
        dz = float(ray_dir[2])

        Cx, Cy, Cz = cam_cx, cam_cy, cam_cz

        # 四条边：0-1,1-2,2-3,3-0
        edges = [(0, 1), (1, 2), (2, 3), (3, 0)]

        best_lambda = None
        best_z_base = None

        for i0, i1 in edges:
            x0, y0 = bottom_corners_xy[i0]
            x1, y1 = bottom_corners_xy[i1]
            ex, ey = (x1 - x0), (y1 - y0)

            det = dx * (-ey) - dy * (-ex)  # = -(d_xy × e_xy)
            if abs(det) < eps:
                continue

            inv11 = -ey / det
            inv12 = ex / det
            inv21 = -dy / det
            inv22 = dx / det

            bx = x0 - Cx
            by = y0 - Cy

            lam = inv11 * bx + inv12 * by
            mu = inv21 * bx + inv22 * by

            if lam <= eps:
                continue
            if mu < -eps or mu > 1.0 + eps:
                continue

            z_base = Cz + lam * dz

            if best_lambda is None or lam < best_lambda:
                best_lambda = lam
                best_z_base = z_base

        return best_z_base, best_lambda

    # ---------- 新增：射线与底面圆求交 -> 反推 z_base（圆形底面，行人） ----------
    def _intersect_ray_with_bottom_circle_z(
            self,
            cam_cx: float,
            cam_cy: float,
            cam_cz: float,
            ray_dir: np.ndarray,
            circle_center_xy: tuple[float, float],
            radius: float,
            *,
            eps: float = 1e-9,
    ) -> tuple[float | None, float | None]:
        """
        行人圆柱体底面：在 XY 平面把底面看作圆（中心 O_xy，半径 r）。

        已知：
          - 相机中心 C = (cam_cx, cam_cy, cam_cz)
          - 世界系射线：P(λ) = C + λ * d
          - 圆方程（XY）：||P_xy(λ) - O_xy||^2 = r^2
        求：
          - 射线与圆周的交点参数 λ（取最小的正根，表示“沿射线前方的第一次相交”）
          - 再用 z_base = cam_cz + λ * dz 反推出底面高度

        返回：
          - (z_base, best_lambda)
          - 若无解/退化，则返回 (None, None)
        """
        if radius <= 0.0 or not math.isfinite(radius):
            return None, None

        dx = float(ray_dir[0])
        dy = float(ray_dir[1])
        dz = float(ray_dir[2])

        # 射线在 XY 平面的方向长度平方
        A = dx * dx + dy * dy
        if A < eps:
            # 射线在 XY 几乎没有分量（几乎竖直），无法与圆做稳定求交
            return None, None

        ox, oy = float(circle_center_xy[0]), float(circle_center_xy[1])

        # 设 P_xy(λ) = (cam_cx,cam_cy) + λ*(dx,dy)
        # 令 q = C_xy - O_xy
        qx = cam_cx - ox
        qy = cam_cy - oy

        # 二次方程：A λ^2 + B λ + C = 0
        B = 2.0 * (dx * qx + dy * qy)
        C = (qx * qx + qy * qy) - radius * radius

        disc = B * B - 4.0 * A * C
        if disc < 0.0:
            return None, None

        sdisc = math.sqrt(disc)
        lam1 = (-B - sdisc) / (2.0 * A)
        lam2 = (-B + sdisc) / (2.0 * A)

        best = None
        if lam1 > eps:
            best = lam1
        if lam2 > eps and (best is None or lam2 < best):
            best = lam2

        if best is None:
            # 两个交点都在射线后方或过近
            return None, None

        z_base = cam_cz + best * dz
        return float(z_base), float(best)

    # ---------- 新版：由 2D 框 + 先验尺寸 + mask 底点 → 3D 立方体 ----------
    def cuboids_from_boxes(
            self,
            objs: np.ndarray,                 # (N,7)  [x1,y1,x2,y2,conf,cls,theta_rel]
            z: float | int | np.ndarray,      # 标量或 (N,) —— 先验 / fallback
            *,
            masks: np.ndarray | None = None,  # (N,H0,W0) uint8，坐标与 objs 同一尺度
            track_ids: np.ndarray | None = None,  # (N,) 跟踪 ID；用于上边界深度稳定
    ) -> np.ndarray:
        """
        从 2D 旋转框推算 3D cuboids。

        输出:
            cuboids: (N,9)
                [cx, cy, cz, l, w, h, conf, cls, theta_abs]

            对于几何/先验都无法解出的目标：
                cx, cy, cz = 0
                l, w, h   = -1
            conf, cls, theta_abs 仍来自原始 objs（theta_abs 解不出时为 0）。
        """
        arr = np.asarray(objs, dtype=np.float64)
        if arr.ndim != 2 or arr.shape[1] != 7:
            raise ValueError("objs 形状需为 (N,7)=[x1,y1,x2,y2,conf,cls,theta_rel]")
        N = arr.shape[0]

        # z 向量化（仍保留，作为先验 / fallback）
        if isinstance(z, (int, float)):
            z_vec = np.full((N,), float(z), dtype=np.float64)
        else:
            z_vec = np.asarray(z, dtype=np.float64).reshape(-1)
        if z_vec.shape[0] != N:
            raise ValueError("z 为向量时必须与 objs 数量一致")

        if masks is not None:
            masks = np.asarray(masks)
            if masks.shape[0] != N:
                raise ValueError("masks 第 0 维应与 objs 数量一致")
        if track_ids is not None:
            track_ids = np.asarray(track_ids).reshape(-1)
            if track_ids.shape[0] != N:
                raise ValueError("track_ids 长度必须与 objs 数量一致")

        cls_ids = arr[:, 5].astype(np.int64)
        confs = arr[:, 4].astype(np.float64)
        theta_rel = arr[:, 6].astype(np.float64)

        # 2D 框（像素坐标）
        x1, y1, x2, y2 = arr[:, 0], arr[:, 1], arr[:, 2], arr[:, 3]

        # 相机中心在世界系坐标（扇形坐标系原点）
        cam_cx = float(self.C_w[0])
        cam_cy = float(self.C_w[1])
        cam_cz = float(self.C_w[2])

        # 输出：预先分配 (N,9)
        cuboids = np.zeros((N, 9), dtype=np.float32)
        cuboids[:, 3:6] = -1.0  # l,w,h = -1

        for i in range(N):
            cid = int(cls_ids[i])
            conf = float(confs[i])

            # 默认：把 conf / cls 抄进去，即使后面几何失败，行里也有检测信息
            cuboids[i, 6] = conf
            cuboids[i, 7] = float(cid)

            size = self.id2size.get(cid)
            if size is None:
                continue

            l, w, h = size
            z_plane_prior = float(z_vec[i])  # 作为 fallback

            # 左右边界默认取 box，下游可被 mask 采样平均结果覆盖
            # Use objs bbox as-is by default; no extra mask-based bbox correction here.
            u_left_i = float(x1[i])
            u_right_i = float(x2[i])
            v_bot_i = float(y2[i])
            if self._use_mask_bbox_refine_in_cuboids and (masks is not None):
                u_left_m, u_right_m, v_bot_m, ok_lr = self._estimate_lr_points_from_mask_levels(
                    mask_i=np.asarray(masks[i]),
                    x1=u_left_i,
                    x2=u_right_i,
                    y2=v_bot_i,
                )
                if ok_lr:
                    u_left_i, u_right_i, v_bot_i = u_left_m, u_right_m, v_bot_m

            u_mid_i = 0.5 * (u_left_i + u_right_i)

            # 左下 / 右下 像素对应的世界射线方向
            dL = self._world_ray_dir(u_left_i, v_bot_i)
            dR = self._world_ray_dir(u_right_i, v_bot_i)

            dxL, dyL = float(dL[0]), float(dL[1])
            dxR, dyR = float(dR[0]), float(dR[1])

            if not (math.isfinite(dxL) and math.isfinite(dyL) and math.isfinite(dxR) and math.isfinite(dyR)):
                continue
            if math.hypot(dxL, dyL) < 1e-8 or math.hypot(dxR, dyR) < 1e-8:
                continue

            alpha = math.atan2(dyL, dxL)
            beta = math.atan2(dyR, dxR)

            # 中点射线方位 -> 把相对角 theta_rel 变成绝对朝向 theta_abs
            d_mid = self._world_ray_dir(u_mid_i, v_bot_i)
            dxM, dyM = float(d_mid[0]), float(d_mid[1])
            if math.isfinite(dxM) and math.isfinite(dyM) and math.hypot(dxM, dyM) >= 1e-8:
                az = math.atan2(dyM, dxM)
                theta_abs = float(theta_rel[i]) + az
                theta_abs = (theta_abs + math.pi) % (2.0 * math.pi) - math.pi
            else:
                theta_abs = 0.0

            cuboids[i, 8] = float(theta_abs)

            if self._solver_mode == "ground0_dynamic_width":
                # Use objs bbox directly (already post-processed upstream).
                # center <- (bottom mid ray) ∩ z=0
                # width  <- distance between bottom-left / bottom-right ray intersections on z=0
                u_mid_i = 0.5 * (u_left_i + u_right_i)

                dL_box = self._world_ray_dir(u_left_i, v_bot_i)
                dR_box = self._world_ray_dir(u_right_i, v_bot_i)
                dM_box = self._world_ray_dir(u_mid_i, v_bot_i)

                pC, _ = self._intersect_ray_with_plane_z(
                    cam_cx=cam_cx,
                    cam_cy=cam_cy,
                    cam_cz=cam_cz,
                    ray_dir=dM_box,
                    z_plane=0.0,
                )
                pL, _ = self._intersect_ray_with_plane_z(
                    cam_cx=cam_cx,
                    cam_cy=cam_cy,
                    cam_cz=cam_cz,
                    ray_dir=dL_box,
                    z_plane=0.0,
                )
                pR, _ = self._intersect_ray_with_plane_z(
                    cam_cx=cam_cx,
                    cam_cy=cam_cy,
                    cam_cz=cam_cz,
                    ray_dir=dR_box,
                    z_plane=0.0,
                )
                if pC is None or pL is None or pR is None:
                    continue

                xC, yC, _ = pC
                xL, yL, _ = pL
                xR, yR, _ = pR

                dyn_w = float(math.hypot(float(xR - xL), float(yR - yL)))
                if (not math.isfinite(dyn_w)) or dyn_w <= 1e-4:
                    continue

                cx = float(xC)
                cy = float(yC)
                z_base = 0.0

                h_use = float(h)
                l_use = float(l)
                w_use = float(dyn_w)
                if cid == 0:
                    l_use = w_use

                cuboids[i, 0] = float(cx)
                cuboids[i, 1] = float(cy)
                cuboids[i, 2] = float(z_base + 0.5 * h_use)
                cuboids[i, 3] = float(l_use)
                cuboids[i, 4] = float(w_use)
                cuboids[i, 5] = float(h_use)
                continue

            r_person = None  # 行人底面圆半径（仅 cid==0 有效），后面算 z_base 时会复用
            if cid == 0:
                # 行人特例：用底面矩形外接圆，与两射线相切求圆心
                # 半径采用外接圆：r = 0.5*sqrt(l^2+w^2)
                r_person = 0.5 * math.hypot(float(l), float(w))
                cc = self._find_circle_center_tangent_to_rays(
                    radius=r_person,
                    alpha=alpha,
                    beta=beta,
                )
                if cc is None:
                    continue
                cx_rel, cy_rel = cc
            else:
                # 其他类别：保持原来的矩形解算（依赖 theta_abs）
                try:
                    centers = self._find_rectangle_centers_between_rays(
                        length=float(l),
                        width=float(w),
                        theta=theta_abs,
                        alpha=alpha,
                        beta=beta,
                        check_inside_wedge=True,
                    )
                except ValueError:
                    continue
                if not centers:
                    continue
                cx_rel, cy_rel = centers[0]

            # 转回世界系 XY：原点在相机中心
            cx = cam_cx + float(cx_rel)
            cy = cam_cy + float(cy_rel)

            # 用 2D box 上边界稳定深度（按 track_id 建立时序约束）
            tid = None if track_ids is None else int(track_ids[i])
            cx, cy = self._stabilize_depth_with_top_boundary(
                track_id=tid,
                cx=cx,
                cy=cy,
                cam_cx=cam_cx,
                cam_cy=cam_cy,
                y_top=float(y1[i]),
                y_bottom=float(y2[i]),
            )

            # 用 mask 估计底面 Z：由 mask 底点射线与“底面轮廓”求交来反推 z_base
            #
            # 修改点：
            #   - 行人（圆柱体）用“射线与底面圆”求交
            #   - 其他类别（矩形底面）用“射线与底面矩形边”求交
            z_base: float | None = None

            if masks is not None:
                mask_i = masks[i]
                ys, xs = np.where(mask_i > 0)
                if ys.size > 0:
                    # mask 最底部像素行（视作“接地位置”的候选）
                    v_max = int(ys.max())
                    xs_bottom = xs[ys == v_max]
                    if xs_bottom.size > 0:
                        # 底边像素 x 的均值（你也可以改成 median 更稳，这里保持原逻辑）
                        u_mask = float(xs_bottom.mean())
                        v_mask = float(v_max)

                        # mask 底点射线（世界系）
                        d_mask = self._world_ray_dir(u_mask, v_mask)

                        if cid == 0:
                            # ============ 行人：底面圆求交（与圆柱体建模一致）============
                            # 圆心就是刚才求出的 (cx, cy)，半径 r_person 复用
                            if r_person is not None:
                                z_base_mask, _ = self._intersect_ray_with_bottom_circle_z(
                                    cam_cx=cam_cx,
                                    cam_cy=cam_cy,
                                    cam_cz=cam_cz,
                                    ray_dir=d_mask,
                                    circle_center_xy=(cx, cy),
                                    radius=float(r_person),
                                )
                                if z_base_mask is not None:
                                    z_base = float(z_base_mask)
                        else:
                            # ============ 其他：底面矩形边求交（保持原逻辑）============
                            # 底面四个角在 XY 平面的坐标
                            half_l = 0.5 * float(l)
                            half_w = 0.5 * float(w)
                            local_xy = np.array(
                                [
                                    [half_l, half_w],
                                    [half_l, -half_w],
                                    [-half_l, -half_w],
                                    [-half_l, half_w],
                                ],
                                dtype=np.float64,
                            )

                            cth = math.cos(theta_abs)
                            sth = math.sin(theta_abs)
                            Rz_xy = np.array([[cth, -sth], [sth, cth]], dtype=np.float64)

                            bottom_corners_xy = (local_xy @ Rz_xy.T) + np.array([cx, cy], dtype=np.float64)

                            z_base_mask, _ = self._intersect_ray_with_bottom_edges_z(
                                cam_cx=cam_cx,
                                cam_cy=cam_cy,
                                cam_cz=cam_cz,
                                ray_dir=d_mask,
                                bottom_corners_xy=bottom_corners_xy,
                            )
                            if z_base_mask is not None:
                                z_base = float(z_base_mask)

            # mask 解不出，就退回到先验平面 z
            if z_base is None:
                z_base = z_plane_prior

            if not math.isfinite(z_base):
                continue

            cz = z_base + 0.5 * float(h)

            # 填写成功解出的 cuboid
            cuboids[i, 0] = float(cx)
            cuboids[i, 1] = float(cy)
            cuboids[i, 2] = float(cz)
            cuboids[i, 3] = float(l)
            cuboids[i, 4] = float(w)
            cuboids[i, 5] = float(h)

        return cuboids

    def draw_3dbox_on_cyl(
            self,
            cyl_img: np.ndarray,
            cuboid: tuple[
                float | int, float | int, float | int,
                float | int, float | int, float | int,
                float | int, float | int, float | int
            ],
            *,
            thickness: int = 2,
            label: str | None = None,
            show_info: bool = False,
            samples_per_edge: int = 8,
            class_colors: dict[int, tuple[int, int, int]] | None = None,
    ) -> np.ndarray:
        """
        在圆柱图上以曲线采样绘制 3D 框，并画出“底面中心→前底面棱中心”的朝向线。
        cuboid: (cx,cy,cz,l,w,h,conf,cls,theta_abs)，theta_abs 为弧度。
        返回绘制后的新图。
        """
        assert samples_per_edge >= 1
        if len(cuboid) != 9:
            raise ValueError("cuboid 需为长度 9 的 (cx,cy,cz,l,w,h,conf,cls,theta_abs)")

        default_palette: dict[int, tuple[int, int, int]] = {
            0: (0, 255, 0),        # person
            1: (255, 0, 0),        # bicycle
            2: (0, 165, 255),      # car
            3: (255, 0, 255),      # motorcycle
            5: (0, 255, 255),      # bus
            7: (255, 255, 0),      # truck
        }

        def _color_for_cls(cid: int) -> tuple[int, int, int]:
            if class_colors is not None and cid in class_colors:
                return tuple(map(int, class_colors[cid]))
            if cid in default_palette:
                return default_palette[cid]
            r = (37 * cid + 127) % 256
            g = (17 * cid + 191) % 256
            b = (29 * cid + 159) % 256
            return (b, g, r)

        img = cyl_img.copy()

        f = self.f
        cu = self.cu
        cv_ = self.cv
        R_w2c = self.R_w2c
        t_act = self.t
        tan_pitch = self.tan_pitch
        H, W = img.shape[:2]

        cx, cy, cz, l, w, h, conf, cls_id_f, theta = [float(x) for x in cuboid]
        cls_id = int(round(cls_id_f))
        color = _color_for_cls(cls_id)

        hl, hw, hh = 0.5 * l, 0.5 * w, 0.5 * h
        corners_local = np.array(
            [
                [hl, hw, hh],
                [hl, -hw, hh],
                [-hl, -hw, hh],
                [-hl, hw, hh],
                [hl, hw, -hh],
                [hl, -hw, -hh],
                [-hl, -hw, -hh],
                [-hl, hw, -hh],
            ],
            dtype=np.float64,
        )

        c, s = math.cos(theta), math.sin(theta)
        Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
        center_w = np.array([cx, cy, cz], dtype=np.float64)
        corners_w = (corners_local @ Rz.T) + center_w

        def project_world_to_uv(Pw: np.ndarray) -> tuple[float, float, float, bool]:
            Pc = R_w2c @ Pw + t_act
            x, y, zc = float(Pc[0]), float(Pc[1]), float(Pc[2])
            kxy = math.hypot(x, y)
            if kxy <= 1e-12:
                return 0.0, 0.0, 0.0, False

            phi = math.atan2(-y, x)
            Y_abs = zc / kxy
            Y_rel = Y_abs + tan_pitch
            u = cu + f * phi
            v = cv_ - f * Y_rel
            if not (math.isfinite(u) and math.isfinite(v)):
                return 0.0, 0.0, 0.0, False
            return u, v, phi, True

        def unwrap(prev_phi_unw: float | None, phi_now: float) -> float:
            if prev_phi_unw is None:
                return phi_now
            d = phi_now - prev_phi_unw
            if d > math.pi:
                return phi_now - 2.0 * math.pi
            if d < -math.pi:
                return phi_now + 2.0 * math.pi
            return phi_now

        def draw_segment_world(P0_w: np.ndarray, P1_w: np.ndarray) -> None:
            pts: list[tuple[int, int]] = []
            prev_phi_unw: float | None = None
            prev_valid = False

            for k in range(samples_per_edge + 1):
                tt = k / samples_per_edge
                Pw = (1.0 - tt) * P0_w + tt * P1_w

                u_raw, v, phi_raw, ok = project_world_to_uv(Pw)
                if not ok:
                    if len(pts) >= 2:
                        cv2.polylines(img, [np.asarray(pts, dtype=np.int32)], False, color, thickness, cv2.LINE_AA)
                    pts.clear()
                    prev_phi_unw = None
                    prev_valid = False
                    continue

                phi_unw = unwrap(prev_phi_unw, phi_raw)
                u = cu + f * phi_unw

                if not (0.0 <= u < W and 0.0 <= v < H):
                    if len(pts) >= 2:
                        cv2.polylines(img, [np.asarray(pts, dtype=np.int32)], False, color, thickness, cv2.LINE_AA)
                    pts.clear()
                    prev_phi_unw = phi_unw
                    prev_valid = False
                    continue

                pt = (int(round(u)), int(round(v)))
                if not prev_valid and len(pts) >= 2:
                    cv2.polylines(img, [np.asarray(pts, dtype=np.int32)], False, color, thickness, cv2.LINE_AA)
                    pts.clear()

                pts.append(pt)
                prev_phi_unw = phi_unw
                prev_valid = True

            if len(pts) >= 2:
                cv2.polylines(img, [np.asarray(pts, dtype=np.int32)], False, color, thickness, cv2.LINE_AA)

        edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        ]
        for a, b in edges:
            draw_segment_world(corners_w[a], corners_w[b])

        bottom_idx = [4, 5, 6, 7]
        bottom_center_w = np.mean(corners_w[bottom_idx], axis=0)
        forward_w = (Rz @ np.array([1.0, 0.0, 0.0], dtype=np.float64)).reshape(3)
        bottom_edges = [(4, 5), (5, 6), (6, 7), (7, 4)]
        edge_centers_w = [0.5 * (corners_w[a] + corners_w[b]) for a, b in bottom_edges]
        scores = [float(np.dot(ec - bottom_center_w, forward_w)) for ec in edge_centers_w]
        k_best = int(np.argmax(scores))
        draw_segment_world(bottom_center_w, edge_centers_w[k_best])

        if show_info or label is not None:
            u_b, v_b, phi_b, ok_b = project_world_to_uv(bottom_center_w)
            if ok_b:
                phi_b_unw = unwrap(0.0, phi_b)
                u_b = cu + f * phi_b_unw
                if 0.0 <= u_b < W and 0.0 <= v_b < H:
                    pb = (int(round(u_b)), int(round(v_b)))
                    cv2.circle(img, pb, 3, color, -1, lineType=cv2.LINE_AA)

                    lines: list[str] = []
                    if show_info:
                        lines.extend([
                            f"x={cx:.2f}, y={cy:.2f}, z={cz:.2f}",
                            f"cls={cls_id}, conf={conf:.2f}",
                        ])
                    if label is not None:
                        lines.append(str(label))

                    if lines:
                        sizes = [cv2.getTextSize(s, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)[0] for s in lines]
                        tw = max(sz[0] for sz in sizes)
                        th_total = sum(sz[1] for sz in sizes)
                        pad = 6
                        x0, y0 = pb[0] + 6, max(0, pb[1] - th_total - pad)
                        x1_box = x0 + tw + 2 * pad
                        y1_box = y0 + th_total + 2 * pad
                        cv2.rectangle(img, (x0, y0), (x1_box, y1_box), (0, 0, 0), -1)

                        y_cursor = y0 + pad + sizes[0][1]
                        for s in lines:
                            cv2.putText(
                                img,
                                s,
                                (x0 + pad, y_cursor),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.45,
                                (255, 255, 255),
                                1,
                                cv2.LINE_AA,
                            )
                            (twi, thi), _ = cv2.getTextSize(s, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
                            y_cursor += thi

        return img

    def draw_3dboxes_on_cyl(
            self,
            cyl_img: np.ndarray,
            cuboids: np.ndarray,
            *,
            thickness: int = 1,
            labels: list[str] | None = None,
            samples_per_edge: int = 8,
            class_colors: dict[int, tuple[int, int, int]] | None = None,
            show_info: bool = False,
    ) -> np.ndarray:
        """
        在圆柱图上绘制多个 3D 框（每类不同颜色），并画出朝向线。

        cuboids:
            (N,9)  = [cx,cy,cz,l,w,h,conf,cls,theta_abs]
            或
            (N,11) = [cx,cy,cz,l,w,h,conf,cls,theta_abs,track_id,idx]

        实际绘制时只使用前 9 项。
        """
        arr = np.asarray(cuboids, dtype=np.float64)
        if arr.ndim != 2 or arr.shape[1] not in (9, 11):
            raise ValueError(
                "cuboids 形状应为 (N,9)=[cx,cy,cz,l,w,h,conf,cls,theta_abs] "
                "或 (N,11)=[cx,cy,cz,l,w,h,conf,cls,theta_abs,track_id,idx]"
            )

        img = cyl_img.copy()

        for i in range(arr.shape[0]):
            lab = labels[i] if (labels is not None and i < len(labels)) else None
            cuboid_9 = tuple(arr[i, :9])
            img = self.draw_3dbox_on_cyl(
                img,
                cuboid_9,  # type: ignore[arg-type]
                thickness=thickness,
                label=lab,
                samples_per_edge=samples_per_edge,
                class_colors=class_colors,
                show_info=show_info,
            )
        return img
