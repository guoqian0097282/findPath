from __future__ import annotations

import math
from typing import Any, Iterable

import numpy as np
from .constants import (
    MOTION_MOVING,
    MOTION_STATIC,
    MOTION_UNKNOWN,
    TRACK_LOST,
    TRACK_NEW,
    TRACK_REMOVED,
    TRACK_TRACKED,
)
from .matching import linear_assignment
from .types import _CuboidTrack


class CuboidTracker:
    _count = 0

    def __init__(
            self,
            *,
            track_high_thresh: float = 0.5,
            track_low_thresh: float = 0.3,
            new_track_thresh: float = 0.6,
            match_thresh: float = 0.65,
            track_buffer: int = 30,
            frame_rate: int = 30,
            default_dt_ms: float = 100.0,
            timestamp_scale: float = 1e-3,
            center_gate_main_m: float = 4.0,
            center_gate_low_m: float = 5.5,
            weight_cls_main: float = 0.55,
            weight_center_main: float = 0.35,
            weight_size_main: float = 0.10,
            weight_cls_low: float = 0.60,
            weight_center_low: float = 0.30,
            weight_size_low: float = 0.10,
            sigma_a: float = 2.0,
            sigma_z: float = 0.18,
            vel_static_thresh: float = 0.20,
            vel_move_thresh: float = 0.50,
            stable_frames: int = 1,
            max_missed: int = 30,
            vel_clip_mps: float = 8.0,
            max_innovation_m: float = 1.2,
            vel_meas_blend: float = 0.18,
            vel_meas_blend_max: float = 0.45,
            vel_meas_innov_for_max: float = 1.2,
            pos_comp_sec: float = 0.06,
            pos_comp_min_speed: float = 0.15,
            pos_comp_max_m: float = 0.60,
            pos_comp_new_scale: float = 0.60,
            pos_comp_innov_for_zero: float = 1.2,
            pos_comp_dir_meas_base: float = 0.20,
            pos_comp_dir_meas_turn_gain: float = 0.60,
            pos_comp_reverse_cos: float = -0.20,
            pos_comp_reverse_hold: int = 3,
            pos_comp_reverse_speed: float = 0.40,
            turn_recover_cos: float = 0.30,
            turn_recover_innov_m: float = 0.10,
            turn_recover_hold: int = 6,
            turn_recover_smooth_frames: int = 3,
            turn_recover_vel_blend: float = 0.65,
            turn_recover_pos_blend: float = 0.70,
            turn_recover_vel_blend_smooth: float = 0.20,
            turn_recover_pos_blend_smooth: float = 0.25,
            turn_recover_comp_boost: float = 1.50,
            turn_recover_min_speed: float = 0.25,
            turn_recover_comp_freeze: int = 3,
    ) -> None:
        self.tracked: list[_CuboidTrack] = []
        self.lost: list[_CuboidTrack] = []
        self.removed: list[_CuboidTrack] = []

        self.frame_id = 0
        self.track_high_thresh = float(track_high_thresh)
        self.track_low_thresh = float(track_low_thresh)
        self.new_track_thresh = float(new_track_thresh)
        self.match_thresh = float(match_thresh)
        self.match_thresh_low = min(0.95, self.match_thresh + 0.15)
        self.match_thresh_new = min(0.95, max(0.70, self.match_thresh))
        self.track_buffer = int(track_buffer)
        self.max_time_lost = int(frame_rate / 30.0 * self.track_buffer)

        self.default_dt_sec = float(default_dt_ms) / 1000.0
        self.timestamp_scale = float(timestamp_scale)
        self.min_dt_sec = 1e-6
        self.min_effective_dt_sec = 5e-3
        self.max_effective_dt_sec = 0.5

        self.center_gate_main_m = float(max(0.1, center_gate_main_m))
        self.center_gate_low_m = float(max(self.center_gate_main_m, center_gate_low_m))

        self.weight_cls_main = float(weight_cls_main)
        self.weight_center_main = float(weight_center_main)
        self.weight_size_main = float(weight_size_main)
        self.weight_cls_low = float(weight_cls_low)
        self.weight_center_low = float(weight_center_low)
        self.weight_size_low = float(weight_size_low)

        self.sigma_a = float(max(1e-6, sigma_a))
        self.sigma_z = float(max(1e-6, sigma_z))
        self.vel_static_thresh = float(vel_static_thresh)
        self.vel_move_thresh = float(vel_move_thresh)
        self.stable_frames = int(max(1, stable_frames))
        self.max_missed = int(max(1, max_missed))
        self.vel_clip_mps = float(max(0.1, vel_clip_mps))
        self.max_innovation_m = float(max(0.1, max_innovation_m))
        self.vel_meas_blend = float(np.clip(vel_meas_blend, 0.0, 1.0))
        self.vel_meas_blend_max = float(np.clip(max(self.vel_meas_blend, vel_meas_blend_max), 0.0, 1.0))
        self.vel_meas_innov_for_max = float(max(1e-3, vel_meas_innov_for_max))
        self.pos_comp_sec = float(max(0.0, pos_comp_sec))
        self.pos_comp_min_speed = float(max(0.0, pos_comp_min_speed))
        self.pos_comp_max_m = float(max(0.0, pos_comp_max_m))
        self.pos_comp_new_scale = float(np.clip(pos_comp_new_scale, 0.0, 1.0))
        self.pos_comp_innov_for_zero = float(max(1e-3, pos_comp_innov_for_zero))
        self.pos_comp_dir_meas_base = float(np.clip(pos_comp_dir_meas_base, 0.0, 1.0))
        self.pos_comp_dir_meas_turn_gain = float(max(0.0, pos_comp_dir_meas_turn_gain))
        self.pos_comp_reverse_cos = float(np.clip(pos_comp_reverse_cos, -1.0, 1.0))
        self.pos_comp_reverse_hold = int(max(0, pos_comp_reverse_hold))
        self.pos_comp_reverse_speed = float(max(0.0, pos_comp_reverse_speed))
        self.turn_recover_cos = float(np.clip(turn_recover_cos, -1.0, 1.0))
        self.turn_recover_innov_m = float(max(0.0, turn_recover_innov_m))
        self.turn_recover_hold = int(max(0, turn_recover_hold))
        self.turn_recover_smooth_frames = int(np.clip(turn_recover_smooth_frames, 0, self.turn_recover_hold))
        self.turn_recover_pull_frames = int(max(1, self.turn_recover_hold - self.turn_recover_smooth_frames))
        self.turn_recover_vel_blend = float(np.clip(turn_recover_vel_blend, 0.0, 1.0))
        self.turn_recover_pos_blend = float(np.clip(turn_recover_pos_blend, 0.0, 1.0))
        self.turn_recover_vel_blend_smooth = float(np.clip(turn_recover_vel_blend_smooth, 0.0, 1.0))
        self.turn_recover_pos_blend_smooth = float(np.clip(turn_recover_pos_blend_smooth, 0.0, 1.0))
        self.turn_recover_comp_boost = float(max(0.0, turn_recover_comp_boost))
        self.turn_recover_min_speed = float(max(0.0, turn_recover_min_speed))
        self.turn_recover_comp_freeze = int(max(0, turn_recover_comp_freeze))

        self.mode_id_ego = 0
        self.mode_id_world = 1
        self._large_cost = 1e6

        self.reset_id()

        # 新增：方向历史
        self.dir_history: list[tuple[float, float]] = []  # [(cos, sin), ...]
        self.max_dir_history = 10
        
        # 新增：稳定方向
        self.stable_dir_x = 1.0
        self.stable_dir_y = 0.0
        self.dir_stable_count = 0
        # 新增：方向平滑参数
        self.max_movement_per_frame = 0.8  # 单帧最大位移
        self.position_smooth_alpha = 0.4   # 位置平滑系数
        self.direction_median_window = 7   # 方向中值窗口
        self.direction_angle_threshold = 0.3  # 方向偏差阈值(rad)
        self.still_speed_threshold = 0.3   # 静止速度阈值
        self.slow_speed_threshold = 1.0    # 慢速速度阈值

    @classmethod
    def next_id(cls) -> int:
        cls._count += 1
        return cls._count

    @classmethod
    def reset_id(cls) -> None:
        cls._count = 0

    def reset(self) -> None:
        self.tracked = []
        self.lost = []
        self.removed = []
        self.frame_id = 0
        self.reset_id()

    def _compute_dt(self, timestamp: int, last_timestamp: int) -> float:
        ts = int(timestamp)
        last = int(last_timestamp)
        if ts <= 0 or last <= 0:
            return self.default_dt_sec
        dt = (ts - last) * self.timestamp_scale
        if (not math.isfinite(dt)) or (dt <= self.min_dt_sec):
            return self.default_dt_sec
        if (dt < self.min_effective_dt_sec) or (dt > self.max_effective_dt_sec):
            return self.default_dt_sec
        return float(dt)

    @staticmethod
    def _sanitize_cuboids(cuboids: np.ndarray | None) -> np.ndarray:
        if cuboids is None:
            return np.zeros((0, 9), dtype=np.float32)
        arr = np.asarray(cuboids, dtype=np.float32)
        if arr.ndim != 2:
            raise ValueError(f"cuboids must be 2D, got {arr.ndim}D")
        if arr.shape[1] < 9:
            raise ValueError(f"cuboids shape[1] must be >=9, got {arr.shape[1]}")
        if arr.shape[1] > 9:
            arr = arr[:, :9]
        return np.ascontiguousarray(arr, dtype=np.float32)

    @staticmethod
    def _det_valid_mask(cuboids: np.ndarray) -> np.ndarray:
        if cuboids.size == 0:
            return np.zeros((0,), dtype=bool)
        finite = np.isfinite(cuboids).all(axis=1)
        size_ok = (cuboids[:, 3] > 0.0) & (cuboids[:, 4] > 0.0) & (cuboids[:, 5] > 0.0)
        conf_ok = np.isfinite(cuboids[:, 6]) & (cuboids[:, 6] > 0.0)
        cls_ok = np.isfinite(cuboids[:, 7])
        return finite & size_ok & conf_ok & cls_ok

    @staticmethod
    def _cls_to_int(v: float) -> int:
        return int(round(float(v)))

    def _init_kf_state(self, px: float, py: float) -> tuple[np.ndarray, np.ndarray]:
        x = np.zeros((4, 1), dtype=np.float64)
        x[0, 0] = float(px)
        x[1, 0] = float(py)
        P = np.eye(4, dtype=np.float64)
        P[0, 0] = 1.0
        P[1, 1] = 1.0
        P[2, 2] = 4.0
        P[3, 3] = 4.0
        return x, P

    def _kf_predict(self, x: np.ndarray, P: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
        F = np.eye(4, dtype=np.float64)
        F[0, 2] = dt
        F[1, 3] = dt

        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt2 * dt2
        sa2 = self.sigma_a * self.sigma_a

        q11 = 0.25 * dt4 * sa2
        q12 = 0.5 * dt3 * sa2
        q22 = dt2 * sa2

        Q = np.zeros((4, 4), dtype=np.float64)
        Q[0, 0] = q11
        Q[0, 2] = q12
        Q[2, 0] = q12
        Q[2, 2] = q22
        Q[1, 1] = q11
        Q[1, 3] = q12
        Q[3, 1] = q12
        Q[3, 3] = q22

        x_pre = F @ x
        P_pre = F @ P @ F.T + Q
        return x_pre, P_pre

    def _kf_update(self, x_pre: np.ndarray, P_pre: np.ndarray, z: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        H = np.zeros((2, 4), dtype=np.float64)
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        R = np.eye(2, dtype=np.float64) * (self.sigma_z * self.sigma_z)

        y = z - (H @ x_pre)
        S = H @ P_pre @ H.T + R
        K = P_pre @ H.T @ np.linalg.inv(S)

        x_post = x_pre + K @ y
        I = np.eye(4, dtype=np.float64)
        IKH = I - K @ H
        P_post = IKH @ P_pre @ IKH.T + K @ R @ K.T
        return x_post, P_post

    def _clip_vel(self, vx: float, vy: float) -> tuple[float, float]:
        s = math.hypot(vx, vy)
        if s <= self.vel_clip_mps:
            return float(vx), float(vy)
        scale = self.vel_clip_mps / max(s, 1e-9)
        return float(vx * scale), float(vy * scale)

    def _clip_speed(self, x: np.ndarray) -> np.ndarray:
        vx, vy = self._clip_vel(float(x[2, 0]), float(x[3, 0]))
        x2 = x.copy()
        x2[2, 0] = vx
        x2[3, 0] = vy
        return x2

    def _blend_measured_velocity(
            self,
            trk: _CuboidTrack,
            x_post: np.ndarray,
            z_used: np.ndarray,
            dt: float,
            innovation_m: float,
    ) -> np.ndarray:
        if (not trk.last_meas_valid) or (dt <= self.min_effective_dt_sec):
            return x_post

        vmx = float((z_used[0, 0] - trk.last_meas_x) / dt)
        vmy = float((z_used[1, 0] - trk.last_meas_y) / dt)
        vmx, vmy = self._clip_vel(vmx, vmy)

        ratio = float(np.clip(innovation_m / self.vel_meas_innov_for_max, 0.0, 1.0))
        beta = self.vel_meas_blend + (self.vel_meas_blend_max - self.vel_meas_blend) * ratio

        x2 = x_post.copy()
        x2[2, 0] = (1.0 - beta) * float(x_post[2, 0]) + beta * vmx
        x2[3, 0] = (1.0 - beta) * float(x_post[3, 0]) + beta * vmy
        return x2

    def _recovery_phase(self, trk: _CuboidTrack) -> tuple[int, float]:
        # phase: 0=inactive, 1=smooth window, 2=pull-back window
        if self.turn_recover_hold <= 0:
            return 0, 0.0
        left = int(trk.maneuver_left)
        if left <= 0:
            return 0, 0.0

        elapsed = max(0, self.turn_recover_hold - left)
        smooth_n = self.turn_recover_smooth_frames
        if elapsed < smooth_n:
            return 1, 0.0

        pull_idx = min(max(elapsed - smooth_n + 1, 1), self.turn_recover_pull_frames)
        pull_ratio = float(pull_idx / max(1, self.turn_recover_pull_frames))
        return 2, pull_ratio

    def _apply_turn_recovery(
            self,
            *,
            trk: _CuboidTrack,
            x_post: np.ndarray,
            z_used: np.ndarray,
            vmx: float,
            vmy: float,
            has_meas_velocity: bool,
    ) -> np.ndarray:
        x2 = x_post.copy()
        phase, pull_ratio = self._recovery_phase(trk)
        if phase == 0:
            return self._clip_speed(x2)

        if phase == 1:
            pos_alpha = self.turn_recover_pos_blend_smooth
            vel_alpha = self.turn_recover_vel_blend_smooth
        else:
            pos_alpha = self.turn_recover_pos_blend_smooth + (self.turn_recover_pos_blend - self.turn_recover_pos_blend_smooth) * pull_ratio
            vel_alpha = self.turn_recover_vel_blend_smooth + (self.turn_recover_vel_blend - self.turn_recover_vel_blend_smooth) * pull_ratio

        # Pull center closer to the latest measurement for faster turn/U-turn convergence.
        x2[0, 0] = (1.0 - pos_alpha) * float(x2[0, 0]) + pos_alpha * float(z_used[0, 0])
        x2[1, 0] = (1.0 - pos_alpha) * float(x2[1, 0]) + pos_alpha * float(z_used[1, 0])

        if has_meas_velocity:
            x2[2, 0] = (1.0 - vel_alpha) * float(x2[2, 0]) + vel_alpha * float(vmx)
            x2[3, 0] = (1.0 - vel_alpha) * float(x2[3, 0]) + vel_alpha * float(vmy)

        return self._clip_speed(x2)

    def _update_motion_state(self, trk: _CuboidTrack, speed: float) -> None:
        target_state: int | None
        if speed <= self.vel_static_thresh:
            target_state = MOTION_STATIC
        elif speed >= self.vel_move_thresh:
            target_state = MOTION_MOVING
        else:
            target_state = None

        if target_state is None:
            trk.stable_cnt = 0
            return

        if trk.motion_state == target_state:
            trk.stable_cnt = 0
            return

        trk.stable_cnt += 1
        if trk.stable_cnt >= self.stable_frames:
            trk.motion_state = target_state
            trk.stable_cnt = 0

    @staticmethod
    def _ego_yaw_from_extra(ego: dict[str, Any] | None) -> float:
        if ego is None:
            return 0.0
        try:
            yaw = ego.get("ego_yaw", 0.0)
            if yaw is None:
                return 0.0
            yaw_f = float(yaw)
            return yaw_f if math.isfinite(yaw_f) else 0.0
        except Exception:
            return 0.0

    def _predict_center(self, trk: _CuboidTrack, timestamp: int) -> tuple[float, float]:
        if int(timestamp) == int(trk.last_timestamp):
            return float(trk.x[0, 0]), float(trk.x[1, 0])
        dt = self._compute_dt(timestamp, trk.last_timestamp)
        x_pre, _ = self._kf_predict(trk.x, trk.P, dt)
        return float(x_pre[0, 0]), float(x_pre[1, 0])

    def _position_comp_delta(
            self,
            *,
            trk: _CuboidTrack,
            vx: float,
            vy: float,
            track_state: int,
            motion_state: int,
    ) -> tuple[float, float]:
        if self.pos_comp_sec <= 0.0:
            return 0.0, 0.0

        phase, pull_ratio = self._recovery_phase(trk)
        if phase == 1:
            # First 3 frames: prioritize smoothness, disable forward compensation.
            return 0.0, 0.0

        if int(trk.comp_freeze) > 0:
            return 0.0, 0.0

        speed = math.hypot(vx, vy)
        if speed < self.pos_comp_min_speed:
            return 0.0, 0.0
        if motion_state == MOTION_STATIC:
            return 0.0, 0.0

        state_scale = self.pos_comp_new_scale if int(track_state) == TRACK_NEW else 1.0
        innov_scale = 1.0 - float(np.clip(trk.last_innovation / self.pos_comp_innov_for_zero, 0.0, 1.0))
        if innov_scale <= 0.0:
            return 0.0, 0.0

        # Default: use KF velocity direction.
        dir_x = float(vx / max(speed, 1e-9))
        dir_y = float(vy / max(speed, 1e-9))
        speed_used = float(speed)
        heading_scale = 1.0

        # If measured velocity exists, steer compensation direction toward measured motion,
        # and suppress compensation when heading disagree is large.
        if trk.has_meas_velocity:
            vmx = float(trk.meas_vx)
            vmy = float(trk.meas_vy)
            vm_speed = math.hypot(vmx, vmy)
            if vm_speed > 1e-6:
                kf_x = float(vx / max(speed, 1e-9))
                kf_y = float(vy / max(speed, 1e-9))
                vm_x = float(vmx / vm_speed)
                vm_y = float(vmy / vm_speed)
                cos_h = float(np.clip(kf_x * vm_x + kf_y * vm_y, -1.0, 1.0))

                heading_scale = max(0.0, 0.5 * (cos_h + 1.0))
                beta = float(np.clip(
                    self.pos_comp_dir_meas_base + (1.0 - max(cos_h, 0.0)) * self.pos_comp_dir_meas_turn_gain,
                    0.0,
                    1.0,
                ))
                mix_x = (1.0 - beta) * kf_x + beta * vm_x
                mix_y = (1.0 - beta) * kf_y + beta * vm_y
                mix_n = math.hypot(mix_x, mix_y)
                if mix_n > 1e-9:
                    dir_x = float(mix_x / mix_n)
                    dir_y = float(mix_y / mix_n)
                speed_used = float((1.0 - beta) * speed + beta * vm_speed)

        scale = state_scale * innov_scale * heading_scale
        if scale <= 0.0:
            return 0.0, 0.0

        maneuver_gain = 1.0 + self.turn_recover_comp_boost * pull_ratio if phase == 2 else 1.0
        dist = float(speed_used * self.pos_comp_sec * scale * maneuver_gain)
        if dist > self.pos_comp_max_m:
            dist = float(self.pos_comp_max_m)
        if dist <= 0.0:
            return 0.0, 0.0

        return float(dir_x * dist), float(dir_y * dist)

    @staticmethod
    def _size_cost(track_box: np.ndarray, det_box: np.ndarray, eps: float = 1e-6) -> float:
        tl, tw, th = float(track_box[3]), float(track_box[4]), float(track_box[5])
        dl, dw, dh = float(det_box[3]), float(det_box[4]), float(det_box[5])
        c = (
            abs(np.log((tl + eps) / (dl + eps)))
            + abs(np.log((tw + eps) / (dw + eps)))
            + abs(np.log((th + eps) / (dh + eps)))
        ) / 3.0
        return float(np.clip(c, 0.0, 1.5))

    def _build_cost(
            self,
            tracks: list[_CuboidTrack],
            detections: np.ndarray,
            det_indices: np.ndarray,
            *,
            timestamp: int | None,
            center_gate_m: float,
            w_cls: float,
            w_center: float,
            w_size: float,
    ) -> np.ndarray:
        nt = len(tracks)
        nd = int(detections.shape[0])
        if nt == 0 or nd == 0:
            return np.zeros((nt, nd), dtype=np.float32)

        ws = max(float(w_cls) + float(w_center) + float(w_size), 1e-6)
        cost = np.full((nt, nd), self._large_cost, dtype=np.float32)

        for i, trk in enumerate(tracks):
            if timestamp is None:
                tcx, tcy = float(trk.cuboid[0]), float(trk.cuboid[1])
            else:
                tcx, tcy = self._predict_center(trk, timestamp)
            tdiag = max(1e-3, float(np.hypot(trk.cuboid[3], trk.cuboid[4])))
            tcls = self._cls_to_int(trk.cuboid[7])

            for j in range(nd):
                det = detections[j]
                dcls = self._cls_to_int(det[7])
                if tcls != dcls:
                    continue

                dcx = float(det[0])
                dcy = float(det[1])
                ddiag = max(1e-3, float(np.hypot(det[3], det[4])))
                dist = float(np.hypot(tcx - dcx, tcy - dcy))
                if dist > center_gate_m:
                    continue

                norm = max(1e-3, 0.5 * (tdiag + ddiag))
                center_cost = float(np.clip(dist / (norm * 2.0), 0.0, 1.5))
                cls_cost = 0.0
                size_cost = self._size_cost(trk.cuboid, det)

                c = (w_cls * cls_cost + w_center * center_cost + w_size * size_cost) / ws
                cost[i, j] = float(c)

        return cost

    @staticmethod
    def _linear_assignment(cost: np.ndarray, thresh: float) -> tuple[list[tuple[int, int]], list[int], list[int]]:
        return linear_assignment(cost=cost, thresh=thresh)

    def _activate(self, det: np.ndarray, det_idx: int, timestamp: int) -> _CuboidTrack:
        x, P = self._init_kf_state(px=float(det[0]), py=float(det[1]))
        trk = _CuboidTrack(
            track_id=self.next_id(),
            cuboid=det.astype(np.float32, copy=True),
            state=TRACK_NEW,
            start_frame=self.frame_id,
            frame_id=self.frame_id,
            idx=int(det_idx),
            last_timestamp=int(timestamp),
            x=x,
            P=P,
            last_meas_x=float(det[0]),
            last_meas_y=float(det[1]),
            last_meas_valid=True,
            last_innovation=0.0,
            meas_vx=0.0,
            meas_vy=0.0,
            has_meas_velocity=False,
            comp_freeze=0,
            maneuver_left=0,
            motion_state=MOTION_UNKNOWN,
            stable_cnt=0,
            missed=0,
            hits=1,
        )
        return trk

    def _stabilize_measurement_position(
        self, 
        trk: _CuboidTrack, 
        z: np.ndarray, 
        max_movement_per_frame: float = 0.8
    ) -> np.ndarray:
        """
        限制单帧位置变化，过滤噪声跳变
        
        参数:
            trk: 目标状态
            z: 当前测量位置 [x, y]
            max_movement_per_frame: 单帧最大合理位移 (m)
        
        返回:
            修正后的测量位置
        """
        px = float(z[0, 0])
        py = float(z[1, 0])
        
        # 1. 检查是否为有效的历史
        if not trk.last_meas_valid:
            return z
        
        # 2. 计算位移
        dx = px - trk.last_meas_x
        dy = py - trk.last_meas_y
        dist = math.hypot(dx, dy)
        
        # 3. 如果位移超过物理极限，按比例缩回
        if dist > max_movement_per_frame:
            # 缩回比例
            scale = max_movement_per_frame / dist
            px = trk.last_meas_x + dx * scale
            py = trk.last_meas_y + dy * scale
            
            # 记录日志（可选）
            # LOG_DEBUG("Position jump detected: %.2fm -> %.2fm", dist, max_movement_per_frame)
        
        # 4. 更新
        z[0, 0] = px
        z[1, 0] = py
        return z


    def _smooth_measurement_position(
        self,
        trk: _CuboidTrack,
        z: np.ndarray,
        alpha: float = 0.3  # 平滑系数
    ) -> np.ndarray:
        """
        对测量位置做 EMA 平滑
        
        alpha 越小越平滑，但响应延迟越大
        """
        px = float(z[0, 0])
        py = float(z[1, 0])
        
        # 首次使用直接采用
        if not hasattr(trk, 'smooth_pos_x'):
            trk.smooth_pos_x = px
            trk.smooth_pos_y = py
        else:
            # EMA
            trk.smooth_pos_x = alpha * px + (1 - alpha) * trk.smooth_pos_x
            trk.smooth_pos_y = alpha * py + (1 - alpha) * trk.smooth_pos_y
        
        z[0, 0] = trk.smooth_pos_x
        z[1, 0] = trk.smooth_pos_y
        return z

    def _preprocess_measurement(self, trk: _CuboidTrack, z: np.ndarray) -> np.ndarray:
        """位置预处理流水线"""
        
        # 1. 跳变检测与限幅
        z = self._stabilize_measurement_position(trk, z, self.max_movement_per_frame)
        
        # 2. 位置平滑
        z = self._smooth_measurement_position(trk, z, self.position_smooth_alpha)
        
        return z

    def _smooth_direction(self, trk: _CuboidTrack, vx: float, vy: float) -> tuple[float, float]:
        """
        对速度方向做平滑处理
        
        返回:
            (smooth_vx, smooth_vy) - 方向稳定后的速度
        """
        speed = math.hypot(vx, vy)
        
        # 1. 速度太小，直接返回（或保持原方向）
        if speed < 0.3:  # 0.3 m/s 以下
            # 使用历史稳定方向
            return self.stable_dir_x * speed, trk.stable_dir_y * speed
        
        # 2. 计算当前方向
        dir_x = vx / speed
        dir_y = vy / speed
        
        # 3. 更新方向历史
        self.dir_history.append((dir_x, dir_y))
        if len(self.dir_history) > self.max_dir_history:
            self.dir_history.pop(0)
        
        # 4. 方向一致性检查
        if len(self.dir_history) >= 3:
            # 计算方向变化
            recent_dirs = self.dir_history[-5:]
            cos_sum = sum(d[0] for d in recent_dirs)
            sin_sum = sum(d[1] for d in recent_dirs)
            avg_cos = cos_sum / len(recent_dirs)
            avg_sin = sin_sum / len(recent_dirs)
            avg_len = math.hypot(avg_cos, avg_sin)
            
            # 如果平均方向长度 < 0.7，说明方向分散，使用稳定方向
            if avg_len < 0.7:
                # 方向不稳定
                self.dir_stable_count = 0
                
                # 如果是新目标，使用当前方向
                if trk.age < 5:
                    self.stable_dir_x = dir_x
                    self.stable_dir_y = dir_y
                # 否则使用历史稳定方向
                return self.stable_dir_x * speed, self.stable_dir_y * speed
            
            # 方向稳定，更新稳定方向
            if avg_len > 0.85:
                self.dir_stable_count += 1
                if self.dir_stable_count >= 2:
                    self.stable_dir_x = avg_cos / avg_len
                    self.stable_dir_y = avg_sin / avg_len
            
            # 返回平滑后的方向
            smooth_x = self.stable_dir_x * speed
            smooth_y = self.stable_dir_y * speed
            return smooth_x, smooth_y
        
        # 历史不足，直接返回
        return vx, vy

    def _median_filter_direction(self, trk: _CuboidTrack, vx: float, vy: float) -> tuple[float, float]:
        """
        使用中值滤波滤除方向噪声
        """
        speed = math.hypot(vx, vy)
        if speed < 0.3:
            return vx, vy
        
        # 计算当前角度
        angle = math.atan2(vy, vx)
        
        # 维护角度历史
        if not hasattr(trk, 'angle_history'):
            trk.angle_history = []
        
        trk.angle_history.append(angle)
        if len(trk.angle_history) > self.direction_median_window:
            trk.angle_history.pop(0)
        
        if len(trk.angle_history) >= 3:
            # 处理角度环绕
            angles = np.array(trk.angle_history)
            
            # 将角度映射到 [-pi, pi]
            angles = np.arctan2(np.sin(angles), np.cos(angles))
            
            # 中值
            median_angle = np.median(angles)
            
            # 检查当前角度与中值的偏差
            diff = abs(angle - median_angle)
            diff = min(diff, 2 * math.pi - diff)
            
            # 如果偏差大于阈值，使用中值
            if diff > self.direction_angle_threshold:  # ~17度
                return math.cos(median_angle) * speed, math.sin(median_angle) * speed
        
        return vx, vy

    def _classify_velocity_state(self, trk: _CuboidTrack, speed: float) -> int:
        """
        分类速度状态，用于差异化处理
        
        返回:
            0: 静止 (< 0.3 m/s)
            1: 慢速 (0.3 - 1.0 m/s)
            2: 中速 (1.0 - 3.0 m/s)
            3: 快速 (> 3.0 m/s)
        """
        if speed < self.still_speed_threshold:
            return 0  # 静止
        elif speed < self.slow_speed_threshold:
            return 1  # 慢速
        elif speed < 3.0:
            return 2  # 中速
        else:
            self.max_movement_per_frame=2.0
            self.position_smooth_alpha=0.3
            self.direction_median_window=5
            self.still_speed_threshold = 0.5
            self.slow_speed_threshold=2.0
            return 3  # 快速

    def _apply_velocity_smoothing_strategy(
        self, 
        trk: _CuboidTrack, 
        raw_vx: float, 
        raw_vy: float
    ) -> tuple[float, float]:
        """
        根据速度状态应用不同的平滑策略
        """
        speed = math.hypot(raw_vx, raw_vy)
        state = self._classify_velocity_state(trk, speed)
        
        if state == 0:  # 静止
            # 强平滑，抑制噪声
            alpha = 0.1
            if not hasattr(trk, 'still_counter'):
                trk.still_counter = 0
            trk.still_counter += 1
            if trk.still_counter > 3:
                return 0.0, 0.0  # 强制置零
            return raw_vx * alpha, raw_vy * alpha
        
        elif state == 1:  # 慢速
            # 方向稳定性优先
            vx, vy = self._median_filter_direction(trk, raw_vx, raw_vy)
            vx, vy = self._smooth_direction(trk, vx, vy)
            # 幅值平滑
            alpha = 0.3
            if hasattr(trk, 'slow_vx'):
                vx = alpha * vx + (1 - alpha) * trk.slow_vx
                vy = alpha * vy + (1 - alpha) * trk.slow_vy
            trk.slow_vx = vx
            trk.slow_vy = vy
            return vx, vy
        
        elif state == 2:  # 中速
            # 中等平滑
            vx, vy = self._smooth_direction(trk, raw_vx, raw_vy)
            return vx, vy
        
        else:  # 快速
            # 最小平滑，保持响应
            vx, vy = self._clip_vel(raw_vx, raw_vy)
            return vx, vy            

    def _compute_velocity_confidence(
        self,
        trk: _CuboidTrack,
        vx: float,
        vy: float
    ) -> float:
        """
        计算速度置信度 (0-1)
        """
        speed = math.hypot(vx, vy)
        
        # 1. 速度幅值置信度
        if speed < 0.3:
            speed_conf = 0.1  # 低速不可靠
        elif speed < 1.0:
            speed_conf = 0.5
        else:
            speed_conf = 0.9
        
        # 2. 方向一致性置信度
        dir_conf = 1.0
        if len(self.dir_history) >= 5:
            recent_dirs = self.dir_history[-5:]
            # 计算方向一致性
            cos_sum = sum(math.cos(d[0]) for d in recent_dirs)
            sin_sum = sum(math.sin(d[1]) for d in recent_dirs)
            avg_len = math.hypot(cos_sum, sin_sum) / len(recent_dirs)
            dir_conf = avg_len  # 越接近1越一致
        
        # 3. 跟踪状态置信度
        state_conf = 1.0
        if trk.state == TRACK_NEW:
            state_conf = 0.3
        elif trk.missed > 0:
            state_conf = 0.5
        
        # 4. 综合置信度
        confidence = speed_conf * dir_conf * state_conf
        return float(np.clip(confidence, 0.0, 1.0))

    def _weighted_velocity_output(
        self,
        trk: _CuboidTrack,
        kf_vx: float,
        kf_vy: float,
        meas_vx: float,
        meas_vy: float
    ) -> tuple[float, float]:
        """
        根据置信度加权融合
        """
        confidence = self._compute_velocity_confidence(trk, meas_vx, meas_vy)
        
        # 低置信度时更多依赖卡尔曼滤波
        beta = 1.0 - confidence * 0.8  # 0.2 ~ 1.0
        
        vx = (1 - beta) * kf_vx + beta * meas_vx
        vy = (1 - beta) * kf_vy + beta * meas_vy
        
        # 如果置信度极低，直接使用KF
        if confidence < 0.2:
            return kf_vx, kf_vy
        
        return vx, vy


    def _update_track(self, trk: _CuboidTrack, det: np.ndarray, det_idx: int, timestamp: int, state: int) -> None:
        dt = self._compute_dt(timestamp, trk.last_timestamp)

        x_pre, P_pre = self._kf_predict(trk.x, trk.P, dt)

        if trk.comp_freeze > 0:
            trk.comp_freeze -= 1
        if trk.maneuver_left > 0:
            trk.maneuver_left -= 1

        z = np.array([[float(det[0])], [float(det[1])]], dtype=np.float64)

        # ===== 新增：位置预处理 =====
        z = self._preprocess_measurement(trk, z)

        dx = float(z[0, 0] - x_pre[0, 0])
        dy = float(z[1, 0] - x_pre[1, 0])
        innov = math.hypot(dx, dy)
        if innov > self.max_innovation_m:
            scale = self.max_innovation_m / max(innov, 1e-9)
            z[0, 0] = float(x_pre[0, 0] + dx * scale)
            z[1, 0] = float(x_pre[1, 0] + dy * scale)

        has_meas_velocity = False
        vmx = 0.0
        vmy = 0.0
        if trk.last_meas_valid and dt > self.min_effective_dt_sec:
            vmx = float((z[0, 0] - trk.last_meas_x) / dt)
            vmy = float((z[1, 0] - trk.last_meas_y) / dt)
            vmx, vmy = self._clip_vel(vmx, vmy)
            has_meas_velocity = True

        # # 计算速度
        # vmx, vmy, has_meas_velocity = self._compute_smooth_velocity(trk, z, timestamp)
    
        # ===== 新增：速度方向平滑 =====
        if has_meas_velocity:
            vmx, vmy = self._apply_velocity_smoothing_strategy(trk, vmx, vmy)



        innov_used = math.hypot(float(z[0, 0] - x_pre[0, 0]), float(z[1, 0] - x_pre[1, 0]))
        x_post, P_post = self._kf_update(x_pre, P_pre, z)
        x_post = self._blend_measured_velocity(trk, x_post, z, dt, innov_used)
        x_post = self._clip_speed(x_post)

        kf_vx = float(x_post[2, 0])
        kf_vy = float(x_post[3, 0])
        vx_out, vy_out = self._weighted_velocity_output(
            trk, kf_vx, kf_vy, vmx, vmy
        )
        kf_vx = vx_out
        kf_vy = vy_out
        x_post[2, 0] = vx_out
        x_post[3, 0] = vy_out
        speed = math.hypot(kf_vx, kf_vy)
        vm_speed = math.hypot(vmx, vmy) if has_meas_velocity else 0.0

        trigger_turn_recover = False
        if has_meas_velocity and vm_speed >= self.turn_recover_min_speed:
            if innov_used >= self.turn_recover_innov_m:
                trigger_turn_recover = True
            if speed >= self.turn_recover_min_speed:
                cos_h = float(np.clip((kf_vx * vmx + kf_vy * vmy) / max(speed * vm_speed, 1e-9), -1.0, 1.0))
                if cos_h <= self.turn_recover_cos:
                    trigger_turn_recover = True

        recover_active = trk.maneuver_left > 0
        if trigger_turn_recover and self.turn_recover_hold > 0:
            if not recover_active:
                trk.maneuver_left = self.turn_recover_hold
                recover_active = True
            else:
                trk.maneuver_left = max(trk.maneuver_left, self.turn_recover_pull_frames)
            trk.comp_freeze = max(trk.comp_freeze, self.turn_recover_comp_freeze)

        if recover_active:
            x_post = self._apply_turn_recovery(
                trk=trk,
                x_post=x_post,
                z_used=z,
                vmx=vmx,
                vmy=vmy,
                has_meas_velocity=has_meas_velocity,
            )
            kf_vx = float(x_post[2, 0])
            kf_vy = float(x_post[3, 0])
            speed = math.hypot(kf_vx, kf_vy)

        fx = float(x_post[0, 0])
        fy = float(x_post[1, 0])
        self._update_motion_state(trk, speed)

        trk.x = x_post
        trk.P = P_post
        trk.last_timestamp = int(timestamp)
        trk.last_meas_x = float(z[0, 0])
        trk.last_meas_y = float(z[1, 0])
        trk.last_meas_valid = True
        trk.last_innovation = float(innov_used)
        trk.meas_vx = float(vmx)
        trk.meas_vy = float(vmy)
        trk.has_meas_velocity = bool(has_meas_velocity)
        trk.missed = 0

        if has_meas_velocity:
            if vm_speed >= self.pos_comp_reverse_speed and speed >= self.pos_comp_reverse_speed:
                cos_h = float(np.clip((kf_vx * vmx + kf_vy * vmy) / max(speed * vm_speed, 1e-9), -1.0, 1.0))
                if cos_h <= self.pos_comp_reverse_cos:
                    trk.comp_freeze = max(trk.comp_freeze, self.pos_comp_reverse_hold)

        trk.cuboid = det.astype(np.float32, copy=True)
        trk.cuboid[0] = fx
        trk.cuboid[1] = fy
        trk.idx = int(det_idx)
        trk.state = int(state)
        trk.frame_id = self.frame_id
        trk.hits += 1

    def _predict_track_without_measure(self, trk: _CuboidTrack, timestamp: int) -> None:
        if int(timestamp) == int(trk.last_timestamp):
            return

        dt = self._compute_dt(timestamp, trk.last_timestamp)
        x_pre, P_pre = self._kf_predict(trk.x, trk.P, dt)
        x_pre = self._clip_speed(x_pre)

        trk.x = x_pre
        trk.P = P_pre
        trk.last_timestamp = int(timestamp)
        trk.last_meas_valid = False
        trk.last_innovation = float(self.pos_comp_innov_for_zero)
        trk.has_meas_velocity = False
        trk.meas_vx = 0.0
        trk.meas_vy = 0.0
        trk.missed += 1
        if trk.comp_freeze > 0:
            trk.comp_freeze -= 1
        if trk.maneuver_left > 0:
            trk.maneuver_left -= 1

        trk.cuboid[0] = float(x_pre[0, 0])
        trk.cuboid[1] = float(x_pre[1, 0])

        speed = math.hypot(float(x_pre[2, 0]), float(x_pre[3, 0]))
        self._update_motion_state(trk, speed)

    @staticmethod
    def _joint_tracks(a: Iterable[_CuboidTrack], b: Iterable[_CuboidTrack]) -> list[_CuboidTrack]:
        out: list[_CuboidTrack] = []
        seen: set[int] = set()
        for t in list(a) + list(b):
            if t.track_id in seen:
                continue
            seen.add(t.track_id)
            out.append(t)
        return out

    @staticmethod
    def _sub_tracks(a: Iterable[_CuboidTrack], b: Iterable[_CuboidTrack]) -> list[_CuboidTrack]:
        b_ids = {t.track_id for t in b}
        return [t for t in a if t.track_id not in b_ids]

    def _remove_duplicate(self, tracked: list[_CuboidTrack], lost: list[_CuboidTrack]) -> tuple[list[_CuboidTrack], list[_CuboidTrack]]:
        if len(tracked) == 0 or len(lost) == 0:
            return tracked, lost

        cost = self._build_cost(
            tracks=tracked,
            detections=np.asarray([t.cuboid for t in lost], dtype=np.float32),
            det_indices=np.asarray([t.idx for t in lost], dtype=np.int64),
            timestamp=None,
            center_gate_m=self.center_gate_main_m,
            w_cls=0.60,
            w_center=0.30,
            w_size=0.10,
        )
        pairs = np.where(cost < 0.12)
        drop_t: set[int] = set()
        drop_l: set[int] = set()
        for i, j in zip(pairs[0].tolist(), pairs[1].tolist()):
            age_t = tracked[i].age
            age_l = lost[j].age
            if age_t >= age_l:
                drop_l.add(j)
            else:
                drop_t.add(i)
        tracked2 = [t for k, t in enumerate(tracked) if k not in drop_t]
        lost2 = [t for k, t in enumerate(lost) if k not in drop_l]
        return tracked2, lost2

    def update_and_estimate(
            self,
            *,
            timestamp: int,
            cuboids: np.ndarray | None,
            ego: dict[str, Any] | None = None,
            mode: str = "ego",
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        if mode not in ("ego", "world"):
            raise ValueError(f"velocity mode must be 'ego' or 'world', got {mode}")

        self.frame_id += 1
        ts = int(timestamp)
        mode_id = self.mode_id_ego if mode == "ego" else self.mode_id_world

        arr = self._sanitize_cuboids(cuboids)
        n_det = int(arr.shape[0])
        valid = self._det_valid_mask(arr)

        if n_det > 0:
            scores = arr[:, 6]
            high_mask = valid & (scores >= self.track_high_thresh)
            low_mask = valid & (scores > self.track_low_thresh) & (scores < self.track_high_thresh)
            high_idx = np.where(high_mask)[0]
            low_idx = np.where(low_mask)[0]
            det_high = arr[high_idx]
            det_low = arr[low_idx]
        else:
            high_idx = np.zeros((0,), dtype=np.int64)
            low_idx = np.zeros((0,), dtype=np.int64)
            det_high = np.zeros((0, 9), dtype=np.float32)
            det_low = np.zeros((0, 9), dtype=np.float32)

        activated: list[_CuboidTrack] = []
        refind: list[_CuboidTrack] = []
        lost_now: list[_CuboidTrack] = []
        removed_now: list[_CuboidTrack] = []

        unconfirmed = [t for t in self.tracked if t.state == TRACK_NEW]
        tracked = [t for t in self.tracked if t.state == TRACK_TRACKED]

        # 1) main match: (tracked + lost) x high-conf detections
        pool = self._joint_tracks(tracked, self.lost)
        cost_main = self._build_cost(
            tracks=pool,
            detections=det_high,
            det_indices=high_idx,
            timestamp=ts,
            center_gate_m=self.center_gate_main_m,
            w_cls=self.weight_cls_main,
            w_center=self.weight_center_main,
            w_size=self.weight_size_main,
        )
        matches, u_pool, u_high = self._linear_assignment(cost_main, self.match_thresh)
        for it, idet in matches:
            trk = pool[it]
            det = det_high[idet]
            det_idx = int(high_idx[idet])
            if trk.state == TRACK_TRACKED:
                self._update_track(trk, det, det_idx, ts, TRACK_TRACKED)
                activated.append(trk)
            else:
                self._update_track(trk, det, det_idx, ts, TRACK_TRACKED)
                refind.append(trk)

        # 2) low-conf match for unmatched tracked-only
        rem_tracked = [pool[i] for i in u_pool if pool[i].state == TRACK_TRACKED]
        cost_low = self._build_cost(
            tracks=rem_tracked,
            detections=det_low,
            det_indices=low_idx,
            timestamp=ts,
            center_gate_m=self.center_gate_low_m,
            w_cls=self.weight_cls_low,
            w_center=self.weight_center_low,
            w_size=self.weight_size_low,
        )
        matches2, u_rt, _ = self._linear_assignment(cost_low, self.match_thresh_low)
        for it, idet in matches2:
            trk = rem_tracked[it]
            det = det_low[idet]
            det_idx = int(low_idx[idet])
            self._update_track(trk, det, det_idx, ts, TRACK_TRACKED)
            activated.append(trk)
        for i in u_rt:
            trk = rem_tracked[i]
            trk.state = TRACK_LOST
            trk.missed += 1
            lost_now.append(trk)

        # 3) unconfirmed x unmatched high detections
        rem_high_idx = high_idx[np.asarray(u_high, dtype=np.int64)] if len(u_high) > 0 else np.zeros((0,), dtype=np.int64)
        rem_high = arr[rem_high_idx] if rem_high_idx.size > 0 else np.zeros((0, 9), dtype=np.float32)
        cost_new = self._build_cost(
            tracks=unconfirmed,
            detections=rem_high,
            det_indices=rem_high_idx,
            timestamp=ts,
            center_gate_m=self.center_gate_main_m,
            w_cls=self.weight_cls_main,
            w_center=self.weight_center_main,
            w_size=self.weight_size_main,
        )
        matches3, u_unconfirmed, u_high2 = self._linear_assignment(cost_new, self.match_thresh_new)
        for it, idet in matches3:
            trk = unconfirmed[it]
            det = rem_high[idet]
            det_idx = int(rem_high_idx[idet])
            self._update_track(trk, det, det_idx, ts, TRACK_TRACKED)
            activated.append(trk)
        for i in u_unconfirmed:
            trk = unconfirmed[i]
            trk.state = TRACK_REMOVED
            removed_now.append(trk)

        # 4) spawn new tracks from unmatched high detections
        for i in u_high2:
            det = rem_high[i]
            det_idx = int(rem_high_idx[i])
            if float(det[6]) < self.new_track_thresh:
                continue
            trk = self._activate(det, det_idx, ts)
            activated.append(trk)

        # 5) age-out lost tracks
        refind_ids = {int(t.track_id) for t in refind}
        for trk in self.lost:
            if int(trk.track_id) not in refind_ids:
                self._predict_track_without_measure(trk, ts)
            if (self.frame_id - trk.frame_id > self.max_time_lost) or (trk.missed > self.max_missed):
                trk.state = TRACK_REMOVED
                removed_now.append(trk)

        # 6) merge lists
        self.tracked = [t for t in self.tracked if t.state in (TRACK_NEW, TRACK_TRACKED)]
        self.tracked = self._joint_tracks(self.tracked, activated)
        self.tracked = self._joint_tracks(self.tracked, refind)

        self.lost = self._sub_tracks(self.lost, self.tracked)
        self.lost.extend(lost_now)
        self.lost = self._sub_tracks(self.lost, removed_now)

        self.tracked, self.lost = self._remove_duplicate(self.tracked, self.lost)

        self.removed.extend(removed_now)
        if len(self.removed) > 2000:
            self.removed = self.removed[-1000:]

        active = [t for t in self.tracked if t.state in (TRACK_NEW, TRACK_TRACKED)]
        active.sort(key=lambda x: x.track_id)

        yaw = self._ego_yaw_from_extra(ego)
        c = math.cos(yaw)
        s = math.sin(yaw)

        n = len(active)
        track_info = np.zeros((n, 4), dtype=np.int32)
        tracked_cuboids_raw = np.zeros((n, 9), dtype=np.float32)
        tracked_cuboids = np.zeros((n, 9), dtype=np.float32)
        tracked_cuboids_vel = np.zeros((n, 4), dtype=np.float32)

        for i, trk in enumerate(active):
            track_info[i, 0] = int(trk.track_id)
            track_info[i, 1] = int(trk.state)
            track_info[i, 2] = int(trk.age)
            track_info[i, 3] = int(trk.idx)

            tracked_cuboids[i] = trk.cuboid.astype(np.float32, copy=False)
            if 0 <= int(trk.idx) < n_det:
                tracked_cuboids_raw[i] = arr[int(trk.idx)]
            else:
                tracked_cuboids_raw[i] = trk.cuboid.astype(np.float32, copy=False)

            vx_e = float(trk.x[2, 0])
            vy_e = float(trk.x[3, 0])
            dx_comp, dy_comp = self._position_comp_delta(
                trk=trk,
                vx=vx_e,
                vy=vy_e,
                track_state=int(trk.state),
                motion_state=int(trk.motion_state),
            )
            tracked_cuboids[i, 0] = float(tracked_cuboids[i, 0] + dx_comp)
            tracked_cuboids[i, 1] = float(tracked_cuboids[i, 1] + dy_comp)

            if mode == "world":
                vx_out = c * vx_e - s * vy_e
                vy_out = s * vx_e + c * vy_e
            else:
                vx_out = vx_e
                vy_out = vy_e

            tracked_cuboids_vel[i, 0] = float(mode_id)
            tracked_cuboids_vel[i, 1] = float(trk.motion_state)
            tracked_cuboids_vel[i, 2] = float(vx_out)
            tracked_cuboids_vel[i, 3] = float(vy_out)

        return track_info, tracked_cuboids_raw, tracked_cuboids, tracked_cuboids_vel

    def update(self, *, timestamp: int, cuboids: np.ndarray | None) -> tuple[np.ndarray, np.ndarray]:
        track_info, tracked_cuboids_raw, _, _ = self.update_and_estimate(
            timestamp=timestamp,
            cuboids=cuboids,
            ego=None,
            mode="ego",
        )
        return track_info, tracked_cuboids_raw
