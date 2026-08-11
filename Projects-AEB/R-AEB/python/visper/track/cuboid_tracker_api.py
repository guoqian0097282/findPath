from __future__ import annotations

from typing import Any

import numpy as np

from .cuboid_tracker_impl import CuboidTracker

__all__ = [
    "cuboid_tracker_Init",
    "cuboid_tracker_Track",
    "cuboid_tracker_TrackAndEstimate",
    "cuboid_tracker_Reset",
]


g_cuboid_tracker: CuboidTracker | None = None


def cuboid_tracker_Init(
        *,
        track_high_thresh: float = 0.5,
        track_low_thresh: float = 0.3,
        match_thresh: float = 0.65,
        new_track_thresh: float = 0.6,
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
) -> CuboidTracker:
    global g_cuboid_tracker
    g_cuboid_tracker = CuboidTracker(
        track_high_thresh=track_high_thresh,
        track_low_thresh=track_low_thresh,
        match_thresh=match_thresh,
        new_track_thresh=new_track_thresh,
        track_buffer=track_buffer,
        frame_rate=frame_rate,
        default_dt_ms=default_dt_ms,
        timestamp_scale=timestamp_scale,
        center_gate_main_m=center_gate_main_m,
        center_gate_low_m=center_gate_low_m,
        weight_cls_main=weight_cls_main,
        weight_center_main=weight_center_main,
        weight_size_main=weight_size_main,
        weight_cls_low=weight_cls_low,
        weight_center_low=weight_center_low,
        weight_size_low=weight_size_low,
        sigma_a=sigma_a,
        sigma_z=sigma_z,
        vel_static_thresh=vel_static_thresh,
        vel_move_thresh=vel_move_thresh,
        stable_frames=stable_frames,
        max_missed=max_missed,
        vel_clip_mps=vel_clip_mps,
        max_innovation_m=max_innovation_m,
        vel_meas_blend=vel_meas_blend,
        vel_meas_blend_max=vel_meas_blend_max,
        vel_meas_innov_for_max=vel_meas_innov_for_max,
        pos_comp_sec=pos_comp_sec,
        pos_comp_min_speed=pos_comp_min_speed,
        pos_comp_max_m=pos_comp_max_m,
        pos_comp_new_scale=pos_comp_new_scale,
        pos_comp_innov_for_zero=pos_comp_innov_for_zero,
        pos_comp_dir_meas_base=pos_comp_dir_meas_base,
        pos_comp_dir_meas_turn_gain=pos_comp_dir_meas_turn_gain,
        pos_comp_reverse_cos=pos_comp_reverse_cos,
        pos_comp_reverse_hold=pos_comp_reverse_hold,
        pos_comp_reverse_speed=pos_comp_reverse_speed,
        turn_recover_cos=turn_recover_cos,
        turn_recover_innov_m=turn_recover_innov_m,
        turn_recover_hold=turn_recover_hold,
        turn_recover_smooth_frames=turn_recover_smooth_frames,
        turn_recover_vel_blend=turn_recover_vel_blend,
        turn_recover_pos_blend=turn_recover_pos_blend,
        turn_recover_vel_blend_smooth=turn_recover_vel_blend_smooth,
        turn_recover_pos_blend_smooth=turn_recover_pos_blend_smooth,
        turn_recover_comp_boost=turn_recover_comp_boost,
        turn_recover_min_speed=turn_recover_min_speed,
        turn_recover_comp_freeze=turn_recover_comp_freeze,
    )
    return g_cuboid_tracker


def cuboid_tracker_Track(
        *,
        timestamp: int,
        cuboids: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    if g_cuboid_tracker is None:
        raise RuntimeError("CuboidTracker not initialized: call cuboid_tracker_Init first.")
    return g_cuboid_tracker.update(timestamp=timestamp, cuboids=cuboids)


def cuboid_tracker_TrackAndEstimate(
        *,
        timestamp: int,
        cuboids: np.ndarray,
        ego: dict[str, Any] | None = None,
        mode: str = "ego",
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    if g_cuboid_tracker is None:
        raise RuntimeError("CuboidTracker not initialized: call cuboid_tracker_Init first.")
    return g_cuboid_tracker.update_and_estimate(
        timestamp=timestamp,
        cuboids=cuboids,
        ego=ego,
        mode=mode,
    )


def cuboid_tracker_Reset() -> None:
    if g_cuboid_tracker is not None:
        g_cuboid_tracker.reset()
