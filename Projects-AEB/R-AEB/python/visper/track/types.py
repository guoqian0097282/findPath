from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class _CuboidTrack:
    track_id: int
    cuboid: np.ndarray  # (9,)
    state: int
    start_frame: int
    frame_id: int
    idx: int
    last_timestamp: int
    x: np.ndarray  # (4,1) [px, py, vx, vy]
    P: np.ndarray  # (4,4)
    last_meas_x: float
    last_meas_y: float
    last_meas_valid: bool
    last_innovation: float
    meas_vx: float
    meas_vy: float
    has_meas_velocity: bool
    comp_freeze: int
    maneuver_left: int
    motion_state: int
    stable_cnt: int
    missed: int
    hits: int

    @property
    def age(self) -> int:
        return int(self.frame_id - self.start_frame + 1)

