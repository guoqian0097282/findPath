from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np


def _to_bool(v: Any, default: bool) -> bool:
    if isinstance(v, bool):
        return v
    if isinstance(v, (int, float)):
        return bool(v)
    if isinstance(v, str):
        s = v.strip().lower()
        if s in {"1", "true", "yes", "on"}:
            return True
        if s in {"0", "false", "no", "off"}:
            return False
    return default


def _to_int(v: Any, default: int) -> int:
    try:
        return int(v)
    except Exception:
        return default


def _to_float(v: Any, default: float) -> float:
    try:
        x = float(v)
        if not math.isfinite(x):
            return default
        return x
    except Exception:
        return default


@dataclass(frozen=True)
class AnglePostProcConfig:
    enabled: bool = True
    column: int = 6
    input_unit: str = "rad"  # "rad" | "deg"
    scale: float = 1.0
    offset: float = 0.0
    invert: bool = False
    wrap_mode: str = "pi"  # "pi" -> [-pi,pi], "2pi" -> [0,2pi), "half_pi" -> [-pi/2,pi/2)
    clip_min: float | None = None
    clip_max: float | None = None

    # raw angle decode (from model Angle head)
    decode_mode: str = "auto"  # "auto" | "bins_residual" | "none"
    angle_bins: int = 8
    raw_layout: str = "auto"  # "auto" | "2k_l" | "l_2k"
    raw_logits_first: bool = True

    @staticmethod
    def from_dict(cfg: dict[str, Any] | None) -> "AnglePostProcConfig":
        if cfg is None:
            return AnglePostProcConfig()

        input_unit = str(cfg.get("input_unit", "rad")).strip().lower()
        if input_unit not in {"rad", "deg"}:
            input_unit = "rad"

        wrap_mode = str(cfg.get("wrap_mode", "pi")).strip().lower()
        if wrap_mode not in {"pi", "2pi", "half_pi"}:
            wrap_mode = "pi"

        decode_mode = str(cfg.get("decode_mode", "auto")).strip().lower()
        if decode_mode not in {"auto", "bins_residual", "none"}:
            decode_mode = "auto"

        raw_layout = str(cfg.get("raw_layout", "auto")).strip().lower()
        if raw_layout not in {"auto", "2k_l", "l_2k"}:
            raw_layout = "auto"

        clip_min_v = cfg.get("clip_min", None)
        clip_max_v = cfg.get("clip_max", None)
        clip_min = _to_float(clip_min_v, 0.0) if clip_min_v is not None else None
        clip_max = _to_float(clip_max_v, 0.0) if clip_max_v is not None else None
        if clip_min is not None and clip_max is not None and clip_min > clip_max:
            clip_min, clip_max = clip_max, clip_min

        return AnglePostProcConfig(
            enabled=_to_bool(cfg.get("enabled", True), True),
            column=_to_int(cfg.get("column", 6), 6),
            input_unit=input_unit,
            scale=_to_float(cfg.get("scale", 1.0), 1.0),
            offset=_to_float(cfg.get("offset", 0.0), 0.0),
            invert=_to_bool(cfg.get("invert", False), False),
            wrap_mode=wrap_mode,
            clip_min=clip_min,
            clip_max=clip_max,
            decode_mode=decode_mode,
            angle_bins=max(1, _to_int(cfg.get("angle_bins", 8), 8)),
            raw_layout=raw_layout,
            raw_logits_first=_to_bool(cfg.get("raw_logits_first", True), True),
        )


class AnglePostProcessor:
    def __init__(self, cfg: AnglePostProcConfig) -> None:
        self.cfg = cfg

    @staticmethod
    def _wrap_to_pi(x: np.ndarray) -> np.ndarray:
        return (x + math.pi) % (2.0 * math.pi) - math.pi

    @staticmethod
    def _wrap_to_2pi(x: np.ndarray) -> np.ndarray:
        y = x % (2.0 * math.pi)
        y[y < 0.0] += 2.0 * math.pi
        return y

    @staticmethod
    def _wrap_to_half_pi(x: np.ndarray) -> np.ndarray:
        return (x + 0.5 * math.pi) % math.pi - 0.5 * math.pi

    def _post_theta(self, theta: np.ndarray) -> np.ndarray:
        out = theta.astype(np.float64, copy=False)
        if self.cfg.input_unit == "deg":
            out = np.deg2rad(out)
        out = out * self.cfg.scale + self.cfg.offset
        if self.cfg.invert:
            out = -out

        if self.cfg.clip_min is not None:
            out = np.maximum(out, self.cfg.clip_min)
        if self.cfg.clip_max is not None:
            out = np.minimum(out, self.cfg.clip_max)

        if self.cfg.wrap_mode == "half_pi":
            out = self._wrap_to_half_pi(out)
        elif self.cfg.wrap_mode == "2pi":
            out = self._wrap_to_2pi(out)
        else:
            out = self._wrap_to_pi(out)
        return out

    def _decode_bins_residual(self, angle: np.ndarray) -> np.ndarray:
        arr = np.asarray(angle, dtype=np.float32)
        if arr.ndim != 2:
            return arr.reshape(-1)

        rows, cols = arr.shape
        K_cfg = max(1, int(self.cfg.angle_bins))

        layout = self.cfg.raw_layout
        if layout == "auto":
            if rows == 2 * K_cfg:
                layout = "2k_l"
            elif cols == 2 * K_cfg:
                layout = "l_2k"
            elif rows % 2 == 0 and rows >= 2:
                layout = "2k_l"
            elif cols % 2 == 0 and cols >= 2:
                layout = "l_2k"
            else:
                return arr.reshape(-1)

        if layout == "2k_l":
            two_k, l = rows, cols
            K = two_k // 2
            if K <= 0 or two_k != 2 * K:
                return arr.reshape(-1)
            if self.cfg.raw_logits_first:
                logits = arr[:K, :]
                res = arr[K:, :]
            else:
                res = arr[:K, :]
                logits = arr[K:, :]
        else:
            l, two_k = rows, cols
            K = two_k // 2
            if K <= 0 or two_k != 2 * K:
                return arr.reshape(-1)
            if self.cfg.raw_logits_first:
                logits = arr[:, :K].T
                res = arr[:, K:].T
            else:
                res = arr[:, :K].T
                logits = arr[:, K:].T

        k_star = np.argmax(logits, axis=0).astype(np.int32, copy=False)  # (L,)
        idx = np.arange(k_star.shape[0], dtype=np.int32)
        r_star = np.clip(res[k_star, idx], -1.0, 1.0)

        delta = (2.0 * math.pi) / float(K)
        half = 0.5 * delta
        c_star = math.pi - k_star.astype(np.float32) * delta
        theta = self._wrap_to_pi(c_star + r_star.astype(np.float32) * half)
        return theta.astype(np.float32, copy=False)

    def decode_angle(self, angle: np.ndarray) -> np.ndarray:
        arr = np.asarray(angle)
        mode = self.cfg.decode_mode
        if mode == "none":
            return arr.reshape(-1).astype(np.float32, copy=False)
        if mode == "bins_residual":
            return self._decode_bins_residual(arr)

        # auto: 2D and one axis=2K -> bins+residual decode, else flatten
        if arr.ndim == 2:
            r, c = arr.shape
            k2 = 2 * int(self.cfg.angle_bins)
            if r == k2 or c == k2 or (r % 2 == 0 and r >= 4) or (c % 2 == 0 and c >= 4):
                return self._decode_bins_residual(arr)
        return arr.reshape(-1).astype(np.float32, copy=False)

    def process(self, objs: np.ndarray) -> np.ndarray:
        if (not self.cfg.enabled) or objs is None:
            return objs
        if not isinstance(objs, np.ndarray):
            return objs
        if objs.ndim != 2:
            return objs
        if objs.shape[1] <= self.cfg.column or self.cfg.column < 0:
            return objs

        out = objs.copy()
        theta = out[:, self.cfg.column].astype(np.float64, copy=False)
        theta = self._post_theta(theta)
        out[:, self.cfg.column] = theta.astype(out.dtype, copy=False)
        return out
