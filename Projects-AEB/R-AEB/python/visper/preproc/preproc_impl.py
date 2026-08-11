from __future__ import annotations

import numpy as np
import cv2 as cv


class ImageConvert:
    """
    Python 版本的颜色空间/格式转换器：
    - 输入类型：NV12 / BGR / RGB
    - 目标类型：RGB / BGR / NV12
    - img 为 numpy.ndarray（OpenCV Mat）
    """

    def __init__(self, width: int, height: int, input_type: str, target_type: str) -> None:
        self.width: int = int(width)
        self.height: int = int(height)
        self.input_type: str = (input_type or "").upper()
        self.target_type: str = (target_type or "").upper()

        self._validate_types()

    # ---------- public API ----------
    def convert(self, img: np.ndarray, target_type: str | None = None) -> np.ndarray:
        """
        仅从已有 Mat(ndarray) 转换；空图或形状不匹配会抛出异常。

        参数：
          - target_type: 可选。若为 None 则使用 self.target_type；否则临时覆盖目标类型。

        返回：
          - 目标为 RGB/BGR： (H, W, 3) uint8    （HWC）
          - 目标为 NV12：    (H*3//2, W) uint8  （单平面）
        """
        if img is None or (hasattr(img, "size") and img.size == 0):
            raise ValueError("empty mat")

        self._validate_input_mat_shape(img)

        # 临时目标类型：默认用 self.target_type
        t = self.target_type if target_type is None else (target_type or "").upper()
        if t not in {"RGB", "BGR", "NV12"}:
            raise ValueError("Unsupported target type")

        return self._convert_to_target_with_type(img, t)

    @staticmethod
    def to_data(
            img: np.ndarray,
            expand_axis: int | None = None,
            transpose_axes: tuple[int, ...] | None = None,
            out_dtype: np.dtype | str | None = None,
    ) -> np.ndarray:
        """
        合并版小工具：按需执行 expand_dims / transpose / astype（按顺序依次执行）。

        参数：
          - expand_axis:     传入则执行 np.expand_dims(img, axis=expand_axis)
          - transpose_axes:  传入则执行 np.transpose(img, axes=transpose_axes)
          - out_dtype:       传入则执行 img.astype(dtype=out_dtype)

        示例：
          - 加 batch:          transform(img, expand_axis=0)
          - HWC -> CHW:        transform(img, transpose_axes=(2, 0, 1))
          - uint8 -> float32:  transform(img, out_dtype=np.float32)
          - 组合（先加维再转再改 dtype）：
                              transform(img, expand_axis=0, transpose_axes=(0, 3, 1, 2), out_dtype=np.float32)
        """
        if img is None or (hasattr(img, "size") and img.size == 0):
            raise ValueError("empty input")

        out = img
        if expand_axis is not None:
            out = np.expand_dims(out, axis=expand_axis)
        if transpose_axes is not None:
            out = np.transpose(out, axes=transpose_axes)
        if out_dtype is not None:
            out = out.astype(dtype=out_dtype)
        return out

    # ---------- helpers ----------
    def _validate_types(self) -> None:
        if self.input_type not in {"NV12", "BGR", "RGB"}:
            raise ValueError("unsupported input type")
        if self.target_type not in {"RGB", "BGR", "NV12"}:
            raise ValueError("Unsupported target type")

    def _validate_input_mat_shape(self, img: np.ndarray) -> None:
        h, w = self.height, self.width
        if self.input_type == "NV12":
            # NV12: (h*3/2, w), 单通道
            if img.ndim != 2 or img.dtype != np.uint8 or img.shape != (h * 3 // 2, w):
                raise RuntimeError("invalid NV12 shape")
        else:
            # BGR/RGB: (h, w, 3), uint8
            if img.ndim != 3 or img.dtype != np.uint8 or img.shape[:2] != (h, w) or img.shape[2] != 3:
                if self.input_type == "BGR":
                    raise RuntimeError("invalid BGR shape")
                else:
                    raise RuntimeError("invalid RGB shape")

    @staticmethod
    def _ensure_even_dims(w: int, h: int) -> None:
        if (w & 1) or (h & 1):
            raise RuntimeError("NV12 requires even width and height")

    @staticmethod
    def _i420_to_nv12(i420: np.ndarray, w: int, h: int) -> np.ndarray:
        """
        将 I420(YUV420p, Y + U + V 平面) 拼成 NV12(Y + UV 交错)。
        期望 i420 形状为 (h*3//2, w)，dtype=uint8。
        """
        if i420.ndim != 2 or i420.dtype != np.uint8 or i420.shape != (h * 3 // 2, w):
            raise RuntimeError("invalid I420 shape")

        y_size = w * h
        uv_size = (w // 2) * (h // 2)

        flat = i420.reshape(-1)  # 连续缓冲
        y = flat[:y_size]
        u = flat[y_size: y_size + uv_size]
        v = flat[y_size + uv_size: y_size + 2 * uv_size]

        uv_interleaved = np.empty(2 * uv_size, dtype=np.uint8)
        uv_interleaved[0::2] = u
        uv_interleaved[1::2] = v

        nv12_flat = np.concatenate([y, uv_interleaved], axis=0)
        nv12 = nv12_flat.reshape(h + h // 2, w)
        return nv12

    def _rgb_to_nv12(self, rgb: np.ndarray) -> np.ndarray:
        w, h = self.width, self.height
        self._ensure_even_dims(w, h)
        i420 = cv.cvtColor(rgb, cv.COLOR_RGB2YUV_I420)
        return self._i420_to_nv12(i420, w, h)

    def _bgr_to_nv12(self, bgr: np.ndarray) -> np.ndarray:
        w, h = self.width, self.height
        self._ensure_even_dims(w, h)
        i420 = cv.cvtColor(bgr, cv.COLOR_BGR2YUV_I420)
        return self._i420_to_nv12(i420, w, h)

    def _convert_to_target(self, src: np.ndarray) -> np.ndarray:
        # 保持兼容：仍按 self.target_type 来转
        return self._convert_to_target_with_type(src, self.target_type)

    def _convert_to_target_with_type(self, src: np.ndarray, target_type: str) -> np.ndarray:
        # 同类型：返回拷贝，避免外部缓冲区悬挂
        if self.input_type == target_type:
            return src.copy()

        # 目标 NV12：RGB/BGR -> I420 -> NV12
        if target_type == "NV12":
            if self.input_type == "RGB":
                return self._rgb_to_nv12(src)
            if self.input_type == "BGR":
                return self._bgr_to_nv12(src)
            # 输入 NV12 且目标 NV12 已在前面返回
            raise RuntimeError("unexpected path to NV12")

        # 输入 NV12 -> 直接用 OpenCV 解码到 RGB/BGR
        if self.input_type == "NV12":
            code = cv.COLOR_YUV2RGB_NV12 if target_type == "RGB" else cv.COLOR_YUV2BGR_NV12
            dst = cv.cvtColor(src, code)
            return dst

        # RGB <-> BGR
        if self.input_type == "RGB" and target_type == "BGR":
            return cv.cvtColor(src, cv.COLOR_RGB2BGR)
        if self.input_type == "BGR" and target_type == "RGB":
            return cv.cvtColor(src, cv.COLOR_BGR2RGB)

        raise RuntimeError("unsupported colorspace conversion")
