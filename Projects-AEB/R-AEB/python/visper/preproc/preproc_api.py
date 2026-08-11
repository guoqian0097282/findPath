from __future__ import annotations

from typing import Any

import numpy as np

from .preproc_impl import ImageConvert

__all__ = [
    "preproc_Init",
    "preproc_Convert",
    "preproc_ToData",
]

g_convert: ImageConvert | None = None


def preproc_Init(width: int, height: int, input_type: str, target_type: str) -> None:
    """
    初始化图像预处理器，相当于 C++ 的 preproc_Init。
    """
    global g_convert
    g_convert = ImageConvert(width, height, input_type, target_type)


def preproc_Convert(img: Any, target_type: str | None = None) -> Any:
    """
    执行图像转换，相当于 C++ 的 preproc_Convert。
    - target_type: 可选，临时覆盖目标类型；None 表示使用初始化时的 target_type
    未初始化时抛出与 C++ 相同的错误信息。
    """
    if g_convert is None:
        raise RuntimeError("ImageConvert 未初始化：请先调用 preproc_Init")
    return g_convert.convert(img, target_type=target_type)


def preproc_ToData(img: np.ndarray,
                   expand_axis: int | None = None,
                   transpose_axes: tuple[int, ...] | None = None,
                   out_dtype: np.dtype | str | None = None,
                   ) -> Any:
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
    return g_convert.to_data(img,
                             expand_axis=expand_axis,
                             transpose_axes=transpose_axes,
                             out_dtype=out_dtype)
