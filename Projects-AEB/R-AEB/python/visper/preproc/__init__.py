from __future__ import annotations

"""
图像处理流水线包

公开 API
-------
- preproc_InitRAEB(
      input_c: str,
      target_c: str,
      target_h: int,
      target_w: int,
  ) -> YoloLikePreprocessor

- preproc_ProcRAEB(
      img: np.ndarray,
  ) -> tuple[np.ndarray, tuple[float, float], tuple[float, float]]

说明
----
- 仅处理单张图像，但返回的 tensor 含 batch 维度：[1, C, Hm, Wm]。
- ratio_wh: (r_w, r_h) —— 宽/高方向缩放比（等比时 r_w == r_h）。
- pad_wh  : (pad_w, pad_h) —— 宽/高方向单侧填充像素（float）。
"""

from .preproc_api import preproc_Init, preproc_Convert, preproc_ToData

__all__: list[str] = ["preproc_Init", "preproc_Convert","preproc_ToData"]
