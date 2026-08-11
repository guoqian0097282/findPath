import numpy as np
from typing import Tuple


class Sparse4DPostProcessor:
    """
    det 后处理（numpy，无 batch）

    输入:
      det_box: (Q,8)  [cx,cy,cz,l,w,h,cos_rel,sin_rel]
      det_cls: (Q,C+1) 概率（已 softmax，含背景0）

    输出:
      boxes7:  (N,7) [cx,cy,cz,l,w,h,theta_world]
      scores:  (N,)
      labels:  (N,)  1..C
    """

    def __init__(
        self,
        num_classes: int,
        *,
        score_thr: float = 0.3,
        min_size: float = 1e-3,
        eps: float = 1e-6,
    ):
        self.num_classes = int(num_classes)
        self.score_thr = float(score_thr)
        self.min_size = float(min_size)
        self.eps = float(eps)

    def postprocess(self, det_box: np.ndarray, det_cls: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        if det_box.ndim != 2 or det_box.shape[1] != 8:
            raise ValueError(f"det_box must be (Q,8), got {det_box.shape}")
        if det_cls.ndim != 2 or det_cls.shape[0] != det_box.shape[0]:
            raise ValueError(f"det_cls must be (Q,C+1) and match Q, got {det_cls.shape}")
        if det_cls.shape[1] != self.num_classes + 1:
            raise ValueError(f"det_cls last dim must be {self.num_classes + 1}, got {det_cls.shape[1]}")

        # 1) score/label（前景最大，不含背景0）
        prob_fg = det_cls[:, 1:]  # (Q,C)
        labels = (np.argmax(prob_fg, axis=1).astype(np.int64) + 1)  # 1..C
        scores = np.max(prob_fg, axis=1).astype(np.float32)         # (Q,)

        # 2) rel yaw -> world theta（不用 TopK，不改顺序）
        cx = det_box[:, 0]
        cy = det_box[:, 1]
        cos_r = det_box[:, 6]
        sin_r = det_box[:, 7]

        r = np.sqrt(cx * cx + cy * cy + self.eps)
        cos_phi = cx / r
        sin_phi = cy / r

        cos_w = cos_r * cos_phi - sin_r * sin_phi
        sin_w = sin_r * cos_phi + cos_r * sin_phi
        theta = np.arctan2(sin_w, cos_w).astype(np.float32)  # (Q,)

        # 3) 过滤：分数阈值 + 基础几何
        ok_size = np.all(det_box[:, 3:6] > self.min_size, axis=1)
        ok_ctr = (np.sum(np.abs(det_box[:, 0:3]), axis=1) > 0.0)
        keep = (scores >= self.score_thr) & ok_size & ok_ctr

        idx = np.where(keep)[0]
        if idx.size == 0:
            return (
                np.zeros((0, 7), dtype=np.float32),
                np.zeros((0,), dtype=np.float32),
                np.zeros((0,), dtype=np.int64),
            )

        boxes7 = np.concatenate(
            [det_box[idx, 0:6].astype(np.float32, copy=False), theta[idx][:, None]],
            axis=1,
        )
        return boxes7, scores[idx], labels[idx]
