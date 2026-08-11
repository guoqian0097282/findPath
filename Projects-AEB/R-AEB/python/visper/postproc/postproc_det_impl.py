import numpy as np
import cv2
from typing import Optional, Sequence, Tuple

from visper.common.ops import xywh2xyxy
from visper.common.vis import draw_2d_boxes
from visper.postproc.postproc_nms_impl import NumpyNMS


class DetPostProcessor:
    def __init__(
        self,
        nc: int = 80,
        id2name: dict[int, str] | None = None,
        whitelist: dict[int, dict] | None = None,
        conf_thresh: float = 0.5,
        iou_thresh: float = 0.3,
    ) -> None:
        self.nc = int(nc)
        self.conf_thr = float(conf_thresh)
        self.iou_thr = float(iou_thresh)

        id2name = id2name or {}
        self.id2name: dict[int, str] = {int(k): v for k, v in id2name.items()}
        self.whitelist_ids: set[int] = {int(k) for k in (whitelist or {}).keys()}

    @staticmethod
    def _undo_letterbox_xyxy(
        xyxy_in: np.ndarray,             # (N,4) in input(letterbox) pixels
        ori_hw: Tuple[int, int],         # (h0,w0)
        scale: float,                    # r
        pad: Tuple[int, int],            # (pad_left, pad_top)
    ) -> np.ndarray:
        if xyxy_in.size == 0:
            return xyxy_in.astype(np.float32, copy=False)

        r = float(scale)
        if r <= 0:
            raise ValueError(f"Invalid scale={scale}")

        pad_left, pad_top = int(pad[0]), int(pad[1])
        h0, w0 = int(ori_hw[0]), int(ori_hw[1])

        xyxy = xyxy_in.astype(np.float32, copy=False).copy()
        xyxy[:, [0, 2]] = (xyxy[:, [0, 2]] - pad_left) / r
        xyxy[:, [1, 3]] = (xyxy[:, [1, 3]] - pad_top) / r

        # 直接裁剪到原图范围（输出是原图坐标，裁剪是必须的基本防御，不加开关）
        xyxy[:, [0, 2]] = np.clip(xyxy[:, [0, 2]], 0.0, float(w0 - 1))
        xyxy[:, [1, 3]] = np.clip(xyxy[:, [1, 3]], 0.0, float(h0 - 1))
        return xyxy

    def postprocess(
        self,
        det_cat: np.ndarray,
        *,
        ori_hw: Tuple[int, int],
        scale: float,
        pad: Tuple[int, int],
    ) -> np.ndarray:
        """
        输入：
            det_cat: (C,L) 或 (L,C), C = 4 + nc
                    前4维为 [cx,cy,w,h]（模型输入/letterbox 后像素坐标）
                    后 nc 维为类别概率（已归一化）
            ori_hw : 原图 (h0,w0)
            scale  : letterbox 缩放 r
            pad    : letterbox pad (pad_left, pad_top)

        输出：
            objs: (N,6) float32 -> [x1,y1,x2,y2,conf,cls]（原图坐标）
        """
        if det_cat.ndim != 2:
            raise ValueError("det_cat must be 2D")

        c_expected = 4 + self.nc

        # (C,L) or (L,C) -> (L,C)
        if det_cat.shape[0] == c_expected:
            x = det_cat.T
        elif det_cat.shape[1] == c_expected:
            x = det_cat
        else:
            raise ValueError(f"Invalid det_cat shape={det_cat.shape}, expect C=4+nc={c_expected}")

        x = x.astype(np.float32, copy=False)
        L = x.shape[0]

        box_cxcywh = x[:, :4]              # (L,4) input coords
        cls_prob = x[:, 4:4 + self.nc]     # (L,nc) prob

        cls_id = np.argmax(cls_prob, axis=1).astype(np.int64)
        cls_conf = cls_prob[np.arange(L, dtype=np.int64), cls_id].astype(np.float32)

        keep = cls_conf > self.conf_thr
        if self.whitelist_ids:
            keep &= np.isin(cls_id, list(self.whitelist_ids))

        if not np.any(keep):
            return np.empty((0, 6), dtype=np.float32)

        box_cxcywh = box_cxcywh[keep]
        cls_conf = cls_conf[keep]
        cls_id = cls_id[keep]

        xyxy_in = xywh2xyxy(box_cxcywh).astype(np.float32, copy=False)

        keep_idx = NumpyNMS.fast_nms(
            boxes=xyxy_in,
            scores=cls_conf,
            iou_threshold=self.iou_thr,
            use_triu=True,
            exit_early=True,
        )
        if keep_idx.size == 0:
            return np.empty((0, 6), dtype=np.float32)

        xyxy_in = xyxy_in[keep_idx]
        cls_conf = cls_conf[keep_idx]
        cls_id = cls_id[keep_idx]

        xyxy_ori = self._undo_letterbox_xyxy(
            xyxy_in,
            ori_hw=ori_hw,
            scale=scale,
            pad=pad,
        )

        objs = np.concatenate(
            [
                xyxy_ori,
                cls_conf.reshape(-1, 1),
                cls_id.reshape(-1, 1).astype(np.float32),
            ],
            axis=1,
        ).astype(np.float32, copy=False)

        return objs

    def draw_dets(
        self,
        img: np.ndarray,
        objs: np.ndarray,
        *,
        txts: Optional[Sequence[str]] = None,
        draw_bbox: bool = True,
    ) -> np.ndarray:
        if objs.size == 0 or not draw_bbox:
            return img
        out = img.copy()
        return draw_2d_boxes(out, objs, id2name=self.id2name, txts=txts)
