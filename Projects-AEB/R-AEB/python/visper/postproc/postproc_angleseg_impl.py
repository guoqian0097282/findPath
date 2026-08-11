from __future__ import annotations
import math
import numpy as np
import cv2
from typing import Tuple, Sequence, Optional

from visper.common.ops import xywh2xyxy
from visper.common.vis import draw_2d_instances, draw_2d_boxes, color_for_cls
from visper.postproc.postproc_nms_impl import NumpyNMS


class AngleSegPostProcessor:
    def __init__(self,
                 nc: int = 80,
                 nm: int = 32,
                 id2name: dict[int, str] | None = None,
                 whitelist: dict[int, dict] | None = None,
                 conf_thresh: float = 0.5,
                 iou_thresh: float = 0.3,
                 mask_thresh: float = 0.5,
                 mask_up: int = 4,  # NEW: proto(Hm,Wm) -> mask(H0,W0) 的上采样倍数
                 refine_bbox_with_mask: bool = True,
                 ):
        """
        参数：
            conf_thresh : 分类置信度阈值
            iou_thresh  : NMS IoU 阈值（类无关）
            nm          : 原型掩膜系数维度（mask coeff 通道数），用于辅助判别 proto layout
            mask_thresh : 掩膜二值化阈值
            mask_up     : mask 输出尺寸 = (Hm*mask_up, Wm*mask_up)
            refine_bbox_with_mask : 是否利用 mask 前景修正 bbox
            id2name     : id->名称 映射
            whitelist   : id->名称 映射
        """
        self.nc = int(nc)
        self.nm = int(nm)
        self.conf_thr = float(conf_thresh)
        self.iou_thr = float(iou_thresh)
        self.mask_thresh = float(mask_thresh)
        self.mask_up = int(mask_up)
        self.refine_bbox_with_mask = bool(refine_bbox_with_mask)

        id2name = id2name or {}
        self.id2name: dict[int, str] = {int(k): v for k, v in id2name.items()}
        self.class_alias = self._build_class_alias(self.id2name)

        # whitelist 是输出类别白名单；后处理会把原始模型类别归并到这些输出类别。
        self.whitelist_ids: set[int] = {int(k) for k in (whitelist or {}).keys()}
        scan_ids = set(self.whitelist_ids)
        for src_id, dst_id in self.class_alias.items():
            if dst_id in self.whitelist_ids:
                scan_ids.add(src_id)
        self.whitelist_class_ids = np.array(sorted(i for i in scan_ids if 0 <= i < self.nc), dtype=np.int64)

    @staticmethod
    def _sigmoid(x: np.ndarray) -> np.ndarray:
        # stable enough for float32 typical ranges
        return 1.0 / (1.0 + np.exp(-x))

    @staticmethod
    def _merge_target_name(name: str) -> str:
        base = str(name).strip()
        if base.startswith("fake "):
            base = base[len("fake "):]
        if base == "bike":
            base = "bicycle"
        if base in {"bicycle", "motorcycle"}:
            return "bicycle"
        return base

    @classmethod
    def _build_class_alias(cls, id2name: dict[int, str]) -> dict[int, int]:
        target_name_to_id: dict[str, int] = {}
        for cid, name in id2name.items():
            target = cls._merge_target_name(name)
            if str(name).strip() == target and target not in target_name_to_id:
                target_name_to_id[target] = int(cid)

        alias: dict[int, int] = {}
        for cid, name in id2name.items():
            target = cls._merge_target_name(name)
            target_id = target_name_to_id.get(target)
            if target_id is not None and target_id != int(cid):
                alias[int(cid)] = target_id
        return alias

    def _canonical_class_ids(self, cls_id: np.ndarray) -> np.ndarray:
        out = cls_id.astype(np.int64, copy=True)
        if not self.class_alias:
            return out
        for src_id, dst_id in self.class_alias.items():
            out[cls_id == src_id] = dst_id
        return out

    @staticmethod
    def _compose_masks(protos: np.ndarray, coeff: np.ndarray) -> np.ndarray:
        # protos: (nm,Hm,Wm), coeff: (N,nm) -> (N,Hm,Wm)
        nm, Hm, Wm = protos.shape
        m = coeff @ protos.reshape(nm, -1)  # (N, Hm*Wm)
        m = m.reshape(-1, Hm, Wm).astype(np.float32)  # (N,Hm,Wm)
        return AngleSegPostProcessor._sigmoid(m)

    @staticmethod
    def _resize_masks_cv2(masks: np.ndarray, out_hw: Tuple[int, int]) -> np.ndarray:
        """
        masks: (N,H,W) float32
        out_hw: (out_h, out_w)
        returns: (N,out_h,out_w) float32

        使用 OpenCV INTER_LINEAR（bilinear）上采样。
        """
        out_h, out_w = int(out_hw[0]), int(out_hw[1])

        if masks.size == 0:
            return np.zeros((0, out_h, out_w), dtype=np.float32)

        if masks.ndim != 3:
            raise ValueError("masks must be (N,H,W)")

        N, in_h, in_w = masks.shape
        if (in_h, in_w) == (out_h, out_w):
            return masks.astype(np.float32, copy=False)

        masks_f = masks.astype(np.float32, copy=False)

        # 把 N 当作通道： (N,H,W) -> (H,W,N)，一次 resize
        img = np.moveaxis(masks_f, 0, -1)  # (H,W,N)

        resized = cv2.resize(img, (out_w, out_h), interpolation=cv2.INTER_LINEAR)

        # N==1 时 OpenCV 可能返回 (H,W)，需要补回通道维
        if N == 1 and resized.ndim == 2:
            resized = resized[:, :, None]

        out = np.moveaxis(resized, -1, 0)  # (N,out_h,out_w)
        return out.astype(np.float32, copy=False)

    @staticmethod
    def _crop_masks_to_boxes(masks: np.ndarray, boxes_xyxy: np.ndarray) -> np.ndarray:
        # masks: (N,H,W), boxes_xyxy: (N,4) in same coord system as H,W
        N, H, W = masks.shape
        if N == 0:
            return masks
        out = masks.copy()

        x1 = np.floor(boxes_xyxy[:, 0]).astype(np.int64)
        y1 = np.floor(boxes_xyxy[:, 1]).astype(np.int64)
        x2 = np.ceil(boxes_xyxy[:, 2]).astype(np.int64)
        y2 = np.ceil(boxes_xyxy[:, 3]).astype(np.int64)

        x1 = np.clip(x1, 0, W - 1)
        y1 = np.clip(y1, 0, H - 1)
        x2 = np.clip(x2, 0, W)
        y2 = np.clip(y2, 0, H)

        for i in range(N):
            if x1[i] >= x2[i] or y1[i] >= y2[i]:
                out[i].fill(0)
                continue
            if y1[i] > 0:
                out[i, :y1[i], :] = 0
            if y2[i] < H:
                out[i, y2[i]:, :] = 0
            if x1[i] > 0:
                out[i, :, :x1[i]] = 0
            if x2[i] < W:
                out[i, :, x2[i]:] = 0
        return out

    @staticmethod
    def _refine_boxes_from_uncropped_masks(
            masks: np.ndarray,
            boxes_xyxy: np.ndarray,
            *,
            mask_thresh: float,
            sample_levels: int = 9,
            top_ratio: float = 0.1,
            bottom_ratio: float = 0.9,
            min_pixels_per_row: int = 2,
    ) -> np.ndarray:
        """
        Refine boxes from uncropped masks by sampling left/right boundaries at multiple heights.
        """
        if masks.ndim != 3:
            raise ValueError("masks must be (N,H,W)")
        if boxes_xyxy.ndim != 2 or boxes_xyxy.shape[1] != 4:
            raise ValueError("boxes_xyxy must be (N,4)")
        if masks.shape[0] != boxes_xyxy.shape[0]:
            raise ValueError("masks and boxes_xyxy must have same N")

        N, H, W = masks.shape
        if N == 0:
            return boxes_xyxy.astype(np.float32, copy=True)

        refined = boxes_xyxy.astype(np.float32, copy=True)

        for i in range(N):
            x1, y1, x2, y2 = refined[i]
            xi1 = int(np.clip(np.floor(x1), 0, W - 1))
            yi1 = int(np.clip(np.floor(y1), 0, H - 1))
            xi2 = int(np.clip(np.ceil(x2), 0, W))
            yi2 = int(np.clip(np.ceil(y2), 0, H))
            if xi2 <= xi1 or yi2 <= yi1:
                continue

            # Use uncropped mask, but keep refinement inside current box ROI.
            roi_fg = masks[i, yi1:yi2, xi1:xi2] > mask_thresh
            ys, _ = np.where(roi_fg)
            if ys.size == 0:
                continue

            ys = ys + yi1
            y_min = int(ys.min())
            y_max = int(ys.max())
            if y_max <= y_min:
                continue

            h = float(y_max - y_min)
            v_start = int(round(y_min + top_ratio * h))
            v_end = int(round(y_min + bottom_ratio * h))
            v_start = max(y_min, min(y_max, v_start))
            v_end = max(y_min, min(y_max, v_end))
            if v_end < v_start:
                v_start, v_end = v_end, v_start

            if sample_levels <= 1:
                sample_vs = [int(round(0.5 * (v_start + v_end)))]
            else:
                sample_vs = [
                    int(round(v_start + (v_end - v_start) * k / (sample_levels - 1)))
                    for k in range(sample_levels)
                ]

            lefts: list[float] = []
            rights: list[float] = []
            for v in sample_vs:
                row_fg = masks[i, v, xi1:xi2] > mask_thresh
                xs_row = np.where(row_fg)[0]
                if xs_row.size < min_pixels_per_row:
                    continue
                lefts.append(float(xi1 + xs_row.min()))
                rights.append(float(xi1 + xs_row.max() + 1))

            if not lefts or not rights:
                continue

            nx1 = float(np.mean(lefts))
            nx2 = float(np.mean(rights))
            if not (np.isfinite(nx1) and np.isfinite(nx2)):
                continue
            if nx2 - nx1 < 1.0:
                continue

            refined[i, 0] = float(np.clip(nx1, 0.0, float(W - 1)))
            refined[i, 2] = float(np.clip(nx2, 0.0, float(W)))
            refined[i, 1] = float(np.clip(float(y_min), 0.0, float(H - 1)))
            refined[i, 3] = float(np.clip(float(y_max + 1), 0.0, float(H)))

        return refined

    # ----------------- main (single frame, no batch dim) -----------------
    def postprocess(
            self,
            det_cat: np.ndarray,  # (C,L), C=4+nc+nm
            proto: np.ndarray,  # (nm,Hm,Wm) or (Hm,Wm,nm)
            angle: np.ndarray,  # (L,)
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        Returns:
            objs:  (N,7) float32 -> [x1,y1,x2,y2,conf,cls,theta]
            masks: (N,H0,W0) uint8 -> 0/1, H0=Hm*mask_up, W0=Wm*mask_up, cropped to bbox
        """
        if det_cat.ndim != 2:
            raise ValueError("det_cat must be (C, L)")
        if proto.ndim != 3:
            raise ValueError("proto must be 3D (nm,Hm,Wm) or (Hm,Wm,nm)")
        if angle.ndim != 1:
            raise ValueError("angle must be (L,)")

        C, L = det_cat.shape
        if angle.shape[0] != L:
            raise ValueError(f"angle length mismatch: angle={angle.shape[0]} vs det_cat L={L}")

        # ---- 统一维度顺序 (nm,Hm,Wm) ----
        if proto.shape[0] == self.nm:
            nm = int(proto.shape[0])
            proto_norm = proto
        elif proto.shape[-1] == self.nm:
            nm = int(proto.shape[-1])
            proto_norm = np.moveaxis(proto, -1, 0)
        else:
            # fallback heuristic: nm is usually smaller than H/W
            if proto.shape[0] <= proto.shape[-1]:
                nm = int(proto.shape[0])
                proto_norm = proto
            else:
                nm = int(proto.shape[-1])
                proto_norm = np.moveaxis(proto, -1, 0)

        if nm <= 0:
            raise ValueError("invalid nm from proto")
        if C <= 4 + nm:
            raise ValueError(f"invalid channel layout: C={C}, nm={nm} (need C > 4 + nm)")

        nc = C - 4 - nm
        if nc <= 0:
            raise ValueError(f"invalid nc computed: nc={nc} from C={C}, nm={nm}")

        x = det_cat.T.astype(np.float32, copy=False)  # (L,C)
        protos = proto_norm.astype(np.float32, copy=False)  # (nm,Hm,Wm)
        angle_full = angle.astype(np.float32, copy=False)  # (L,)

        _, Hm, Wm = protos.shape
        H0 = int(Hm * self.mask_up)
        W0 = int(Wm * self.mask_up)
        if H0 <= 0 or W0 <= 0:
            raise ValueError(f"invalid output size: (Hm,Wm)=({Hm},{Wm}), mask_up={self.mask_up}")

        box_cxcywh = x[:, :4]  # (L,4)
        cls_scores = x[:, 4:4 + nc]  # (L,nc)
        mask_coeff = x[:, 4 + nc:4 + nc + nm]  # (L,nm)

        if mask_coeff.shape[1] != nm:
            raise ValueError(f"mask coeff dim mismatch: expect nm={nm}, got {mask_coeff.shape[1]}")

        # ---- class score + conf thresh ----
        if len(self.whitelist_ids) > 0:
            if self.whitelist_class_ids.size == 0:
                return (
                    np.empty((0, 7), np.float32),
                    np.zeros((0, H0, W0), np.uint8),
                )
            scan_scores = cls_scores[:, self.whitelist_class_ids]
            best_pos = np.argmax(scan_scores, axis=1).astype(np.int64)
            cls_id = self.whitelist_class_ids[best_pos].astype(np.int64)
            cls_conf = scan_scores[np.arange(L, dtype=np.int64), best_pos].astype(np.float32)
        else:
            cls_id = np.argmax(cls_scores, axis=1).astype(np.int64)  # (L,)
            cls_conf = cls_scores[np.arange(L, dtype=np.int64), cls_id].astype(np.float32)  # (L,)
        keep = cls_conf > self.conf_thr

        if not np.any(keep):
            return (
                np.empty((0, 7), np.float32),
                np.zeros((0, H0, W0), np.uint8),
            )

        box_cxcywh = box_cxcywh[keep]
        mask_coeff = mask_coeff[keep]
        cls_conf = cls_conf[keep]
        cls_id = self._canonical_class_ids(cls_id[keep])
        angle_full = angle_full[keep]

        # ---- cxcywh -> xyxy ----
        xyxy = xywh2xyxy(box_cxcywh)

        # ---- class-agnostic NMS using NumpyNMS.fast_nms ----
        keep_idx = NumpyNMS.fast_nms(
            boxes=xyxy,
            scores=cls_conf,
            iou_threshold=self.iou_thr,
            use_triu=True,
            exit_early=True,
        )
        if keep_idx.size == 0:
            return (
                np.empty((0, 7), np.float32),
                np.zeros((0, H0, W0), np.uint8),
            )

        xyxy = xyxy[keep_idx]
        mask_coeff = mask_coeff[keep_idx]
        cls_conf = cls_conf[keep_idx]
        cls_id = cls_id[keep_idx]
        angle_keep = angle_full[keep_idx]

        # ---- masks: compose -> upsample -> optional refine box -> crop -> binarize ----
        masks_low = self._compose_masks(protos, mask_coeff.astype(np.float32))  # (N,Hm,Wm)
        masks_img_uncropped = self._resize_masks_cv2(masks_low, (H0, W0))  # (N,H0,W0)
        if self.refine_bbox_with_mask:
            boxes_for_output = self._refine_boxes_from_uncropped_masks(
                masks_img_uncropped,
                xyxy,
                mask_thresh=self.mask_thresh,
            )
        else:
            boxes_for_output = xyxy.astype(np.float32, copy=True)
        masks_img = self._crop_masks_to_boxes(masks_img_uncropped, boxes_for_output)  # (N,H0,W0)
        masks_bin = (masks_img > self.mask_thresh).astype(np.uint8)  # (N,H0,W0)

        # ---- objs: [x1,y1,x2,y2,conf,cls,theta] ----
        objs = np.concatenate(
            [
                boxes_for_output.astype(np.float32),
                cls_conf.reshape(-1, 1).astype(np.float32),
                cls_id.reshape(-1, 1).astype(np.float32),
                angle_keep.reshape(-1, 1).astype(np.float32),
            ],
            axis=1,
        ).astype(np.float32)

        return objs, masks_bin

    def draw_angleins(
            self,
            img: np.ndarray,  # (H,W,3)
            objs: np.ndarray,  # (N,7) [x1,y1,x2,y2,conf,cls,theta]
            masks: np.ndarray,  # (N,H,W)
            *,
            txts: Optional[Sequence[str]] = None,  # (N,) 自定义文本
            draw_mask: bool = True,
            draw_bbox: bool = True,
            draw_angle: bool = True,
    ) -> np.ndarray:

        if objs.size == 0:
            return img

        out = img.copy()
        objs6 = objs[:, :6]

        # 1) mask + bbox + label：复用 draw_2d_instances
        if draw_mask:
            out = draw_2d_instances(out, objs6, masks, id2name=self.id2name, txts=txts)
        elif draw_bbox:
            out = draw_2d_boxes(out, objs6, id2name=self.id2name, txts=txts)

        # 2) 角度箭头：自己画
        if draw_angle:
            H, W = out.shape[:2]
            for x1, y1, x2, y2, conf, cls_id, theta in objs[np.argsort(objs[:, 4])]:
                color = color_for_cls(int(cls_id))

                x1 = int(np.clip(round(x1), 0, W - 1))
                y1 = int(np.clip(round(y1), 0, H - 1))
                x2 = int(np.clip(round(x2), 0, W - 1))
                y2 = int(np.clip(round(y2), 0, H - 1))

                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                L = 0.35 * max(x2 - x1, y2 - y1)
                dx, dy = int(-L * math.sin(theta)), int(-L * math.cos(theta))

                cv2.arrowedLine(out, (cx, cy), (cx + dx, cy + dy),
                                color, 1, cv2.LINE_AA, tipLength=0.2)

        return out

    def draw_angleins_with_track_info(
            self,
            img: np.ndarray,  # (H,W,3)
            objs: np.ndarray,  # (Nt,6/7) [x1,y1,x2,y2,conf,cls(,theta)]
            masks: np.ndarray,  # (Nt,H,W)
            track_info: np.ndarray,  # (Nt,4) [track_id, track_state, track_age, idx]
            *,
            draw_mask: bool = True,
            draw_bbox: bool = True,
            draw_angle: bool = True,
    ) -> np.ndarray:
        """
        在 img 上绘制跟踪后的实例结果并返回新图。

        约定：
          - track_info[i] = [track_id, track_state, track_age, idx]
          - objs[i]       = [x1,y1,x2,y2,conf,cls(,theta)]
          - masks[i]      = (H,W) mask，与 objs 同顺序
        """
        if track_info.size == 0 or objs.size == 0:
            return img

        # 保证 objs 是 (Nt,7)：没有 theta 就补 0
        if objs.shape[1] == 6:
            z = np.zeros((objs.shape[0], 1), dtype=objs.dtype)
            objs7 = np.concatenate([objs, z], axis=1)
        else:
            objs7 = objs

        # 为每个跟踪目标构造一条自定义文本（与 objs / masks 的顺序一一对应）
        # 文本格式：ID <track_id> <class_name> <conf> A<track_age>
        txts = [
            f"ID {int(track_info[i, 0])} "  # track_id：跟踪器分配的目标 ID
            f"{self.id2name.get(int(objs7[i, 5]), str(int(objs7[i, 5])))} "  # 类别名（无映射则用 cls_id）
            f"{float(objs7[i, 4]):.2f} "  # 置信度 conf，保留两位小数
            f"A{int(track_info[i, 2])}"  # track_age：目标已持续跟踪的帧数/年龄
            for i in range(objs7.shape[0])
        ]

        return self.draw_angleins(
            img,
            objs7,
            masks,
            txts=txts,
            draw_mask=draw_mask,
            draw_bbox=draw_bbox,
            draw_angle=draw_angle,
        )
