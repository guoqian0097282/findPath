import sys
import time
import torch
import numpy as np


class TorchNMS:
    """
    Ultralytics custom NMS implementation optimized for YOLO.

    This class provides static methods for performing non-maximum suppression (NMS) operations on bounding boxes,
    including both standard NMS and batched NMS for multi-class scenarios.

    Methods:
        nms: Optimized NMS with early termination that matches torchvision behavior exactly.
        batched_nms: Batched NMS for class-aware suppression.

    Examples:
        Perform standard NMS on boxes and scores
        >>> boxes = torch.tensor([[0, 0, 10, 10], [5, 5, 15, 15]])
        >>> scores = torch.tensor([0.9, 0.8])
        >>> keep = TorchNMS.nms(boxes, scores, 0.5)
    """

    @staticmethod
    def box_iou(box1: torch.Tensor, box2: torch.Tensor, eps: float = 1e-7) -> torch.Tensor:
        """
        Calculate intersection-over-union (IoU) of boxes.

        Args:
            box1 (torch.Tensor): A tensor of shape (N, 4) representing N bounding boxes in (x1, y1, x2, y2) format.
            box2 (torch.Tensor): A tensor of shape (M, 4) representing M bounding boxes in (x1, y1, x2, y2) format.
            eps (float, optional): A small value to avoid division by zero.

        Returns:
            (torch.Tensor): An NxM tensor containing the pairwise IoU values for every element in box1 and box2.

        References:
            https://github.com/pytorch/vision/blob/main/torchvision/ops/boxes.py
        """
        # NOTE: Need .float() to get accurate iou values
        # inter(N,M) = (rb(N,M,2) - lt(N,M,2)).clamp(0).prod(2)
        (a1, a2), (b1, b2) = box1.float().unsqueeze(1).chunk(2, 2), box2.float().unsqueeze(0).chunk(2, 2)
        inter = (torch.min(a2, b2) - torch.max(a1, b1)).clamp_(0).prod(2)

        # IoU = inter / (area1 + area2 - inter)
        return inter / ((a2 - a1).prod(2) + (b2 - b1).prod(2) - inter + eps)

    @staticmethod
    def fast_nms(
            boxes: torch.Tensor,
            scores: torch.Tensor,
            iou_threshold: float,
            use_triu: bool = True,
            exit_early: bool = True,
    ) -> torch.Tensor:
        """
        Fast-NMS implementation from https://arxiv.org/pdf/1904.02689 using upper triangular matrix operations.

        Args:
            boxes (torch.Tensor): Bounding boxes with shape (N, 4) in xyxy format.
            scores (torch.Tensor): Confidence scores with shape (N,).
            iou_threshold (float): IoU threshold for suppression.
            use_triu (bool): Whether to use torch.triu operator for upper triangular matrix operations.
            iou_func (callable): Function to compute IoU between boxes.
            exit_early (bool): Whether to exit early if there are no boxes.

        Returns:
            (torch.Tensor): Indices of boxes to keep after NMS.

        Examples:
            Apply NMS to a set of boxes
            >>> boxes = torch.tensor([[0, 0, 10, 10], [5, 5, 15, 15]])
            >>> scores = torch.tensor([0.9, 0.8])
            >>> keep = TorchNMS.nms(boxes, scores, 0.5)
        """
        if boxes.numel() == 0 and exit_early:
            return torch.empty((0,), dtype=torch.int64, device=boxes.device)

        sorted_idx = torch.argsort(scores, descending=True)
        boxes = boxes[sorted_idx]
        ious = TorchNMS.box_iou(boxes, boxes)
        if use_triu:
            ious = ious.triu_(diagonal=1)
            # NOTE: handle the case when len(boxes) hence exportable by eliminating if-else condition
            pick = torch.nonzero((ious >= iou_threshold).sum(0) <= 0).squeeze_(-1)
        else:
            n = boxes.shape[0]
            row_idx = torch.arange(n, device=boxes.device).view(-1, 1).expand(-1, n)
            col_idx = torch.arange(n, device=boxes.device).view(1, -1).expand(n, -1)
            upper_mask = row_idx < col_idx
            ious = ious * upper_mask
            # Zeroing these scores ensures the additional indices would not affect the final results
            scores_ = scores[sorted_idx]
            scores_[~((ious >= iou_threshold).sum(0) <= 0)] = 0
            scores[sorted_idx] = scores_  # update original tensor for NMSModel
            # NOTE: return indices with fixed length to avoid TFLite reshape error
            pick = torch.topk(scores_, scores_.shape[0]).indices
        return sorted_idx[pick]

    @staticmethod
    def nms(boxes: torch.Tensor, scores: torch.Tensor, iou_threshold: float) -> torch.Tensor:
        """
        Prefer torchvision.ops.nms if torchvision is available; otherwise fallback to pure torch NMS.
        """
        if boxes.numel() == 0:
            return torch.empty((0,), dtype=torch.int64, device=boxes.device)

        # Try torchvision first (import is slow the first time, cached afterwards)
        try:
            import torchvision  # scope as slow import

            return torchvision.ops.nms(boxes, scores, iou_threshold)
        except Exception:
            # Fallback to pure torch implementation below
            pass

        # ---------- Fallback: pure torch NMS ----------
        x1, y1, x2, y2 = boxes.unbind(1)
        areas = (x2 - x1) * (y2 - y1)

        order = scores.argsort(0, descending=True)

        keep = torch.zeros(order.numel(), dtype=torch.int64, device=boxes.device)
        keep_idx = 0

        while order.numel() > 0:
            i = order[0]
            keep[keep_idx] = i
            keep_idx += 1

            if order.numel() == 1:
                break

            rest = order[1:]
            xx1 = torch.maximum(x1[i], x1[rest])
            yy1 = torch.maximum(y1[i], y1[rest])
            xx2 = torch.minimum(x2[i], x2[rest])
            yy2 = torch.minimum(y2[i], y2[rest])

            w = (xx2 - xx1).clamp_(min=0)
            h = (yy2 - yy1).clamp_(min=0)
            inter = w * h

            if inter.sum() == 0:
                order = rest
                continue

            iou = inter / (areas[i] + areas[rest] - inter)
            order = rest[iou <= iou_threshold]

        return keep[:keep_idx]

    @staticmethod
    def batched_nms(
            boxes: torch.Tensor,
            scores: torch.Tensor,
            idxs: torch.Tensor,
            iou_threshold: float,
            use_fast_nms: bool = False,
    ) -> torch.Tensor:
        """
        Batched NMS for class-aware suppression.

        Args:
            boxes (torch.Tensor): Bounding boxes with shape (N, 4) in xyxy format.
            scores (torch.Tensor): Confidence scores with shape (N,).
            idxs (torch.Tensor): Class indices with shape (N,).
            iou_threshold (float): IoU threshold for suppression.
            use_fast_nms (bool): Whether to use the Fast-NMS implementation.

        Returns:
            (torch.Tensor): Indices of boxes to keep after NMS.

        Examples:
            Apply batched NMS across multiple classes
            >>> boxes = torch.tensor([[0, 0, 10, 10], [5, 5, 15, 15]])
            >>> scores = torch.tensor([0.9, 0.8])
            >>> idxs = torch.tensor([0, 1])
            >>> keep = TorchNMS.batched_nms(boxes, scores, idxs, 0.5)
        """
        if boxes.numel() == 0:
            return torch.empty((0,), dtype=torch.int64, device=boxes.device)

        # Strategy: offset boxes by class index to prevent cross-class suppression
        max_coordinate = boxes.max()
        offsets = idxs.to(boxes) * (max_coordinate + 1)
        boxes_for_nms = boxes + offsets[:, None]

        return (
            TorchNMS.fast_nms(boxes_for_nms, scores, iou_threshold)
            if use_fast_nms
            else TorchNMS.nms(boxes_for_nms, scores, iou_threshold)
        )


class NumpyNMS:
    @staticmethod
    def box_iou(boxes1: np.ndarray, boxes2: np.ndarray) -> np.ndarray:
        # boxes1: (M,4), boxes2: (N,4) in xyxy
        if boxes1.size == 0 or boxes2.size == 0:
            return np.zeros((boxes1.shape[0], boxes2.shape[0]), dtype=np.float32)

        x11 = boxes1[:, 0:1]
        y11 = boxes1[:, 1:2]
        x12 = boxes1[:, 2:3]
        y12 = boxes1[:, 3:4]

        x21 = boxes2[:, 0][None, :]
        y21 = boxes2[:, 1][None, :]
        x22 = boxes2[:, 2][None, :]
        y22 = boxes2[:, 3][None, :]

        xx1 = np.maximum(x11, x21)
        yy1 = np.maximum(y11, y21)
        xx2 = np.minimum(x12, x22)
        yy2 = np.minimum(y12, y22)

        w = np.clip(xx2 - xx1, 0, None)
        h = np.clip(yy2 - yy1, 0, None)
        inter = w * h

        area1 = np.clip(x12 - x11, 0, None) * np.clip(y12 - y11, 0, None)  # (M,1)
        area2 = np.clip(x22 - x21, 0, None) * np.clip(y22 - y21, 0, None)  # (1,N)
        union = area1 + area2 - inter

        return inter / np.clip(union, 1e-12, None)

    @staticmethod
    def fast_nms(
            boxes: np.ndarray,
            scores: np.ndarray,
            iou_threshold: float,
            use_triu: bool = True,
            exit_early: bool = True,
    ) -> np.ndarray:
        if boxes.size == 0 and exit_early:
            return np.empty((0,), dtype=np.int64)

        sorted_idx = np.argsort(-scores).astype(np.int64)
        boxes_s = boxes[sorted_idx]
        ious = NumpyNMS.box_iou(boxes_s, boxes_s)

        if use_triu:
            ious = np.triu(ious, k=1)
            pick = np.nonzero((ious >= iou_threshold).sum(axis=0) <= 0)[0]
        else:
            n = boxes_s.shape[0]
            upper_mask = np.triu(np.ones((n, n), dtype=bool), k=1)
            ious = ious * upper_mask

            scores_ = scores[sorted_idx].copy()
            scores_[~((ious >= iou_threshold).sum(axis=0) <= 0)] = 0
            scores[sorted_idx] = scores_  # update original array (match torch version behavior)

            pick = np.argsort(-scores_).astype(np.int64)  # like topk(full).indices

        return sorted_idx[pick]

    @staticmethod
    def nms(boxes: np.ndarray, scores: np.ndarray, iou_threshold: float) -> np.ndarray:
        if boxes.size == 0:
            return np.empty((0,), dtype=np.int64)

        x1 = boxes[:, 0]
        y1 = boxes[:, 1]
        x2 = boxes[:, 2]
        y2 = boxes[:, 3]
        areas = (x2 - x1) * (y2 - y1)

        order = np.argsort(-scores).astype(np.int64)

        keep = np.empty(order.size, dtype=np.int64)
        keep_idx = 0

        while order.size > 0:
            i = order[0]
            keep[keep_idx] = i
            keep_idx += 1

            if order.size == 1:
                break

            rest = order[1:]
            xx1 = np.maximum(x1[i], x1[rest])
            yy1 = np.maximum(y1[i], y1[rest])
            xx2 = np.minimum(x2[i], x2[rest])
            yy2 = np.minimum(y2[i], y2[rest])

            w = np.clip(xx2 - xx1, 0, None)
            h = np.clip(yy2 - yy1, 0, None)
            inter = w * h

            if inter.sum() == 0:
                order = rest
                continue

            iou = inter / (areas[i] + areas[rest] - inter)
            order = rest[iou <= iou_threshold]

        return keep[:keep_idx]

    @staticmethod
    def batched_nms(
            boxes: np.ndarray,
            scores: np.ndarray,
            idxs: np.ndarray,
            iou_threshold: float,
            use_fast_nms: bool = False,
    ) -> np.ndarray:
        if boxes.size == 0:
            return np.empty((0,), dtype=np.int64)

        max_coordinate = boxes.max()
        offsets = idxs.astype(boxes.dtype) * (max_coordinate + 1)
        boxes_for_nms = boxes + offsets[:, None]

        if use_fast_nms:
            return NumpyNMS.fast_nms(boxes_for_nms, scores, iou_threshold)
        else:
            return NumpyNMS.nms(boxes_for_nms, scores, iou_threshold)
