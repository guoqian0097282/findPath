from __future__ import annotations
# -*- coding: utf-8 -*-
import math
import copy
from typing import List, Tuple, Dict, Optional

import torch
import torch.nn.functional as F
from torch import nn, Tensor
# 依赖 ultralytics 的工具
from ultralytics.nn.modules import Conv, C2f, DWConv
from ultralytics.utils.loss import BboxLoss
from ultralytics.utils.ops import xyxy2xywh, xywh2xyxy, crop_mask
from ultralytics.utils.tal import TaskAlignedAssigner, dist2bbox, make_anchors

from .backbones.yolo11_backbone import YOLO11Backbone
from .backbones.yolov5_backbone import YOLOv5Backbone


class DFL(nn.Module):
    """
    Integral module of Distribution Focal Loss (DFL).

    Proposed in Generalized Focal Loss https://ieeexplore.ieee.org/document/9792391
    """

    def __init__(self, c1: int = 16):
        """
        Initialize a convolutional layer with a given number of input channels.

        Args:
            c1 (int): Number of input channels.
        """
        super().__init__()
        self.conv = nn.Conv2d(c1, 1, 1, bias=False).requires_grad_(False)
        x = torch.arange(c1, dtype=torch.float)
        self.conv.weight.data[:] = nn.Parameter(x.view(1, c1, 1, 1))
        self.c1 = c1

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """Apply the DFL module to input tensor and return transformed output."""
        b, _, a = x.shape  # batch, channels, anchors
        return self.conv(x.view(b, 4, self.c1, a).softmax(2).transpose(2, 1)).view(b, 4, a)


class Detect(nn.Module):
    """
    YOLO Detect head for object detection models.

    This class implements the detection head used in YOLO models for predicting bounding boxes and class probabilities.
    It supports both training and inference modes, with optional end-to-end detection capabilities.

    Attributes:
        dynamic (bool): Force grid reconstruction.
        export (bool): Export mode flag.
        format (str): Export format.
        end2end (bool): End-to-end detection mode.
        max_det (int): Maximum detections per image.
        shape (tuple): Input shape.
        anchors (torch.Tensor): Anchor points.
        strides (torch.Tensor): Feature map strides.
        legacy (bool): Backward compatibility for v3/v5/v8/v9 models.
        xyxy (bool): Output format, xyxy or xywh.
        nc (int): Number of classes.
        nl (int): Number of detection layers.
        reg_max (int): DFL channels.
        no (int): Number of outputs per anchor.
        stride (torch.Tensor): Strides computed during build.
        cv2 (nn.ModuleList): Convolution layers for box regression.
        cv3 (nn.ModuleList): Convolution layers for classification.
        dfl (nn.Module): Distribution Focal Loss layer.
        one2one_cv2 (nn.ModuleList): One-to-one convolution layers for box regression.
        one2one_cv3 (nn.ModuleList): One-to-one convolution layers for classification.

    Methods:
        forward: Perform forward pass and return predictions.
        forward_end2end: Perform forward pass for end-to-end detection.
        bias_init: Initialize detection head biases.
        decode_bboxes: Decode bounding boxes from predictions.
        postprocess: Post-process model predictions.

    Examples:
        Create a detection head for 80 classes
        >>> detect = Detect(nc=80, ch=(256, 512, 1024))
        >>> x = [torch.randn(1, 256, 80, 80), torch.randn(1, 512, 40, 40), torch.randn(1, 1024, 20, 20)]
        >>> outputs = detect(x)
    """

    dynamic = False  # force grid reconstruction
    export = False  # export mode
    format = None  # export format
    end2end = False  # end2end
    max_det = 300  # max_det
    shape = None
    anchors = torch.empty(0)  # init
    strides = torch.empty(0)  # init
    legacy = False  # backward compatibility for v3/v5/v8/v9 models
    xyxy = False  # xyxy or xywh output

    def __init__(self, nc: int = 80, ch: tuple = ()):
        """
        Initialize the YOLO detection layer with specified number of classes and channels.

        Args:
            nc (int): Number of classes.
            ch (tuple): Tuple of channel sizes from backbone feature maps.
        """
        super().__init__()
        self.nc = nc  # number of classes
        self.nl = len(ch)  # number of detection layers
        self.reg_max = 16  # DFL channels (ch[0] // 16 to scale 4/8/12/16/20 for n/s/m/l/x)
        self.no = nc + self.reg_max * 4  # number of outputs per anchor
        self.stride = torch.zeros(self.nl)  # strides computed during build
        c2, c3 = max((16, ch[0] // 4, self.reg_max * 4)), max(ch[0], min(self.nc, 100))  # channels
        self.cv2 = nn.ModuleList(
            nn.Sequential(Conv(x, c2, 3), Conv(c2, c2, 3), nn.Conv2d(c2, 4 * self.reg_max, 1)) for x in ch
        )
        self.cv3 = (
            nn.ModuleList(nn.Sequential(Conv(x, c3, 3), Conv(c3, c3, 3), nn.Conv2d(c3, self.nc, 1)) for x in ch)
            if self.legacy
            else nn.ModuleList(
                nn.Sequential(
                    nn.Sequential(DWConv(x, x, 3), Conv(x, c3, 1)),
                    nn.Sequential(DWConv(c3, c3, 3), Conv(c3, c3, 1)),
                    nn.Conv2d(c3, self.nc, 1),
                )
                for x in ch
            )
        )
        self.dfl = DFL(self.reg_max) if self.reg_max > 1 else nn.Identity()

        if self.end2end:
            self.one2one_cv2 = copy.deepcopy(self.cv2)
            self.one2one_cv3 = copy.deepcopy(self.cv3)

    def forward(self, x: list[torch.Tensor]) -> list[torch.Tensor] | tuple:
        """Concatenate and return predicted bounding boxes and class probabilities."""
        if self.end2end:
            return self.forward_end2end(x)

        for i in range(self.nl):
            x[i] = torch.cat((self.cv2[i](x[i]), self.cv3[i](x[i])), 1)
        if self.training:  # Training path
            return x
        y = self._inference(x)
        return y if self.export else (y, x)

    def forward_end2end(self, x: list[torch.Tensor]) -> dict | tuple:
        """
        Perform forward pass of the v10Detect module.

        Args:
            x (list[torch.Tensor]): Input feature maps from different levels.

        Returns:
            outputs (dict | tuple): Training mode returns dict with one2many and one2one outputs.
                Inference mode returns processed detections or tuple with detections and raw outputs.
        """
        x_detach = [xi.detach() for xi in x]
        one2one = [
            torch.cat((self.one2one_cv2[i](x_detach[i]), self.one2one_cv3[i](x_detach[i])), 1) for i in range(self.nl)
        ]
        for i in range(self.nl):
            x[i] = torch.cat((self.cv2[i](x[i]), self.cv3[i](x[i])), 1)
        if self.training:  # Training path
            return {"one2many": x, "one2one": one2one}

        y = self._inference(one2one)
        y = self.postprocess(y.permute(0, 2, 1), self.max_det, self.nc)
        return y if self.export else (y, {"one2many": x, "one2one": one2one})

    def _inference(self, x: list[torch.Tensor]) -> torch.Tensor:
        """
        Decode predicted bounding boxes and class probabilities based on multiple-level feature maps.

        Args:
            x (list[torch.Tensor]): List of feature maps from different detection layers.

        Returns:
            (torch.Tensor): Concatenated tensor of decoded bounding boxes and class probabilities.
        """
        # Inference path
        shape = x[0].shape  # BCHW
        x_cat = torch.cat([xi.view(shape[0], self.no, -1) for xi in x], 2)
        if self.dynamic or self.shape != shape:
            self.anchors, self.strides = (x.transpose(0, 1) for x in make_anchors(x, self.stride, 0.5))
            self.shape = shape

        box, cls = x_cat.split((self.reg_max * 4, self.nc), 1)
        dbox = self.decode_bboxes(self.dfl(box), self.anchors.unsqueeze(0)) * self.strides
        return torch.cat((dbox, cls.sigmoid()), 1)

    def bias_init(self):
        """Initialize Detect() biases, WARNING: requires stride availability."""
        m = self  # self.model[-1]  # Detect() module
        # cf = torch.bincount(torch.tensor(np.concatenate(dataset.labels, 0)[:, 0]).long(), minlength=nc) + 1
        # ncf = math.log(0.6 / (m.nc - 0.999999)) if cf is None else torch.log(cf / cf.sum())  # nominal class frequency
        for a, b, s in zip(m.cv2, m.cv3, m.stride):  # from
            a[-1].bias.data[:] = 1.0  # box
            b[-1].bias.data[: m.nc] = math.log(5 / m.nc / (640 / s) ** 2)  # cls (.01 objects, 80 classes, 640 img)
        if self.end2end:
            for a, b, s in zip(m.one2one_cv2, m.one2one_cv3, m.stride):  # from
                a[-1].bias.data[:] = 1.0  # box
                b[-1].bias.data[: m.nc] = math.log(5 / m.nc / (640 / s) ** 2)  # cls (.01 objects, 80 classes, 640 img)

    def decode_bboxes(self, bboxes: torch.Tensor, anchors: torch.Tensor, xywh: bool = True) -> torch.Tensor:
        """Decode bounding boxes from predictions without calling dist2bbox (no sub op; neg allowed)."""
        use_xywh = xywh and (not self.end2end) and (not self.xyxy)

        lt, rb = bboxes.chunk(2, dim=1)  # (..,2,..) + (..,2,..)
        x1y1 = anchors + lt.neg()  # anchors - lt
        x2y2 = anchors + rb  # anchors + rb

        if use_xywh:
            c_xy = (x1y1 + x2y2) * 0.5  # (x1y1 + x2y2) / 2
            wh = x2y2 + x1y1.neg()  # x2y2 - x1y1
            return torch.cat((c_xy, wh), dim=1)

        return torch.cat((x1y1, x2y2), dim=1)

    @staticmethod
    def postprocess(preds: torch.Tensor, max_det: int, nc: int = 80) -> torch.Tensor:
        """
        Post-process YOLO model predictions.

        Args:
            preds (torch.Tensor): Raw predictions with shape (batch_size, num_anchors, 4 + nc) with last dimension
                format [x, y, w, h, class_probs].
            max_det (int): Maximum detections per image.
            nc (int, optional): Number of classes.

        Returns:
            (torch.Tensor): Processed predictions with shape (batch_size, min(max_det, num_anchors), 6) and last
                dimension format [x, y, w, h, max_class_prob, class_index].
        """
        batch_size, anchors, _ = preds.shape  # i.e. shape(16,8400,84)
        boxes, scores = preds.split([4, nc], dim=-1)
        index = scores.amax(dim=-1).topk(min(max_det, anchors))[1].unsqueeze(-1)
        boxes = boxes.gather(dim=1, index=index.repeat(1, 1, 4))
        scores = scores.gather(dim=1, index=index.repeat(1, 1, nc))
        scores, index = scores.flatten(1).topk(min(max_det, anchors))
        i = torch.arange(batch_size)[..., None]  # batch indices
        return torch.cat([boxes[i, index // nc], scores[..., None], (index % nc)[..., None].float()], dim=-1)


class Proto(nn.Module):
    """Ultralytics YOLO models mask Proto module for segmentation models."""

    def __init__(self, c1: int, c_: int = 256, c2: int = 32):
        """
        Initialize the Ultralytics YOLO models mask Proto module with specified number of protos and masks.

        Args:
            c1 (int): Input channels.
            c_ (int): Intermediate channels.
            c2 (int): Output channels (number of protos).
        """
        super().__init__()
        self.cv1 = Conv(c1, c_, k=3)
        self.upsample = nn.ConvTranspose2d(c_, c_, 2, 2, 0, bias=True)  # nn.Upsample(scale_factor=2, mode='nearest')
        # self.upsample = nn.Upsample(scale_factor=2, mode='nearest') #TODO TI兼容修改

        self.cv2 = Conv(c_, c_, k=3)
        self.cv3 = Conv(c_, c2)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """Perform a forward pass through layers using an upsampled input image."""
        return self.cv3(self.cv2(self.upsample(self.cv1(x))))


class Segment(Detect):
    """
    YOLO Segment head for segmentation models.

    This class extends the Detect head to include mask prediction capabilities for instance segmentation tasks.

    Attributes:
        nm (int): Number of masks.
        npr (int): Number of protos.
        proto (Proto): Prototype generation module.
        cv4 (nn.ModuleList): Convolution layers for mask coefficients.

    Methods:
        forward: Return model outputs and mask coefficients.

    Examples:
        Create a segmentation head
        >>> segment = Segment(nc=80, nm=32, npr=256, ch=(256, 512, 1024))
        >>> x = [torch.randn(1, 256, 80, 80), torch.randn(1, 512, 40, 40), torch.randn(1, 1024, 20, 20)]
        >>> outputs = segment(x)
    """

    def __init__(self, nc: int = 80, nm: int = 32, npr: int = 256, ch: tuple = ()):
        """
        Initialize the YOLO model attributes such as the number of masks, prototypes, and the convolution layers.

        Args:
            nc (int): Number of classes.
            nm (int): Number of masks.
            npr (int): Number of protos.
            ch (tuple): Tuple of channel sizes from backbone feature maps.
        """
        super().__init__(nc, ch)
        self.nm = nm  # number of masks
        self.npr = npr  # number of protos
        self.proto = Proto(ch[0], self.npr, self.nm)  # protos

        c4 = max(ch[0] // 4, self.nm)
        self.cv4 = nn.ModuleList(nn.Sequential(Conv(x, c4, 3), Conv(c4, c4, 3), nn.Conv2d(c4, self.nm, 1)) for x in ch)

    def forward(self, x: list[torch.Tensor]) -> tuple | list[torch.Tensor]:
        """Return model outputs and mask coefficients if training, otherwise return outputs and mask coefficients."""
        p = self.proto(x[0])  # mask protos
        bs = p.shape[0]  # batch size

        mc = torch.cat([self.cv4[i](x[i]).view(bs, self.nm, -1) for i in range(self.nl)], 2)  # mask coefficients
        x = Detect.forward(self, x)
        if self.training:
            return x, mc, p
        return (torch.cat([x, mc], 1), p) if self.export else (torch.cat([x[0], mc], 1), (x[1], mc, p))


class Angle(nn.Module):
    r"""
    Angle 分支：相对角的分桶 + 残差回归（k=0 对应 +π，随后按中心角递减）

    角度约定：
    - 桶宽 Δ = 2π / K，半宽 Δ/2 = π / K
    - 桶中心：center_k = π - k*Δ
    - 本分支统一把角度规整到区间 [-π + π/K,  π + π/K]
    """

    def __init__(self, angle_bins: int, ch: tuple[int, ...]) -> None:
        super().__init__()
        self.angle_bins = int(angle_bins)
        self.nl = len(ch)

        self.bin_size: float = (2.0 * math.pi) / float(self.angle_bins)  # Δ
        self.bin_half: float = 0.5 * self.bin_size  # Δ/2 = π/K

        centers = math.pi - torch.arange(self.angle_bins, dtype=torch.float32) * self.bin_size
        self.register_buffer("angle_centers", centers)

        c4 = max(ch[0] // 4, self.angle_bins * 2)
        self.cv_angle = nn.ModuleList(
            nn.Sequential(
                Conv(c, c4, 3),
                Conv(c4, c4, 3),
                nn.Conv2d(c4, self.angle_bins * 2, 1),
            )
            for c in ch
        )

    @staticmethod
    def _wrap_to_pi(theta: Tensor) -> Tensor:
        return (theta + math.pi) % (2.0 * math.pi) - math.pi

    @classmethod
    def encode(cls, theta: Tensor, angle_bins: int) -> tuple[Tensor, Tensor]:
        """
        theta: (B, M)
        return:
            k:      (B, M)  # 桶索引
            r_norm: (B, M)  # 归一化残差，范围 [-1, 1]
        """
        assert theta.dim() == 2, "theta must be (B, M)"
        theta = cls._wrap_to_pi(theta)

        pi = math.pi
        delta = (2.0 * math.pi) / float(angle_bins)
        half = delta * 0.5

        pi_t = theta.new_tensor(pi)
        delta_t = theta.new_tensor(delta)
        half_t = theta.new_tensor(half)

        # 环形最近桶：-π 附近应该映射到 k=0(+π) 桶，而不是被夹到最后一桶。
        k = torch.floor((pi_t + half_t - theta) / delta_t).long()
        k = torch.remainder(k, angle_bins)

        # 桶中心
        center = pi_t - k.to(theta.dtype) * delta_t  # (B, M)

        # 环形最短残差 -> 归一化残差
        r_phys = cls._wrap_to_pi(theta - center)  # (B, M)
        r_norm = (r_phys / half_t).clamp_(-1.0, 1.0)  # (B, M)

        return k, r_norm

    @classmethod
    def decode(cls, k: Tensor, r_norm: Tensor, angle_bins: int) -> Tensor:
        """
        k:      (B, M)
        r_norm: (B, M)  # [-1, 1]
        return:
            theta: (B, M)
        """
        assert k.dim() == 2 and r_norm.dim() == 2, "k and r_norm must be (B, M)"
        assert k.shape == r_norm.shape, "k and r_norm must have same shape"

        pi = r_norm.new_tensor(math.pi)
        delta = r_norm.new_tensor((2.0 * math.pi) / float(angle_bins))
        half = 0.5 * delta  # Δ/2

        # 中心：π - k*Δ
        c_star = pi - k.to(r_norm.dtype) * delta  # (B, M)

        # 残差要乘回物理量
        r_star = r_norm.clamp(-1.0, 1.0) * half  # (B, M)

        theta = cls._wrap_to_pi(c_star + r_star)  # (B, M)
        return theta

    # def forward(self, x: list[Tensor]) -> tuple[list[Tensor], list[Tensor]] | Tensor:
    #     """适用于sigmastar"""
    #     ys = [self.cv_angle[i](x[i]) for i in range(self.nl)]
    #     K = self.angle_bins
    #
    #     logits_list = [y[:, :K, ...] for y in ys]
    #     res_list = [y[:, K:, ...].tanh() for y in ys]  # 统一 tanh 一次
    #
    #     if self.training:
    #         return logits_list, res_list
    #
    #     outs: list[Tensor] = []
    #     base_centers = self.angle_centers.view(1, K, 1, 1)
    #
    #     for logits, r_norm in zip(logits_list, res_list):
    #         B, _, H, W = logits.shape
    #
    #         _, k_star = logits.max(dim=1, keepdim=True)      # (B,1,H,W)
    #         r_star = r_norm.gather(1, k_star) * self.bin_half
    #
    #         c_full = base_centers.to(logits.dtype).expand(B, K, H, W)
    #         c_star = c_full.gather(1, k_star)
    #
    #         theta = c_star + r_star                          # (B,1,H,W)
    #         outs.append(theta.flatten(1))
    #
    #     return torch.cat(outs, dim=1)

    def forward(self, x: list[Tensor]) -> tuple[list[Tensor], list[Tensor]] | Tensor:
        """适用于ti"""
        ys = [self.cv_angle[i](x[i]) for i in range(self.nl)]
        K = self.angle_bins

        # split once
        logits_list = [y[:, :K, ...] for y in ys]  # list[(B,K,H,W)]
        res_list = [y[:, K:, ...].tanh() for y in ys]  # list[(B,K,H,W)] in [-1,1]

        if self.training:
            return logits_list, res_list

        # inference: NO gather/argmax decode here
        # yolo-style flatten per level then concat across levels
        logits_flat = [l.flatten(2) for l in logits_list]  # list[(B,K,HW)]
        res_flat = [r.flatten(2) for r in res_list]  # list[(B,K,HW)]

        logits_cat = torch.cat(logits_flat, dim=2)  # (B,K,sumHW)
        res_cat = torch.cat(res_flat, dim=2)  # (B,K,sumHW)

        out = torch.cat((logits_cat, res_cat), dim=1)  # (B,2K,sumHW)
        return out


class YOLO11AngleSeg(YOLO11Backbone):
    """
    基于 YOLO11Backbone 的分割 + 角度多任务模型（继承版）：
    - 直接继承 backbone+neck（P3, P4, P5）
    - 只在 P3/P4/P5 上接 Segment 和 Angle 任务头
    """

    def __init__(
            self,
            scale: str = "n",
            in_ch: int = 3,
            nc: int = 80,
            nm: int = 32,
            angle_bins: int = 8,
    ) -> None:
        # 先初始化 backbone（会推断出 self.out_chs / self.strides）
        super().__init__(scale=scale, in_ch=in_ch)

        ch_p3, ch_p4, ch_p5 = self.out_chs
        ch_tuple = (ch_p3, ch_p4, ch_p5)

        # 分割头
        self.segment = Segment(ch=ch_tuple, nc=nc, nm=nm, npr=ch_p3)

        # 角度头
        self.angle = Angle(ch=ch_tuple, angle_bins=angle_bins)

        # 让新加的头也保持 Ultralytics 一致设置
        for m in self.modules():
            if isinstance(m, nn.BatchNorm2d):
                m.momentum = 0.03
                m.eps = 0.001
            if isinstance(m, C2f):
                m.forward = m.forward_split
            if isinstance(m, (Detect, Segment)):
                m.stride = self.strides

    def forward(self, x: torch.Tensor):
        # 直接用父类 forward 拿到 P3/P4/P5
        p3, p4, p5 = super().forward(x)

        seg_out = self.segment([p3, p4, p5])
        ang_out = self.angle([p3, p4, p5])

        if self.training:
            feats_list, mask_coef, proto = seg_out
            angle_logits, angle_res = ang_out
            return feats_list, mask_coef, proto, angle_logits, angle_res

        det_cat = seg_out[0]
        proto = seg_out[1][2]
        angle = ang_out
        return det_cat, proto, angle


class YOLOv5AngleSeg(YOLOv5Backbone):
    """
    基于 YOLO11Backbone 的分割 + 角度多任务模型：

    - 复用 YOLO11Backbone 的 backbone+neck（P3, P4, P5）
    - 只在 P3/P4/P5 上接 Segment 和 Angle 任务头
    - 训练态(self.training=True)：返回
        feats_list, mask_coef, proto, angle_logits, angle_res
    - 推理态(self.training=False)：返回
        det_cat, proto, angle
    """

    def __init__(
            self,
            scale: str = "n",
            in_ch: int = 3,
            nc: int = 80,
            nm: int = 32,
            angle_bins: int = 8,
    ) -> None:
        # 先初始化 backbone（会推断出 self.out_chs / self.strides）
        super().__init__(scale=scale, in_ch=in_ch)

        ch_p3, ch_p4, ch_p5 = self.out_chs
        ch_tuple = (ch_p3, ch_p4, ch_p5)

        # 分割头
        self.segment = Segment(ch=ch_tuple, nc=nc, nm=nm, npr=ch_p3)

        # 角度头
        self.angle = Angle(ch=ch_tuple, angle_bins=angle_bins)

        # 让新加的头也保持 Ultralytics 一致设置
        for m in self.modules():
            if isinstance(m, nn.BatchNorm2d):
                m.momentum = 0.03
                m.eps = 0.001
            if isinstance(m, C2f):
                m.forward = m.forward_split
            if isinstance(m, (Detect, Segment)):
                m.stride = self.strides

    def forward(self, x: torch.Tensor):
        # backbone 输出三个尺度的特征图：P3, P4, P5
        p3, p4, p5 = super().forward(x)

        # 任务头
        seg_out = self.segment([p3, p4, p5])
        ang_out = self.angle([p3, p4, p5])

        if self.training:
            # 训练态: Segment -> (feats_list, mask_coef, proto)
            #        Angle -> (logits_list, residual_list)
            feats_list, mask_coef, proto = seg_out
            angle_logits, angle_res = ang_out
            return feats_list, mask_coef, proto, angle_logits, angle_res

        # 推理态:
        # seg_out -> (torch.cat([y_decoded, mc], 1), (raw_feats, mc, proto))
        det_cat = seg_out[0]
        proto = seg_out[1][2]
        angle = ang_out
        return det_cat, proto, angle


class ConditionalSegAngleLossV8(nn.Module):
    def __init__(
            self,
            *,
            stride: List[int],
            nc: int,
            reg_max: int = 16,
            # gains
            box_gain: float = 7.5,
            cls_gain: float = 0.5,
            dfl_gain: float = 1.5,
            seg_gain: Optional[float] = None,
            overlap_mask: bool = False,
            # angle
            angle_bins: int = 8,
            angle_ce_weight: float = 1.0,
            angle_reg_weight: float = 1.0,
            angle_use_huber: bool = True,
            angle_huber_beta: float = 0.1,
            # TAL
            tal_topk: int = 10,
            tal_alpha: float = 0.5,
            tal_beta: float = 6.0,
    ) -> None:
        super().__init__()
        self.stride = list(stride)
        self.nc = int(nc)
        self.reg_max = int(reg_max)
        self.no = self.nc + self.reg_max * 4
        self.use_dfl = self.reg_max > 1

        self.box_gain = float(box_gain)
        self.cls_gain = float(cls_gain)
        self.dfl_gain = float(dfl_gain)
        self.seg_gain = float(box_gain if seg_gain is None else seg_gain)
        self.overlap_mask = bool(overlap_mask)

        self.angle_bins = int(angle_bins)
        self.angle_ce_w = float(angle_ce_weight)
        self.angle_reg_w = float(angle_reg_weight)
        self._angle_use_huber = bool(angle_use_huber)
        self._angle_beta = float(angle_huber_beta)

        self.bce = nn.BCEWithLogitsLoss(reduction="none")
        self.assigner = TaskAlignedAssigner(
            topk=tal_topk, num_classes=self.nc, alpha=tal_alpha, beta=tal_beta
        )
        self.bbox_loss = BboxLoss(self.reg_max)

        self.register_buffer("proj", torch.arange(self.reg_max, dtype=torch.float), persistent=False)
        self._lazy_device_bound = False

    # ---------- utils ----------
    def _bind_device_if_needed(self, ref: torch.Tensor):
        if not self._lazy_device_bound:
            dev = ref.device
            self.proj = self.proj.to(dev)
            self.bbox_loss = self.bbox_loss.to(dev)
            self._lazy_device_bound = True

    def _preprocess_targets_padded(
            self,
            *,
            gt_cls_bk: torch.Tensor,  # (B,K)
            gt_bboxes_xywh_norm_bk: torch.Tensor,  # (B,K,4)
            imgsz_hw: torch.Tensor,  # (H,W)
            device: torch.device,
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        输出：
          gt_labels : (B,K,1) Long
          gt_bboxes : (B,K,4) Float，像素坐标 xyxy
          mask_gt   : (B,K,1) Bool，有没有这个 GT
        """
        gt_cls_bk = gt_cls_bk.to(device)
        boxes = gt_bboxes_xywh_norm_bk.to(device)

        whwh = torch.tensor(
            [imgsz_hw[1], imgsz_hw[0], imgsz_hw[1], imgsz_hw[0]],
            device=device,
            dtype=boxes.dtype,
        )
        gt_bboxes = xywh2xyxy(boxes * whwh)
        gt_labels = gt_cls_bk.unsqueeze(-1).long()
        mask_gt = (gt_cls_bk >= 0).unsqueeze(-1)
        return gt_labels, gt_bboxes, mask_gt

    def _bbox_decode(self, anchor_points: torch.Tensor, pred_dist: torch.Tensor) -> torch.Tensor:
        if self.use_dfl:
            b, a, c = pred_dist.shape
            pd = pred_dist.view(b, a, 4, c // 4).softmax(3).matmul(self.proj.type(pred_dist.dtype))
        else:
            pd = pred_dist
        return dist2bbox(pd, anchor_points, xywh=False)

    @staticmethod
    def _pack_angle_lists(angle_logits_list: List[torch.Tensor],
                          angle_res_list: List[torch.Tensor]) -> torch.Tensor:
        """
        多层 -> [B, 2*bins, L]
        """
        B = angle_logits_list[0].shape[0]
        bins = angle_logits_list[0].shape[1]
        flat_logits = [t.reshape(B, bins, -1) for t in angle_logits_list]
        flat_res = [t.reshape(B, bins, -1) for t in angle_res_list]
        logits_cat = torch.cat(flat_logits, dim=2)
        res_cat = torch.cat(flat_res, dim=2)
        return torch.cat([logits_cat, res_cat], dim=1)

    @staticmethod
    def _single_mask_loss(gt_mask: torch.Tensor, pred: torch.Tensor, proto: torch.Tensor,
                          xyxy: torch.Tensor, area: torch.Tensor) -> torch.Tensor:
        pred_mask = torch.einsum("in,nhw->ihw", pred, proto)
        loss = F.binary_cross_entropy_with_logits(pred_mask, gt_mask, reduction="none")
        return (crop_mask(loss, xyxy).mean(dim=(1, 2)) / (area + 1e-6)).sum()

    def _segmentation_loss(
            self,
            *,
            fg_mask: torch.Tensor,
            masks: torch.Tensor,
            target_gt_idx: torch.Tensor,
            target_bboxes: torch.Tensor,
            batch_size: int,
            proto: torch.Tensor,
            pred_masks: torch.Tensor,
            imgsz_hw: torch.Tensor,
            overlap: bool,
    ) -> torch.Tensor:
        B, nm, Hm, Wm = proto.shape
        device = proto.device
        loss = pred_masks.sum() * 0.0

        mxyxy = (target_bboxes / imgsz_hw[[1, 0, 1, 0]]) * torch.tensor([Wm, Hm, Wm, Hm], device=device)
        marea = xyxy2xywh(target_bboxes / imgsz_hw[[1, 0, 1, 0]])[..., 2:].prod(2)

        for i in range(B):
            fg_i = fg_mask[i]
            if fg_i.any():
                gt_ids = target_gt_idx[i, fg_i]
                proto_i = proto[i]
                pred_i = pred_masks[i, fg_i]
                xyxy_i = mxyxy[i, fg_i]
                area_i = marea[i, fg_i]
                if overlap:
                    gt_mask = (masks[i] == (gt_ids.view(-1, 1, 1) + 1)).float()
                else:
                    gt_mask = masks[i][gt_ids]
                loss = loss + self._single_mask_loss(gt_mask, pred_i, proto_i, xyxy_i, area_i)
            else:
                loss = loss + (proto[i] * 0).sum() + (pred_masks[i] * 0).sum()
        return loss / (fg_mask.sum() + 1e-6)

    # ---------- forward ----------
    def forward(
            self,
            *,
            feats_list: List[torch.Tensor],
            mask_coef: torch.Tensor,
            proto: torch.Tensor,
            angle_logits_list: List[torch.Tensor],
            angle_res_list: List[torch.Tensor],
            gt_cls: torch.Tensor,  # (B,K)
            gt_bboxes_xywh_norm: torch.Tensor,  # (B,K,4)
            gt_masks: Optional[torch.Tensor] = None,
            gt_angle: Optional[torch.Tensor] = None,  # (B,K) 弧度；None 表示不监督角度
    ) -> Tuple[torch.Tensor, Dict[str, torch.Tensor]]:
        r"""
        计算 Detect + Segment + Angle 的联合损失。

        Args:
            feats_list:
                List[Tensor]，每层 (B, no, Hi, Wi)，no = nc + 4*reg_max。
                注意这是 raw head 输出（logits/DFL logits），不是归一化坐标。

            mask_coef:
                Tensor，分割系数。通常 (B, nm, L)，loss 内部会视作可变换到 (B, L, nm)。

            proto:
                Tensor，shape (B, nm, Hm, Wm)，分割原型。

            angle_logits_list:
                List[Tensor]，每层 (B, angle_bins, Hi, Wi)，角度 bin 分类 logits。

            angle_res_list:
                List[Tensor]，每层 (B, angle_bins, Hi, Wi)，角度残差（推荐已 tanh 到 [-1,1]）。

            gt_cls:
                Tensor (B, K)。类别 id，padding 位置必须为 -1。

            gt_bboxes_xywh_norm:
                Tensor (B, K, 4)。
                bbox 为 xywh（center x/y, width/height），并且是相对网络输入尺寸归一化到 [0,1] 的比例值。

            gt_masks:
                可选。若 overlap_mask=False：
                    (B, M, H, W) 的实例二值 mask 堆叠，实例顺序需与 GT 索引一致；
                若 overlap_mask=True：
                    (B, H, W) 的实例 id 图，背景0，实例j对应像素值 j+1。
                尺寸若非 (Hm,Wm) 会用 nearest resize。

            gt_angle:
                可选。Tensor (B, K) 弧度制角度，与 gt_cls 的 K 维对齐；
                padding 由 gt_cls=-1 决定是否有效。None 则不监督角度。

        Returns:
            total_loss: Tensor
            parts: dict(str -> Tensor) with keys:
                "box", "cls", "dfl", "seg", "angle_ce", "angle_reg"
        """
        B = feats_list[0].shape[0]
        self._bind_device_if_needed(feats_list[0])
        device = feats_list[0].device
        dtype = feats_list[0].dtype

        # 展开预测（YOLOv8 风格）
        imgsz_hw = torch.tensor(feats_list[0].shape[2:], device=device, dtype=dtype) * self.stride[0]
        pd_cat = torch.cat([xi.view(B, self.no, -1) for xi in feats_list], dim=2)
        pred_distri, pred_scores = pd_cat.split((self.reg_max * 4, self.nc), dim=1)
        pred_scores = pred_scores.permute(0, 2, 1).contiguous()  # (B,L,nc)
        pred_distri = pred_distri.permute(0, 2, 1).contiguous()  # (B,L,4*reg_max)
        pred_masks = mask_coef.permute(0, 2, 1).contiguous()  # (B,L,nm)

        # Anchors/stride
        anchor_points, stride_tensor = make_anchors(feats_list, self.stride, 0.5)

        # 目标预处理
        gt_labels, gt_bboxes_xyxy, mask_gt = self._preprocess_targets_padded(
            gt_cls_bk=gt_cls,
            gt_bboxes_xywh_norm_bk=gt_bboxes_xywh_norm,
            imgsz_hw=imgsz_hw,
            device=device,
        )

        # 解码 + TAL
        pred_bboxes = self._bbox_decode(anchor_points, pred_distri)
        _, target_bboxes, target_scores, fg_mask, target_gt_idx = self.assigner(
            pred_scores.detach().sigmoid(),
            (pred_bboxes.detach() * stride_tensor).type(gt_bboxes_xyxy.dtype),
            anchor_points * stride_tensor,
            gt_labels,
            gt_bboxes_xyxy,
            mask_gt,
        )
        target_scores_sum = max(target_scores.sum(), 1)

        # 分类
        loss_cls = self.bce(pred_scores, target_scores.to(pred_scores.dtype)).sum() / target_scores_sum

        # 框/DFL
        if fg_mask.sum():
            loss_box, loss_dfl = self.bbox_loss(
                pred_distri, pred_bboxes, anchor_points,
                target_bboxes / stride_tensor, target_scores, target_scores_sum, fg_mask,
            )
        else:
            z = pred_bboxes.sum() * 0.0
            loss_box, loss_dfl = z, z

        # 分割
        if gt_masks is not None:
            Bp, nm, Hm, Wm = proto.shape
            assert Bp == B, "proto batch size mismatch"
            masks_gt = gt_masks.to(device).float()
            if masks_gt.dim() == 3:
                if tuple(masks_gt.shape[-2:]) != (Hm, Wm):
                    masks_gt = F.interpolate(masks_gt[:, None], (Hm, Wm), mode="nearest")[:, 0]
            elif masks_gt.dim() == 4:
                if tuple(masks_gt.shape[-2:]) != (Hm, Wm):
                    masks_gt = F.interpolate(masks_gt, (Hm, Wm), mode="nearest")
            else:
                raise ValueError("gt_masks 维度应为 (B,Hm,Wm) 或 (B,M,Hm,Wm)")

            loss_seg = self._segmentation_loss(
                fg_mask=fg_mask, masks=masks_gt, target_gt_idx=target_gt_idx, target_bboxes=target_bboxes,
                batch_size=B, proto=proto, pred_masks=pred_masks, imgsz_hw=imgsz_hw, overlap=self.overlap_mask,
            )
        else:
            loss_seg = pred_masks.sum() * 0.0

        # ---------- 角度分支 ----------
        angle_cat = self._pack_angle_lists(angle_logits_list, angle_res_list)  # [B, 2*bins, L]
        bins = angle_cat.shape[1] // 2
        if bins != self.angle_bins:
            raise ValueError(f"angle_bins 不一致：pred={bins}, expected={self.angle_bins}")
        logits = angle_cat[:, :bins, :]  # (B,bins,L)
        resid = angle_cat[:, bins:, :]  # (B,bins,L)

        if gt_angle is None:
            loss_angle_ce = logits.sum() * 0.0
            loss_angle_reg = logits.sum() * 0.0
        else:
            # gt_angle 跟 gt_cls 对齐
            if gt_angle.shape[:2] != gt_cls.shape[:2]:
                raise ValueError("gt_angle 形状应与 (B,K) 的 gt_cls 对齐")

            # 先把 per-GT 角度编码成 (桶, 归一化残差)
            gt_angle_bin, gt_angle_rnorm = Angle.encode(gt_angle.to(device), self.angle_bins)  # (B,K),(B,K)

            B_, L = logits.shape[0], logits.shape[2]
            tgt_bin = torch.zeros((B_, L), device=device, dtype=torch.long)
            tgt_res = torch.zeros((B_, L), device=device, dtype=logits.dtype)
            valid = torch.zeros((B_, L), device=device, dtype=torch.bool)

            for b in range(B_):
                pos = fg_mask[b]  # (L,)
                if not pos.any():
                    continue
                gi = target_gt_idx[b, pos].long()  # (P,)

                # 这些 gi 本来就是 assigner 按有效 GT 匹配出来的，所以对应的 gt_cls >= 0
                k_b = gt_angle_bin[b][gi]  # (P,)
                r_b = gt_angle_rnorm[b][gi]  # (P,)

                pos_idx = torch.nonzero(pos, as_tuple=False).squeeze(1)  # (P,)
                tgt_bin[b, pos_idx] = k_b
                tgt_res[b, pos_idx] = r_b.to(tgt_res.dtype)
                valid[b, pos_idx] = True

            # CE：显式 mask
            if valid.any():
                ce_all = F.cross_entropy(logits, tgt_bin, reduction='none')  # (B,L)
                loss_angle_ce = (ce_all * valid.float()).sum() / valid.float().sum()
            else:
                loss_angle_ce = logits.sum() * 0.0

            # 残差：显式 mask
            if valid.any():
                resid_t = resid.permute(0, 2, 1)  # (B,L,bins)
                idx = tgt_bin.unsqueeze(-1)  # (B,L,1)
                picked_pred = torch.gather(resid_t, 2, idx).squeeze(-1)  # (B,L)
                if self._angle_use_huber:
                    reg_all = F.smooth_l1_loss(
                        picked_pred, tgt_res, beta=self._angle_beta, reduction='none'
                    )
                else:
                    reg_all = (picked_pred - tgt_res).abs()
                loss_angle_reg = (reg_all * valid.float()).sum() / valid.float().sum()
            else:
                loss_angle_reg = logits.sum() * 0.0

        # ---------- 汇总 ----------
        loss_box *= self.box_gain
        loss_cls *= self.cls_gain
        loss_dfl *= self.dfl_gain
        loss_seg *= self.seg_gain
        loss_ang_ce = loss_angle_ce * self.angle_ce_w
        loss_ang_reg = loss_angle_reg * self.angle_reg_w

        total = loss_box + loss_cls + loss_dfl + loss_seg + loss_ang_ce + loss_ang_reg
        parts: Dict[str, torch.Tensor] = {
            "box": loss_box.detach(),
            "cls": loss_cls.detach(),
            "dfl": loss_dfl.detach(),
            "seg": loss_seg.detach(),
            "angle_ce": loss_ang_ce.detach(),
            "angle_reg": loss_ang_reg.detach(),
        }
        return total, parts


if __name__ == '__main__':
    dummy = torch.zeros(1, 3, 640, 640)
    model = YOLO11AngleSeg().eval()
    out = model(dummy)
    print(out)
