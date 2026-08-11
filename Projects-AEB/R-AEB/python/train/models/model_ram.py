# -*- coding: utf-8 -*-
from __future__ import annotations


import torch
from torch import nn, Tensor
import torch.nn.functional as F
from typing import List, Tuple, Dict

# ultralytics：只保留 det 需要的组件
from ultralytics.nn.modules import Detect, C2f
from ultralytics.utils.loss import BboxLoss
from ultralytics.utils.ops import xywh2xyxy
from ultralytics.utils.tal import TaskAlignedAssigner, dist2bbox, make_anchors

from .backbones.yolo11_backbone import YOLO11Backbone
from .backbones.yolov5_backbone import YOLOv5Backbone


class YOLO11Det(YOLO11Backbone):
    """
    基于 YOLO11Backbone 的纯检测模型：
    - backbone+neck 输出 P3/P4/P5
    - 仅接 Detect 头
    """

    def __init__(
            self,
            scale: str = "n",
            in_ch: int = 3,
            nc: int = 80,
    ) -> None:
        super().__init__(scale=scale, in_ch=in_ch)

        ch_p3, ch_p4, ch_p5 = self.out_chs
        self.detect = Detect(nc=nc, ch=(ch_p3, ch_p4, ch_p5))

        # 保持 Ultralytics 一致设置
        for m in self.modules():
            if isinstance(m, nn.BatchNorm2d):
                m.momentum = 0.03
                m.eps = 0.001
            if isinstance(m, C2f):
                m.forward = m.forward_split
            if isinstance(m, Detect):
                m.stride = self.strides

    def forward(self, x: torch.Tensor):
        """
        Training (self.training == True):
            returns:
                feats_list: list[Tensor], len=nl
                feats_list[i].shape = (B, no, Hi, Wi)
                no = nc + 4*reg_max
                内容：raw head 输出（bbox DFL logits + cls logits），未 decode，未 sigmoid。

        Inference (self.training == False):
            Detect.forward() returns:
                y: Tensor, shape (B, 4 + nc, L)
                   y[:, 0:4, :] 为 decode 后 bbox（像素尺度；xywh/xyxy 由 Detect.xyxy 决定，默认 xywh）
                   y[:, 4:,  :] 为 cls 概率（已 sigmoid）
                feats_list: list[Tensor]，同训练态 raw 输出
            本 forward 只把 y 用 det_cat 名字返回：
                det_cat = y
        """
        p3, p4, p5 = super().forward(x)

        if self.training:
            feats_list = self.detect([p3, p4, p5])
            return feats_list

        det_cat, feats_list = self.detect([p3, p4, p5])
        return det_cat


# =========================
# Model
# =========================
def conv_block(in_channels: int, out_channels: int, norm: str | None = "BN") -> nn.Sequential:
    if norm == "IN":
        return nn.Sequential(
            nn.Conv2d(in_channels, out_channels, 3, padding="same"),
            nn.InstanceNorm2d(out_channels),
            nn.LeakyReLU(),
        )
    if norm == "BN":
        return nn.Sequential(
            nn.Conv2d(in_channels, out_channels, 3, padding="same"),
            nn.BatchNorm2d(out_channels),
            nn.LeakyReLU(),
        )
    if norm is None:
        return nn.Sequential(
            nn.Conv2d(in_channels, out_channels, 3, padding="same"),
            nn.LeakyReLU(),
        )
    raise ValueError(f"Unknown norm: {norm}")


class RPEncoder(nn.Module):
    """
    Raw Parameter Encoder: input image -> feature vector
    """

    def __init__(self, img_size: int = 256, in_channels: int = 3, out_channels: int = 128) -> None:
        super().__init__()
        self.img_size = img_size
        self.seq = nn.Sequential(
            nn.Conv2d(in_channels, 16, 7, padding="same"),
            nn.BatchNorm2d(16),
            nn.LeakyReLU(),
            nn.MaxPool2d(2, 2),

            nn.Conv2d(16, 32, 5, padding="same"),
            nn.BatchNorm2d(32),
            nn.LeakyReLU(),
            nn.MaxPool2d(2, 2),

            nn.Conv2d(32, out_channels, 3, padding="same"),
            nn.BatchNorm2d(out_channels),
            nn.LeakyReLU(),
            nn.MaxPool2d(2, 2),

            nn.AdaptiveAvgPool2d((1, 1)),
            nn.Flatten(),
        )

    def forward(self, x: Tensor) -> Tensor:
        x = F.interpolate(x, size=(self.img_size, self.img_size), mode="bilinear", align_corners=False)
        return self.seq(x)


class RPDecoder(nn.Module):
    """
    Raw Parameter Decoder: feature vector -> ISP params
    """

    def __init__(self, out_channels: int, in_channels: int = 128) -> None:
        super().__init__()
        self.mlp = nn.Sequential(
            nn.Linear(in_channels, in_channels),
            nn.LeakyReLU(),
            nn.Linear(in_channels, out_channels),
        )

    def forward(self, x: Tensor) -> Tensor:
        return self.mlp(x)


def define_feature_fusion(in_channels: int, out_channels: int = 3, ffm_params: dict | None = None) -> nn.Module:
    # default: BN_HG
    if ffm_params is None:
        ffm_type = "BN_HG"
        mid_channels = 64
    else:
        ffm_params = dict(ffm_params)  # avoid in-place pop on caller dict
        ffm_type = ffm_params.pop("ffm_type", "BN_HG")
        mid_channels = ffm_params.pop("mid_channels", 64)

    if ffm_type == "IN":
        return nn.Sequential(
            conv_block(in_channels, 16, "IN"),
            conv_block(16, mid_channels, "IN"),
            nn.Conv2d(mid_channels, out_channels, 1, padding="same"),
        )
    if ffm_type == "HG":
        return nn.Sequential(
            conv_block(in_channels, 16, None),
            conv_block(16, mid_channels, None),
            conv_block(mid_channels, 16, None),
            nn.Conv2d(16, out_channels, 1, padding="same"),
        )
    if ffm_type == "IN_HG":
        return nn.Sequential(
            conv_block(in_channels, 16, "IN"),
            conv_block(16, mid_channels, "IN"),
            conv_block(mid_channels, 16, "IN"),
            nn.Conv2d(16, out_channels, 1, padding="same"),
        )
    if ffm_type == "BN_HG":
        return nn.Sequential(
            conv_block(in_channels, 16, "BN"),
            conv_block(16, mid_channels, "BN"),
            conv_block(mid_channels, 16, "BN"),
            nn.Conv2d(16, out_channels, 1, padding="same"),
        )
    if ffm_type == "BN":
        return nn.Sequential(
            conv_block(in_channels, 16, "BN"),
            conv_block(16, mid_channels, "BN"),
            nn.Conv2d(mid_channels, out_channels, 1, padding="same"),
        )
    raise ValueError(f"Unknown ffm_type: {ffm_type}")


class RawAdaptationModule(nn.Module):
    """
    Raw Adaptation Module (RAM)
    """

    def __init__(
            self,
            in_channels: int = 3,
            img_size: int = 256,
            out_channels: int = 128,
            functions: list[str] | None = None,
            ffm_params: dict | None = None,
            clamp_values: bool = False,
    ) -> None:
        super().__init__()
        self.in_channels = int(in_channels)
        self.functions = list(functions or [])
        self.clamp_values = bool(clamp_values)

        assert len(self.functions) > 0, "functions list is empty"

        self.encoder = RPEncoder(img_size=img_size, in_channels=in_channels, out_channels=out_channels)

        for fn in self.functions:
            self.define_function(fn, rpe_out_channels=out_channels, input_channels=in_channels)

        self.norm_layer = nn.BatchNorm2d(in_channels, affine=True)

        ffm_in_channels = in_channels * max(len(self.functions), 1)
        self.ffm = define_feature_fusion(in_channels=ffm_in_channels, ffm_params=ffm_params)

        # optional cache for logging
        self.last_gamma_mean: float | None = None

    def forward(self, x: Tensor, training: bool = True) -> Tensor:
        if self.clamp_values:
            x = torch.clamp(x, min=0)

        input_ = x
        feat = self.encoder(x)

        outputs: list[Tensor] = []
        for fn in self.functions:
            out = self.apply_function(input_, fn, feat, training)
            outputs.append(out)

        x_cat = torch.cat(outputs, dim=1)
        y = self.ffm(x_cat)
        y = self.norm_layer(y)
        return y

    def define_function(self, function: str, rpe_out_channels: int, input_channels: int) -> None:
        if function == "gamma":
            self.gamma = nn.Sequential(
                RPDecoder(in_channels=rpe_out_channels, out_channels=1),
                nn.Sigmoid(),
            )
        elif function == "ccm":
            self.conv_cc = RPDecoder(in_channels=rpe_out_channels, out_channels=(input_channels ** 2))
        elif function == "wb":
            self.conv_wb = RPDecoder(in_channels=rpe_out_channels, out_channels=input_channels)
        elif function == "brightness":
            self.conv_bright = nn.Sequential(
                RPDecoder(in_channels=rpe_out_channels, out_channels=1),
                nn.Sigmoid(),
            )
        else:
            raise ValueError(f"Unknown function: {function}")

    def learn_gamma(self, feat: Tensor, training: bool) -> Tensor:
        gamma = self.gamma(feat).view(-1, 1, 1, 1)
        if training:
            self.last_gamma_mean = float(gamma.mean().detach().cpu())
        return gamma

    def apply_function(self, input_: Tensor, function: str, feat: Tensor, training: bool) -> Tensor:
        bs = feat.size(0)

        if function == "gamma":
            gamma = self.learn_gamma(feat, training)
            out = input_ ** gamma
        elif function == "ccm":
            params_cc = self.conv_cc(feat).reshape(bs, self.in_channels, self.in_channels)
            out = torch.einsum("bcij,bnc->bnij", input_, params_cc)
        elif function == "wb":
            params_wb = self.conv_wb(feat).reshape(bs, self.in_channels, 1, 1)
            out = input_ * params_wb
        elif function == "brightness":
            params_bright = self.conv_bright(feat).view(-1, 1, 1, 1)
            out = input_ + params_bright
        else:
            raise ValueError(f"Unknown function: {function}")

        return out


# =========================
# Loss
# =========================
def ssim_approx(x: Tensor, y: Tensor, data_range: float = 1.0) -> Tensor:
    """
    lightweight SSIM approx with 3x3 avgpool, returns scalar mean SSIM.
    x,y: (B,C,H,W) in [0, data_range]
    """
    c1 = (0.01 * data_range) ** 2
    c2 = (0.03 * data_range) ** 2

    mu_x = F.avg_pool2d(x, 3, 1, 1)
    mu_y = F.avg_pool2d(y, 3, 1, 1)

    sigma_x = F.avg_pool2d(x * x, 3, 1, 1) - mu_x * mu_x
    sigma_y = F.avg_pool2d(y * y, 3, 1, 1) - mu_y * mu_y
    sigma_xy = F.avg_pool2d(x * y, 3, 1, 1) - mu_x * mu_y

    num = (2 * mu_x * mu_y + c1) * (2 * sigma_xy + c2)
    den = (mu_x * mu_x + mu_y * mu_y + c1) * (sigma_x + sigma_y + c2)
    ssim_map = num / (den + 1e-12)
    return ssim_map.mean()


def total_variation(x: Tensor) -> Tensor:
    dh = (x[:, :, 1:, :] - x[:, :, :-1, :]).abs().mean()
    dw = (x[:, :, :, 1:] - x[:, :, :, :-1]).abs().mean()
    return dh + dw


class ISPReconstructionLoss(nn.Module):
    """
    Loss = w_char * Charbonnier + w_ssim * (1-SSIM) + w_tv * TV
    """

    def __init__(
            self,
            w_char: float = 1.0,
            w_ssim: float = 0.2,
            w_tv: float = 0.0,
            charbonnier_eps: float = 1e-3,
    ) -> None:
        super().__init__()
        self.w_char = float(w_char)
        self.w_ssim = float(w_ssim)
        self.w_tv = float(w_tv)
        self.eps = float(charbonnier_eps)

    def forward(self, pred: Tensor, target: Tensor) -> Tensor:
        pred_c = pred.clamp(0.0, 1.0)
        target_c = target.clamp(0.0, 1.0)

        diff = pred_c - target_c
        charbonnier = torch.mean(torch.sqrt(diff * diff + (self.eps * self.eps)))

        ssim_val = ssim_approx(pred_c, target_c, data_range=1.0)
        ssim_loss = 1.0 - ssim_val

        tv = total_variation(pred_c) if self.w_tv > 0 else pred_c.new_tensor(0.0)

        return self.w_char * charbonnier + self.w_ssim * ssim_loss + self.w_tv * tv


class ConditionalDetLossV8(nn.Module):
    """
    纯检测 loss（TAL + BCE cls + BboxLoss(box+dfl)）：
    - 输入：Detect 训练态输出 feats_list: list[(B,no,Hi,Wi)]
    - GT：gt_cls (B,K) padding=-1；gt_bboxes_xywh_norm (B,K,4) in [0,1]
    """

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

        self.bce = nn.BCEWithLogitsLoss(reduction="none")
        self.assigner = TaskAlignedAssigner(
            topk=tal_topk, num_classes=self.nc, alpha=tal_alpha, beta=tal_beta
        )
        self.bbox_loss = BboxLoss(self.reg_max)

        self.register_buffer("proj", torch.arange(self.reg_max, dtype=torch.float), persistent=False)
        self._lazy_device_bound = False

    def _bind_device_if_needed(self, ref: torch.Tensor):
        if not self._lazy_device_bound:
            dev = ref.device
            self.proj = self.proj.to(dev)
            self.bbox_loss = self.bbox_loss.to(dev)
            self._lazy_device_bound = True

    def _preprocess_targets_padded(
            self,
            *,
            gt_cls_bk: torch.Tensor,  # (B,K), padding=-1
            gt_bboxes_xywh_norm_bk: torch.Tensor,  # (B,K,4), xywh in [0,1]
            imgsz_hw: torch.Tensor,  # (H,W) pixels
            device: torch.device,
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """
        输出：
          gt_labels : (B,K,1) Long
          gt_bboxes : (B,K,4) Float，像素坐标 xyxy
          mask_gt   : (B,K,1) Bool
        """
        gt_cls_bk = gt_cls_bk.to(device)
        boxes = gt_bboxes_xywh_norm_bk.to(device)

        whwh = torch.tensor(
            [imgsz_hw[1], imgsz_hw[0], imgsz_hw[1], imgsz_hw[0]],
            device=device,
            dtype=boxes.dtype,
        )
        gt_bboxes = xywh2xyxy(boxes * whwh)  # (B,K,4) pixel xyxy
        gt_labels = gt_cls_bk.unsqueeze(-1).long()  # (B,K,1)
        mask_gt = (gt_cls_bk >= 0).unsqueeze(-1)  # (B,K,1)
        return gt_labels, gt_bboxes, mask_gt

    def _bbox_decode(self, anchor_points: torch.Tensor, pred_dist: torch.Tensor) -> torch.Tensor:
        # pred_dist: (B,L,4*reg_max)
        if self.use_dfl:
            b, a, c = pred_dist.shape
            pd = pred_dist.view(b, a, 4, c // 4).softmax(3).matmul(self.proj.type(pred_dist.dtype))
        else:
            pd = pred_dist
        return dist2bbox(pd, anchor_points, xywh=False)  # (B,L,4) in anchor units

    def forward(
            self,
            *,
            feats_list: List[torch.Tensor],  # list of (B,no,Hi,Wi)
            gt_cls: torch.Tensor,  # (B,K)
            gt_bboxes_xywh_norm: torch.Tensor,  # (B,K,4)
    ) -> Tuple[torch.Tensor, Dict[str, torch.Tensor]]:
        self._bind_device_if_needed(feats_list[0])
        device = feats_list[0].device
        dtype = feats_list[0].dtype
        B = feats_list[0].shape[0]

        # imgsz: 用 P3 的空间尺寸 * stride[0]
        imgsz_hw = torch.tensor(feats_list[0].shape[2:], device=device, dtype=dtype) * self.stride[0]

        # (B,no,Hi,Wi) -> (B,no,L)
        pd_cat = torch.cat([xi.view(B, self.no, -1) for xi in feats_list], dim=2)  # (B,no,L)

        pred_distri, pred_scores = pd_cat.split((self.reg_max * 4, self.nc), dim=1)
        pred_scores = pred_scores.permute(0, 2, 1).contiguous()  # (B,L,nc) logits
        pred_distri = pred_distri.permute(0, 2, 1).contiguous()  # (B,L,4*reg_max)

        # anchors / stride tensor（与 ultralytics 保持一致）
        anchor_points, stride_tensor = make_anchors(feats_list, self.stride, 0.5)  # (L,2), (L,1)

        # GT preprocess
        gt_labels, gt_bboxes_xyxy, mask_gt = self._preprocess_targets_padded(
            gt_cls_bk=gt_cls,
            gt_bboxes_xywh_norm_bk=gt_bboxes_xywh_norm,
            imgsz_hw=imgsz_hw,
            device=device,
        )

        # decode bbox（anchor units）
        pred_bboxes = self._bbox_decode(anchor_points, pred_distri)  # (B,L,4)

        # TAL assign（坐标用像素尺度：pred_bboxes*stride_tensor）
        _, target_bboxes, target_scores, fg_mask, _ = self.assigner(
            pred_scores.detach().sigmoid(),
            (pred_bboxes.detach() * stride_tensor).type(gt_bboxes_xyxy.dtype),
            anchor_points * stride_tensor,
            gt_labels,
            gt_bboxes_xyxy,
            mask_gt,
        )
        target_scores_sum = max(target_scores.sum(), 1)

        # cls
        loss_cls = self.bce(pred_scores, target_scores.to(pred_scores.dtype)).sum() / target_scores_sum

        # box + dfl
        if fg_mask.sum():
            loss_box, loss_dfl = self.bbox_loss(
                pred_distri,
                pred_bboxes,
                anchor_points,
                target_bboxes / stride_tensor,  # 回到 anchor units
                target_scores,
                target_scores_sum,
                fg_mask,
            )
        else:
            z = pred_bboxes.sum() * 0.0
            loss_box, loss_dfl = z, z

        # gains
        loss_box = loss_box * self.box_gain
        loss_cls = loss_cls * self.cls_gain
        loss_dfl = loss_dfl * self.dfl_gain

        total = loss_box + loss_cls + loss_dfl
        parts: Dict[str, torch.Tensor] = {
            "box": loss_box.detach(),
            "cls": loss_cls.detach(),
            "dfl": loss_dfl.detach(),
        }
        return total, parts


if __name__ == "__main__":

    # 1) 构建你的纯 det 模型（yolo11n -> "n"）
    model = YOLO11Det(scale="n", nc=80)

    # 2) 直接加载导出的官方 state_dict（先做 key 前缀修复：model.23. -> detect.）
    state_dict = torch.load(
        "../../../assets/weights/yolo11n-dict.pth",
        map_location="cpu",
    )

    fixed_state_dict = {}
    for k, v in state_dict.items():
        # 把 Detect head 的前缀从 model.23. 改成 detect.
        if k.startswith("model.23."):
            k2 = "detect." + k[len("model.23."):]
        else:
            k2 = k

        # 过滤掉 BN 的 num_batches_tracked（你的模型里通常没有这个 buffer，避免 unexpected）
        if k2.endswith("num_batches_tracked"):
            continue

        fixed_state_dict[k2] = v

    load_ret = model.load_state_dict(fixed_state_dict, strict=False)

    # 3) 打印不匹配项
    missing = list(load_ret.missing_keys)
    unexpected = list(load_ret.unexpected_keys)

    print(f"Missing keys in checkpoint (model needs but ckpt lacks): {len(missing)}")
    for k in missing[:200]:
        print(f"  [MISSING] {k}")
    if len(missing) > 200:
        print(f"  ... ({len(missing) - 200} more)")

    print(f"Unexpected keys in checkpoint (ckpt has but model lacks): {len(unexpected)}")
    for k in unexpected[:200]:
        print(f"  [UNEXPECTED] {k}")
    if len(unexpected) > 200:
        print(f"  ... ({len(unexpected) - 200} more)")

    # 4) 跑一下推理/训练输出形状（确保 forward 路径 OK）
    dummy = torch.zeros(1, 3, 640, 640)

    model.eval()
    with torch.no_grad():
        det_cat = model(dummy)  # inference: det_cat = y, (B, 4+nc, L)
    print(f"det_cat: {tuple(det_cat.shape)}")

    model.train()
    feats_list = model(dummy)  # training: list[(B,no,Hi,Wi)]
    print(f"len(feats_list): {len(feats_list)}")
    print([tuple(t.shape) for t in feats_list])
