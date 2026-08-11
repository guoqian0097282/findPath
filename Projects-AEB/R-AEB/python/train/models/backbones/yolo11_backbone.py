from __future__ import annotations
from __future__ import annotations
# -*- coding: utf-8 -*-
import math
from typing import List, Tuple

import torch
from torch import nn
from ultralytics.nn.modules import Conv, C3k2, SPPF, C2PSA, Concat, C2f


class SPPF_k3(nn.Module):
    """Spatial Pyramid Pooling - Fast (SPPF) layer for YOLOv5 by Glenn Jocher.

    Drop-in equivalent implementation using ONLY 3x3 MaxPool internally.
    For odd k, repeated 3x3 pooling matches effective kernels: k, 2k-1, 3k-2.
    """

    def __init__(self, c1: int, c2: int, k: int = 5):
        """Initialize the SPPF layer with given input/output channels and kernel size.

        Args:
            c1 (int): Input channels.
            c2 (int): Output channels.
            k (int): Kernel size (must be odd). Kept for signature compatibility.

        Notes:
            This module is equivalent to SPP(k=(k, 2k-1, 3k-2)).
            Internally uses only 3x3 MaxPool repeated multiple times.
        """
        super().__init__()
        if k < 3 or (k % 2) == 0:
            raise ValueError(f"k must be an odd integer >= 3, got {k}.")

        c_ = c1 // 2  # hidden channels
        self.cv1 = Conv(c1, c_, 1, 1)
        self.cv2 = Conv(c_ * 4, c2, 1, 1)

        # Fixed 3x3 maxpool only
        self.m = nn.MaxPool2d(kernel_size=3, stride=1, padding=1)

        # Steps to match effective kernels: k, 2k-1, 3k-2
        self._t1 = (k - 1) // 2
        self._t2 = (k - 1)
        self._t3 = (3 * (k - 1)) // 2

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """Apply sequential pooling operations to input and return concatenated feature maps."""
        x = self.cv1(x)
        y = [x]

        z = x
        for i in range(1, self._t3 + 1):
            z = self.m(z)
            if i == self._t1 or i == self._t2 or i == self._t3:
                y.append(z)

        return self.cv2(torch.cat(y, 1))


class YOLO11Backbone(nn.Module):
    """
    显式 YOLO11 (n/s/m/l/x) Backbone+Neck（无任务头）：
    对齐官方 yolo11.yaml + parse_model 的 depth/width/max_channels 逻辑，
    并按 scale 自动设置 C3k2 的 c3k 开关（m/l/x 强制 True）。
    """

    # [depth, width, max_channels]
    SCALES: dict[str, Tuple[float, float, int]] = {
        "n": (0.50, 0.25, 1024),
        "s": (0.50, 0.50, 1024),
        "m": (0.50, 1.00, 512),
        "l": (1.00, 1.00, 512),
        "x": (1.00, 1.50, 512),
    }

    @staticmethod
    def _make_divisible(x: int | float, divisor: int = 8) -> int:
        return int(math.ceil(x / divisor) * divisor)

    @staticmethod
    def _round_width(ch: int, w: float, max_ch: int) -> int:
        """
        对齐官方 parse_model 的宽度缩放：
            c2 = make_divisible(min(c2, max_channels) * width, 8)
        先截断到 max_ch，再乘以 width，最后 make_divisible。
        """
        return YOLO11Backbone._make_divisible(min(ch, max_ch) * w, 8)

    @staticmethod
    def _round_depth(n: int, d: float) -> int:
        return max(1, int(round(n * d)))

    def __init__(self, scale: str = "n", in_ch: int = 3) -> None:
        super().__init__()
        assert scale in self.SCALES, f"unknown scale: {scale}"
        d, w, max_ch = self.SCALES[scale]
        self.scale = scale

        # width & depth（按 yaml 基准通道数 + 官方缩放公式）
        c64 = self._round_width(64, w, max_ch)
        c128 = self._round_width(128, w, max_ch)
        c256 = self._round_width(256, w, max_ch)
        c512 = self._round_width(512, w, max_ch)
        c1024 = self._round_width(1024, w, max_ch)
        r2 = self._round_depth(2, d)

        # 对应 parse_model 中：if m is C3k2 and scale in "mlx": args[3] = True
        use_c3k = scale in ("m", "l", "x")

        M = nn.ModuleList()
        # -------- backbone (0..10) --------
        M.append(Conv(in_ch, c64, 3, 2))  # 0
        M.append(Conv(c64, c128, 3, 2))  # 1
        M.append(C3k2(c128, c256, n=r2, c3k=use_c3k, e=0.25))  # 2
        M.append(Conv(c256, c256, 3, 2))  # 3
        M.append(C3k2(c256, c512, n=r2, c3k=use_c3k, e=0.25))  # 4
        M.append(Conv(c512, c512, 3, 2))  # 5
        M.append(C3k2(c512, c512, n=r2, c3k=True))  # 6
        M.append(Conv(c512, c1024, 3, 2))  # 7
        M.append(C3k2(c1024, c1024, n=r2, c3k=True))  # 8
        M.append(SPPF_k3(c1024, c1024, k=5))  # 9
        M.append(C2PSA(c1024, c1024, n=r2))  # 10

        # -------- neck/head (11..22, 无任务头) --------
        # 对应 yolo11.yaml head 的 Detect 之前 0..22 的部分
        M.append(nn.Upsample(scale_factor=2, mode="nearest"))  # 11
        M.append(Concat(dimension=1))  # 12
        M.append(C3k2(c1024 + c512, c512, n=r2, c3k=use_c3k))  # 13

        M.append(nn.Upsample(scale_factor=2, mode="nearest"))  # 14
        M.append(Concat(dimension=1))  # 15
        M.append(C3k2(c512 + c512, c256, n=r2, c3k=use_c3k))  # 16 -> P3

        M.append(Conv(c256, c256, 3, 2))  # 17
        M.append(Concat(dimension=1))  # 18
        M.append(C3k2(c256 + c512, c512, n=r2, c3k=use_c3k))  # 19 -> P4

        M.append(Conv(c512, c512, 3, 2))  # 20
        M.append(Concat(dimension=1))  # 21
        M.append(C3k2(c512 + c1024, c1024, n=r2, c3k=True))  # 22 -> P5

        # BN 和 C2f 的配置，保持和 Ultralytics 一致
        for m in M.modules():
            if isinstance(m, nn.BatchNorm2d):
                m.momentum = 0.03
                m.eps = 0.001
            if isinstance(m, C2f):
                m.forward = m.forward_split

        self.model = M

        # 用假输入跑一遍，自动推断 strides 和 out_ch
        self._init_runtime_info()

    def _init_runtime_info(self) -> None:
        # 任意能被 32 整除的尺寸都行，这里取 256
        dummy_size = 128
        x = torch.zeros(1, 3, dummy_size, dummy_size)

        # 避免污染 BN 的 running stats，临时切到 eval 再恢复
        was_training = self.training
        self.eval()
        with torch.no_grad():
            p3, p4, p5 = YOLO11Backbone.forward(self, x)
        self.train(was_training)

        s3 = dummy_size // p3.shape[-1]
        s4 = dummy_size // p4.shape[-1]
        s5 = dummy_size // p5.shape[-1]

        # 计算 strides：输入尺寸 / 输出特征尺寸
        self.strides: list[int] = [s3, s4, s5]
        # 输出通道数
        self.out_chs: list[int] = [p3.shape[1], p4.shape[1], p5.shape[1]]

    def forward(self, x: torch.Tensor):
        cache: List[torch.Tensor] = [None] * len(self.model)  # type: ignore

        # 0..10 backbone
        x = self.model[0](x)
        cache[0] = x
        x = self.model[1](x)
        cache[1] = x
        x = self.model[2](x)
        cache[2] = x
        x = self.model[3](x)
        cache[3] = x
        x = self.model[4](x)
        cache[4] = x
        x = self.model[5](x)
        cache[5] = x
        x = self.model[6](x)
        cache[6] = x
        x = self.model[7](x)
        cache[7] = x
        x = self.model[8](x)
        cache[8] = x
        x = self.model[9](x)
        cache[9] = x
        x = self.model[10](x)
        cache[10] = x  # x10

        # 11..22 head （P3, P4, P5）
        x11 = self.model[11](cache[10])  # up
        x12 = self.model[12]([x11, cache[6]])  # cat P4
        x13 = self.model[13](x12)

        x14 = self.model[14](x13)
        x15 = self.model[15]([x14, cache[4]])  # cat P3
        x16 = self.model[16](x15)  # -> P3

        x17 = self.model[17](x16)
        x18 = self.model[18]([x17, x13])
        x19 = self.model[19](x18)  # -> P4

        x20 = self.model[20](x19)
        x21 = self.model[21]([x20, cache[10]])
        x22 = self.model[22](x21)  # -> P5

        return x16, x19, x22
