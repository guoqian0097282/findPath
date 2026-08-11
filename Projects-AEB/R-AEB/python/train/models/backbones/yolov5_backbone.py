from __future__ import annotations
# -*- coding: utf-8 -*-
import math
from typing import List, Tuple

import torch
from torch import nn
from ultralytics.nn.modules import Conv, C3, SPPF, Concat, C2f


class YOLOv5Backbone(nn.Module):
    """
    显式 YOLOv5 (n/s/m/l/x) Backbone+Neck（无 Detect 头）：
    - 对齐官方 yolov5.yaml + parse_model 的 depth/width/max_channels 缩放逻辑
    - 输出三个尺度的特征图：P3(小)、P4(中)、P5(大)，用于后续 Detect 头
    """

    # [depth_multiple, width_multiple, max_channels]
    SCALES: dict[str, Tuple[float, float, int]] = {
        "n": (0.33, 0.25, 1024),
        "s": (0.33, 0.50, 1024),
        "m": (0.67, 0.75, 1024),
        "l": (1.00, 1.00, 1024),
        "x": (1.33, 1.25, 1024),
    }

    @staticmethod
    def _make_divisible(x: int | float, divisor: int = 8) -> int:
        return int(math.ceil(x / divisor) * divisor)

    @staticmethod
    def _round_width(ch: int, w: float, max_ch: int) -> int:
        """
        对齐 Ultralytics parse_model 的宽度缩放逻辑：
            c2 = make_divisible(min(c2, max_channels) * width_multiple, 8)
        """
        return YOLOv5Backbone._make_divisible(min(ch, max_ch) * w, 8)

    @staticmethod
    def _round_depth(n: int, d: float) -> int:
        """
        深度缩放：按 depth_multiple 缩放后至少为 1
        """
        return max(1, int(round(n * d)))

    def __init__(self, scale: str = "n", in_ch: int = 3) -> None:
        super().__init__()
        assert scale in self.SCALES, f"unknown scale: {scale}"
        d, w, max_ch = self.SCALES[scale]
        self.scale = scale

        # -------- 通道数按宽度系数缩放 --------
        c64 = self._round_width(64, w, max_ch)
        c128 = self._round_width(128, w, max_ch)
        c256 = self._round_width(256, w, max_ch)
        c512 = self._round_width(512, w, max_ch)
        c1024 = self._round_width(1024, w, max_ch)

        # -------- 各个 C3 模块的深度缩放 --------
        r3 = self._round_depth(3, d)
        r6 = self._round_depth(6, d)
        r9 = self._round_depth(9, d)

        M = nn.ModuleList()

        # ================= backbone (0..9) =================
        # 对应 yolov5.yaml backbone
        M.append(Conv(in_ch, c64, 6, 2, 2))  # 0-P1/2
        M.append(Conv(c64, c128, 3, 2))  # 1-P2/4
        M.append(C3(c128, c128, n=r3))  # 2
        M.append(Conv(c128, c256, 3, 2))  # 3-P3/8
        M.append(C3(c256, c256, n=r6))  # 4
        M.append(Conv(c256, c512, 3, 2))  # 5-P4/16
        M.append(C3(c512, c512, n=r9))  # 6
        M.append(Conv(c512, c1024, 3, 2))  # 7-P5/32
        M.append(C3(c1024, c1024, n=r3))  # 8
        M.append(SPPF(c1024, c1024, k=5))  # 9

        # ================= neck/head (10..23, 无 Detect 头) =================
        # 对应 yolov5.yaml head 中 Detect 之前的部分
        M.append(Conv(c1024, c512, 1, 1))  # 10
        M.append(nn.Upsample(scale_factor=2, mode="nearest"))  # 11
        M.append(Concat(dimension=1))  # 12, cat [x11, x6]
        M.append(C3(c512 + c512, c512, n=r3, shortcut=False))  # 13

        M.append(Conv(c512, c256, 1, 1))  # 14
        M.append(nn.Upsample(scale_factor=2, mode="nearest"))  # 15
        M.append(Concat(dimension=1))  # 16, cat [x15, x4]
        M.append(C3(c256 + c256, c256, n=r3, shortcut=False))  # 17 -> P3

        M.append(Conv(c256, c256, 3, 2))  # 18
        M.append(Concat(dimension=1))  # 19, cat [x18, x14]
        M.append(C3(2 * c256, c512, n=r3, shortcut=False))  # 20 -> P4

        M.append(Conv(c512, c512, 3, 2))  # 21
        M.append(Concat(dimension=1))  # 22, cat [x21, x10]
        M.append(C3(c512 + c512, c1024, n=r3, shortcut=False))  # 23 -> P5

        # BN 和 C2f 的配置，保持和 Ultralytics 一致
        for m in M.modules():
            if isinstance(m, nn.BatchNorm2d):
                m.momentum = 0.03
                m.eps = 0.001
            if isinstance(m, C2f):
                m.forward = m.forward_split

        self.model = M

        # 用假输入跑一遍，自动推断 strides 和 out_chs
        self._init_runtime_info()

    def _init_runtime_info(self) -> None:
        # 任意能被 32 整除的尺寸都行
        dummy_size = 128
        x = torch.zeros(1, 3, dummy_size, dummy_size)

        was_training = self.training
        self.eval()
        with torch.no_grad():
            p3, p4, p5 = YOLOv5Backbone.forward(self, x)
        if was_training:
            self.train()

        s3 = dummy_size // p3.shape[-1]
        s4 = dummy_size // p4.shape[-1]
        s5 = dummy_size // p5.shape[-1]

        # 输入尺寸 / 输出特征尺寸 = stride
        self.strides: List[int] = [s3, s4, s5]
        self.out_chs: List[int] = [p3.shape[1], p4.shape[1], p5.shape[1]]

    def forward(self, x: torch.Tensor):
        cache: List[torch.Tensor] = [None] * len(self.model)  # type: ignore

        # -------- backbone 0..9 --------
        x = self.model[0](x)
        cache[0] = x
        x = self.model[1](x)
        cache[1] = x
        x = self.model[2](x)
        cache[2] = x
        x = self.model[3](x)
        cache[3] = x
        x = self.model[4](x)
        cache[4] = x  # P3
        x = self.model[5](x)
        cache[5] = x
        x = self.model[6](x)
        cache[6] = x  # P4
        x = self.model[7](x)
        cache[7] = x
        x = self.model[8](x)
        cache[8] = x
        x = self.model[9](x)
        cache[9] = x  # backbone 输出 (1024 通道)

        # -------- head / neck 10..23 -> P3, P4, P5 --------
        x10 = self.model[10](cache[9])  # 10
        x11 = self.model[11](x10)  # up
        x12 = self.model[12]([x11, cache[6]])  # cat P4
        x13 = self.model[13](x12)  # 中层特征

        x14 = self.model[14](x13)
        x15 = self.model[15](x14)
        x16 = self.model[16]([x15, cache[4]])  # cat P3
        x17 = self.model[17](x16)  # -> P3 (small)

        x18 = self.model[18](x17)
        x19 = self.model[19]([x18, x14])  # 而不是 x13
        x20 = self.model[20](x19)         # -> P4

        x21 = self.model[21](x20)
        x22 = self.model[22]([x21, x10])
        x23 = self.model[23](x22)  # -> P5 (large)

        # 返回三个尺度
        return x17, x20, x23

if __name__ == '__main__':

    sd = torch.load("/opt_disk2/rd23442/Projects-SGS/R-AEB/assets/weights/yolov5xu-dict.pth", map_location='cpu')
    model = YOLOv5Backbone(scale='x')
    missing, unexpected = model.load_state_dict(sd, strict=False)
    print(f"加载预训练：missing={missing}\nunexpected={unexpected}")



