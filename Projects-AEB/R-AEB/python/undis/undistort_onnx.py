# -*- coding: utf-8 -*-
from __future__ import annotations

import cv2
import numpy as np
import onnxruntime as ort
import torch
import torch.nn as nn
from pathlib import Path

from python.undis.undistort_rear import build_map_fish_to_cyl, build_map_cyl_to_fish


# -------------------- 推理：用 float map_xy 做最近邻 gather --------------------
class RemapGather(nn.Module):
    """
    x:      (B, C, H_src, W_src)
    map_xy: (H_dst, W_dst, 2) float32, stores (u_src, v_src)
    mask:   (H_dst, W_dst) float32, 1(valid) / 0(invalid)
    out:    (B, C, H_dst, W_dst)

    mode:
      - "nearest": round(u,v) then gather
      - "bilinear": 4-point gather + weights
    """

    def __init__(
            self,
            map_xy: np.ndarray,
            mask: np.ndarray,
            src_wh: tuple[int, int],
            mode: str = "nearest",
            eps: float = 1e-6,
    ) -> None:
        super().__init__()
        W_src, H_src = src_wh
        self.W_src = int(W_src)
        self.H_src = int(H_src)
        self.mode = str(mode)
        self.eps = float(eps)

        H_dst, W_dst, _ = map_xy.shape
        self.H_dst = int(H_dst)
        self.W_dst = int(W_dst)
        N = self.H_dst * self.W_dst

        flat = map_xy.reshape(N, 2).astype(np.float32)
        self.register_buffer("u", torch.from_numpy(flat[:, 0]).view(1, 1, N))  # (1,1,N)
        self.register_buffer("v", torch.from_numpy(flat[:, 1]).view(1, 1, N))  # (1,1,N)

        self.register_buffer("mask", torch.from_numpy(mask.reshape(N).astype(np.float32)).view(1, 1, N))

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        B, C, H, W = x.shape
        x_flat = x.view(B, C, H * W)

        if self.mode == "nearest":
            x_nn = torch.round(self.u).to(torch.int64).clamp(0, self.W_src - 1)
            y_nn = torch.round(self.v).to(torch.int64).clamp(0, self.H_src - 1)

            idx = (y_nn * self.W_src + x_nn).expand(B, C, -1)
            m = self.mask.expand(B, C, -1)

            y_flat = torch.gather(x_flat, 2, idx) * m
            return y_flat.view(B, C, self.H_dst, self.W_dst)

        if self.mode == "bilinear":
            # floor/ceil
            u0 = torch.floor(self.u)
            v0 = torch.floor(self.v)
            u1 = u0 + 1.0
            v1 = v0 + 1.0

            # clamp to image bounds
            u0i = u0.to(torch.int64).clamp(0, self.W_src - 1)
            v0i = v0.to(torch.int64).clamp(0, self.H_src - 1)
            u1i = u1.to(torch.int64).clamp(0, self.W_src - 1)
            v1i = v1.to(torch.int64).clamp(0, self.H_src - 1)

            # weights
            du = (self.u - u0).clamp(0.0, 1.0)
            dv = (self.v - v0).clamp(0.0, 1.0)

            w00 = (1.0 - du) * (1.0 - dv)
            w10 = du * (1.0 - dv)
            w01 = (1.0 - du) * dv
            w11 = du * dv

            # linear indices
            idx00 = (v0i * self.W_src + u0i).expand(B, C, -1)
            idx10 = (v0i * self.W_src + u1i).expand(B, C, -1)
            idx01 = (v1i * self.W_src + u0i).expand(B, C, -1)
            idx11 = (v1i * self.W_src + u1i).expand(B, C, -1)

            # gather 4 corners
            p00 = torch.gather(x_flat, 2, idx00)
            p10 = torch.gather(x_flat, 2, idx10)
            p01 = torch.gather(x_flat, 2, idx01)
            p11 = torch.gather(x_flat, 2, idx11)

            # apply weights + mask (mask来自几何有效性；边界clamp导致的重复采样也会被这个 mask 控制“外面为0”)
            m = self.mask.expand(B, C, -1)
            y_flat = (p00 * w00 + p10 * w10 + p01 * w01 + p11 * w11) * m

            return y_flat.view(B, C, self.H_dst, self.W_dst)

        raise ValueError(f"Unsupported mode: {self.mode} (use 'nearest' or 'bilinear')")


class RemapGridSample(nn.Module):
    """
    x:      (B, C, H_src, W_src)
    map_xy: (H_dst, W_dst, 2) float32, stores (u_src, v_src) in pixel coords
    mask:   (H_dst, W_dst) float32
    out:    (B, C, H_dst, W_dst)

    mode: "nearest" or "bilinear"
    """

    def __init__(
            self,
            map_xy: np.ndarray,
            mask: np.ndarray,
            src_wh: tuple[int, int],
            mode: str = "bilinear",
            align_corners: bool = True,
            padding_mode: str = "zeros",
    ) -> None:
        super().__init__()
        W_src, H_src = src_wh
        self.W_src = int(W_src)
        self.H_src = int(H_src)

        self.mode = str(mode)
        self.align_corners = bool(align_corners)
        self.padding_mode = str(padding_mode)

        H_dst, W_dst, _ = map_xy.shape
        self.H_dst = int(H_dst)
        self.W_dst = int(W_dst)

        # map_xy: pixel (u,v) -> normalized (x,y) for grid_sample
        uv = map_xy.astype(np.float32)
        u = uv[:, :, 0]
        v = uv[:, :, 1]

        if self.align_corners:
            # u=0 -> -1, u=W-1 -> +1
            x_norm = (2.0 * u / max(self.W_src - 1, 1)) - 1.0
            y_norm = (2.0 * v / max(self.H_src - 1, 1)) - 1.0
        else:
            # pixel-center convention
            x_norm = (2.0 * (u + 0.5) / self.W_src) - 1.0
            y_norm = (2.0 * (v + 0.5) / self.H_src) - 1.0

        grid = np.stack([x_norm, y_norm], axis=-1)  # (H_dst, W_dst, 2)

        # buffers
        self.register_buffer("grid", torch.from_numpy(grid).unsqueeze(0))  # (1,Hd,Wd,2)
        self.register_buffer("mask", torch.from_numpy(mask.astype(np.float32)).unsqueeze(0).unsqueeze(0))  # (1,1,Hd,Wd)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # x: (B,C,H,W)
        B, C, H, W = x.shape

        grid = self.grid.expand(B, -1, -1, -1)  # (B,Hd,Wd,2)

        y = torch.nn.functional.grid_sample(
            x,
            grid,
            mode=self.mode,  # "nearest" / "bilinear"
            padding_mode=self.padding_mode,  # "zeros" / "border" / "reflection"
            align_corners=self.align_corners
        )

        # mask: (1,1,Hd,Wd) -> (B,C,Hd,Wd)
        return y * self.mask.expand(B, C, -1, -1)


def export_onnx(model: nn.Module, onnx_path: Path | str, src_wh: tuple[int, int]) -> None:
    W, H = src_wh
    dummy = torch.randn(1, 3, H, W, dtype=torch.float32)
    torch.onnx.export(
        model,
        dummy,
        str(onnx_path),
        opset_version=17,
        input_names=["img"],
        output_names=["out"]
    )

    import onnx
    from onnx import shape_inference

    m = onnx.load(str(onnx_path))
    m = shape_inference.infer_shapes(m)
    onnx.save(m, str(onnx_path))
    print("[OK] shape inference")

    so = ort.SessionOptions()
    so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_BASIC
    so.optimized_model_filepath = str(onnx_path)
    _ = ort.InferenceSession(str(onnx_path), so, providers=["CPUExecutionProvider"])
    print("[OK] ORT optimize")


# -------------------- 主程序：导出并用 ORT 验证 --------------------
if __name__ == "__main__":
    out_dir = Path("./out")
    out_dir.mkdir(parents=True, exist_ok=True)

    onnx_f2c_path = "fisheye_to_cyl.onnx"
    onnx_c2f_path = "cyl_to_fisheye.onnx"

    map_f2c, mask_f2c, fish_size, cyl_size = build_map_fish_to_cyl()
    map_c2f, mask_c2f, _, _ = build_map_cyl_to_fish()

    model_f2c = RemapGather(map_f2c, mask_f2c, src_wh=fish_size).eval()
    model_c2f = RemapGather(map_c2f, mask_c2f, src_wh=cyl_size).eval()

    export_onnx(model_f2c, onnx_f2c_path, fish_size)  # 输入：鱼眼 full
    export_onnx(model_c2f, onnx_c2f_path, cyl_size)  # 输入：圆柱缩放图

    # ORT 验证
    img_path = Path(f"./test_data/Fishb_Outdoor_cld_LuxG_Park_20251103_Huizhou-Longhorn-fakechild_2_NA_1.jpg")
    img = cv2.imread(str(img_path), cv2.IMREAD_COLOR)
    if img is None:
        raise FileNotFoundError(f"Failed to read image: {img_path}")

    img = cv2.resize(img, (464, 328), interpolation=cv2.INTER_LINEAR)
    img_nchw = np.transpose(img, (2, 0, 1))[None, ...].astype(np.float32)
    stem = img_path.stem
    ext = img_path.suffix

    cyl_out = out_dir / f"{stem}_cyl_onnx{ext}"
    fish_out = out_dir / f"{stem}_fisheye_onnx{ext}"

    sess_f2c = ort.InferenceSession(str(onnx_f2c_path), providers=["CPUExecutionProvider"])
    cyl = sess_f2c.run(["out"], {"img": img_nchw})[0]
    cyl_u8 = np.clip(np.transpose(cyl[0], (1, 2, 0)), 0.0, 255.0).astype(np.uint8)
    cv2.imwrite(str(cyl_out), cyl_u8)

    sess_c2f = ort.InferenceSession(str(onnx_c2f_path), providers=["CPUExecutionProvider"])
    fish = sess_c2f.run(["out"], {"img": cyl})[0]
    fish_u8 = np.clip(np.transpose(fish[0], (1, 2, 0)), 0.0, 255.0).astype(np.uint8)
    cv2.imwrite(str(fish_out), fish_u8)

    print(f"[OK] wrote outputs: {cyl_out} , {fish_out}")
