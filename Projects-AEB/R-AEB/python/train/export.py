# -*- coding: utf-8 -*-
from __future__ import annotations

from pathlib import Path
import onnx
import onnxruntime as ort
import torch
import torch.nn as nn

from models.backbones.yolo11_backbone import YOLO11Backbone
from models.model_angleseg import YOLO11AngleSeg

YOLO_SCALE: str = "n"  # "n/s/m/l/x" 任选
ONNX_PATH = Path(__file__).resolve().parents[2] / "assets" / "RAEB" / "SGS_miao.onnx"
WEIGHT_PATH = Path(
    '/opt_disk3/rd234421/Projects-SGS/R-AEB/python/train/logs/angle_2026-06-23_01-13-18/weight_dict/epoch-180.pt')


class Wrapper(nn.Module):
    """
    输入:  (N,3,Hf,Wf) 鱼眼 RGB/float32
    训练导出: feats_list, mask_coef, proto, angle_logits, angle_res
    """
    def __init__(self) -> None:
        super().__init__()
        self.yolo = YOLO11AngleSeg(scale=YOLO_SCALE, in_ch=3, nc=8)
        sd = torch.load(WEIGHT_PATH, map_location="cpu")

        incompatible = self.yolo.load_state_dict(sd, strict=True)

        print("[load_state_dict] missing_keys:")
        for k in incompatible.missing_keys:
            print(k)

        print("[load_state_dict] unexpected_keys:")
        for k in incompatible.unexpected_keys:
            print(k)

    def forward(self, x: torch.Tensor):
        # 不依赖 self.training，导出时固定走训练输出
        return self.yolo.forward(x / 255.0)

if __name__ == "__main__":

    model = Wrapper().eval()
    dummy = torch.randn(1, 3, 480, 960, dtype=torch.float32)

    # 导出 ONNX
    torch.onnx.export(
        model,
        dummy,
        str(ONNX_PATH),
        opset_version=17,
        input_names=["img"],
        output_names=["det_cat", "proto", "angle"],
        do_constant_folding=True,
    )

    m = onnx.load(str(ONNX_PATH))
    found = False
    for o in m.opset_import:
        if o.domain == "ai.onnx.ml":
            o.version = 3
            found = True
            break
    if not found:
        op = m.opset_import.add()
        op.domain = "ai.onnx.ml"
        op.version = 3
    onnx.save(m, str(ONNX_PATH))
    # ===== end patch =====

    # 2) 形状推理
    import onnx
    from onnx import shape_inference

    m = onnx.load(str(ONNX_PATH))
    m = shape_inference.infer_shapes(m)
    onnx.save(m, str(ONNX_PATH))
    print(f"[OK] 形状推理完成 ")

    # 3) ONNX Runtime 图优化（BASIC 级别）
    so = ort.SessionOptions()
    so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_BASIC
    so.optimized_model_filepath = str(ONNX_PATH)
    _ = ort.InferenceSession(str(ONNX_PATH), so, providers=["CPUExecutionProvider"])
    print(f"[OK] 图优化完成")
