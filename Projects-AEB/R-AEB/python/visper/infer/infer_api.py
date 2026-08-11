from __future__ import annotations
import numpy as np
from ..common.logger import logger
from .infer_ort_impl import ONNXModel, ONNXModelOpenPilot

# 全局模型变量
g_model_raeb: ONNXModel | None = None
g_model_op: ONNXModelOpenPilot | None = None


@logger.measure_time
def infer_InitRAEB(model_path: str) -> None:
    """初始化对象检测模型。

    Args:
        model_path (str): 模型文件路径。

    Raises:
        Exception: ONNXModel 构造失败时抛出。
    """
    global g_model_raeb
    g_model_raeb = ONNXModel(model_path)


def infer_InferRAEB(
        data: np.ndarray,
        *,
        squeeze_batch: bool = False
) -> list[np.ndarray]:
    """对象检测模型推理。

    参数
    ----
    data : np.ndarray
        预处理后的输入张量，形状通常为 [N, C, H, W]（N 为 batch 大小）。
    squeeze_batch : bool, 可选
        若为 True 且 N==1，则将每个输出的 batch 维（axis=0）去除，由 [1, ...] 变为 [...].
        默认 False（保持原始输出形状）。

    返回
    ----
    list[np.ndarray]
        模型推理输出列表，每个元素为一个输出张量。

    异常
    ----
    RuntimeError
        当模型未初始化时抛出。
    """
    if g_model_raeb is None:
        raise RuntimeError("RAEB 模型未初始化，请先调用 infer_InitRAEB")
    return g_model_raeb.infer(data, squeeze_batch=squeeze_batch)


@logger.measure_time
def infer_InitOP(model_path: str, device: int) -> None:
    """初始化 OpenPilot 模型。

    Args:
        model_path (str): 模型文件路径。
        device (int): 使用的 GPU 设备 ID。

    Raises:
        Exception: ONNXModelOpenPilot 构造失败时抛出。
    """
    global g_model_op
    g_model_op = ONNXModelOpenPilot(model_path, device)


@logger.measure_time
def infer_InferOP(data: np.ndarray) -> list[np.ndarray]:
    """OpenPilot 模型推理。

    Args:
        data (np.ndarray): 输入数据数组，通常为预处理后的特征张量。

    Returns:
        list[np.ndarray]:
            - prob_logits: 轨迹概率。
            - trajectory: 轨迹位置。

    Raises:
        RuntimeError: 模型未初始化时抛出。
        Exception: 推理过程中出现的其它异常。
    """
    if g_model_op is None:
        raise RuntimeError("OpenPilot 模型未初始化，请先调用 infer_InitOP")
    return g_model_op.infer(data)
