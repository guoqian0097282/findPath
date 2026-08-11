from __future__ import annotations
import re
from typing import Optional, Tuple, List

import numpy as np
import onnxruntime

from ..common.logger import logger



class ONNXModel:
    """ONNX Runtime 推理封装（基于 device: 'cpu' | 'cuda:N' 选择后端）。"""

    def __init__(self, model_path: str, device: str = "cpu") -> None:
        """
        Args:
            model_path: ONNX 模型文件路径。
            device: "cpu" 或 "cuda:<index>"（例如 "cuda:0"、"cuda:1"）。
        """
        backend, cuda_index = self._parse_device(device)

        # 判定可用 provider
        available = set(onnxruntime.get_available_providers())

        # 期望用 CUDA，但不可用 -> 警告并强制 CPU
        use_cuda = backend == "cuda" and "CUDAExecutionProvider" in available
        if backend == "cuda" and not use_cuda:
            logger.warning("请求使用 CUDA，但系统不可用，自动回退到 CPUExecutionProvider")

        # 组装 providers 与 options
        if use_cuda:
            providers: List[str] = ["CUDAExecutionProvider", "CPUExecutionProvider"]
            provider_options = [{"device_id": cuda_index}, {}]  # 先尝试 CUDA，再回退 CPU
        else:
            providers = ["CPUExecutionProvider"]
            provider_options = None

        # 创建 Session，显存不足时回退 CPU
        try:
            if provider_options is not None:
                self.session = onnxruntime.InferenceSession(
                    model_path, providers=providers, provider_options=provider_options
                )
                self._using_gpu = True
                logger.debug(f"ONNX Runtime 使用 CUDAExecutionProvider, device_id={cuda_index}")
            else:
                self.session = onnxruntime.InferenceSession(
                    model_path, providers=providers
                )
                self._using_gpu = False
                logger.debug("ONNX Runtime 使用 CPUExecutionProvider")
        except RuntimeError as e:
            # 显存不足等 CUDA 失败 -> 回退 CPU
            if "out of memory" in str(e).lower() or "cuda" in str(e).lower():
                logger.warning(f"CUDA 失败（{e}），回退到 CPUExecutionProvider")
                self.session = onnxruntime.InferenceSession(
                    model_path, providers=["CPUExecutionProvider"]
                )
                self._using_gpu = False
            else:
                raise

        self.input_names = [inp.name for inp in self.session.get_inputs()]
        self.output_names = [out.name for out in self.session.get_outputs()]

        logger.debug(
            f"ONNXModel initialized from '{model_path}', "
            f"device='{device}', using_gpu={self._using_gpu}, "
            f"inputs={self.input_names}, outputs={self.output_names}"
        )

    @staticmethod
    def _parse_device(device: str) -> Tuple[str, Optional[int]]:
        """
        仅接受 'cpu' 或 'cuda:<index>'。返回 ('cpu', None) 或 ('cuda', index)。
        """
        if device == "cpu":
            return "cpu", None
        m = re.fullmatch(r"cuda:(\d+)", device)
        if m:
            return "cuda", int(m.group(1))
        raise ValueError("device 仅支持 'cpu' 或 'cuda:<index>'，例如 'cuda:0'")

    def infer(self, input_data: np.ndarray, *, squeeze_batch: bool = False) -> list[np.ndarray]:
        """
        执行推理。

        参数
        ----
        input_data     : 输入张量，形状通常为 [N, ...]（N 为 batch 大小）
        squeeze_batch  : 若为 True，则对每个输出在 batch 维 (axis=0) 为 1 时去掉该维度，
                         即由 [1, ...] 变为 [...]

        返回
        ----
        outputs : list[np.ndarray]
            - 当 squeeze_batch=False（默认）：原始输出，形如 [ [N, ...], ... ]
            - 当 squeeze_batch=True ：若某个输出的第 0 维为 1，则返回时变为 [...,]（去掉第 0 维）
        """
        if input_data.ndim == 0:
            raise ValueError("input_data 至少应包含 batch 维（形如 [N, ...]）")

        feed = {self.input_names[0]: input_data}
        outputs = self.session.run(self.output_names, feed)

        if squeeze_batch:
            outputs = [
                (out[0] if isinstance(out, np.ndarray) and out.ndim >= 1 and out.shape[0] == 1 else out)
                for out in outputs
            ]
        return outputs


class ONNXModelOpenPilot:
    """带循环状态的 OpenPilot ONNX 模型封装。"""

    def __init__(self, model_path: str, gpu_id: int) -> None:
        """初始化 OpenPilot 模型并设置循环状态。

        Args:
            model_path: ONNX 模型文件的路径。
            gpu_id: GPU 设备编号，0 表示 GPU0，1 表示 GPU1；若传入负值，则强制只用 CPU。
        """
        # 根据 gpu_id 及可用 provider 决定 provider 列表和 options
        if gpu_id >= 0 and "CUDAExecutionProvider" in onnxruntime.get_available_providers():
            providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]
            provider_options = [{"device_id": gpu_id}, {}]
        else:
            providers = ["CPUExecutionProvider"]
            provider_options = None

        # 尝试用 GPU（或 CPU-only）创建 Session，遇到显存不足时回落到纯 CPU
        try:
            if provider_options:
                self.session = onnxruntime.InferenceSession(
                    model_path,
                    providers=providers,
                    provider_options=provider_options
                )
                self._using_gpu = True
                logger.debug(f"ONNX Runtime 使用 GPU:{gpu_id} 执行")
            else:
                self.session = onnxruntime.InferenceSession(
                    model_path,
                    providers=providers
                )
                self._using_gpu = False
                logger.debug("ONNX Runtime 使用 CPU 执行")
        except RuntimeError as e:
            if "out of memory" in str(e).lower():
                logger.warning(f"GPU:{gpu_id} 显存不足，回退到 CPUExecutionProvider")
                self.session = onnxruntime.InferenceSession(
                    model_path,
                    providers=["CPUExecutionProvider"]
                )
                self._using_gpu = False
            else:
                # 不是显存问题，继续向上抛
                raise

        self.input_names = [inp.name for inp in self.session.get_inputs()]
        self.output_names = [out.name for out in self.session.get_outputs()]

        logger.debug(
            f"ONNXModelOpenPilot initialized from '{model_path}', "
            f"inputs: {self.input_names}, "
            f"outputs: {self.output_names} "
        )

        # 隐状态，shape=(2, 1, 512)
        self.recurrent_state = np.zeros((2, 1, 512), dtype=np.float32)
        self.cached_input: np.ndarray | None = None

    def infer(self, input_data: np.ndarray) -> list[np.ndarray]:
        """执行时序推理并更新循环状态。

        首次调用时，上一帧用全 0 张量占位。

        Args:
            input_data: 形状 (batch, seq_len, feature_dim) 的 numpy 数组。

        Returns:
            list[np.ndarray]:
                - prob_logits: 轨迹概率。
                - trajectory: 轨迹位置。
        """
        if self.cached_input is None:
            self.cached_input = input_data

        combined = np.concatenate((self.cached_input, input_data), axis=1)
        self.cached_input = input_data

        feed = {
            self.input_names[0]: combined,
            self.input_names[1]: self.recurrent_state,
        }
        outputs = self.session.run(self.output_names, feed)
        prob_logits, trajectory, self.recurrent_state = outputs

        return [prob_logits, trajectory]
