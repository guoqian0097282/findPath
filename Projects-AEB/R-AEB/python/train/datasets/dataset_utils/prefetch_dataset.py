from __future__ import annotations

import queue
import threading
from itertools import count
from concurrent.futures import ProcessPoolExecutor, Future
from typing import Any, Callable, Sequence
from torch.utils.data import Dataset

from visper.common.logger import logger


class SamplePrefetchDataset(Dataset):
    """
    基于多进程 sample 级预取的通用 Dataset 父类。

    设计目标：
        - 将「单个样本的耗时处理逻辑」抽象为一个可并行的函数 db_fn；
        - 通过多进程进程池提前把样本处理好，放进内部的优先级队列中；
        - __getitem__ 不再按 idx 实时计算样本，而是从预取队列中取出已经准备好的样本，
          并在后台持续提交新的任务，保持队列中有一定数量的缓存 sample。

    使用方式（子类需要做的事）：
        1. 准备 self.db: Sequence[Any]，例如文件路径列表、样本索引列表等；
        2. 提供一个 db_fn: Callable，签名类似 db_fn(db[i]) -> sample，
           由该函数完成「从 db 元素生成最终训练样本」的完整处理流程。

    重要说明：
        - 本类更适合「流式消费」的场景，即 DataLoader 顺序访问
          idx = 0, 1, 2, ...，而不是依赖严格的「索引 -> 样本」映射。
        - __getitem__ 返回的样本是「队列里已经完成的结果」，并不保证与传入的 idx
          一一对应，也就是说：
              dataset[i] 返回的 sample 可能并不是 self.db[i] 生成的，
              而是某个之前预取完成的样本。
        - 因此，不要在上层逻辑里依赖
              「给定 idx 一定能取到对应的那条 raw db 记录」
          的语义；更应该把它看成一个「带多进程预取的样本生成器」。
    """

    def __init__(
            self,
            db: Sequence,
            db_fn: Callable,
            prefetch_size: int = 16,
    ) -> None:
        super().__init__()

        self.db: list[Any] = list(db)
        if not self.db:
            raise RuntimeError("db 为空，无法构建数据集")

        self._db_fn = db_fn
        self._prefetch_size = prefetch_size
        self._prefetch_workers = prefetch_size // 2

        # 1) 队列与序号
        self._queue = queue.PriorityQueue(maxsize=prefetch_size)
        self._seq = count()

        # 2) 处理 start_method / mp_context
        import multiprocessing as mp

        try:
            mp_ctx = mp.get_context("fork")
            logger.info("使用 mp start method: fork")
        except Exception:
            try:
                mp_ctx = mp.get_context("spawn")
                logger.info("回退使用 mp start method: spawn")
            except Exception as e:
                mp_ctx = None
                logger.warning(f"无法显式设置 mp start method，使用默认：{e}")

        self._executor = ProcessPoolExecutor(
            max_workers=self._prefetch_workers,
            mp_context=mp_ctx,
        )

        self._lock = threading.Lock()
        self._active_futures: set[Future] = set()

        # 4) 预提交任务
        for i in range(0, min(prefetch_size, len(self.db))):
            self._submit_task(i)

    # ================== 通用的并行 / 预取逻辑 ==================
    def _submit_task(self, idx: int) -> None:
        """提交一个处理 db[idx] 的任务到进程池。"""
        with self._lock:
            fut = self._executor.submit(
                self._db_fn, self.db[idx],
            )
            self._active_futures.add(fut)
        fut.add_done_callback(self._on_task_done)

    def _on_task_done(self, fut: Future) -> None:
        try:
            try:
                res = fut.result()
                prio = 1
            except Exception as e:
                logger.error(f"Producer 出错：{e}")
                res = e
                prio = 0
            try:
                self._queue.put_nowait((prio, next(self._seq), res))
                logger.debug(f"Queue put {self._queue.qsize()}/{self._queue.maxsize}")
            except queue.Full:
                logger.error("队列已满，丢弃一个结果/异常")
        finally:
            with self._lock:
                self._active_futures.discard(fut)

    def __len__(self) -> int:
        return len(self.db)

    def __getitem__(self, idx: int) -> any:
        # 从队列里取已经完成的 sample
        _, _, payload = self._queue.get(block=True)
        logger.debug(f"Queue get {self._queue.qsize()}/{self._queue.maxsize}")

        # 立刻补一个新的任务，保持队列有货
        self._submit_task((idx + self._prefetch_size) % len(self.db))

        if isinstance(payload, Exception):
            return None
        return payload

    def shutdown(self, wait: bool = True) -> None:
        self._executor.shutdown(wait=wait)
