# train_sparse4d_lightning.py
# -*- coding: utf-8 -*-

import datetime
import math
import os
from pathlib import Path
from typing import Any, Optional
import cv2

import numpy as np
import torch
import lightning as L
from lightning.pytorch.loggers import TensorBoardLogger
from lightning.pytorch.callbacks import ModelCheckpoint, RichProgressBar
from torch.utils.data import DataLoader

from datasets.nuscenes_dataset import NuScenesSceneDataset
from models.model_sparse4d import Sparse4DMultiTaskNet
from visper.common.cimg import ImageProcessor
from visper.common.logger import logger, INFO
from visper.common.vis import boxes_to_corners3d, draw_3d_boxes, as_numpy
from visper.postproc.postproc_sparse_impl import Sparse4DPostProcessor

logger.init_logger(level=INFO)

# =========================
# 常量（不使用 argparse）
# =========================
CUR_DIR = Path(__file__).resolve().parent
WORK_DIR = CUR_DIR
ROOT_DIR = CUR_DIR.parent

MAX_EPOCHS = 100
PRECISION = "16-mixed"
GRAD_CLIP_VAL = 1.0

BATCH_SIZE = 16
PREFETCH_SIZE = 8

LR = 2e-5
WEIGHT_DECAY = 1e-2
WARMUP_EPOCHS = 5

CKPT_EVERY_EPOCHS = 2
VIS_EVERY_STEPS = 500  # 每多少个 step 可视化一次

UPDATE_EVERY_FRAMES = 8
TEACHER_RATIO = 0.0

EXP_NAME = f"sparse4d_{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}"
LOG_DIR = WORK_DIR / "logs" / EXP_NAME

NUM_DET_QUERIES = 100
NUM_DET_CLS = 5


class Sparse4DDetLightning(L.LightningModule):
    def __init__(self):
        super().__init__()
        self.net = Sparse4DMultiTaskNet(enable_lane=False,
                                        num_det_queries=NUM_DET_QUERIES,
                                        num_det_cls=NUM_DET_CLS,
                                        enable_trj=False,
                                        enable_det=True)

        self.num_det_cls = int(self.net.num_det_cls)
        self.update_every = int(UPDATE_EVERY_FRAMES)

        self.lr = float(LR)
        self.weight_decay = float(WEIGHT_DECAY)
        self.teacher_ratio = float(TEACHER_RATIO)

        self.automatic_optimization = False

    def configure_optimizers(self):
        opt = torch.optim.AdamW(self.parameters(), lr=self.lr, weight_decay=self.weight_decay)

        def lr_lambda(epoch: int) -> float:
            warm = max(int(WARMUP_EPOCHS), 1)
            total = max(int(MAX_EPOCHS), warm + 1)

            if epoch < warm:
                return float(epoch + 1) / float(warm)

            prog = float(epoch - warm) / float(max(1, total - warm))
            prog = float(min(max(prog, 0.0), 1.0))

            min_ratio = 0.01
            return min_ratio + (1.0 - min_ratio) * 0.5 * (1.0 + math.cos(math.pi * prog))

        sch = torch.optim.lr_scheduler.LambdaLR(opt, lr_lambda=lr_lambda)

        # interval 写 epoch（但注意：你是 manual optimization，Lightning 不会自动 step scheduler）
        return {"optimizer": opt, "lr_scheduler": {"scheduler": sch, "interval": "epoch"}}

    @staticmethod
    @torch.no_grad()
    def build_targets_from_nuscenes(frame: dict[str, Any]) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        images = frame["big_img_data"]
        cls = frame["det3d_cls"]
        box7 = frame["det3d_box"]

        if box7.size(-1) != 7:
            raise ValueError(f"det3d_box last dim must be 7, got {tuple(box7.shape)}")

        ctr_size = box7[..., 0:6]
        theta = box7[..., 6]
        cs = torch.stack([torch.cos(theta), torch.sin(theta)], dim=-1)
        box8 = torch.cat([ctr_size, cs], dim=-1)

        return images, box8, cls

    def on_train_epoch_end(self):
        sch = self.lr_schedulers()
        if sch is None:
            return
        # manual optimization 下需要手动 step scheduler :contentReference[oaicite:0]{index=0}
        if isinstance(sch, list):
            for s in sch:
                s.step()
        else:
            sch.step()

    def training_step(self, batch: Any, batch_idx: int):
        if not isinstance(batch, list):
            raise TypeError(f"collate_fn must return list[dict], got {type(batch)}")
        frames: list[dict] = batch

        opt = self.optimizers()
        opt.zero_grad(set_to_none=True)

        prev_q_det = None
        prev_box_det = None

        loss_keys = ["det_center", "det_size", "det_yaw", "det_cls", "det_grid"]
        window_sum: dict[str, Optional[torch.Tensor]] = {k: None for k in loss_keys}
        window_total_sum: Optional[torch.Tensor] = None
        window_count = 0

        update_loss_last = None

        for t, frame in enumerate(frames):
            images, tgt_box8, tgt_cls = self.build_targets_from_nuscenes(
                frame=frame,
            )

            # ===== 新增：初始化 prev（不能为 None）=====
            if prev_q_det is None:
                B = int(images.size(0))
                Q = int(self.net.num_det_queries)
                C = int(self.net.embed_dim)
                dev = images.device
                dt = images.dtype

                prev_q_det = torch.zeros(B, Q, C, device=dev, dtype=dt)
                prev_box_det = torch.zeros(B, Q, 8, device=dev, dtype=dt)

            # ===== 新增：每帧传 is_first（首帧=1，其余=0）=====
            B = int(images.size(0))
            is_first = torch.ones(B, device=images.device, dtype=images.dtype) if t == 0 else \
                torch.zeros(B, device=images.device, dtype=images.dtype)

            # detach 仍然保留（你现在是按帧截断 BPTT）
            prev_q_det = prev_q_det.detach()
            prev_box_det = prev_box_det.detach()

            det_box, det_cls, det_q, det_loss = self.net(
                images,
                prev_q_det=prev_q_det,
                prev_box_det=prev_box_det,
                is_first=is_first,  # ✅ 新增
                target_det_box=tgt_box8,
                target_det_cls=tgt_cls,
                teacher_ratio=self.teacher_ratio,
            )

            prev_q_det = det_q
            prev_box_det = det_box

            loss_frame = (
                    det_loss["det_center"]
                    + det_loss["det_size"]
                    + det_loss["det_yaw"]
                    + det_loss["det_cls"]
                    + det_loss["det_grid"]
            )

            for k in loss_keys:
                window_sum[k] = det_loss[k] if window_sum[k] is None else (window_sum[k] + det_loss[k])
            window_total_sum = loss_frame if window_total_sum is None else (window_total_sum + loss_frame)
            window_count += 1

            loss_scaled = loss_frame / float(self.update_every)
            will_step = ((t + 1) % self.update_every == 0) or ((t + 1) == len(frames))

            if (self.trainer.world_size > 1) and (not will_step):
                with self.trainer.strategy.model.no_sync():
                    self.manual_backward(loss_scaled)
            else:
                self.manual_backward(loss_scaled)

            if will_step:
                denom = float(max(window_count, 1))
                update_total = window_total_sum / denom
                update_parts = {k: (window_sum[k] / denom) for k in loss_keys}

                lr_now = float(opt.param_groups[0]["lr"])

                self.clip_gradients(
                    opt,
                    gradient_clip_val=GRAD_CLIP_VAL,
                    gradient_clip_algorithm="norm",
                )
                opt.step()
                opt.zero_grad(set_to_none=True)

                bs = int(images.size(0))  # 当前 frame 的 batch size

                self.log_dict(
                    {
                        "loss/total": update_total,
                        "loss/center": update_parts["det_center"],
                        "loss/size": update_parts["det_size"],
                        "loss/yaw": update_parts["det_yaw"],
                        "loss/cls": update_parts["det_cls"],
                        "loss/grid": update_parts["det_grid"],
                        "lr/lr": lr_now,
                    },
                    on_step=True,
                    on_epoch=True,
                    prog_bar=True,
                    batch_size=bs,  # ✅ 关键：跳过 batch_size 推断
                )

                update_loss_last = update_total  # ✅ 让 return 有意义

                window_sum = {k: None for k in loss_keys}
                window_total_sum = None
                window_count = 0

        return update_loss_last if update_loss_last is not None else torch.tensor(0.0, device=self.device)


class VisualizeCallback(L.Callback):
    def __init__(
        self,
        every_n_steps: int,
        vis_dir: Path | str,
        *,
        num_det_classes: int,
        score_thr: float = 0.3,
        catch_up: bool = False,  # True: 跨过多个整点会补画；False: 只画最新那个整点
    ) -> None:
        super().__init__()
        self.every_n_steps = int(every_n_steps)
        self.vis_dir = Path(vis_dir)
        os.makedirs(self.vis_dir, exist_ok=True)

        self.catch_up = bool(catch_up)
        self._last_bucket = 0  # 记录上一次已经可视化过的 bucket

        self.post = Sparse4DPostProcessor(
            num_classes=int(num_det_classes),
            score_thr=float(score_thr),
            min_size=1e-3,
            eps=1e-6,
        )

    @torch.no_grad()
    def on_train_batch_end(
        self,
        trainer: L.Trainer,
        pl_module: L.LightningModule,
        outputs,
        batch,
        batch_idx: int,
    ) -> None:
        # DDP：只让 rank0 做可视化
        if hasattr(trainer, "is_global_zero") and (not trainer.is_global_zero):
            return

        cur_step = int(trainer.global_step)
        if cur_step <= 0 or self.every_n_steps <= 0:
            return

        # bucket：每 every_n_steps 一个区间
        cur_bucket = cur_step // self.every_n_steps
        if cur_bucket <= 0:
            return

        # 没跨区间 => 本 batch 内没有“跨过新的整点”
        if cur_bucket == self._last_bucket:
            return

        # 决定这次要画哪些整点
        if self.catch_up:
            buckets = list(range(self._last_bucket + 1, cur_bucket + 1))
        else:
            buckets = [cur_bucket]

        self._last_bucket = cur_bucket

        # 你的 collate 是 list[dict]（序列帧）；也兼容单帧 dict
        frames = batch if isinstance(batch, list) else [batch]
        T = len(frames)
        if T == 0:
            return

        net = getattr(pl_module, "net", pl_module)
        was_training = net.training
        net.eval()

        try:
            # ---- 初始化 prev（按序列滚动）----
            first_frame = frames[0]
            imgs0 = first_frame["big_img_data"]
            B = int(imgs0.shape[0])

            Q = int(getattr(net, "num_det_queries"))
            C = int(getattr(net, "embed_dim"))
            device = pl_module.device

            prev_q_det = torch.zeros((B, Q, C), device=device, dtype=torch.float32)
            prev_box_det = torch.zeros((B, Q, 8), device=device, dtype=torch.float32)

            # 固定只画 b=0
            b = 0

            for bucket in buckets:
                step_tag = bucket * self.every_n_steps  # 用整点命名，避免 504 这种“非整点文件名”

                # 每次触发都用同一个 batch 的序列做可视化
                prev_q_det.zero_()
                prev_box_det.zero_()

                for t, frame in enumerate(frames):
                    imgs = frame["big_img_data"].to(
                        device=device, dtype=torch.float32, non_blocking=True
                    )

                    is_first = (
                        torch.ones((B,), device=device, dtype=torch.float32)
                        if t == 0
                        else torch.zeros((B,), device=device, dtype=torch.float32)
                    )

                    det_box, det_cls, det_q = net(
                        imgs,
                        prev_q_det=prev_q_det,
                        prev_box_det=prev_box_det,
                        is_first=is_first,
                    )

                    # roll prev
                    prev_box_det = det_box.detach()
                    prev_q_det = det_q.detach()

                    # ---- 取原图/相机 ----
                    cimg = frame["big_cimgs"][b]
                    scene_name = frame["scene_name"][b] if "scene_name" in frame else "scene"
                    canvas = cimg.image.copy()  # BGR uint8
                    H, W = canvas.shape[:2]

                    # =======================
                    # Pred
                    # =======================
                    pred_img = canvas.copy()

                    boxes7, scores, labels = self.post.postprocess(
                        det_box=as_numpy(det_box[b]),
                        det_cls=as_numpy(det_cls[b]),
                    )

                    if boxes7.shape[0] > 0:
                        p_corners3d = boxes_to_corners3d(
                            boxes7.astype(np.float32, copy=False)
                        )  # (N,8,3)
                        p_uv = ImageProcessor.world_points_to_uv(cimg, p_corners3d)  # (N,8,2)

                        x1 = np.clip(np.min(p_uv[..., 0], axis=1), 0, W - 1)
                        y1 = np.clip(np.min(p_uv[..., 1], axis=1), 0, H - 1)
                        x2 = np.clip(np.max(p_uv[..., 0], axis=1), 0, W - 1)
                        y2 = np.clip(np.max(p_uv[..., 1], axis=1), 0, H - 1)
                        p_2d = np.stack([x1, y1, x2, y2], axis=1).astype(np.float32)

                        p_objs6 = np.concatenate(
                            [
                                p_2d,
                                scores.reshape(-1, 1).astype(np.float32),
                                labels.reshape(-1, 1).astype(np.float32),
                            ],
                            axis=1,
                        )  # (N,6) [x1,y1,x2,y2,score,cls]
                        pred_img = draw_3d_boxes(pred_img, p_uv, p_objs6)

                    cv2.imwrite(
                        str(self.vis_dir / f"{scene_name}_step_{step_tag:06d}_t{t:03d}_pred.jpg"),
                        pred_img,
                    )

                    # =======================
                    # GT
                    # =======================
                    gt_img = canvas.copy()

                    gt_cls = frame["det3d_cls"][b].detach().cpu().numpy()  # (K,) padding=-1
                    gt_box7 = frame["det3d_box"][b].detach().cpu().numpy()  # (K,7)

                    valid = (gt_cls >= 0) & (np.sum(np.abs(gt_box7), axis=1) > 0.0)
                    if np.any(valid):
                        g7 = gt_box7[valid].astype(np.float32, copy=False)
                        gc = gt_cls[valid].astype(np.float32, copy=False)

                        g_corners3d = boxes_to_corners3d(g7)
                        g_uv = ImageProcessor.world_points_to_uv(cimg, g_corners3d)

                        x1 = np.clip(np.min(g_uv[..., 0], axis=1), 0, W - 1)
                        y1 = np.clip(np.min(g_uv[..., 1], axis=1), 0, H - 1)
                        x2 = np.clip(np.max(g_uv[..., 0], axis=1), 0, W - 1)
                        y2 = np.clip(np.max(g_uv[..., 1], axis=1), 0, H - 1)
                        g_2d = np.stack([x1, y1, x2, y2], axis=1).astype(np.float32)

                        g_objs6 = np.concatenate(
                            [g_2d, np.ones((g_2d.shape[0], 1), np.float32), gc.reshape(-1, 1)],
                            axis=1,
                        )
                        gt_img = draw_3d_boxes(gt_img, g_uv, g_objs6)

                    cv2.imwrite(
                        str(self.vis_dir / f"{scene_name}_step_{step_tag:06d}_t{t:03d}_gt.jpg"),
                        gt_img,
                    )
        finally:
            if was_training:
                net.train()


# =========================
# build_model：负责初始化标定信息
# =========================
def cimg_to_KRt_for_model(cimg):
    # 1. K（像素）
    K = torch.tensor([
        [cimg.focal_u, 0.0, cimg.center_u],
        [0.0, cimg.focal_v, cimg.center_v],
        [0.0, 0.0, 1.0]
    ])

    # 2. 用你已有的主动外参（世界→相机_原生坐标）
    M_act = ImageProcessor._Rt_active(cimg)  # (3,4) numpy, 其中 R_act=[:,:3], t_act=[:,3:]
    R_act = M_act[:, :3]
    t_act = M_act[:, 3:].reshape(3, 1)

    # 3. 轴系对齐到 OpenCV（x右、y下、z前）
    Q = np.array([
        [0.0, -1.0, 0.0],
        [0.0, 0.0, -1.0],
        [1.0, 0.0, 0.0]
    ], dtype=np.float64)

    R_cv = Q @ R_act
    t_cv = Q @ t_act

    # 4. 转 torch
    R = torch.tensor(R_cv)
    t = torch.tensor(t_cv.reshape(3))  # (3,)

    return K, R, t


def build_model(dataset: NuScenesSceneDataset) -> "Sparse4DDetLightning":
    pl_model = Sparse4DDetLightning()

    sample = next((s for i in range(BATCH_SIZE) if (s := dataset[i]) is not None), None)
    H, W = int(sample.big_cimg[0].image_height), int(sample.big_cimg[0].image_width)
    K, R, t = cimg_to_KRt_for_model(cimg=sample.big_cimg[0])

    # ✅ 在这里一次性 set_calibration（LightningModule 内不再涉及标定逻辑）
    pl_model.net.set_calibration(H=H, W=W, K=K, R=R, t=t)

    p = Path('/opt_disk3/rd234421/Projects-SGS/R-AEB/assets/weights/yolo11x-seg-dict.pth')
    if p.exists():
        sd = torch.load(str(p), map_location="cpu")
        missing, unexpected = pl_model.net.backbone.load_state_dict(sd, strict=False)
        logger.info(f"加载预训练：missing={missing}\nunexpected={unexpected}")
    else:
        logger.warning(f"未找到预训练权重：{p}")

    return pl_model


def main():
    os.makedirs(LOG_DIR / "ckpt", exist_ok=True)

    dataset = NuScenesSceneDataset(split="trainval", prefetch_size=PREFETCH_SIZE)

    train_loader = DataLoader(
        dataset=dataset,
        batch_size=BATCH_SIZE,
        shuffle=False,
        num_workers=0,
        prefetch_factor=None,
        collate_fn=dataset.collate_fn,  # 你指定的 collate_fn
    )

    # ✅ 按你要求：用 build_model 初始化（包含标定）
    pl_model = build_model(dataset)

    tb_logger = TensorBoardLogger(save_dir=str(LOG_DIR.parent), name=LOG_DIR.name)

    ckpt_best = ModelCheckpoint(
        dirpath=str(LOG_DIR / "ckpt"),
        filename="best-{epoch:03d}",
        monitor="loss/total_epoch",
        mode="min",
        save_top_k=1,
        save_last=True,
        auto_insert_metric_name=False,
    )

    progress = RichProgressBar()

    vis_cb = VisualizeCallback(
        every_n_steps=VIS_EVERY_STEPS,
        vis_dir=str(LOG_DIR / "vis"),
        num_det_classes=NUM_DET_CLS,
        score_thr=0.35,
    )

    trainer = L.Trainer(
        logger=tb_logger,
        callbacks=[ckpt_best, progress, vis_cb],
        max_epochs=MAX_EPOCHS,
        precision=PRECISION,
        deterministic=False,
        benchmark=True,
        log_every_n_steps=5,
        accelerator="gpu" if torch.cuda.is_available() else "cpu",
        devices=[0] if torch.cuda.is_available() else None,
        accumulate_grad_batches=1,  # 你已经按帧手动累积了
    )

    try:
        trainer.fit(
            model=pl_model,
            train_dataloaders=train_loader,
        )
    finally:
        # 关掉数据集内部的进程池
        dataset.shutdown()
        logger.info("Training completed.")


if __name__ == "__main__":
    main()
