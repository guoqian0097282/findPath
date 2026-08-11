# -*- coding: utf-8 -*-
import datetime
import math
import os
from pathlib import Path
from typing import Tuple, Dict, Any

import cv2
import numpy as np
import torch
import lightning as L
from lightning.pytorch.loggers import TensorBoardLogger
from lightning.pytorch.callbacks import (ModelCheckpoint, LearningRateMonitor, RichProgressBar)
from torch import nn, Tensor
from torch.utils.data import DataLoader

from datasets.nuscenes_dataset import NuScenesSceneDataset
from models.model_angleseg import ConditionalSegAngleLossV8, YOLO11AngleSeg

from visper.common.logger import logger, INFO, DEBUG
from visper.common.ops import xyxy2xywh
from visper.postproc.postproc_angleseg_impl import AngleSegPostProcessor

# ========== 常量（不使用 argparse） ==========
CUR_DIR = Path(__file__).resolve().parent
WORK_DIR = CUR_DIR
ROOT_DIR = CUR_DIR.parent

MAX_EPOCHS = 400
PRECISION = "16-mixed"  # 显卡不支持就改成 "32-true"
GRAD_CLIP_VAL = 1.0

BATCH_SIZE = 24,

LR = 2e-4,
WEIGHT_DECAY = 0.05
WARMUP_STEPS = 1000
MAX_STEPS = 200000

CKPT_EVERY_EPOCHS = 2
VIS_EVERY_STEPS = 50  # 每多少个 step 可视化一次

EXP_NAME = f"angle_{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}"
LOG_DIR = WORK_DIR / "logs" / EXP_NAME

# 预训练权重：只有在“没恢复 ckpt”的时候才会用
# WEIGHTS_PATH = None
# RESUME_CKPT = Path("/opt_disk2/rd23442/Projects-SGS/R-AEB/python/train/logs/angle_2025-12-05_10-17-03/ckpt/last.ckpt")

WEIGHTS_PATH = Path('/opt_disk2/rd23442/Projects-SGS/R-AEB/python/train/logs/angle_2025-12-05_10-17-03/weight_dict/best.pt')
RESUME_CKPT = None

# ========== cuDNN / matmul 等设置 ==========
torch.backends.cudnn.enabled = True
torch.backends.cudnn.benchmark = True
torch.backends.cudnn.deterministic = False
torch.autograd.set_detect_anomaly(False)
torch.backends.cuda.matmul.allow_tf32 = True
torch.set_float32_matmul_precision("high")

# ========= IoU 最大匹配：把 NMS 后的框映射回 L 级别锚索引 =========
COCO_CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
    "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle",
    "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant", "bed",
    "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven",
    "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
]
class_id2name = {i: n for i, n in enumerate(COCO_CLASSES)}


class YOLO11AngleSegLightning(L.LightningModule):
    """
    训练态 forward(images) -> feats_list, mask_coef, proto, angle_logits_list, angle_res_list
    batch 需包含：images, cls, bboxes(xywh_norm), masks, angle_bin, angle_residual
    """

    def __init__(
            self,
            scale: str = "s",
            in_ch: int = 3,
    ) -> None:
        super().__init__()
        self.save_hyperparameters(ignore=["net"])

        self.net = YOLO11AngleSeg(
            scale=scale,
            in_ch=in_ch,
        )

        # 显式固定损失参数
        self.criterion = ConditionalSegAngleLossV8(
            stride=[8, 16, 32],
            nc=80,
            reg_max=16,
            box_gain=7.5,
            cls_gain=0.5,
            dfl_gain=1.5,
            overlap_mask=False,
            angle_bins=8,
            angle_ce_weight=1.0,
            angle_reg_weight=1.0,
            angle_use_huber=True,
            angle_huber_beta=0.1,
            tal_topk=10,
            tal_alpha=0.5,
            tal_beta=6.0,
        )

        self.lr = LR
        self.weight_decay = WEIGHT_DECAY
        self.warmup_steps = WARMUP_STEPS
        self.max_steps = MAX_STEPS

    # ========= Lightning 常规接口 =========
    def forward(self, images: Tensor):
        return self.net(images)

    def configure_optimizers(self):
        optim = torch.optim.AdamW(self.parameters(), lr=self.lr, weight_decay=self.weight_decay)

        def lr_lambda(step: int) -> float:
            if step < self.warmup_steps:
                return (step + 1) / max(1, self.warmup_steps)
            progress = (step - self.warmup_steps) / max(1, self.max_steps - self.warmup_steps)
            progress = float(min(max(progress, 0.0), 1.0))
            min_ratio = 0.01
            return min_ratio + (1.0 - min_ratio) * 0.5 * (1.0 + math.cos(math.pi * progress))

        sched = torch.optim.lr_scheduler.LambdaLR(optim, lr_lambda=lr_lambda)
        return {"optimizer": optim, "lr_scheduler": {"scheduler": sched, "interval": "step"}}

    # ========= 训练 / 验证 =========
    def training_step(self, batch: Dict[str, Any]) -> Tensor:

        images, gt_cls, gt_bboxes_xywh_norm, gt_angle_rel = self.build_targets_from_nuscenes(batch)

        feats_list, mask_coef, proto, angle_logits_list, angle_res_list = self(images)

        total, parts = self.criterion(
            feats_list=feats_list,
            mask_coef=mask_coef,
            proto=proto,
            angle_logits_list=angle_logits_list,
            angle_res_list=angle_res_list,
            gt_cls=gt_cls,
            gt_bboxes_xywh_norm=gt_bboxes_xywh_norm,
            gt_masks=None,
            gt_angle=gt_angle_rel,
        )

        self.log_dict(
            {
                "train/loss": total,
                "train/box": parts["box"],
                "train/seg": parts["seg"],
                "train/cls": parts["cls"],
                "train/dfl": parts["dfl"],
                "train/angle_ce": parts["angle_ce"],
                "train/angle_reg": parts["angle_reg"],
            },
            on_step=True, on_epoch=True, prog_bar=True,
        )
        return total

    @staticmethod
    @torch.no_grad()
    def build_targets_from_nuscenes(batch):

        images = batch["big_img_data"]  # (B,C,H,W)
        cls2d = batch["det3d_cls"]  # (B,K)
        box2d = batch["det2d_box"]  # (B,K,4)
        box3d = batch["det3d_box"]  # (B,K,7), theta在 [:,:,6]

        device = images.device
        dtype_boxes = box2d.dtype

        # ===== 0) 类别映射：DET6 → COCO80（仅对 >=0 的有效标签做查表）=====
        #  0:Background→-1, 1:Car→2, 2:Truck→7, 3:Bus→5, 4:Pedestrian→0, 5:Cyclist→1
        lut = torch.tensor([-1, 2, 7, 5, 0, 1], device=device, dtype=torch.long)
        cls2d_in = cls2d.to(device).long()
        valid_cls = cls2d_in >= 0
        gt_cls = torch.full_like(cls2d_in, fill_value=-1)
        if valid_cls.any():
            gt_cls[valid_cls] = lut[cls2d_in[valid_cls]]

        # ===== 1) 2D 框归一化 (xyxy -> xywh_norm) =====
        xywh_pix = xyxy2xywh(box2d)  # (B,K,4)
        B, _, H, W = images.shape
        wh_norm = torch.tensor([W, H, W, H], device=device, dtype=dtype_boxes)
        gt_bboxes_xywh_norm = xywh_pix / wh_norm  # (B,K,4)

        # ===== 2) 相对角（以 ego 原点为参考射线）=====
        cx, cy = box3d[..., 0], box3d[..., 1]  # (B,K)
        theta_abs = box3d[..., 6]  # (B,K)
        bearing = torch.atan2(cy, cx)  # (B,K)
        gt_angle_rel = (theta_abs - bearing + math.pi) % (2.0 * math.pi) - math.pi  # (-π, π]

        # ===== 5) 返回 =====
        return images, gt_cls, gt_bboxes_xywh_norm, gt_angle_rel


class VisualizeCallback(L.Callback):
    def __init__(
            self,
            every_n_steps: int,
            vis_dir: Path,
            class_id2name: dict[int, str],
            conf_thr: float = 0.40,
            iou_thr: float = 0.30,
            mask_thr: float = 0.50,
    ) -> None:
        super().__init__()
        self.every_n_steps = int(every_n_steps)
        self.vis_dir = Path(vis_dir)
        os.makedirs(self.vis_dir, exist_ok=True)

        # SegPostProcessor 需要 name->id
        self.post = AngleSegPostProcessor(
            id2name=class_id2name,
            conf_thresh=conf_thr,
            iou_thresh=iou_thr,
            mask_thresh=mask_thr,
            mask_up=4,  # 上采样4倍
        )

    @torch.no_grad()
    def on_train_batch_end(
            self,
            trainer: L.Trainer,
            pl_module: L.LightningModule,
            outputs,
            batch: dict,
            batch_idx: int,
    ) -> None:

        if hasattr(trainer, "is_global_zero") and (not trainer.is_global_zero):
            return
        step = int(trainer.global_step)
        if step == 0 or (step % self.every_n_steps) != 0:
            return

        was_training = pl_module.training
        pl_module.eval()

        imgs = batch["big_img_data"].to(pl_module.device, non_blocking=True)  # (B,3,Hn,Wn)
        det_cat, proto, angle_cat = pl_module(imgs)

        b = 0  # 取 b=0
        canvas = batch["big_cimgs"][b].image  # 原图（BGR）

        # 单帧 numpy
        det_np = det_cat[b].detach().cpu().numpy()  # (C, L)
        proto_np = proto[b].detach().cpu().numpy()  # (nm, Hm, Wm)
        theta_np = angle_cat[b].detach().cpu().numpy()  # (L,)

        # 后处理
        # objs_m: (N,7) [x1,y1,x2,y2,conf,cls,theta]
        # masks_m: (N,H,W) uint8(0/1)  (原图尺寸)
        objs_m, masks_m = self.post.postprocess(det_np, proto_np, theta_np)

        out_img = self.post.draw_angleins(
            canvas, objs_m, masks_m,
        )

        cv2.imwrite(str(self.vis_dir / f"step_{step:06d}_pred.jpg"), out_img)

        if was_training:
            pl_module.train()

        # ----------- GT 可视化 -----------
        gt_img = canvas.copy()  # 默认：啥都不画也要保存

        gt_cls_2d: torch.Tensor = batch["det3d_cls"][b]  # (K,)
        gt_box3d: torch.Tensor = batch["det3d_box"][b]  # (K,7)
        gt_box_xyxy: torch.Tensor = batch["det2d_box"][b]  # (K,4) 大图尺度 xyxy

        valid = gt_cls_2d >= 0
        if bool(valid.any()):
            gt_cls_2d_v = gt_cls_2d[valid].long()
            gt_box_xyxy_v = gt_box_xyxy[valid].to(torch.float32)
            gt_box3d_v = gt_box3d[valid].to(torch.float32)

            # 映射要和训练一致
            cls_lut = torch.tensor([-1, 2, 7, 5, 0, 1], dtype=torch.long, device=gt_cls_2d_v.device)
            coco_cls_i64 = cls_lut[gt_cls_2d_v]  # (N,)

            # 过滤掉 background(-1)
            valid2 = coco_cls_i64 >= 0
            if bool(valid2.any()):
                gt_box_xyxy_v = gt_box_xyxy_v[valid2]
                gt_box3d_v = gt_box3d_v[valid2]
                coco_cls = coco_cls_i64[valid2].to(torch.float32)

                # theta_rel = theta_abs - bearing
                cx = gt_box3d_v[:, 0]
                cy = gt_box3d_v[:, 1]
                theta_abs = gt_box3d_v[:, 6]
                bearing = torch.atan2(cy, cx)
                theta_rel = (theta_abs - bearing + math.pi) % (2.0 * math.pi) - math.pi

                N = gt_box_xyxy_v.shape[0]
                gt_objs7 = torch.zeros((N, 7), dtype=torch.float32, device=gt_box_xyxy_v.device)
                gt_objs7[:, 0:4] = gt_box_xyxy_v
                gt_objs7[:, 4] = 1.0
                gt_objs7[:, 5] = coco_cls
                gt_objs7[:, 6] = theta_rel

                gt_objs7_np = gt_objs7.detach().cpu().numpy().astype(np.float32)
                H0, W0 = gt_img.shape[:2]
                gt_masks0 = np.zeros((N, H0, W0), dtype=np.uint8)

                gt_img = self.post.draw_angleins(
                    gt_img,
                    gt_objs7_np,
                    gt_masks0,
                    draw_mask=False,
                )
        # 无论有没有 GT，都保存
        cv2.imwrite(str(self.vis_dir / f"step_{step:06d}_gt.jpg"), gt_img)


# ========= 导出 state_dict 的回调 =========
class ExportWeightsCallback(L.Callback):
    def __init__(
            self,
            out_dir: Path,
            monitor: str | None = "train/loss_epoch",  # 监控指标；None 表示不存 best.pt
            mode: str = "min",  # "min" 或 "max"
            every_n_epochs: int | None = 1,  # 按频率导出 epoch-xxx.pt；None 表示不按频率
    ) -> None:
        super().__init__()
        self.out_dir = out_dir
        self.monitor = monitor
        self.mode = mode
        self.every_n_epochs = every_n_epochs
        self.best: float | None = None

    def _is_main_process(self, trainer: L.Trainer) -> bool:
        # 只在 rank0 落盘，避免 DDP 重复写
        return getattr(trainer, "is_global_zero", True)

    def _improved(self, current: float) -> bool:
        if self.best is None:
            return True
        return current < self.best if self.mode == "min" else current > self.best

    def _save(self, trainer: L.Trainer, pl_module: L.LightningModule, filename: str) -> None:
        if not self._is_main_process(trainer):
            return
        self.out_dir.mkdir(parents=True, exist_ok=True)
        path = self.out_dir / filename
        sd = pl_module.net.state_dict()  # 始终导出整网（YOLO11AngleSeg）的权重字典
        torch.save(sd, str(path))

    def on_fit_start(self, trainer: L.Trainer, pl_module: L.LightningModule) -> None:
        if self.monitor is not None:
            val = trainer.callback_metrics.get(self.monitor)
            if val is not None:
                self.best = float(val.detach().cpu()) if torch.is_tensor(val) else float(val)

    def on_train_epoch_end(self, trainer: L.Trainer, pl_module: L.LightningModule) -> None:
        # 1) 按频率导出：epoch-XXX.pt
        if self.every_n_epochs is not None:
            ep = trainer.current_epoch
            if (ep + 1) % self.every_n_epochs == 0:
                self._save(trainer, pl_module, f"epoch-{ep + 1:03d}.pt")

        # 2) 监控 best：best.pt（可选）
        if self.monitor is not None:
            metric = trainer.callback_metrics.get(self.monitor)
            if metric is None:
                return
            m = float(metric.detach().cpu()) if torch.is_tensor(metric) else float(metric)
            if self._improved(m):
                self.best = m
                self._save(trainer, pl_module, "best.pt")


# ========= 模型构建 =========
def build_model(from_ckpt: bool) -> YOLO11AngleSegLightning:
    """
    from_ckpt=True 时，只构建空模型，参数恢复交给 trainer.fit(...)
    from_ckpt=False 时，加载预训练权重
    """
    pl_model = YOLO11AngleSegLightning(scale="n", in_ch=3)

    if not from_ckpt:
        if WEIGHTS_PATH:
            p = Path(WEIGHTS_PATH)
            if p.exists():
                sd = torch.load(str(p), map_location="cpu")
                missing, unexpected = pl_model.net.load_state_dict(sd, strict=False)
                logger.info(f"加载预训练：missing={missing}\nunexpected={unexpected}")
            else:
                logger.warning(f"未找到预训练权重：{p}")
    else:
        logger.info("将从 ckpt 恢复，初始化时不加载预训练权重。")

    # 冻住所有参数
    for p in pl_model.net.parameters():
        p.requires_grad = False

    # 前面的层不再记录 BN 的 running_mean / running_var
    for m in pl_model.net.model[:24].modules():
        if isinstance(m, nn.BatchNorm2d):
            m.track_running_stats = False

    # 只训练第角度头
    for p in pl_model.net.angle.parameters():
        p.requires_grad = True

    return pl_model


# ========= 主入口 =========
def main() -> None:
    os.makedirs(LOG_DIR / "ckpt", exist_ok=True)
    logger.init_logger(save_path=f"{LOG_DIR}/log.txt", detailed=True, level=INFO)

    # 1) 数据集
    dataset = NuScenesSceneDataset(
        split="trainval", prefetch_size=32,
    )

    train_loader = DataLoader(
        dataset=dataset,
        batch_size=4,
        shuffle=True,
        num_workers=0,
        prefetch_factor=None,
        collate_fn=dataset.collate_fn_bt,
    )

    # 2) 构建模型 / 是否从 ckpt 恢复
    resume_flag = bool(RESUME_CKPT and RESUME_CKPT.is_file())
    pl_model = build_model(from_ckpt=resume_flag)

    # 3) Logger & Callbacks
    tb_logger = TensorBoardLogger(
        save_dir=str(LOG_DIR.parent),
        name=LOG_DIR.name
    )

    ckpt_best = ModelCheckpoint(
        dirpath=str(LOG_DIR / "ckpt"),
        filename="best-{epoch:03d}-{train_loss:.4f}",
        monitor="train/loss_epoch",
        mode="min",
        save_top_k=1,
        save_last=True,
        auto_insert_metric_name=False,
    )
    ckpt_every = ModelCheckpoint(
        dirpath=str(LOG_DIR / "ckpt"),
        filename="epoch-{epoch:03d}",
        every_n_epochs=CKPT_EVERY_EPOCHS,
        save_top_k=-1,
        auto_insert_metric_name=False,
    )

    lr_monitor = LearningRateMonitor(
        logging_interval="step"
    )
    progress = RichProgressBar()

    vis_cb = VisualizeCallback(
        every_n_steps=VIS_EVERY_STEPS,
        vis_dir=LOG_DIR / "vis",
        class_id2name=class_id2name
    )

    export_cb = ExportWeightsCallback(
        out_dir=LOG_DIR / "weight_dict",
        monitor="train/loss_epoch",  # 与你的 ckpt_best 的 monitor 一致
        mode="min",
        every_n_epochs=CKPT_EVERY_EPOCHS,  # 保存频率沿用你的常量
    )

    # 4) Trainer
    trainer = L.Trainer(
        logger=tb_logger,
        callbacks=[ckpt_best, ckpt_every, lr_monitor, progress, vis_cb, export_cb],
        max_epochs=MAX_EPOCHS,
        gradient_clip_val=GRAD_CLIP_VAL,
        precision=PRECISION,
        deterministic=False,
        benchmark=True,
        enable_model_summary=True,
        log_every_n_steps=50,
        accelerator="gpu" if torch.cuda.is_available() else "cpu",
        devices=[0] if torch.cuda.is_available() else None,
        reload_dataloaders_every_n_epochs=0,
    )

    # 5) 训练（Lightning 会在恢复时同步优化器/调度器/global_step）
    try:
        trainer.fit(
            model=pl_model,
            train_dataloaders=train_loader,
            ckpt_path=str(RESUME_CKPT) if resume_flag else None,
        )
    finally:
        # 6) 关掉数据集内部的进程池
        dataset.shutdown()
        logger.info("Training completed.")


if __name__ == "__main__":
    main()
