# -*- coding: utf-8 -*-
import datetime
import math
import os
from pathlib import Path
from typing import Dict, Any

import cv2
import lightning as L
import numpy as np
import torch
from lightning.pytorch.callbacks import (ModelCheckpoint, LearningRateMonitor, TQDMProgressBar)
from lightning.pytorch.loggers import TensorBoardLogger
from torch import nn, Tensor
from torch.utils.data import DataLoader

from datasets.huizhou_dataset import HuizhouDataset, ID2NAME, NUM_CLASSES
from models.model_angleseg import Angle, ConditionalSegAngleLossV8, YOLO11AngleSeg
from visper.common.logger import logger, INFO
from visper.common.ops import xyxy2xywh
from visper.common.vis import as_numpy
from visper.postproc.postproc_angleseg_impl import AngleSegPostProcessor

# ========== 常量（不使用 argparse） ==========
CUR_DIR = Path(__file__).resolve().parent
WORK_DIR = CUR_DIR
ROOT_DIR = CUR_DIR.parent

MAX_EPOCHS = 400
PRECISION = "16-mixed"  # 显卡不支持就改成 "32-true"
ACCUMULATE_STEPS = 1
GRAD_CLIP_VAL = 1.0
ENABLE_CKPT_EVERY_N_EPOCHS = 2
VIS_EVERY_STEPS = 50  # 每多少个 step 可视化一次

EXP_NAME = f"angle_{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}"
LOG_DIR = WORK_DIR / "logs" / EXP_NAME

# 预训练权重：只有在“没恢复 ckpt”的时候才会用
WEIGHTS_PATH = Path("/opt_disk3/rd234421/Projects-SGS/R-AEB-lyl/python/train/logs/20260526_102248_hardneg_mutex_clean_ft/hardneg_mutex_ft/train/hardneg_mutex_ft_hardneg_mutex_ft_ep10_angle_huizhou_960x480/ckpt/best-008-0.0000.ckpt")
RESUME_CKPT = Path("/opt_disk3/rd234421/Projects-SGS/R-AEB/python/train/logs/angle_2026-06-22_03-08-01/ckpt/last.ckpt")

# WEIGHTS_PATH = Path('/opt_disk3/rd234421/Projects-SGS/R-AEB/python/train/logs/angle_2025-12-18_09-11-29/weight_dict/best.pt')
# RESUME_CKPT = None

# ========== cuDNN / matmul 等设置 ==========
torch.backends.cudnn.enabled = True
torch.backends.cudnn.benchmark = True
torch.backends.cudnn.deterministic = False
torch.autograd.set_detect_anomaly(False)
torch.backends.cuda.matmul.allow_tf32 = True
torch.set_float32_matmul_precision("high")


class YOLO11AngleSegLightning(L.LightningModule):
    """
    训练态 forward(images) -> feats_list, mask_coef, proto, angle_logits_list, angle_res_list
    batch 需包含：images, cls, bboxes(xywh_norm), masks, angle_bin, angle_residual
    """

    def __init__(
            self,
            scale: str = "s",
            in_ch: int = 3,
            *,
            lr: float = 2e-4,
            weight_decay: float = 0.05,
            warmup_steps: int = 1000,
            max_steps: int = 200000,
    ) -> None:
        super().__init__()
        self.save_hyperparameters(ignore=["net"])

        self.net = YOLO11AngleSeg(
            scale=scale,
            in_ch=in_ch,
            nc=NUM_CLASSES,
        )

        # 显式固定损失参数
        self.criterion = ConditionalSegAngleLossV8(
            stride=[8, 16, 32],
            nc=NUM_CLASSES,
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

        self.lr = lr
        self.weight_decay = weight_decay
        self.warmup_steps = warmup_steps
        self.max_steps = max_steps

    def on_load_checkpoint(self, checkpoint: Dict[str, Any]) -> None:
        state_dict = checkpoint.get("state_dict")
        if isinstance(state_dict, dict):
            state_dict.pop("criterion.cls_pos_weight", None)

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

        images, gt_cls, gt_bboxes_xywh_norm, gt_angle_rel = self.build_targets_from_huizhou(batch)

        feats_list, mask_coef, proto, angle_logits_list, angle_res_list = self(images)

        total, parts = self.criterion(
            feats_list=feats_list,
            mask_coef=mask_coef,
            proto=proto,
            angle_logits_list=angle_logits_list,
            angle_res_list=angle_res_list,
            gt_cls=gt_cls,
            gt_bboxes_xywh_norm=gt_bboxes_xywh_norm,
            gt_masks=batch["masks"],
            gt_angle=gt_angle_rel,
        )
        bsz = images.shape[0]  # 你这里已经有 images
        self.log_dict(
            {
                "loss/total": total.detach(),
                "loss/box": parts["box"].detach(),
                "loss/seg": parts["seg"].detach(),
                "loss/cls": parts["cls"].detach(),
                "loss/dfl": parts["dfl"].detach(),
                "loss/angle_ce": parts["angle_ce"].detach(),
                "loss/angle_reg": parts["angle_reg"].detach(),
            },
            on_step=True,
            on_epoch=True,
            prog_bar=True,
            batch_size=bsz,  # ⭐关键：避免 Lightning 推断 batch_size 扫描整个 batch
        )

        return total


    @staticmethod
    @torch.no_grad()
    def build_targets_from_huizhou(batch):

        images = batch["img_data"]  # (B,C,H,W)
        gt_cls = batch["labels"]  # (B,K)
        box2d = batch["boxes"]  # (B,K,4)
        gt_angle_rel = batch["directions"]
        device = images.device
        dtype_boxes = box2d.dtype

        # ===== 1) 2D 框归一化 (xyxy -> xywh_norm) =====
        xywh_pix = xyxy2xywh(box2d)  # (B,K,4)
        B, _, H, W = images.shape
        wh_norm = torch.tensor([W, H, W, H], device=device, dtype=dtype_boxes)
        gt_bboxes_xywh_norm = xywh_pix / wh_norm  # (B,K,4)

        # ===== 5) 返回 =====
        return images, gt_cls, gt_bboxes_xywh_norm, gt_angle_rel


class VisualizeCallback(L.Callback):
    def __init__(
            self,
            every_n_steps: int,
            vis_dir: Path,
            id2name: dict[int, str],
            conf_thr: float = 0.40,
            iou_thr: float = 0.30,
            mask_thr: float = 0.50,
    ) -> None:
        super().__init__()
        self.every_n_steps = int(every_n_steps)
        self.vis_dir = Path(vis_dir)
        self.conf_thr = float(conf_thr)
        self.iou_thr = float(iou_thr)
        self.mask_thr = float(mask_thr)

        # SegPostProcessor 需要 name->id
        self.pp = AngleSegPostProcessor(
            nc=NUM_CLASSES,
            id2name=id2name,
            conf_thresh=self.conf_thr,
            iou_thresh=self.iou_thr,
            mask_thresh=self.mask_thr,
            mask_up=4,  # 上采样4倍
            refine_bbox_with_mask=False,
        )

    @staticmethod
    def _decode_angle_cat(angle_cat_b: Tensor) -> Tensor:
        bins2, length = angle_cat_b.shape
        if bins2 % 2 != 0:
            raise ValueError(f"angle_cat channel must be even, got {bins2}")
        bins = bins2 // 2
        logits = angle_cat_b[:bins]
        residual = angle_cat_b[bins:]
        k = logits.argmax(dim=0, keepdim=True)
        r = residual.gather(0, k)
        return Angle.decode(k, r, bins).reshape(length)

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

        imgs = batch["img_data"].to(pl_module.device, non_blocking=True)  # (B,3,Hn,Wn)
        det_cat, proto, angle_cat = pl_module(imgs)

        b = 0  # 取 b=0
        canvas = batch["ori_img"][b]  # 原图（BGR）

        # 单帧 numpy
        det_np = det_cat[b].detach().cpu().numpy()  # (C, L)
        proto_np = proto[b].detach().cpu().numpy()  # (nm, Hm, Wm)
        theta_np = self._decode_angle_cat(angle_cat[b]).detach().cpu().numpy()  # (L,)

        # 后处理
        # objs_m: (N,7) [x1,y1,x2,y2,conf,cls,theta]
        # masks_m: (N,H,W) uint8(0/1)  (原图尺寸)
        objs_m, masks_m = self.pp.postprocess(det_np, proto_np, theta_np)

        out_img = self.pp.draw_angleins(
            canvas, objs_m, masks_m,
        )

        self.vis_dir.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(self.vis_dir / f"step_{step:06d}_pred.jpg"), out_img)

        if was_training:
            pl_module.train()

        # ----------- GT 可视化 -----------
        gt_img = canvas.copy()  # 默认：啥都不画也要保存

        gt_cls_2d = batch["labels"][b]  # (K,)
        gt_box_xyxy= batch["boxes"][b]  # (K,4) 大图尺度 xyxy
        gt_angle = batch["directions"][b]
        gt_mask = batch["masks"][b]

        valid = gt_cls_2d >= 0
        if bool(valid.any()):
            gt_cls_2d = gt_cls_2d[valid]
            gt_box_xyxy = gt_box_xyxy[valid]
            gt_angle = gt_angle[valid]
            gt_mask = gt_mask[valid]

            N = gt_box_xyxy.shape[0]
            gt_objs7 = torch.zeros((N, 7), dtype=torch.float32, device=gt_box_xyxy.device)
            gt_objs7[:, 0:4] = gt_box_xyxy
            gt_objs7[:, 4] = 1.0
            gt_objs7[:, 5] = gt_cls_2d
            gt_objs7[:, 6] = gt_angle

            gt_objs7 = as_numpy(gt_objs7)
            gt_mask = as_numpy(gt_mask)

            gt_img = self.pp.draw_angleins(
                gt_img,
                gt_objs7,
                gt_mask,
                draw_mask=True,
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


def load_net_pretrained(net: nn.Module, path: Path) -> None:
    ckpt = torch.load(str(path), map_location="cpu")
    sd = ckpt.get("state_dict", ckpt) if isinstance(ckpt, dict) else ckpt
    if not isinstance(sd, dict):
        raise TypeError(f"不支持的权重格式：{path}")

    net_sd = net.state_dict()
    filtered = {}
    skipped = []
    for key, value in sd.items():
        if not torch.is_tensor(value):
            continue
        name = key[4:] if key.startswith("net.") else key
        if name not in net_sd:
            skipped.append(key)
            continue
        if tuple(value.shape) != tuple(net_sd[name].shape):
            skipped.append(key)
            continue
        filtered[name] = value

    missing, unexpected = net.load_state_dict(filtered, strict=False)
    logger.info(
        f"加载预训练：loaded={len(filtered)}, skipped={len(skipped)}, "
        f"missing={len(missing)}, unexpected={len(unexpected)}"
    )
    if missing:
        logger.info(f"预训练未加载参数示例：{missing[:20]}")
    if skipped:
        logger.info(f"预训练跳过参数示例：{skipped[:20]}")


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
                load_net_pretrained(pl_model.net, p)
            else:
                logger.warning(f"未找到预训练权重：{p}")
    else:
        logger.info("将从 ckpt 恢复，初始化时不加载预训练权重。")

    # 冻住 backbone/neck，只训练任务头
    for p in pl_model.net.model.parameters():
        p.requires_grad = False

    # 前面的层不再记录 BN 的 running_mean / running_var
    for m in pl_model.net.model[:24].modules():
        if isinstance(m, nn.BatchNorm2d):
            m.track_running_stats = False

    # 解冻检测/分割头和角度头。Segment 内含 box/cls/mask 三个分支。
    for p in pl_model.net.segment.parameters():
        p.requires_grad = True
    for p in pl_model.net.angle.parameters():
        p.requires_grad = True

    return pl_model


# ========= 主入口 =========
def main() -> None:
    os.makedirs(LOG_DIR / "ckpt", exist_ok=True)
    logger.init_logger(save_path=f"{LOG_DIR}/log.txt", detailed=True, level=INFO)

    dataset = HuizhouDataset(
        image_dir="/opt_disk3/rd234421/Projects-SGS/R-AEB/data/R-AEB_Data/All-Labeled-Data/cyl_img",
        label_dir="/opt_disk3/rd234421/Projects-SGS/R-AEB/data/R-AEB_Data/All-Labeled-Data/new_labels_direction",
    )

    train_loader = DataLoader(
        dataset=dataset,
        batch_size=48,
        shuffle=True,
        num_workers=8,
        collate_fn=dataset.collate_fn,
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
        every_n_epochs=ENABLE_CKPT_EVERY_N_EPOCHS,
        save_top_k=-1,
        auto_insert_metric_name=False,
    )

    lr_monitor = LearningRateMonitor(
        logging_interval="step"
    )
    progress = TQDMProgressBar()

    vis_cb = VisualizeCallback(
        every_n_steps=VIS_EVERY_STEPS,
        vis_dir=LOG_DIR / "vis",
        id2name=ID2NAME
    )

    export_cb = ExportWeightsCallback(
        out_dir=LOG_DIR / "weight_dict",
        monitor="train/loss_epoch",  # 与你的 ckpt_best 的 monitor 一致
        mode="min",
        every_n_epochs=ENABLE_CKPT_EVERY_N_EPOCHS,  # 保存频率沿用你的常量
    )

    # 4) Trainer
    use_gpu = torch.cuda.is_available()
    trainer = L.Trainer(
        logger=tb_logger,
        callbacks=[ckpt_best, ckpt_every, lr_monitor, progress, vis_cb, export_cb],
        max_epochs=MAX_EPOCHS,
        accumulate_grad_batches=ACCUMULATE_STEPS,
        gradient_clip_val=GRAD_CLIP_VAL,
        precision=PRECISION,
        deterministic=False,
        benchmark=True,
        enable_model_summary=True,
        log_every_n_steps=50,
        accelerator="gpu" if use_gpu else "cpu",
        devices=[0] if use_gpu else 1,
    )

    # 5) 训练（Lightning 会在恢复时同步优化器/调度器/global_step）
    trainer.fit(
        model=pl_model,
        train_dataloaders=train_loader,
        ckpt_path=str(RESUME_CKPT) if resume_flag else None,
    )

    # 6) 关掉数据集内部的进程池
    # dataset.shutdown()
    logger.info("Training completed.")


if __name__ == "__main__":
    main()
