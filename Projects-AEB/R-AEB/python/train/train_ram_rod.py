# -*- coding: utf-8 -*-
import os
from datetime import datetime
from typing import Dict, Optional

import cv2
import numpy as np
import torch
import pytorch_lightning as pl
from torch.utils.data import DataLoader, random_split
from pytorch_lightning.callbacks import ModelCheckpoint, LearningRateMonitor
from pytorch_lightning.loggers import TensorBoardLogger

from visper.common.logger import logger
from visper.common.ops import xyxy2xywh
from visper.common.vis import draw_2d_boxes, as_numpy

# ---- your dataset (the RodDataset you wrote) ----
# 假设你把 RodDataset 放在 datasets/rod_dataset.py
from datasets.rod_dataset import RodDataset, ID2NAME  # 确保这里能 import 到

# ---- model/loss/postproc (same style as your sample) ----
from models.model_ram import YOLO11Det, ConditionalDetLossV8
from visper.postproc.postproc_det_impl import DetPostProcessor


# =========================
# 常改参数（常量写这里，不用 argparse）
# =========================
SEED = 42

TRAIN_IMAGE_DIR = "/opt_disk3/rd234421/Projects-SGS/R-AEB/data/SIED/ROD-dataset/00Train"
TRAIN_LABEL_DIR = "/opt_disk3/rd234421/Projects-SGS/R-AEB/data/SIED/ROD-dataset/00Train-json"

OUT_SIZE = 960
NUM_CLASSES = 80  # Pedestrian/Cyclist/Car/Tram/Truck（按你的 ID2NAME）
MODEL_SCALE = "n"

BATCH_SIZE = 16
NUM_WORKERS = 4

MAX_EPOCHS = 200
BASE_LR = 2e-4
WEIGHT_DECAY = 0.05
WARMUP_EPOCHS = 3

CHECK_VAL_EVERY_N_EPOCH = 10
PRECISION = "16-mixed"

DEVICES = [0]

# 继续支持断点/预训练
RESUME_CKPT_PATH: Optional[str] = None
PRETRAIN_PATH: Optional[str] = "/opt_disk3/rd234421/Projects-SGS/R-AEB/assets/weights/yolo11n-dict.pth"

# dataset split
VAL_RATIO = 0.05

# visualize
VIS_EVERY_N_EPOCHS = 10
VIS_MAX_IMAGES = 4
VIS_CONF_THRESH = 0.5
VIS_IOU_THRESH = 0.3


def make_run_version(log_root: str, exp_name: str) -> str:
    date = datetime.now().strftime("%Y%m%d")
    exp_dir = os.path.join(log_root, exp_name)

    if not os.path.exists(exp_dir):
        return f"{date}_001"

    existed = [
        d for d in os.listdir(exp_dir)
        if d.startswith(f"{date}_") and os.path.isdir(os.path.join(exp_dir, d))
    ]
    return f"{date}_{len(existed) + 1:03d}"


def wb_gray_world_rgb01(img_rgb01: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    """WB for visualization only. img: HWC RGB float [0,1]."""
    if img_rgb01.ndim != 3 or img_rgb01.shape[2] != 3:
        return img_rgb01
    img = img_rgb01.astype(np.float32, copy=True)
    m = img.mean(axis=(0, 1))
    mr, mg, mb = float(m[0]), float(m[1]), float(m[2])
    sr = mg / max(mr, eps)
    sb = mg / max(mb, eps)
    img[:, :, 0] *= sr
    img[:, :, 2] *= sb
    return np.clip(img, 0.0, 1.0)


class VisualizeDetCallback(pl.Callback):
    def __init__(
        self,
        *,
        nc: int,
        id2name: Dict[int, str],
        conf_thresh: float,
        iou_thresh: float,
        every_n_epochs: int,
        max_images: int,
    ) -> None:
        super().__init__()
        self.every_n_epochs = every_n_epochs
        self.max_images = max_images
        self.pp = DetPostProcessor(nc=nc, id2name=id2name, conf_thresh=conf_thresh, iou_thresh=iou_thresh)

    def on_validation_epoch_end(self, trainer, pl_module) -> None:
        if (not trainer.is_global_zero) or ((trainer.current_epoch + 1) % self.every_n_epochs != 0):
            return

        save_dir = os.path.join(trainer.logger.log_dir, "vis")
        os.makedirs(save_dir, exist_ok=True)

        batch = next(iter(trainer.datamodule.val_dataloader()))
        imgs = batch["img_data"].to(pl_module.device, non_blocking=True)

        pl_module.model.eval()
        with torch.no_grad():
            out = pl_module.model(imgs)
        det_cat_b = out[0] if isinstance(out, (tuple, list)) else out

        save_n = min(self.max_images, len(batch["img_name"]))
        ep = trainer.current_epoch + 1

        for i in range(save_n):
            meta = batch["meta"][i]
            ori_hw, scale, pad = meta["ori_hw"], meta["scale"], meta["pad"]

            objs_pred = self.pp.postprocess(as_numpy(det_cat_b[i]), ori_hw=ori_hw, scale=scale, pad=pad)

            m = as_numpy(batch["valid_mask"][i]).astype(bool)
            if m.any():
                gt_xyxy_ori = self.pp._undo_letterbox_xyxy(
                    as_numpy(batch["box"][i])[m], ori_hw=ori_hw, scale=scale, pad=pad
                )
                gt_cls = as_numpy(batch["cls"][i])[m].reshape(-1, 1).astype(np.float32)
                objs_gt = np.concatenate([gt_xyxy_ori, np.ones((len(gt_xyxy_ori), 1), np.float32), gt_cls], axis=1)
            else:
                objs_gt = np.empty((0, 6), np.float32)

            # 你 RodDataset 返回的是 img_vis(BGR u8)，这里我们直接用
            img0 = batch["img_vis"][i].copy()

            # 额外：给可视化做 WB（基于 img0 转回 RGB 做一下再转回 BGR）
            img0_rgb = cv2.cvtColor(img0, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
            img0_rgb = wb_gray_world_rgb01(img0_rgb)
            img0 = cv2.cvtColor((img0_rgb * 255.0).clip(0, 255).astype(np.uint8), cv2.COLOR_RGB2BGR)

            stem = os.path.splitext(os.path.basename(batch["img_name"][i]))[0]

            if objs_gt.size:
                vis_gt = draw_2d_boxes(img0.copy(), objs_gt, id2name=self.pp.id2name, draw_label=True)
            else:
                vis_gt = img0.copy()

            if objs_pred.size:
                vis_pred = self.pp.draw_dets(img0.copy(), objs_pred)
            else:
                vis_pred = img0.copy()

            cv2.imwrite(os.path.join(save_dir, f"ep{ep:04d}_{i}_{stem}_gt.jpg"), vis_gt)
            cv2.imwrite(os.path.join(save_dir, f"ep{ep:04d}_{i}_{stem}_pred.jpg"), vis_pred)


class RodDataModule(pl.LightningDataModule):
    def setup(self, stage: Optional[str] = None) -> None:
        if stage in (None, "fit"):
            full = RodDataset(
                image_dir=TRAIN_IMAGE_DIR,
                label_dir=TRAIN_LABEL_DIR,
                out_size=OUT_SIZE,
                half_size=True,
                out_dtype=torch.float16,
            )

            n_total = len(full)
            n_val = max(1, int(round(n_total * VAL_RATIO)))
            n_train = n_total - n_val
            self.train_ds, self.val_ds = random_split(
                full,
                [n_train, n_val],
                generator=torch.Generator().manual_seed(SEED),
            )
            logger.info(f"RodDataModule split: total={n_total}, train={n_train}, val={n_val}")

            # random_split 后 collate_fn 还在原 dataset 上
            self._collate_fn = full.collate_fn

    def train_dataloader(self) -> DataLoader:
        return DataLoader(
            self.train_ds,
            batch_size=BATCH_SIZE,
            shuffle=True,
            num_workers=NUM_WORKERS,
            pin_memory=True,
            drop_last=True,
            collate_fn=self._collate_fn,
        )

    def val_dataloader(self) -> DataLoader:
        return DataLoader(
            self.val_ds,
            batch_size=BATCH_SIZE,
            shuffle=False,
            num_workers=NUM_WORKERS,
            pin_memory=True,
            drop_last=False,
            collate_fn=self._collate_fn,
        )


class RodDetModule(pl.LightningModule):
    def __init__(self) -> None:
        super().__init__()
        self.model = YOLO11Det(scale=MODEL_SCALE, nc=NUM_CLASSES)
        reg_max = int(getattr(self.model.detect, "reg_max", 16))
        self.criterion = ConditionalDetLossV8(stride=list(self.model.strides), nc=NUM_CLASSES, reg_max=reg_max)

    def on_fit_start(self) -> None:
        if getattr(self.trainer, "ckpt_path", None) or not PRETRAIN_PATH:
            return

        sd = torch.load(PRETRAIN_PATH, map_location="cpu")
        ret = self.model.load_state_dict(sd, strict=False)

        missing = list(ret.missing_keys)
        unexpected = list(ret.unexpected_keys)

        logger.info(f"[PRETRAIN] loaded from {PRETRAIN_PATH}")
        logger.warning(f"[PRETRAIN] missing_keys={len(missing)}, unexpected_keys={len(unexpected)}")

        for k in missing:
            logger.warning(f"[PRETRAIN][MISSING] {k}")
        for k in unexpected:
            logger.warning(f"[PRETRAIN][UNEXPECTED] {k}")

    def _gt(self, cls_pad: torch.Tensor, box_xyxy: torch.Tensor, m: torch.Tensor):
        # box_xyxy is in OUT_SIZE coordinate (letterboxed), normalize to [0,1]
        box = box_xyxy.clone()
        box[~m] = 0.0
        box_xywh = xyxy2xywh(box) / box.new_tensor([OUT_SIZE, OUT_SIZE, OUT_SIZE, OUT_SIZE]).view(1, 1, 4)

        gt_cls = cls_pad.clone()
        gt_cls[~m] = -1
        return gt_cls, box_xywh.clamp(0.0, 1.0)

    def training_step(self, batch, batch_idx: int) -> torch.Tensor:
        imgs = batch["img_data"].to(self.device, non_blocking=True)
        gt_cls, gt_xywh = self._gt(batch["cls"].to(self.device), batch["box"].to(self.device), batch["valid_mask"].to(self.device))

        loss, parts = self.criterion(feats_list=self.model(imgs), gt_cls=gt_cls, gt_bboxes_xywh_norm=gt_xywh)

        self.log_dict(
            {"train/loss": loss, "train/box": parts["box"], "train/cls": parts["cls"], "train/dfl": parts["dfl"]},
            prog_bar=True,
            on_step=True,
            on_epoch=True,
            batch_size=imgs.shape[0],
        )
        return loss

    @torch.no_grad()
    def validation_step(self, batch, batch_idx: int) -> None:
        imgs = batch["img_data"].to(self.device, non_blocking=True)
        gt_cls, gt_xywh = self._gt(batch["cls"].to(self.device), batch["box"].to(self.device), batch["valid_mask"].to(self.device))

        old = self.model.training
        self.model.train()
        try:
            feats = self.model(imgs)
        finally:
            self.model.train(old)

        loss, parts = self.criterion(feats_list=feats, gt_cls=gt_cls, gt_bboxes_xywh_norm=gt_xywh)
        self.log_dict(
            {"val/loss": loss, "val/box": parts["box"], "val/cls": parts["cls"], "val/dfl": parts["dfl"]},
            prog_bar=True,
            on_step=False,
            on_epoch=True,
            batch_size=imgs.shape[0],
        )

    def configure_optimizers(self):
        opt = torch.optim.AdamW(self.parameters(), lr=BASE_LR, weight_decay=WEIGHT_DECAY)

        total = int(self.trainer.estimated_stepping_batches)
        warmup = int(WARMUP_EPOCHS * total / MAX_EPOCHS)
        warmup = max(warmup, 1)

        sch1 = torch.optim.lr_scheduler.LinearLR(opt, start_factor=1.0 / warmup, end_factor=1.0, total_iters=warmup)
        sch2 = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=max(1, total - warmup))
        sch = torch.optim.lr_scheduler.SequentialLR(opt, schedulers=[sch1, sch2], milestones=[warmup])

        return {"optimizer": opt, "lr_scheduler": {"scheduler": sch, "interval": "step"}}


def main() -> None:
    pl.seed_everything(SEED, workers=True)

    log_root = "./logs"
    exp_name = f"rod_det_yolo11{MODEL_SCALE}_sz{OUT_SIZE}"
    run_version = make_run_version(log_root, exp_name)

    tb_logger = TensorBoardLogger(save_dir=log_root, name=exp_name, version=run_version)

    ckpt_dir = os.path.join(tb_logger.log_dir, "checkpoints")
    os.makedirs(ckpt_dir, exist_ok=True)

    trainer = pl.Trainer(
        accelerator="gpu" if torch.cuda.is_available() else "cpu",
        devices=DEVICES,
        strategy="auto",
        precision=PRECISION,
        max_epochs=MAX_EPOCHS,
        logger=tb_logger,
        check_val_every_n_epoch=CHECK_VAL_EVERY_N_EPOCH,
        callbacks=[
            ModelCheckpoint(
                dirpath=ckpt_dir,
                filename="{epoch:03d}-{val_loss:.4f}",
                monitor="val/loss",
                mode="min",
                save_top_k=3,
                save_last=True,
                auto_insert_metric_name=False,
            ),
            LearningRateMonitor(logging_interval="step"),
            VisualizeDetCallback(
                nc=NUM_CLASSES,
                id2name=ID2NAME,
                conf_thresh=VIS_CONF_THRESH,
                iou_thresh=VIS_IOU_THRESH,
                every_n_epochs=VIS_EVERY_N_EPOCHS,
                max_images=VIS_MAX_IMAGES,
            ),
        ],
        gradient_clip_val=5.0,
        gradient_clip_algorithm="norm",
        log_every_n_steps=20,
        benchmark=True,
    )

    trainer.fit(RodDetModule(), datamodule=RodDataModule(), ckpt_path=RESUME_CKPT_PATH)


if __name__ == "__main__":
    main()
