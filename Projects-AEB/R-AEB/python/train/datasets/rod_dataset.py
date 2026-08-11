import os
import json
import time
from typing import List, Dict, Any, Optional

import numpy as np
import cv2
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import Dataset, DataLoader

from visper.common.logger import logger

# ----------------------------
# Constants (no argparse)
# ----------------------------
ID2NAME = {
    0: "Pedestrian",
    1: "Cyclist",
    2: "Car",
    5: "Tram",
    7: "Truck",
}
NAME2ID = {name: i for i, name in ID2NAME.items()}

BIT8 = 2 ** 8
BIT16 = 2 ** 16
BIT24 = 2 ** 24
INV_MAX24_F32 = np.float32(1.0 / (BIT24 - 1))

# Raw geometry must match your data
RAW_H = 1856
RAW_W = 2880

# Debayer device: keep CPU for stability (especially if DataLoader num_workers > 0)
DEBAYER_DEVICE = "cpu"


def read_raw_24b_to_nchw_f32(raw_path: str, h: int = RAW_H, w: int = RAW_W) -> np.ndarray:
    """
    Read 24-bit packed raw (3 bytes per pixel, little-endian) -> NCHW float32, shape=(1,1,H,W)
    """
    buf = np.fromfile(raw_path, dtype=np.uint8)
    expect = h * w * 3
    if buf.size != expect:
        raise RuntimeError(f"RAW size mismatch: path={raw_path}, bytes={buf.size}, expect={expect}")

    v = (
            buf[0::3].astype(np.uint32)
            + buf[1::3].astype(np.uint32) * BIT8
            + buf[2::3].astype(np.uint32) * BIT16
    )
    v = v.reshape((1, 1, h, w)).astype(np.float32)
    return v


def wb_gray_world_rgb01(img_rgb01: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    """
    Gray-world WB for visualization only.
    img_rgb01: HWC RGB float in [0,1] (or close), returns same shape/dtype float32.
    """
    if img_rgb01.ndim != 3 or img_rgb01.shape[2] != 3:
        return img_rgb01

    img = img_rgb01.astype(np.float32, copy=True)
    m = img.mean(axis=(0, 1))
    mr, mg, mb = float(m[0]), float(m[1]), float(m[2])

    # scale R and B to match G
    sr = mg / max(mr, eps)
    sb = mg / max(mb, eps)

    img[:, :, 0] *= sr
    img[:, :, 2] *= sb
    return np.clip(img, 0.0, 1.0)


class Debayer3x3(nn.Module):
    # Exactly the same logic as your previous debayer code (kernels + index fixed)
    def __init__(self):
        super().__init__()

        self.kernels = nn.Parameter(
            torch.tensor(
                [
                    [0, 0, 0],
                    [0, 1, 0],
                    [0, 0, 0],

                    [0, 0.25, 0],
                    [0.25, 0, 0.25],
                    [0, 0.25, 0],

                    [0.25, 0, 0.25],
                    [0, 0, 0],
                    [0.25, 0, 0.25],

                    [0, 0, 0],
                    [0.5, 0, 0.5],
                    [0, 0, 0],

                    [0, 0.5, 0],
                    [0, 0, 0],
                    [0, 0.5, 0],
                ],
                dtype=torch.float32,
            ).view(5, 1, 3, 3),
            requires_grad=False,
        )

        self.index = nn.Parameter(
            torch.tensor(
                [
                    # dest channel r
                    [0, 3],  # pixel is R,G1
                    [4, 2],  # pixel is G2,B
                    # dest channel g
                    [1, 0],  # pixel is R,G1
                    [0, 1],  # pixel is G2,B
                    # dest channel b
                    [2, 4],  # pixel is R,G1
                    [3, 0],  # pixel is G2,B
                ],
                dtype=torch.int64,
            ).view(1, 3, 2, 2),
            requires_grad=False,
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # x: (B,1,H,W) -> rgb: (B,3,H,W)
        b, c, h, w = x.shape
        x = F.pad(x, (1, 1, 1, 1), mode="replicate")
        c5 = F.conv2d(x, self.kernels, stride=1)  # (B,5,H,W)
        rgb = torch.gather(c5, 1, self.index.repeat(b, 1, h // 2, w // 2))
        return rgb


class RodDataset(Dataset):
    def __init__(
            self,
            image_dir: str,
            label_dir: str,
            out_size: int = 960,
            half_size: bool = True,
            out_dtype: torch.dtype = torch.float16,
            allowed_raw_exts: Optional[set[str]] = None,
    ):
        self.image_dir = image_dir
        self.label_dir = label_dir

        self.out_h = out_size
        self.out_w = out_size

        self.half_size = half_size
        self.out_dtype = out_dtype

        self.allowed_raw_exts = allowed_raw_exts or {".raw"}

        # Debayer module: init once
        self._debayer_device = DEBAYER_DEVICE
        self._debayer = Debayer3x3().to(self._debayer_device)
        self._debayer.eval()

        # 1) Build RAW lookup: stem -> path (lowercase)
        self.raw_lookup: Dict[str, str] = {}
        t_img = time.time()
        for fn in os.listdir(self.image_dir):
            p = os.path.join(self.image_dir, fn)
            if not os.path.isfile(p):
                continue
            ext = os.path.splitext(fn)[1].lower()
            if ext not in self.allowed_raw_exts:
                continue
            stem = os.path.splitext(fn)[0].lower()
            if stem in self.raw_lookup:
                logger.warning(f"RodDataset: duplicate raw stem='{stem}', keep={self.raw_lookup[stem]}, skip={p}")
                continue
            self.raw_lookup[stem] = p

        if len(self.raw_lookup) == 0:
            raise RuntimeError(
                f"No raw files found under image_dir={self.image_dir}, exts={sorted(self.allowed_raw_exts)}")

        logger.info(f"RodDataset: built raw lookup, n_raw={len(self.raw_lookup)}, time_sec={time.time() - t_img:.2f}")

        label_files = [f for f in os.listdir(label_dir) if f.endswith(".json")]
        label_files.sort()

        self.items: List[Dict[str, Any]] = []

        t0 = time.time()
        logger.info(f"RodDataset: start caching jsons, label_dir={label_dir}, num_json={len(label_files)}")

        skipped_no_label = 0
        skipped_bad_json = 0

        for k, json_name in enumerate(label_files):
            json_path = os.path.join(self.label_dir, json_name)
            try:
                with open(json_path, "r", encoding="utf-8") as f:
                    ann = json.load(f)
            except Exception as e:
                skipped_bad_json += 1
                logger.warning(f"RodDataset: bad json, path={json_path}, err={e}")
                continue

            if "imageData" in ann:
                ann["imageData"] = None

            image_path = ann.get("imagePath", None)
            if not isinstance(image_path, str) or len(image_path) == 0:
                raise KeyError(f"json missing imagePath: {json_name}")

            # json has jpg name; use stem to find raw
            jpg_name = os.path.basename(image_path)
            stem = os.path.splitext(jpg_name)[0].lower()
            raw_path = self.raw_lookup.get(stem)
            if raw_path is None:
                raise FileNotFoundError(
                    f"RAW not found: json_imagePath={image_path}, stem={stem}, image_dir={self.image_dir}"
                )

            raw_name = os.path.basename(raw_path)

            boxes = []
            clses = []
            shapes = ann.get("shapes", [])
            if isinstance(shapes, list):
                for sh in shapes:
                    if not isinstance(sh, dict):
                        continue
                    label = sh.get("label", None)
                    if not isinstance(label, str) or label not in NAME2ID:
                        continue
                    pts = sh.get("points", None)
                    if not isinstance(pts, list) or len(pts) < 2:
                        continue

                    pts_np = np.array(pts, dtype=np.float32)
                    if pts_np.ndim != 2 or pts_np.shape[1] != 2:
                        continue

                    x1 = float(np.min(pts_np[:, 0]))
                    y1 = float(np.min(pts_np[:, 1]))
                    x2 = float(np.max(pts_np[:, 0]))
                    y2 = float(np.max(pts_np[:, 1]))

                    if x2 < x1:
                        x1, x2 = x2, x1
                    if y2 < y1:
                        y1, y2 = y2, y1

                    if (x2 - x1) < 1.0 or (y2 - y1) < 1.0:
                        continue

                    boxes.append([x1, y1, x2, y2])
                    clses.append(NAME2ID[label])

            if len(boxes) == 0:
                skipped_no_label += 1
                box_np = np.zeros((0, 4), dtype=np.float32)
                cls_np = np.zeros((0,), dtype=np.int64)
            else:
                box_np = np.asarray(boxes, dtype=np.float32)
                cls_np = np.asarray(clses, dtype=np.int64)

            self.items.append(
                {
                    "img_name": raw_name,
                    "img_path": raw_path,
                    "box": box_np,  # json coords (assumed original raw size)
                    "cls": cls_np,
                }
            )

            if (k + 1) % 500 == 0:
                logger.info(f"RodDataset: cached {k + 1}/{len(label_files)} jsons")

        dt = time.time() - t0
        logger.info(
            f"RodDataset: finished caching, total_items={len(self.items)}, "
            f"skipped_bad_json={skipped_bad_json}, no_label_json={skipped_no_label}, "
            f"time_sec={dt:.2f}"
        )

    def __len__(self) -> int:
        return len(self.items)

    @torch.no_grad()
    def _read_raw_to_rgb01_f32(self, img_path: str, half_size: bool) -> np.ndarray:
        """
        Use 24bit unpack + Debayer3x3.
        Return HWC RGB float32 in [0,1], no white balance.
        """
        raw_nchw = read_raw_24b_to_nchw_f32(img_path, h=RAW_H, w=RAW_W)  # (1,1,H,W)
        x = torch.from_numpy(raw_nchw).to(device=self._debayer_device, dtype=torch.float32)

        rgb = self._debayer(x)  # (1,3,H,W)
        rgb = rgb.squeeze(0).permute(1, 2, 0).contiguous().cpu().numpy()  # HWC

        # normalize to [0,1] using 24bit full scale
        rgb = np.clip(rgb, 0.0, float(BIT24 - 1)) * INV_MAX24_F32

        if half_size:
            h, w = rgb.shape[:2]
            rgb = cv2.resize(rgb, (w // 2, h // 2), interpolation=cv2.INTER_LINEAR)

        return np.ascontiguousarray(rgb, dtype=np.float32)

    def __getitem__(self, idx: int) -> Dict[str, Any]:
        item = self.items[idx]
        img_name = item["img_name"]
        img_path = item["img_path"]

        img_rgb01 = self._read_raw_to_rgb01_f32(img_path, half_size=self.half_size)
        if img_rgb01 is None or img_rgb01.ndim != 3 or img_rgb01.shape[2] != 3:
            raise RuntimeError(f"RAW read failed or channel!=3: {img_path}")

        h0, w0 = img_rgb01.shape[:2]

        box_np = item["box"].copy()
        cls_np = item["cls"].copy()

        # half_size=True -> output is 1/2, bbox scale to 1/2
        if self.half_size and box_np.shape[0] > 0:
            box_np *= 0.5

        if box_np.shape[0] > 0:
            box_np[:, 0] = np.clip(box_np[:, 0], 0, w0 - 1)
            box_np[:, 2] = np.clip(box_np[:, 2], 0, w0 - 1)
            box_np[:, 1] = np.clip(box_np[:, 1], 0, h0 - 1)
            box_np[:, 3] = np.clip(box_np[:, 3], 0, h0 - 1)

            bw = box_np[:, 2] - box_np[:, 0]
            bh = box_np[:, 3] - box_np[:, 1]
            keep = (bw >= 1.0) & (bh >= 1.0)
            box_np = box_np[keep]
            cls_np = cls_np[keep]

        # letterbox to out_size, padding=0
        new_h, new_w = self.out_h, self.out_w
        r = min(new_w / w0, new_h / h0)
        resized_w = int(round(w0 * r))
        resized_h = int(round(h0 * r))

        img_resized = cv2.resize(img_rgb01, (resized_w, resized_h), interpolation=cv2.INTER_LINEAR)

        pad_w = new_w - resized_w
        pad_h = new_h - resized_h
        pad_left = pad_w // 2
        pad_top = pad_h // 2
        pad_right = pad_w - pad_left
        pad_bottom = pad_h - pad_top

        img_lb = cv2.copyMakeBorder(
            img_resized,
            pad_top, pad_bottom, pad_left, pad_right,
            borderType=cv2.BORDER_CONSTANT,
            value=(0.0, 0.0, 0.0),
        )

        if box_np.shape[0] > 0:
            box_np[:, [0, 2]] = box_np[:, [0, 2]] * r + pad_left
            box_np[:, [1, 3]] = box_np[:, [1, 3]] * r + pad_top
            box_np[:, 0] = np.clip(box_np[:, 0], 0, new_w - 1)
            box_np[:, 2] = np.clip(box_np[:, 2], 0, new_w - 1)
            box_np[:, 1] = np.clip(box_np[:, 1], 0, new_h - 1)
            box_np[:, 3] = np.clip(box_np[:, 3], 0, new_h - 1)

        img_lb = np.ascontiguousarray(img_lb)

        # img_data: CHW float RGB [0,1]
        image_tensor = torch.from_numpy(img_lb).permute(2, 0, 1).contiguous().to(self.out_dtype)

        # img_vis: HWC uint8 BGR for visualization
        # WB only for visualization (do NOT change img_data)
        img_lb_vis = wb_gray_world_rgb01(img_lb)

        # keep your visualization scaling line unchanged
        img_vis_rgb_u8 = (img_lb_vis * 255.0 * 255.0).clip(0, 255).astype(np.uint8)
        img_vis_bgr_u8 = cv2.cvtColor(img_vis_rgb_u8, cv2.COLOR_RGB2BGR)

        img_vis_bgr_u8 = np.ascontiguousarray(img_vis_bgr_u8)

        cls_tensor = torch.from_numpy(cls_np)
        box_tensor = torch.from_numpy(box_np).to(torch.float32)

        return {
            "img_name": img_name,
            "img_path": img_path,
            "img_data": image_tensor,
            "img_vis": img_vis_bgr_u8,
            "cls": cls_tensor,
            "box": box_tensor,
            "meta": {
                "ori_hw": (h0, w0),
                "out_hw": (new_h, new_w),
                "scale": float(r),
                "pad": (int(pad_left), int(pad_top)),
            },
        }

    @staticmethod
    def collate_fn(batch: List[Dict[str, Any]]) -> Dict[str, Any]:
        img_data = torch.stack([b["img_data"] for b in batch], dim=0)

        bsz = len(batch)
        nums = [int(b["cls"].numel()) for b in batch]
        max_n = max(nums) if len(nums) > 0 else 0

        cls_pad = torch.full((bsz, max_n), -1, dtype=torch.long)
        box_pad = torch.full((bsz, max_n, 4), -1.0, dtype=torch.float32)
        valid_mask = torch.zeros((bsz, max_n), dtype=torch.bool)

        for i, b in enumerate(batch):
            n = int(b["cls"].numel())
            if n == 0:
                continue
            cls_pad[i, :n] = b["cls"]
            box_pad[i, :n, :] = b["box"].to(torch.float32)
            valid_mask[i, :n] = True

        return {
            "img_data": img_data,
            "img_name": [b["img_name"] for b in batch],
            "img_path": [b["img_path"] for b in batch],
            "img_vis": [b["img_vis"] for b in batch],
            "cls": cls_pad,
            "box": box_pad,
            "valid_mask": valid_mask,
            "meta": [b["meta"] for b in batch],
        }


if __name__ == "__main__":
    from visper.common.vis import draw_2d_boxes

    # ----------------------------
    # Hard-coded test config
    # ----------------------------
    IMAGE_DIR = "/opt_disk3/rd234421/Projects-SGS/R-AEB/data/SIED/ROD-dataset/00Train"
    LABEL_DIR = "/opt_disk3/rd234421/Projects-SGS/R-AEB/data/SIED/ROD-dataset/00Train-json"
    OUT_SIZE = 960
    HALF_SIZE = True
    OUT_DTYPE = torch.float16

    BATCH_SIZE = 2
    NUM_WORKERS = 0
    MAX_BATCHES = 5
    MAX_IMAGES_PER_BATCH = 999999
    OUT_DIR = "rod_output"

    ds = RodDataset(
        image_dir=IMAGE_DIR,
        label_dir=LABEL_DIR,
        out_size=OUT_SIZE,
        half_size=HALF_SIZE,
        out_dtype=OUT_DTYPE,
    )

    loader = DataLoader(
        dataset=ds,
        batch_size=BATCH_SIZE,
        shuffle=False,
        num_workers=NUM_WORKERS,
        collate_fn=ds.collate_fn,
        pin_memory=True,
    )

    os.makedirs(OUT_DIR, exist_ok=True)
    logger.info(f"dataset size={len(ds)}, save_dir={OUT_DIR}, max_batches={MAX_BATCHES}")

    for bidx, batch in enumerate(loader):
        if bidx >= MAX_BATCHES:
            logger.info(f"stop: reached max_batches={MAX_BATCHES}")
            break

        cls_pad = batch["cls"]
        box_pad = batch["box"]
        valid_mask = batch["valid_mask"]

        bsz = int(cls_pad.shape[0])
        save_n = min(bsz, MAX_IMAGES_PER_BATCH)
        logger.info(f"batch {bidx}: batch_size={bsz}, save_images={save_n}")

        for i in range(save_n):
            img_bgr = batch["img_vis"][i].copy()

            m = valid_mask[i].cpu().numpy()
            if m.any():
                boxes = box_pad[i][m].cpu().numpy().astype(np.float32)
                clses = cls_pad[i][m].cpu().numpy().astype(np.int64)
                conf = np.ones((boxes.shape[0], 1), dtype=np.float32)
                objs = np.concatenate([boxes, conf, clses.reshape(-1, 1)], axis=1)
            else:
                objs = np.zeros((0, 6), dtype=np.float32)

            vis = draw_2d_boxes(img_bgr, objs, id2name=ID2NAME, draw_label=True, thickness=2)

            name = batch["img_name"][i]
            stem = os.path.splitext(name)[0]
            out_path = os.path.join(OUT_DIR, f"{bidx:03d}_{i}_{stem}.jpg")
            ok = cv2.imwrite(out_path, vis)

            if ok:
                logger.info(f"saved: {out_path} (n_objs={int(m.sum())})")
            else:
                logger.warning(f"save failed: {out_path}")

    logger.info(f"done -> {OUT_DIR}")
