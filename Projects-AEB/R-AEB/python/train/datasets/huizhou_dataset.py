import math
import os
import json
import sys
from typing import List, Dict, Any, Tuple

import numpy as np
import cv2
import torch
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms

from visper.common.logger import logger
from visper.common.vis import draw_2d_instances, color_for_cls

ID2NAME = {
    0: "person",
    1: "bicycle",
    2: "car",
    3: "motorcycle",
    4: "fake person",
    5: "bus",
    6: "fake bicycle",
    7: "truck",
}

NAME2ID = {name: i for i, name in ID2NAME.items()}
NUM_CLASSES = max(ID2NAME) + 1


class HuizhouDataset(Dataset):
    def __init__(self, image_dir: str, label_dir: str):
        self.image_dir = image_dir
        self.label_dir = label_dir

        self.label_files = [
            f for f in os.listdir(label_dir)
            if f.endswith(".json")
        ]
        self.label_files.sort()

        self.transform = transforms.ToTensor()

        # 目标尺寸
        self.out_w = 640
        self.out_h = 320
        self.target_aspect = self.out_w / self.out_h  # 2:1

    def __len__(self) -> int:
        return len(self.label_files)

    def _load_json(self, json_path: str) -> Dict[str, Any]:
        with open(json_path, "r", encoding="utf-8") as f:
            data = json.load(f)
        return data

    def _polygons_to_mask(self, points, height: int, width: int) -> torch.Tensor:
        mask = np.zeros((height, width), dtype=np.uint8)
        pts = np.array(points, dtype=np.int32).reshape(-1, 1, 2)
        cv2.fillPoly(mask, [pts], 1)
        return torch.from_numpy(mask)

    def _mask_to_box(self, mask: torch.Tensor) -> Tuple[int, int, int, int]:
        ys, xs = torch.nonzero(mask, as_tuple=True)
        if len(xs) == 0 or len(ys) == 0:
            return 0, 0, 0, 0
        xmin = int(xs.min().item())
        xmax = int(xs.max().item())
        ymin = int(ys.min().item())
        ymax = int(ys.max().item())
        return xmin, ymin, xmax, ymax

    def _center_crop_to_aspect(
            self, img_bgr: np.ndarray, aspect: float
    ) -> Tuple[np.ndarray, int, int]:
        """
        中心裁剪到指定宽高比 aspect = W/H，返回裁剪后图像和左上角偏移 (crop_x0, crop_y0)
        """
        h, w = img_bgr.shape[:2]
        cur_aspect = w / h

        if abs(cur_aspect - aspect) < 1e-6:
            return img_bgr, 0, 0

        if cur_aspect > aspect:
            # 图太宽，裁宽度
            new_w = int(round(aspect * h))
            x0 = (w - new_w) // 2
            x1 = x0 + new_w
            y0 = 0
            img_cropped = img_bgr[y0:h, x0:x1]
        else:
            # 图太高，裁高度
            new_h = int(round(w / aspect))
            y0 = (h - new_h) // 2
            y1 = y0 + new_h
            x0 = 0
            img_cropped = img_bgr[y0:y1, x0:w]

        return img_cropped, x0, y0

    def __getitem__(self, idx: int):
        json_name = self.label_files[idx]
        json_path = os.path.join(self.label_dir, json_name)

        data = self._load_json(json_path)

        image_path = data.get("imagePath", None)
        if image_path is None:
            base = os.path.splitext(json_name)[0]
            image_path = f"{base}.jpg"

        img_path_full = os.path.join(self.image_dir, image_path)
        img_name = os.path.basename(img_path_full)

        img_bgr_orig = cv2.imread(img_path_full)
        if img_bgr_orig is None:
            raise FileNotFoundError(f"Cannot read image: {img_path_full}")

        # 1) 先中心裁剪到 2:1
        img_cropped, crop_x0, crop_y0 = self._center_crop_to_aspect(
            img_bgr_orig, self.target_aspect
        )
        crop_h, crop_w = img_cropped.shape[:2]

        # 2) 再缩放到 640x320
        img_bgr = cv2.resize(
            img_cropped, (self.out_w, self.out_h), interpolation=cv2.INTER_LINEAR
        )
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

        # 缩放因子（理论上两者相等，但写全更稳妥）
        scale_x = self.out_w / crop_w
        scale_y = self.out_h / crop_h

        masks: List[torch.Tensor] = []
        labels: List[int] = []
        direction_raw_pairs: List[List[int]] = []  # 原始 raw 值对 [k0, k1 or -1]
        direction_angles: List[float] = []        # 弧度
        boxes: List[List[float]] = []

        for shape in data["shapes"]:
            label_name = shape["label"]
            if label_name not in NAME2ID:
                continue

            if shape.get("shape_type", "polygon") != "polygon":
                continue

            # 先取原始类别 id
            cls_id = NAME2ID[label_name]

            # 原始 points 在原图坐标系
            points = shape["points"]

            # 按裁剪 + 缩放，映射到 640x320 坐标系
            points_resized = []
            for x, y in points:
                x_new = (x - crop_x0) * scale_x
                y_new = (y - crop_y0) * scale_y
                points_resized.append([x_new, y_new])

            direction_list = shape.get("direction_number", [])
            raw0, raw1 = -1, -1
            angle_value = -1.0

            if len(direction_list) > 0:
                d0 = int(direction_list[0])
                d1 = int(direction_list[1]) if len(direction_list) > 1 else -1
                raw0, raw1 = d0, d1

                if d0 >= 0 and (d1 == -1 or d1 >= 0):
                    if d1 != -1:
                        d_idx = 0.5 * (float(d0) + float(d1))
                    else:
                        d_idx = float(d0)
                    # 你的角度编码：格子索引 -> 弧度
                    angle_value = d_idx * (-math.pi / 4.0) - math.pi / 2
                    # 归一化到 [-pi, pi)
                    angle_value = (angle_value + math.pi) % (2 * math.pi) - math.pi

            # 如果方向两个 raw 都是 -1，就把 cls 标成 -1（无效实例）
            if raw0 == -1 and raw1 == -1:
                cls_id = -1

            mask = self._polygons_to_mask(points_resized, self.out_h, self.out_w)
            if mask.sum() == 0:
                continue

            xmin, ymin, xmax, ymax = self._mask_to_box(mask)

            masks.append(mask)
            labels.append(cls_id)
            direction_raw_pairs.append([raw0, raw1])
            direction_angles.append(angle_value)
            boxes.append([xmin, ymin, xmax, ymax])

        if len(masks) == 0:
            masks_tensor = torch.zeros((0, self.out_h, self.out_w), dtype=torch.uint8)
            labels_tensor = torch.zeros((0,), dtype=torch.long)
            directions_raw_tensor = torch.full((0, 2), fill_value=-1, dtype=torch.long)
            directions_tensor = torch.zeros((0,), dtype=torch.float32)
            boxes_tensor = torch.zeros((0, 4), dtype=torch.float32)
        else:
            masks_tensor = torch.stack(masks, dim=0)
            labels_tensor = torch.tensor(labels, dtype=torch.long)
            directions_raw_tensor = torch.tensor(direction_raw_pairs, dtype=torch.long)
            directions_tensor = torch.tensor(direction_angles, dtype=torch.float32)
            boxes_tensor = torch.tensor(boxes, dtype=torch.float32)

        image_tensor = self.transform(img_rgb)  # [C, H, W] = [3, 320, 640]

        target = {
            "ori_img": img_bgr,                    # 已经是裁剪+缩放后的 640x320
            "img_data": image_tensor,
            "masks": masks_tensor,
            "labels": labels_tensor,
            "directions_raw": directions_raw_tensor,  # [N, 2]
            "directions": directions_tensor,          # [N], 弧度
            "boxes": boxes_tensor,
            "image_id": torch.tensor([idx], dtype=torch.long),
            "size": torch.tensor([self.out_h, self.out_w], dtype=torch.long),
            "img_name": img_name,
        }

        return target

    @staticmethod
    def collate_fn(batch: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        batch: list of target dict, 每个元素长这样：
          {
            "ori_img": np.ndarray(H, W, 3),
            "img_data": FloatTensor [C, H, W],
            "masks":   ByteTensor [N, H, W],
            "labels":  LongTensor [N],
            "directions_raw": LongTensor [N, 2],
            "directions":     FloatTensor [N],
            "boxes":   FloatTensor [N, 4],
            "img_name": str,
            ...
          }
        """

        # 1) 先堆叠 img_data
        img_tensors = [t["img_data"] for t in batch]           # list of [C, H, W]
        img_data = torch.stack(img_tensors, dim=0)             # [B, C, H, W]
        batch_size, _, H, W = img_data.shape

        # 2) 其他简单的 list
        ori_imgs = [t["ori_img"] for t in batch]
        img_names = [t["img_name"] for t in batch]

        # 3) 统计每张图的实例数，决定 padding 长度
        num_instances_list = [t["labels"].shape[0] for t in batch]
        max_instances = max(num_instances_list) if num_instances_list else 0

        if max_instances == 0:
            masks = torch.zeros((batch_size, 0, H, W), dtype=torch.uint8)
            labels = torch.full((batch_size, 0), fill_value=-1, dtype=torch.long)
            directions_raw = torch.full((batch_size, 0, 2), fill_value=-1, dtype=torch.long)
            directions = torch.zeros((batch_size, 0), dtype=torch.float32)
            boxes = torch.zeros((batch_size, 0, 4), dtype=torch.float32)

            return {
                "ori_img": ori_imgs,
                "img_data": img_data,
                "masks": masks,
                "labels": labels,
                "directions_raw": directions_raw,
                "directions": directions,
                "boxes": boxes,
                "img_name": img_names,
            }

        # 4) 需要 padding 的情况
        masks = torch.zeros((batch_size, max_instances, H, W), dtype=torch.uint8)
        labels = torch.full((batch_size, max_instances), fill_value=-1, dtype=torch.long)
        directions_raw = torch.full((batch_size, max_instances, 2), fill_value=-1, dtype=torch.long)
        directions = torch.zeros((batch_size, max_instances), dtype=torch.float32)
        boxes = torch.zeros((batch_size, max_instances, 4), dtype=torch.float32)

        for i, t in enumerate(batch):
            n = t["labels"].shape[0]
            if n > 0:
                masks[i, :n] = t["masks"]
                labels[i, :n] = t["labels"]
                directions_raw[i, :n] = t["directions_raw"]   # [n, 2]
                directions[i, :n] = t["directions"]           # [n]
                boxes[i, :n] = t["boxes"]

        return {
            "ori_img": ori_imgs,          # list, len B
            "img_data": img_data,         # [B, C, H, W]
            "masks": masks,               # [B, max_N, H, W]
            "labels": labels,             # [B, max_N]
            "directions_raw": directions_raw,  # [B, max_N, 2]
            "directions": directions,          # [B, max_N]
            "boxes": boxes,               # [B, max_N, 4]
            "img_name": img_names,        # list, len B
        }


# ========== Demo 可视化 ==========
if __name__ == "__main__":
    ds = HuizhouDataset(
        image_dir="/opt_disk2/rd23442/Projects-SGS/R-AEB/data/R-AEB_Data/All-Labeled-Data/cyl_img",
        label_dir="/opt_disk2/rd23442/Projects-SGS/R-AEB/data/R-AEB_Data/All-Labeled-Data/new_labels_direction",
    )

    loader = DataLoader(
        dataset=ds,
        batch_size=2,
        shuffle=False,
        num_workers=2,
        prefetch_factor=None,
        collate_fn=ds.collate_fn,
    )

    os.makedirs("huizhou_output", exist_ok=True)

    b = 0
    for i, data in enumerate(loader):
        if i > 200:
            break

        H, W = data["ori_img"][b].shape[:2]
        out_path = "huizhou_output"

        vis_img = data["ori_img"][b]
        img_name = data["img_name"][b]

        gt_2dbox = data["boxes"][b].detach().cpu().numpy()
        gt_cls = data["labels"][b].detach().cpu().numpy()
        gt_mask = data["masks"][b].detach().cpu().numpy()
        gt_angle = data["directions"][b].detach().cpu().numpy()
        gt_dir_raw = data["directions_raw"][b].detach().cpu().numpy()  # (K, 2)

        valid_mask = gt_cls >= 0
        gt_2dbox = gt_2dbox[valid_mask].astype(np.float32, copy=False)
        gt_cls = gt_cls[valid_mask].astype(np.int32, copy=False)
        gt_mask = gt_mask[valid_mask].astype(np.int32, copy=False)
        gt_angle = gt_angle[valid_mask].astype(np.float32, copy=False)
        gt_dir_raw = gt_dir_raw[valid_mask].astype(np.int32, copy=False)

        K = gt_2dbox.shape[0]
        if K == 0:
            save_path = os.path.join(out_path, img_name)
            cv2.imwrite(save_path, vis_img)
            logger.info(f"[INFO] saved (no instances): {save_path}")
            continue

        gt_objs6 = np.concatenate(
            [gt_2dbox, np.ones((K, 1), dtype=np.float32), gt_cls.reshape(K, 1)],
            axis=1,
        )

        txts = [
            f"raw {r0},{r1} angle {a * 180.0 / math.pi:.1f}deg"
            for (r0, r1), a in zip(gt_dir_raw, gt_angle)
        ]

        vis_img = draw_2d_instances(vis_img, gt_objs6, gt_mask, ID2NAME, txts=txts)

        gt_objs7 = np.concatenate([gt_objs6, gt_angle[:, None]], axis=-1)

        for x1, y1, x2, y2, conf, cls_id, angle in gt_objs7[np.argsort(gt_objs6[:, 4])]:
            color = color_for_cls(int(cls_id))

            x1 = int(np.clip(round(x1), 0, W - 1))
            y1 = int(np.clip(round(y1), 0, H - 1))
            x2 = int(np.clip(round(x2), 0, W - 1))
            y2 = int(np.clip(round(y2), 0, H - 1))

            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            L = 0.35 * max(x2 - x1, y2 - y1)
            dx = int(-L * math.sin(angle))
            dy = int(-L * math.cos(angle))

            cv2.arrowedLine(
                vis_img,
                (cx, cy),
                (cx + dx, cy + dy),
                color,
                1,
                cv2.LINE_AA,
                tipLength=0.2,
            )

        save_path = os.path.join(out_path, img_name)
        cv2.imwrite(save_path, vis_img)
        logger.info(f"[INFO] saved: {save_path}")
