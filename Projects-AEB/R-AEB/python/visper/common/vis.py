from __future__ import annotations
from typing import Sequence

import matplotlib
import torch

matplotlib.use("Agg", force=True)

import matplotlib.pyplot as plt
from matplotlib.axes import Axes

import numpy as np
from typing import Dict, Tuple, Optional

COLOR_MAP_BGR = [
    (107, 107, 255),  # id 0  | RGB #FF6B6B | Coral Red (珊瑚红)
    (43, 146, 255),  # id 1  | RGB #FF922B | Vivid Orange (活力橙)
    (59, 212, 255),  # id 2  | RGB #FFD43B | Sunny Yellow (阳光黄)
    (75, 227, 169),  # id 3  | RGB #A9E34B | Lime Green (青柠绿)
    (87, 192, 64),  # id 4  | RGB #40C057 | Fresh Green (清新绿)
    (151, 201, 32),  # id 5  | RGB #20C997 | Mint/Teal (薄荷青)
    (191, 170, 21),  # id 6  | RGB #15AABF | Cyan Teal (青蓝)
    (230, 139, 34),  # id 7  | RGB #228BE6 | Bright Blue (亮蓝)
    (245, 110, 76),  # id 8  | RGB #4C6EF5 | Indigo Blue (靛蓝)
    (242, 80, 121),  # id 9  | RGB #7950F2 | Purple (紫罗兰)
    (219, 75, 190),  # id 10 | RGB #BE4BDB | Neon Purple (霓虹紫)
    (149, 101, 240),  # id 11 | RGB #F06595 | Hot Pink (亮粉)

    (135, 135, 255),  # id 12 | RGB #FF8787 | Soft Coral (浅珊瑚)
    (77, 169, 255),  # id 13 | RGB #FFA94D | Soft Orange (浅橙)
    (102, 224, 255),  # id 14 | RGB #FFE066 | Soft Yellow (浅黄)
    (162, 245, 216),  # id 15 | RGB #D8F5A2 | Pastel Lime (柔青柠)
    (154, 233, 140),  # id 16 | RGB #8CE99A | Pastel Green (柔绿)
    (190, 230, 99),  # id 17 | RGB #63E6BE | Pastel Mint (柔薄荷)
    (232, 217, 102),  # id 18 | RGB #66D9E8 | Pastel Cyan (柔青蓝)
    (252, 192, 116),  # id 19 | RGB #74C0FC | Pastel Sky (柔天蓝)
    (255, 167, 145),  # id 20 | RGB #91A7FF | Pastel Periwinkle (柔紫蓝)
    (252, 151, 177),  # id 21 | RGB #B197FC | Pastel Purple (柔紫)
    (247, 153, 229),  # id 22 | RGB #E599F7 | Pastel Magenta (柔洋红)
    (215, 194, 252),  # id 23 | RGB #FCC2D7 | Pastel Pink (柔粉)

    (49, 49, 224),  # id 24 | RGB #E03131 | Deep Red (深红)
    (92, 37, 194),  # id 25 | RGB #C2255C | Raspberry (树莓红)
    (0, 140, 240),  # id 26 | RGB #F08C00 | Amber Orange (琥珀橙)
    (5, 176, 250),  # id 27 | RGB #FAB005 | Golden Yellow (金黄)
    (68, 158, 47),  # id 28 | RGB #2F9E44 | Emerald Green (祖母绿)
    (120, 166, 12),  # id 29 | RGB #0CA678 | Teal Green (蓝绿)
    (173, 152, 16),  # id 30 | RGB #1098AD | Deep Teal (深青)
    (171, 100, 24),  # id 31 | RGB #1864AB | Deep Blue (深蓝)
    (199, 79, 54),  # id 32 | RGB #364FC7 | Royal Blue (皇家蓝)
    (196, 61, 95),  # id 33 | RGB #5F3DC4 | Deep Purple (深紫)
    (156, 46, 134),  # id 34 | RGB #862E9C | Grape Purple (葡萄紫)
    (77, 30, 166),  # id 35 | RGB #A61E4D | Wine Magenta (酒红紫)
]


def color_for_cls(cid: int) -> Tuple[int, int, int]:
    """
    固定颜色映射表：根据类别 id 返回稳定颜色（BGR）
    """
    return COLOR_MAP_BGR[int(cid) % len(COLOR_MAP_BGR)]


def as_numpy(x) -> np.ndarray:
    """
    将输入转换为 numpy.ndarray。

    支持：
      - numpy.ndarray
      - torch.Tensor（或任意具有 detach().cpu().numpy() 方法的对象）
    """
    if isinstance(x, np.ndarray):
        return x
    # torch-like tensor: 有 detach/cpu/numpy 方法即可
    if hasattr(x, "detach") and hasattr(x, "cpu") and hasattr(x, "numpy"):
        return x.detach().cpu().numpy()
    raise TypeError(f"{x} must be a numpy array or torch tensor, got {type(x)}")


def draw_2d_boxes(
        img: np.ndarray,  # (H,W,3) uint8, BGR
        objs: np.ndarray,  # (N,6) or (6,)
        id2name: Optional[Dict[int, str]] = None,
        *,
        txts: Optional[Sequence[str]] = None,  # (N,) or single str
        draw_label: bool = True,
        thickness: int = 1,
) -> np.ndarray:
    img = as_numpy(img)
    objs = as_numpy(objs)

    if img.ndim != 3 or img.shape[-1] != 3:
        raise ValueError(f"img must be (H,W,3), got {img.shape}")

    if objs is None or objs.size == 0:
        return img

    # ---- 兼容 objs 为 (6,) 的情况 ----
    if objs.ndim == 1:
        if objs.shape[0] != 6:
            raise ValueError(f"objs must be (N,6) or (6,), got {objs.shape}")
        objs = objs.reshape(1, 6)

    if objs.ndim != 2 or objs.shape[1] != 6:
        raise ValueError(f"objs must have shape (N,6), got {objs.shape}")

    # ---- 兼容 txts 传单个字符串 ----
    if isinstance(txts, str):
        txts = [txts]

    if txts is not None and len(txts) != objs.shape[0]:
        raise ValueError(f"txts length must match objs N, got len(txts)={len(txts)} vs N={objs.shape[0]}")

    out = img.copy()
    H, W = out.shape[:2]

    order = np.argsort(objs[:, 4])  # conf 从低到高

    for idx in order:
        x1, y1, x2, y2, conf, cls_id = objs[idx].tolist()
        cls_id = int(cls_id)
        color = color_for_cls(cls_id)

        x1i = int(np.clip(round(x1), 0, W - 1))
        y1i = int(np.clip(round(y1), 0, H - 1))
        x2i = int(np.clip(round(x2), 0, W - 1))
        y2i = int(np.clip(round(y2), 0, H - 1))

        cv2.rectangle(out, (x1i, y1i), (x2i, y2i), color, thickness, cv2.LINE_AA)

        if draw_label:
            if txts is not None:
                txt = "" if txts[idx] is None else str(txts[idx])
            else:
                name = id2name.get(cls_id, str(cls_id)) if id2name is not None else str(cls_id)
                txt = f"{name} {conf:.2f}"

            if txt != "":
                (tw, th), base = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
                pad_x, pad_y = 3, 2

                x0 = max(0, x1i)
                y0 = max(0, y1i - th - 2 * pad_y)
                x1b = min(W - 1, x0 + tw + 2 * pad_x)
                y1b = min(H - 1, y0 + th + 2 * pad_y)

                cv2.rectangle(out, (x0, y0), (x1b, y1b), color, -1)
                text_org = (x0 + pad_x, y0 + th + pad_y - base // 2)
                cv2.putText(out, txt, text_org, cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1, cv2.LINE_8)

    return out


def draw_2d_instances(
        img: np.ndarray,  # (H,W,3) uint8, BGR
        objs: np.ndarray,  # (N,6) or (6,)
        masks: np.ndarray,  # (N,H,W) or (H,W)
        id2name: Optional[Dict[int, str]] = None,
        *,
        txts: Optional[Sequence[str]] = None,  # (N,) or single str
        draw_bbox: bool = True,
        draw_label: bool = True,
        mask_alpha: float = 0.45,
        thickness: int = 1,
) -> np.ndarray:
    img = as_numpy(img)
    objs = as_numpy(objs)
    masks = as_numpy(masks)

    if img.ndim != 3 or img.shape[-1] != 3:
        raise ValueError(f"img must be (H,W,3), got {img.shape}")

    if objs is None or objs.size == 0:
        return img

    # ---- 兼容 objs 为 (6,) ----
    if objs.ndim == 1:
        if objs.shape[0] != 6:
            raise ValueError(f"objs must be (N,6) or (6,), got {objs.shape}")
        objs = objs.reshape(1, 6)

    if objs.ndim != 2 or objs.shape[1] != 6:
        raise ValueError(f"objs must have shape (N,6), got {objs.shape}")

    if masks is None or masks.size == 0:
        # 没 mask：按用户开关决定要不要画框/字
        if draw_bbox or draw_label:
            return draw_2d_boxes(img, objs, id2name=id2name, txts=txts, draw_label=draw_label, thickness=thickness)
        return img

    # ---- 兼容 masks 为 (H,W) ----
    if masks.ndim == 2:
        masks = masks[None, ...]  # (1,H,W)

    if masks.ndim != 3:
        raise ValueError("masks must be (N,H,W) or (H,W)")

    # ---- 兼容 txts 传单个字符串 ----
    if isinstance(txts, str):
        txts = [txts]

    out = img.copy()
    H, W = out.shape[:2]

    N = min(objs.shape[0], masks.shape[0])
    objs = objs[:N]
    masks = masks[:N]
    if txts is not None:
        txts = list(txts)[:N]

    if masks.shape[1] != H or masks.shape[2] != W:
        raise ValueError(
            f"mask size must match image size, got masks (N,{masks.shape[1]},{masks.shape[2]}), img ({H},{W})"
        )

    order = np.argsort(objs[:, 4])  # conf 从低到高

    for idx in order:
        cls_id = int(objs[idx, 5])
        color = color_for_cls(cls_id)

        m = masks[idx]
        if m.shape != (H, W):
            raise ValueError(f"single mask shape must be ({H},{W}), got {m.shape}")

        if m.any():
            overlay = np.zeros_like(out, dtype=np.uint8)
            overlay[m.astype(bool)] = color
            out = cv2.addWeighted(out, 1.0, overlay, float(mask_alpha), 0.0)

    if draw_bbox or draw_label:
        out = draw_2d_boxes(
            out,
            objs,
            id2name=id2name,
            txts=txts,
            draw_label=draw_label,
            thickness=thickness,
        )

    return out


def boxes3d_to_corners3d(boxes):
    """
    将 3D 盒参数 (cx,cy,cz,l,w,h,theta) 转为 8 个角点坐标。

    支持输入:
      - numpy.ndarray: (N,7) 或 (7,)
      - torch.Tensor : (N,7) 或 (7,)
    输出:
      - 若输入为 torch.Tensor -> 输出 torch.Tensor (N,8,3)，dtype/device 跟随输入
      - 若输入为 numpy.ndarray -> 输出 numpy.ndarray (N,8,3) float32

    参数定义 (boxes):
      boxes: (N,7) = (cx, cy, cz, l, w, h, theta)
        - (cx,cy,cz): 盒中心坐标（世界/车体/雷达等坐标系，取决于你的输入定义）
        - l: 长度，沿局部 +x 方向
        - w: 宽度，沿局部 +y 方向
        - h: 高度，沿局部 +z 方向
        - theta: 绕 +z 轴旋转的 yaw（弧度），右手系下：
                 theta>0 表示在 (x,y) 平面内从 +x 朝 +y 的逆时针旋转

    角点顺序（非常重要，所有绘制/反变换都依赖这个约定）:
      在“未旋转、中心在原点”的局部坐标系中，8 个角点为：

      底面 (z = -h/2):
        0: (-l/2, -w/2, -h/2)
        1: (+l/2, -w/2, -h/2)
        2: (+l/2, +w/2, -h/2)
        3: (-l/2, +w/2, -h/2)

      顶面 (z = +h/2):
        4: (-l/2, -w/2, +h/2)
        5: (+l/2, -w/2, +h/2)
        6: (+l/2, +w/2, +h/2)
        7: (-l/2, +w/2, +h/2)

      也就是说：
        - 底面按 0->1->2->3 排列（沿 +x 再沿 +y）
        - 顶面对应底面点整体 +z 平移：0<->4, 1<->5, 2<->6, 3<->7
        - “长度方向”(l) 的参考边为 0->1（局部 +x）
        - “宽度方向”(w) 的参考边为 1->2（局部 +y）

    常用连边（画 3D 框线框时）:
      bottom: (0,1),(1,2),(2,3),(3,0)
      top:    (4,5),(5,6),(6,7),(7,4)
      vertical:(0,4),(1,5),(2,6),(3,7)
    """
    if boxes is None:
        # 无法判别 dtype/device，返回 numpy 空
        return np.zeros((0, 8, 3), dtype=np.float32)

    is_torch = torch.is_tensor(boxes)

    if is_torch:
        x = boxes
        if x.ndim == 1:
            x = x.unsqueeze(0)
        if x.numel() == 0:
            return x.new_zeros((0, 8, 3))
        if x.shape[1] != 7:
            raise ValueError(f"boxes shape must be (N,7), got {tuple(x.shape)}")

        cx, cy, cz, l, w, h, theta = (x[:, i] for i in range(7))

        x_c = torch.stack([-l / 2, l / 2, l / 2, -l / 2, -l / 2, l / 2, l / 2, -l / 2], dim=1)
        y_c = torch.stack([-w / 2, -w / 2, w / 2, w / 2, -w / 2, -w / 2, w / 2, w / 2], dim=1)
        z_c = torch.stack([-h / 2, -h / 2, -h / 2, -h / 2, h / 2, h / 2, h / 2, h / 2], dim=1)

        cos_y = torch.cos(theta).unsqueeze(1)
        sin_y = torch.sin(theta).unsqueeze(1)

        x_w = cos_y * x_c - sin_y * y_c + cx.unsqueeze(1)
        y_w = sin_y * x_c + cos_y * y_c + cy.unsqueeze(1)
        z_w = z_c + cz.unsqueeze(1)

        return torch.stack([x_w, y_w, z_w], dim=-1)  # (N,8,3)

    # numpy branch
    x = np.asarray(boxes, dtype=np.float32)
    if x.ndim == 1:
        x = x[None, :]
    if x.size == 0:
        return np.zeros((0, 8, 3), dtype=np.float32)
    if x.shape[1] != 7:
        raise ValueError(f"boxes shape must be (N,7), got {x.shape}")

    cx, cy, cz, l, w, h, theta = [x[:, i] for i in range(7)]

    x_c = np.stack([-l / 2, l / 2, l / 2, -l / 2, -l / 2, l / 2, l / 2, -l / 2], axis=1)
    y_c = np.stack([-w / 2, -w / 2, w / 2, w / 2, -w / 2, -w / 2, w / 2, w / 2], axis=1)
    z_c = np.stack([-h / 2, -h / 2, -h / 2, -h / 2, h / 2, h / 2, h / 2, h / 2], axis=1)

    cos_y = np.cos(theta)[:, None]
    sin_y = np.sin(theta)[:, None]

    x_w = cos_y * x_c - sin_y * y_c + cx[:, None]
    y_w = sin_y * x_c + cos_y * y_c + cy[:, None]
    z_w = z_c + cz[:, None]

    return np.stack([x_w, y_w, z_w], axis=-1).astype(np.float32)


def corners3d_to_boxes3d(corners):
    """
    将 8 个角点坐标反推回盒参数 (cx,cy,cz,l,w,h,theta)。

    支持输入:
      - numpy.ndarray: (N,8,3) 或 (8,3)
      - torch.Tensor : (N,8,3) 或 (8,3)
    输出:
      - 若输入为 torch.Tensor -> 输出 torch.Tensor (N,7)，dtype/device 跟随输入
      - 若输入为 numpy.ndarray -> 输出 numpy.ndarray (N,7) float32

    【角点顺序要求】（必须与 boxes3d_to_corners3d 完全一致）:
      corners 的索引含义必须是：

      底面 (z = -h/2):
        0,1,2,3 依次围成底面矩形，且 0->1 是长度方向（局部 +x），1->2 是宽度方向（局部 +y）
      顶面 (z = +h/2):
        4,5,6,7 分别对应 0,1,2,3 在 +z 方向的点：
          0<->4, 1<->5, 2<->6, 3<->7

      具体局部坐标（未旋转、中心在原点）对应为：
        0: (-l/2, -w/2, -h/2)
        1: (+l/2, -w/2, -h/2)
        2: (+l/2, +w/2, -h/2)
        3: (-l/2, +w/2, -h/2)
        4: (-l/2, -w/2, +h/2)
        5: (+l/2, -w/2, +h/2)
        6: (+l/2, +w/2, +h/2)
        7: (-l/2, +w/2, +h/2)

    反推策略（跟上述顺序绑定）:
      - center: 8 点平均
      - l: 取底面边 |corner1 - corner0| 的长度（xy 平面）
      - w: 取底面边 |corner2 - corner1| 的长度（xy 平面）
      - h: 取所有角点 z 的 max-min
      - theta: 取底面“长度方向”边向量的方向角
              使用 (corner1 - corner0) 与 (corner2 - corner3) 两条平行边做平均，略增强鲁棒性
    """
    if corners is None:
        return np.zeros((0, 7), dtype=np.float32)

    is_torch = torch.is_tensor(corners)

    if is_torch:
        c = corners
        if c.ndim == 2:
            c = c.unsqueeze(0)
        if c.numel() == 0:
            return c.new_zeros((0, 7))
        if c.shape[1:] != (8, 3):
            raise ValueError(f"corners shape must be (N,8,3), got {tuple(c.shape)}")

        eps = c.new_tensor(1e-8)

        center = c.mean(dim=1)  # (N,3)
        cx, cy, cz = center[:, 0], center[:, 1], center[:, 2]

        v01 = c[:, 1, :2] - c[:, 0, :2]
        v12 = c[:, 2, :2] - c[:, 1, :2]
        l = torch.sqrt((v01 ** 2).sum(dim=1) + eps)
        w = torch.sqrt((v12 ** 2).sum(dim=1) + eps)

        zmin = c[:, :, 2].min(dim=1).values
        zmax = c[:, :, 2].max(dim=1).values
        h = (zmax - zmin).clamp(min=0)

        v32 = c[:, 2, :2] - c[:, 3, :2]
        v01n = v01 / (torch.sqrt((v01 ** 2).sum(dim=1, keepdim=True) + eps))
        v32n = v32 / (torch.sqrt((v32 ** 2).sum(dim=1, keepdim=True) + eps))
        vdir = v01n + v32n
        theta = torch.atan2(vdir[:, 1], vdir[:, 0])

        return torch.stack([cx, cy, cz, l, w, h, theta], dim=1)

    # numpy branch
    c = np.asarray(corners, dtype=np.float32)
    if c.ndim == 2:
        c = c[None, ...]
    if c.size == 0:
        return np.zeros((0, 7), dtype=np.float32)
    if c.shape[1:] != (8, 3):
        raise ValueError(f"corners shape must be (N,8,3), got {c.shape}")

    center = c.mean(axis=1)
    cx, cy, cz = center[:, 0], center[:, 1], center[:, 2]

    v01 = c[:, 1, :2] - c[:, 0, :2]
    v12 = c[:, 2, :2] - c[:, 1, :2]
    l = np.sqrt((v01 ** 2).sum(axis=1) + 1e-8)
    w = np.sqrt((v12 ** 2).sum(axis=1) + 1e-8)

    zmin = c[:, :, 2].min(axis=1)
    zmax = c[:, :, 2].max(axis=1)
    h = np.clip(zmax - zmin, 0.0, None)

    v32 = c[:, 2, :2] - c[:, 3, :2]
    v01n = v01 / (np.sqrt((v01 ** 2).sum(axis=1, keepdims=True) + 1e-8))
    v32n = v32 / (np.sqrt((v32 ** 2).sum(axis=1, keepdims=True) + 1e-8))
    vdir = v01n + v32n
    theta = np.arctan2(vdir[:, 1], vdir[:, 0]).astype(np.float32)

    return np.stack([cx, cy, cz, l, w, h, theta], axis=1).astype(np.float32)


def draw_3d_boxes(
        img: np.ndarray,  # (H,W,3) uint8, BGR
        corners_uv: np.ndarray,  # (N,8,2) or (8,2)  float/int, pixel coords
        objs: Optional[np.ndarray] = None,  # (N,6) [x1,y1,x2,y2,conf,cls_i] 仅用于颜色/label/排序；可为 None
        id2name: Optional[Dict[int, str]] = None,
        *,
        txts: Optional[Sequence[str]] = None,  # (N,) 自定义文本
        draw_label: bool = True,
        thickness: int = 1,
) -> np.ndarray:
    """
    在图像上绘制 3D 框线框（8 个角点的 uv 投影）。

    输入
    ----------
    img : (H,W,3) uint8, BGR
    corners_uv :
        (N,8,2) 或 (8,2)，每个 3D 框的 8 个角点在图像上的像素坐标 (u,v)。
        角点顺序必须为：

        底面:
          0: (-l/2, -w/2, -h/2)   1: (+l/2, -w/2, -h/2)
          2: (+l/2, +w/2, -h/2)   3: (-l/2, +w/2, -h/2)

        顶面:
          4: (-l/2, -w/2, +h/2)   5: (+l/2, -w/2, +h/2)
          6: (+l/2, +w/2, +h/2)   7: (-l/2, +w/2, +h/2)

    objs : 可选 (N,6) [x1,y1,x2,y2,conf,cls]
        - 用于：颜色由 cls 决定；排序按 conf 从低到高；默认 label 文本
        - 若不提供：按输入顺序画；颜色用固定色；label 仅用 txts（若提供）

    txts : 可选 (N,)
        自定义文本；优先级高于默认 "{name} {conf:.2f}"

    draw_label : 是否绘制文本（默认 True）
    thickness : 线宽（默认 1）

    返回
    ----------
    out : (H,W,3) uint8, BGR
    """
    img = as_numpy(img)
    corners_uv = as_numpy(corners_uv)
    objs = as_numpy(objs) if objs is not None else None

    if img.ndim != 3 or img.shape[-1] != 3:
        raise ValueError(f"img must be (H,W,3), got {img.shape}")

    if corners_uv is None or corners_uv.size == 0:
        return img

    if corners_uv.ndim == 2:
        corners_uv = corners_uv[None, ...]  # (1,8,2)

    if corners_uv.ndim != 3 or corners_uv.shape[1:] != (8, 2):
        raise ValueError(f"corners_uv must be (N,8,2) or (8,2), got {corners_uv.shape}")

    N = corners_uv.shape[0]

    if txts is not None and len(txts) != N:
        raise ValueError(f"txts length must match N, got len(txts)={len(txts)} vs N={N}")

    if objs is not None:
        if objs.ndim == 1:
            objs = objs[None, :]
        if objs.shape[0] != N or objs.shape[1] != 6:
            raise ValueError(f"objs must be (N,6) and match corners_uv N, got {objs.shape} vs N={N}")

    out = img.copy()
    H, W = out.shape[:2]

    # 线框连接边：底面 + 顶面 + 竖边
    edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),  # bottom
        (4, 5), (5, 6), (6, 7), (7, 4),  # top
        (0, 4), (1, 5), (2, 6), (3, 7),  # vertical
    ]

    # 可选：加两条“朝向边”，更容易看 front（这里用 0->1 和 4->5）
    front_edges = [(1, 6), (2, 5)]

    # 排序：跟 draw_2d_boxes 一样，按 conf 从低到高
    if objs is not None:
        order = np.argsort(objs[:, 4])
    else:
        order = np.arange(N, dtype=np.int64)

    for idx in order:
        pts = corners_uv[idx].astype(np.float32)  # (8,2)

        # 颜色：优先用 cls；否则固定色
        if objs is not None:
            cls_id = int(objs[idx, 5])
            color = color_for_cls(cls_id)
            conf = float(objs[idx, 4])
        else:
            cls_id = -1
            color = (0, 255, 0)
            conf = 1.0

        # 裁剪并转 int（u 对应 x，v 对应 y）
        p = np.empty((8, 2), dtype=np.int32)
        p[:, 0] = np.clip(np.round(pts[:, 0]), 0, W - 1).astype(np.int32)
        p[:, 1] = np.clip(np.round(pts[:, 1]), 0, H - 1).astype(np.int32)

        # 1) 画线框
        for a, b in edges:
            cv2.line(out, tuple(p[a]), tuple(p[b]), color, int(thickness), cv2.LINE_AA)

        # 2) 画前面(+x面)对角线
        for a, b in front_edges:
            cv2.line(out, tuple(p[a]), tuple(p[b]), color, int(thickness), cv2.LINE_AA)

        # 3) 文本标签：放在“顶面四点的均值”附近更稳
        if draw_label:
            if txts is not None:
                txt = "" if txts[idx] is None else str(txts[idx])
            else:
                if objs is not None:
                    name = id2name.get(cls_id, str(cls_id)) if id2name is not None else str(cls_id)
                    txt = f"{name} {conf:.2f}"
                else:
                    txt = ""

            if txt != "":
                top_center = p[4:8].mean(axis=0).astype(np.int32)  # (2,)
                x1i, y1i = int(top_center[0]), int(top_center[1])

                (tw, th), base = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
                pad_x, pad_y = 3, 2

                x0 = int(np.clip(x1i, 0, W - 1))
                y0 = int(np.clip(y1i - th - 2 * pad_y, 0, H - 1))
                x1b = int(np.clip(x0 + tw + 2 * pad_x, 0, W - 1))
                y1b = int(np.clip(y0 + th + 2 * pad_y, 0, H - 1))

                cv2.rectangle(out, (x0, y0), (x1b, y1b), color, -1)
                text_org = (x0 + pad_x, y0 + th + pad_y - base // 2)
                cv2.putText(out, txt, text_org, cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1, cv2.LINE_8)
    return out


def vis_point_lane(vis_img, lanes, scores=None):
    for pts in lanes:
        pts = np.array(pts, dtype=np.int32)  # 转换为 numpy 数组，类型为 int32
        pts = pts.reshape((-1, 1, 2))  # 重新调整为 (n, 1, 2) 形状，确保符合 cv2.polylines 的要求

        # 绘制车道线
        cv2.polylines(vis_img, [pts], isClosed=False, color=(0, 255, 0), thickness=2)

        # 绘制每个采样点：红色圆点
        for pt in pts:
            cv2.circle(vis_img, tuple(pt[0]), radius=2, color=(0, 0, 255), thickness=-1)

    return vis_img


def draw_trajectory_on_ax(ax: Axes, trajectories, confs, line_type='o-', transparent=True, xlim=(30, -30),
                          ylim=(0, 100)):
    '''
    ax: matplotlib.axes.Axes, the axis to draw trajectories on
    trajectories: List of numpy arrays of shape (num_points, 2 or 3)
    confs: List of numbers, 1 means gt
    '''

    # get the max conf
    max_conf = max([conf for conf in confs if conf != 1])

    for idx, (trajectory, conf) in enumerate(zip(trajectories, confs)):
        label = 'gt' if conf == 1 else 'pred%d (%.3f)' % (idx, conf)
        alpha = 1.0
        if transparent:
            alpha = 1.0 if conf == max_conf else np.clip(conf, 0.5, None)
        plot_args = dict(label=label, alpha=alpha, linewidth=2 if alpha == 1.0 else 1)
        if label == 'gt':
            plot_args['color'] = '#d62728'
        ax.plot(trajectory[:, 1],  # - for nuscenes and + for comma 2k19
                trajectory[:, 0],
                line_type, **plot_args)
    if xlim is not None:
        ax.set_xlim(*xlim)
    if ylim is not None:
        ax.set_ylim(*ylim)
    ax.legend(loc='lower left')

    return ax


def draw_blended_trajectory_on_ax(
        ax: Axes,
        trajectories: list[np.ndarray] | tuple[np.ndarray, ...],
        confs: list[float] | tuple[float, ...],
        line_type: str = 'o-',
        xlim: tuple[float, float] | None = (30, -30),
        ylim: tuple[float, float] | None = (0, 100),
):
    """
    在 ax 上同时绘制：
      1. 所有 GT 轨迹（conf == 1）
      2. 由预测轨迹（conf != 1）按权重线性加权的 blended 轨迹

    Parameters
    ----------
    ax : matplotlib.axes.Axes
    trajectories : list[np.ndarray] | tuple[np.ndarray, ...]
        每个元素形状 (N, 2) 或 (N, 3)
    confs : list[float] | tuple[float, ...]
        与 trajectories 对应；其中 1 表示 GT，其余为预测置信度
    """
    if len(trajectories) != len(confs):
        raise ValueError("`trajectories` 与 `confs` 长度必须一致")

    # --- 拆分 GT 与预测 ---
    gt_trajs = [t for t, c in zip(trajectories, confs) if c == 1]
    pred_trajs = [t for t, c in zip(trajectories, confs) if c != 1]
    pred_w = np.asarray([c for c in confs if c != 1], dtype=float)

    if len(gt_trajs) == 0:
        raise ValueError("未找到 conf == 1 的 GT 轨迹")
    if len(pred_trajs) == 0:
        raise ValueError("未找到预测轨迹")

    # --- 权重归一化 ---
    if pred_w.sum() == 0:
        raise ValueError("预测轨迹的权重总和为 0")
    pred_w /= pred_w.sum()

    # --- 对齐长度（截断到最短） ---
    lens_gt = [t.shape[0] for t in gt_trajs]
    lens_pred = [t.shape[0] for t in pred_trajs]
    min_len = min(min(lens_gt), min(lens_pred))

    gt_trajs = [t[:min_len] for t in gt_trajs]
    pred_trajs = [t[:min_len] for t in pred_trajs]

    # --- 计算 blended 预测 ---
    blended = (np.stack(pred_trajs, axis=0) * pred_w[:, None, None]).sum(axis=0)

    # --- 绘制 GT（红色实线）---
    for i, gt in enumerate(gt_trajs):
        ax.plot(
            gt[:, 1],
            gt[:, 0],
            line_type,
            label=f"GT{i}" if len(gt_trajs) > 1 else "GT",
            linewidth=2,
            color="#d62728",
        )

    # --- 绘制 blended 预测（蓝色粗线）---
    ax.plot(
        blended[:, 1],
        blended[:, 0],
        line_type,
        label="blended_pred",
        linewidth=3,
    )

    if xlim is not None:
        ax.set_xlim(*xlim)
    if ylim is not None:
        ax.set_ylim(*ylim)

    ax.legend(loc="lower left")
    return ax


def draw_path(
        img: np.ndarray,
        img_pts_l: np.ndarray,
        img_pts_r: np.ndarray,
        fill_color=(255, 255, 255),
        line_color=(0, 255, 0),
        alpha=0.5
) -> np.ndarray:
    """
    在 img 上用半透明填充色 + 不透明线条画出左右两条轨迹连成的四边形，
    返回一张新图，不影响原图 img。忽略任何包含负坐标的片段。
    """
    import cv2
    import numpy as np

    # 复制一份作为输出
    out = img.copy()

    # 收集所有有效多边形顶点
    polygons = []
    for i in range(1, len(img_pts_l)):
        u1, v1 = img_pts_l[i - 1]
        u2, v2 = img_pts_r[i - 1]
        u3, v3 = img_pts_l[i]
        u4, v4 = img_pts_r[i]
        if min(u1, v1, u2, v2, u3, v3, u4, v4) < 0:
            continue
        pts = np.array([[u1, v1], [u2, v2], [u4, v4], [u3, v3]],
                       dtype=np.int32).reshape((-1, 1, 2))
        polygons.append(pts)

    # 半透明填充
    if fill_color and polygons:
        overlay = out.copy()
        cv2.fillPoly(overlay, polygons, fill_color)
        # 融合到 out（不修改原 img）
        out = cv2.addWeighted(overlay, alpha, out, 1 - alpha, 0)

    # 不透明画边框
    if line_color and polygons:
        for pts in polygons:
            cv2.polylines(out, [pts], True, line_color)

    return out


import numpy as np
import cv2
from typing import Tuple


def draw_line(
        img: np.ndarray,
        trajectories: np.ndarray | list,  # (num_pts,2) 或 (n,num_pts,2)；支持 list 且各条长度不同
        visibility: np.ndarray | list | None = None,  # (num_pts) 或 (n,num_pts)；list 可与各条长度匹配
        line_color: Tuple[int, int, int] | None = None,  # BGR, e.g. (255,0,0)
) -> np.ndarray:
    """
    在图像上绘制轨迹；对 visibility==1 的点画圆并标注点的索引（与线段颜色的反色）。
    - trajectories: 单条 (T,2) / 多条 (N,T,2)；或 list[ (Ti,2) ]（Ti 可不同）。
    - visibility:   (T,) / (N,T)；或 list[ (Ti,) ]，None 表示所有点可见（填充点(-1,-1)会被过滤）。
    - line_color:   None 则按轨迹索引自动配色，否则所有轨迹同色。
    - 额外规则：过滤坐标恰为 (-1, -1) 的点（不连线、不画点、不标注）。
    返回绘制后的新图像（不会改动原始 img）。
    """
    if img is None or img.ndim != 3 or img.shape[2] != 3:
        raise ValueError("img 必须为 HxWx3 的 BGR 图像")

    out = img.copy()

    # ---------- 规整 trajectories：允许 list 且各条长度不同 ----------
    T_list = None  # 记录每条轨迹的原始长度
    if isinstance(trajectories, np.ndarray):
        traj = trajectories.astype(float)
        if traj.ndim == 2 and traj.shape[1] == 2:
            traj = traj[None, ...]  # (1, T, 2)
        if traj.ndim != 3 or traj.shape[2] != 2:
            raise ValueError("trajectories 形状应为 (T,2) 或 (N,T,2) 或 list[(Ti,2)]")
        N, T, _ = traj.shape
        T_list = [T] * N
    elif isinstance(trajectories, (list, tuple)):
        traj_list = []
        for k, t in enumerate(trajectories):
            arr = np.asarray(t, dtype=float)
            if arr.ndim == 1:
                # 允许 [x, y] 这种，视作单点
                if arr.size == 2:
                    arr = arr.reshape(1, 2)
                else:
                    raise ValueError(f"第{k}条轨迹维度不合法：期望 (Ti,2) 或 (2,)")
            if arr.ndim != 2 or arr.shape[1] != 2:
                raise ValueError(f"第{k}条轨迹形状应为 (Ti,2)，得到 {arr.shape}")
            traj_list.append(arr)
        N = len(traj_list)
        if N == 0:
            return out
        T_list = [a.shape[0] for a in traj_list]
        Tmax = max(T_list)
        # 用 (-1,-1) 补齐到统一长度（稍后会被 valid_mask 过滤）
        traj = np.full((N, Tmax, 2), -1.0, dtype=float)
        for i, a in enumerate(traj_list):
            L = a.shape[0]
            traj[i, :L, :] = a
        T = Tmax
    else:
        raise ValueError("trajectories 类型应为 numpy 数组或 list/tuple")

    # ---------- 处理 visibility：允许与每条轨迹长度分别对应 ----------
    if visibility is None:
        vis = np.ones((N, T), dtype=bool)
        # 也可以选择把补齐处设为 False，但 (-1,-1) 本就会被 valid_mask 过滤
    else:
        if isinstance(visibility, np.ndarray):
            v = visibility
            if v.ndim == 1:
                v = v.astype(bool).ravel()
                vis = np.zeros((N, T), dtype=bool)
                for i in range(N):
                    Li = T_list[i]
                    L = min(Li, v.shape[0])
                    vis[i, :L] = v[:L]
            elif v.ndim == 2:
                if v.shape[0] != N:
                    raise ValueError(f"visibility 第一维应与轨迹数相同：期望 {N}，得到 {v.shape[0]}")
                vis = np.zeros((N, T), dtype=bool)
                v = v.astype(bool)
                for i in range(N):
                    Li = T_list[i]
                    L = min(Li, v.shape[1], T)
                    vis[i, :L] = v[i, :L]
            else:
                raise ValueError("visibility 形状应为 (T,) 或 (N,T) 或 list[(Ti,)]")
        elif isinstance(visibility, (list, tuple)):
            if len(visibility) != N:
                raise ValueError(f"visibility 条数应与轨迹数相同：期望 {N}，得到 {len(visibility)}")
            vis = np.zeros((N, T), dtype=bool)
            for i, vv in enumerate(visibility):
                v_arr = np.asarray(vv).astype(bool).ravel()
                Li = T_list[i]
                L = min(Li, v_arr.shape[0])
                vis[i, :L] = v_arr[:L]
        else:
            raise ValueError("visibility 类型应为 numpy 数组或 list/tuple 或 None")

    # 将坐标转为整数像素（四舍五入）
    pts = np.rint(traj).astype(np.int32)  # (N,T,2)

    # 构建有效掩码
    finite_mask = np.isfinite(traj[..., 0]) & np.isfinite(traj[..., 1])
    neg1_mask = (pts[..., 0] == -1) & (pts[..., 1] == -1)
    valid_mask = finite_mask & (~neg1_mask)

    # 绘制
    line_thickness = 2
    point_radius = 3
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.5
    text_thickness = 1
    text_offset = (4, -4)  # (dx, dy)

    def _auto_bgr(i: int) -> Tuple[int, int, int]:
        """根据轨迹索引 i 生成稳定可区分的 BGR 颜色（HSV 均匀取色）。"""
        h = (i * 37) % 180  # OpenCV HSV 的 H 范围是 [0,180)
        s, v = 200, 255
        hsv = np.uint8([[[h, s, v]]])
        bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0]
        return int(bgr[0]), int(bgr[1]), int(bgr[2])

    for i in range(N):
        # 线颜色
        if line_color is not None:
            lc = tuple(int(c) for c in line_color)
        else:
            lc = _auto_bgr(i)

        # 点/文字颜色：线颜色反色
        inv = (255 - lc[0], 255 - lc[1], 255 - lc[2])

        # 画线段（仅连接相邻且坐标有效的点）
        for t in range(T - 1):
            if valid_mask[i, t] and valid_mask[i, t + 1]:
                p1 = tuple(pts[i, t])
                p2 = tuple(pts[i, t + 1])
                cv2.line(out, p1, p2, lc, thickness=line_thickness, lineType=cv2.LINE_AA)

        # 画点与标注（仅当该点可见且坐标有效）
        for t in range(T):
            if vis[i, t] and valid_mask[i, t]:
                center = tuple(pts[i, t])
                cv2.circle(out, center, point_radius, inv, thickness=-1, lineType=cv2.LINE_AA)
                org = (center[0] + text_offset[0], center[1] + text_offset[1])
                cv2.putText(out, str(t), org, font, font_scale, inv, thickness=text_thickness, lineType=cv2.LINE_AA)

    return out


# 全局缓存
_fig: plt.Figure | None = None
_axes: dict[str, plt.Axes] | None = None


def _init_fig_axes() -> tuple[plt.Figure, dict[str, plt.Axes]]:
    """
    只在首次调用时创建 Figure、Axes 并调用 tight_layout，
    并缓存到全局变量。后续调用直接复用。
    """
    global _fig, _axes
    if _fig is None or _axes is None:
        _fig = plt.figure(figsize=(12, 9))
        spec = _fig.add_gridspec(3, 3)
        ax_prev = _fig.add_subplot(spec[2, 0])
        ax_curr = _fig.add_subplot(spec[2, 1])
        ax_curve = _fig.add_subplot(spec[:, 2])
        ax_overlay = _fig.add_subplot(spec[0:2, 0:2])
        plt.tight_layout()  # 只执行一次
        _axes = {
            "ax_prev": ax_prev,
            "ax_curr": ax_curr,
            "ax_curve": ax_curve,
            "ax_overlay": ax_overlay,
        }
    return _fig, _axes  # type: ignore


def visualize_prediction(
        img_0: np.ndarray,
        img_1: np.ndarray,
        trj: np.ndarray,
        labels: np.ndarray,
        img_pts_l: np.ndarray,
        img_pts_r: np.ndarray,
        pred_conf: np.ndarray,
        origin_img: np.ndarray,
        gt_pts_l: np.ndarray | None = None,
        gt_pts_r: np.ndarray | None = None,
        model: str = "separate",
) -> np.ndarray:
    """
    可视化轨迹预测结果，返回一张 H×W×3 的 uint8 图像
    """
    # ---------- 选取要可视化的轨迹 ----------
    if model == "separate":
        best_idx = int(pred_conf.argmax())
        chosen_trj = trj[best_idx]  # (T,3)
        chosen_pts_l = img_pts_l[best_idx]  # (T,2)
        chosen_pts_r = img_pts_r[best_idx]  # (T,2)
    else:
        w = pred_conf.astype(float)
        if w.sum() == 0:
            raise ValueError("pred_conf sum is zero")
        w /= w.sum()
        chosen_trj = (trj * w[:, None, None]).sum(axis=0)
        chosen_pts_l = (img_pts_l * w[:, None, None]).sum(axis=0)
        chosen_pts_r = (img_pts_r * w[:, None, None]).sum(axis=0)

    current_metric = np.linalg.norm(chosen_trj - labels, axis=-1).mean()

    # ---------- 获取或初始化布局 ----------
    fig, axes = _init_fig_axes()
    ax_prev = axes["ax_prev"]
    ax_curr = axes["ax_curr"]
    ax_curve = axes["ax_curve"]
    ax_overlay = axes["ax_overlay"]

    # 清空旧内容
    for ax in (ax_prev, ax_curr, ax_curve, ax_overlay):
        ax.clear()

    # 左下：网络前一帧
    ax_prev.imshow(img_0)
    ax_prev.set_title("network input [previous]")
    ax_prev.axis("off")

    # 中下：网络当前帧
    ax_curr.imshow(img_1)
    ax_curr.set_title("network input [current]")
    ax_curr.axis("off")

    # 右侧：轨迹曲线
    trajectories = list(trj) + [labels]
    confs = list(pred_conf) + [1.0]
    if model == "separate":
        draw_trajectory_on_ax(ax_curve, trajectories, confs, ylim=(0, 200))
    else:
        draw_blended_trajectory_on_ax(ax_curve, trajectories, confs, ylim=(0, 200))
    ax_curve.set_title(f"Mean L2: {current_metric:.2f}")
    ax_curve.grid()

    # 左上：原图叠加轨迹
    vis_img = draw_path(
        origin_img,
        chosen_pts_l,
        chosen_pts_r,
        fill_color=(255, 255, 255),
        line_color=(0, 255, 0),
    )
    if gt_pts_l is not None and gt_pts_r is not None:
        vis_img = draw_path(
            vis_img, gt_pts_l, gt_pts_r,
            fill_color=None,
            line_color=(255, 165, 0),
        )
    ax_overlay.imshow(vis_img)
    ax_overlay.set_title("Project on current frame")
    ax_overlay.axis("off")

    # canvas 转 numpy
    fig.canvas.draw()
    w, h = fig.canvas.get_width_height()
    buf = np.frombuffer(fig.canvas.buffer_rgba(), dtype=np.uint8)
    out_img = buf.reshape(h, w, 4)[..., :3]

    return out_img


import numpy as np


def boxes_to_corners3d(boxes: np.ndarray) -> np.ndarray:
    """
    boxes: (N,7) = (cx,cy,cz,l,w,h,theta)
    返回:  (N,8,3) 8个角点，顺序与 draw_detections_2d 的 edges 对应
    约定: 长度沿 x，宽度沿 y，高度沿 z；绕 z 轴旋转 yaw=theta（弧度）。
    """
    if boxes is None:
        return np.zeros((0, 8, 3), dtype=np.float32)

    boxes = np.asarray(boxes, dtype=np.float32)
    if boxes.ndim == 1:
        boxes = boxes[None, :]
    if boxes.size == 0:
        return np.zeros((0, 8, 3), dtype=np.float32)
    if boxes.shape[1] != 7:
        raise ValueError(f"boxes shape must be (N,7), got {boxes.shape}")

    cx, cy, cz, l, w, h, theta = [boxes[:, i] for i in range(7)]

    # 局部坐标系下的 8 点（以中心为原点）
    # 底面 z = -h/2，顶面 z = +h/2
    x_c = np.stack([-l / 2, l / 2, l / 2, -l / 2, -l / 2, l / 2, l / 2, -l / 2], axis=1)
    y_c = np.stack([-w / 2, -w / 2, w / 2, w / 2, -w / 2, -w / 2, w / 2, w / 2], axis=1)
    z_c = np.stack([-h / 2, -h / 2, -h / 2, -h / 2, h / 2, h / 2, h / 2, h / 2], axis=1)

    # 旋转 + 平移（yaw = theta）
    cos_y = np.cos(theta)[:, None]
    sin_y = np.sin(theta)[:, None]
    x_w = cos_y * x_c - sin_y * y_c + cx[:, None]
    y_w = sin_y * x_c + cos_y * y_c + cy[:, None]
    z_w = z_c + cz[:, None]

    corners = np.stack([x_w, y_w, z_w], axis=-1).astype(np.float32)  # (N,8,3)
    return corners
