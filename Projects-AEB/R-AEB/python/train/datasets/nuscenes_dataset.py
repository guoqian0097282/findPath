# nuscenes_scene_dataset.py
# -*- coding: utf-8 -*-
"""
nuScenes 场景级（sample=整段 scene）前视单目 3D 检测数据集
- 坐标轴：nuScenes camera(x右,y下,z前) → cimg camera(x前,y左,z上) （R_fix 已修正）
- 3D 框：global → ego；所有过滤（FOV/BEV）在 ego 系下进行
- 组件化：LoadSceneCAMFront / BuildCalibForCImg / LoadDet3DNuScenes / NuscMapExtractorComponent / CImgProcess / AlignLength
- sample 级并行（ProcessPoolExecutor + 优先级队列）
- collate 返回含 t 维 (T,B,...)
依赖：
  pip install nuscenes-devkit pyquaternion torch numpy opencv-python shapely
"""

from __future__ import annotations

import gzip
import hashlib
import json
import math
import os
import pickle
import random
import sys
from dataclasses import dataclass
from typing import Dict, Any, List, Tuple
from typing import Optional

import cv2
import numpy as np
import torch
from nuscenes.map_expansion.map_api import NuScenesMap, NuScenesMapExplorer
from nuscenes.nuscenes import NuScenes
from nuscenes.utils.splits import create_splits_scenes
from pyquaternion import Quaternion
from shapely import ops, strtree
from shapely.geometry import LineString, Polygon, LinearRing, box

from visper.common.cimg import CalibratedImage, ImageProcessor
from visper.common.logger import logger
from .dataset_utils.prefetch_dataset import SamplePrefetchDataset

ID2NAME = {0: "Background",
           1: "Car",
           2: "Truck",
           3: "Bus",
           4: "Pedestrian",
           5: "Cyclist"}

NAME2ID = {name: i for i, name in ID2NAME.items()}


# ---------------- 数据容器 ----------------
@dataclass
class Data:
    scene_name: str | None = None
    scene_token: str | None = None

    video_path: str | None = None
    ori_img_size: Tuple[int, int] | None = None  # (H, W)
    sd_tokens: List[str] | None = None
    img_paths: List[str] | None = None

    cam_intrinsics: np.ndarray | None = None  # (T,3,3)
    ego_from_sensor: np.ndarray | None = None  # (T,4,4) sensor->ego
    ego_pose: np.ndarray | None = None  # (T,4,4) ego->global

    big_cimg: list | None = None
    ori_cimg: list | None = None

    small_img_data: torch.Tensor | None = None  # (T,3,H,W) float16
    big_img_data: torch.Tensor | None = None

    # 3D 框改为 7 维：最后一维为 theta（弧度）
    det3d_box: torch.Tensor | None = None  # (T,K,7) float32 (cx,cy,cz,l,w,h,theta)  (ego)
    det3d_cls: torch.Tensor | None = None  # (T,K)   int64   类别 id（-1 padding）

    calib: dict | None = None

    det2d_box: torch.Tensor | None = None  # (T,K,4) float32, xyxy (big 图)


# ---------------- SE(3) 工具 ----------------
def _se3(translation: List[float], rot_wxyz: List[float]) -> np.ndarray:
    R = Quaternion(rot_wxyz).rotation_matrix
    M = np.eye(4, dtype=np.float64)
    M[:3, :3] = R
    M[:3, 3] = np.array(translation, dtype=np.float64)
    return M


class NuScenesCachedLoader:
    """
    带指纹缓存的 NuScenes 加载器工厂类。

    使用方式：
        nusc = NuScenesCachedLoader(
            version="v1.0-trainval",
            dataroot="/path/to/all_nuScenes",
            use_gzip=True,
            cache_dir=None,
            verbose=True,
        )

    注意：__new__ 直接返回 NuScenes 实例，本类不会真正被实例化成 Loader 对象。
    """

    def __new__(
            cls,
            version: str,
            dataroot: str,
            use_gzip: bool = True,
            cache_dir: str | None = None,
            verbose: bool = True,
    ) -> "NuScenes":
        cache_root = cache_dir or os.path.join(dataroot, ".cache")
        os.makedirs(cache_root, exist_ok=True)

        fp = cls._fingerprint(dataroot, version)
        cache_name = (
                f"nuscenes_{version}_{fp['hash']}.pkl"
                + (".gz" if use_gzip else "")
        )
        cache_path = os.path.join(cache_root, cache_name)

        # 1) 命中缓存：直接读
        if os.path.isfile(cache_path):
            logger.info("Load NuScenes from cache: %s", cache_path)
            with (
                    gzip.open(cache_path, "rb")
                    if use_gzip
                    else open(cache_path, "rb")
            ) as f:
                payload = pickle.load(f)
            state = payload["state"]
            return cls._rebuild_from_state(state)

        # 2) 未命中：构建 + 写入缓存
        logger.info("Build NuScenes (first time) ...")

        nusc = NuScenes(
            version=version,
            dataroot=dataroot,
            verbose=False,
        )

        state = cls._extract_picklable_state(nusc)
        payload = {"fingerprint": fp, "state": state}
        tmp = cache_path + ".tmp"

        with (
                gzip.open(tmp, "wb")
                if use_gzip
                else open(tmp, "wb")
        ) as f:
            pickle.dump(payload, f, protocol=pickle.HIGHEST_PROTOCOL)

        os.replace(tmp, cache_path)
        logger.info("NuScenes cached → %s", cache_path)
        return nusc

    # ====================== 内部辅助方法 ======================

    @staticmethod
    def _walk_json_files(dataroot: str, version: str) -> List[str]:
        """收集该版本目录下所有 .json 文件。"""
        version_dir = os.path.join(dataroot, version)
        out: List[str] = []
        if not os.path.isdir(version_dir):
            return out
        for name in os.listdir(version_dir):
            if name.endswith(".json"):
                out.append(os.path.join(version_dir, name))
        return sorted(out)

    @classmethod
    def _fingerprint(cls, dataroot: str, version: str) -> Dict[str, Any]:
        """
        构造指纹：
            - dataroot / version
            - 所有 json 的 (文件名, mtime, size)
            - python 版本、nuscenes_devkit 模块名
        """
        files = cls._walk_json_files(dataroot, version)
        stats: List[Tuple[str, int, int]] = []

        for fp in files:
            try:
                st = os.stat(fp)
                stats.append(
                    (os.path.basename(fp), int(st.st_mtime), int(st.st_size))
                )
            except FileNotFoundError:
                stats.append((os.path.basename(fp), 0, 0))

        env = {
            "python": sys.version.split()[0],
            "nuscenes_devkit": getattr(NuScenes, "__module__", "nuscenes"),
        }

        payload: Dict[str, Any] = {
            "version": version,
            "dataroot": os.path.abspath(dataroot),
            "files": stats,
            "env": env,
        }
        payload_bytes = json.dumps(payload, sort_keys=True).encode("utf-8")
        payload["hash"] = hashlib.md5(payload_bytes).hexdigest()
        return payload

    @staticmethod
    def _extract_picklable_state(nusc: NuScenes) -> Dict[str, Any]:
        """只保留易序列化的简单字段（dict/list/tuple/标量等）。"""
        keep: Dict[str, Any] = {}
        for k, v in nusc.__dict__.items():
            if isinstance(
                    v,
                    (dict, list, tuple, str, int, float, bool, type(None)),
            ):
                keep[k] = v
        return keep

    @staticmethod
    def _rebuild_from_state(state: Dict[str, Any]) -> NuScenes:
        """通过 __new__ + 直接赋 __dict__ 的方式，还原一个 NuScenes 实例。"""
        obj = NuScenes.__new__(NuScenes)
        obj.__dict__.update(state)
        return obj


# ========== 组件1：加载 scene 的 CAM_FRONT 序列 ==========
class LoadSceneCAMFront:
    def __init__(self, cam_channel: str = "CAM_FRONT", max_length: int = 2000):
        self.cam = cam_channel
        self.max_length = max_length

    def __call__(self, data: Data, item: Dict[str, Any]) -> Data:
        global g_nusc
        scene_token = item["scene_token"]
        scene = g_nusc.get("scene", scene_token)

        sd_tokens, img_paths = [], []
        intrs, T_se2ego, T_ego2global = [], [], []

        tok = scene["first_sample_token"]
        while tok and len(sd_tokens) < self.max_length:
            sample = g_nusc.get("sample", tok)
            sd_tok = sample["data"].get(self.cam, None)
            if sd_tok is not None:
                sd = g_nusc.get("sample_data", sd_tok)
                img_path = os.path.join(g_nusc.dataroot, sd["filename"])
                cs = g_nusc.get("calibrated_sensor", sd["calibrated_sensor_token"])
                ep = g_nusc.get("ego_pose", sd["ego_pose_token"])

                intrs.append(np.array(cs["camera_intrinsic"], dtype=np.float64))
                T_se2ego.append(_se3(cs["translation"], cs["rotation"]))  # sensor->ego
                T_ego2global.append(_se3(ep["translation"], ep["rotation"]))  # ego->global

                sd_tokens.append(sd_tok)
                img_paths.append(img_path)

            tok = sample["next"]

        if not sd_tokens:
            raise RuntimeError(f"Scene {scene['name']} 无 {self.cam} 数据")

        first = cv2.imread(img_paths[0], cv2.IMREAD_COLOR)
        if first is None:
            raise RuntimeError(f"读取失败: {img_paths[0]}")
        H, W = first.shape[:2]

        data.scene_token = scene_token
        data.scene_name = scene["name"]
        data.video_path = data.scene_name
        data.sd_tokens = sd_tokens
        data.img_paths = img_paths
        data.ori_img_size = (H, W)

        data.cam_intrinsics = np.stack(intrs, axis=0)
        data.ego_from_sensor = np.stack(T_se2ego, axis=0)
        data.ego_pose = np.stack(T_ego2global, axis=0)
        return data


# ========== 组件2：构建 cimg calib（世界=ego），sample 级缓存 ==========
class BuildCalibForCImg:
    """
    使用第一帧 calibrated_sensor + 图像尺寸，构建 CalibratedImage 所需的 calib（世界系=ego）。
    R_passive = R_se @ R_fix，其中：
      - R_se : sensor->ego（nuScenes）
      - R_fix: cimg(x前,y左,z上) → nuScenes camera(x右,y下,z前) 的**被动**旋转
        正确的 R_fix（列向量为 cimg 基向量在 nuScenes 相机系中的坐标）：
          ex_cimg(1,0,0)->(0,0,1) ; ey_cimg(0,1,0)->(-1,0,0) ; ez_cimg(0,0,1)->(0,-1,0)
          R_fix = [[0,-1, 0],
                   [0, 0,-1],
                   [1, 0, 0]]
    """

    def __init__(self, cam_channel: str = "CAM_FRONT"):
        self.cam = cam_channel
        self._cache: dict[tuple[str, int, int], dict] = {}
        self.R_fix = np.array([[0., -1., 0.],
                               [0., 0., -1.],
                               [1., 0., 0.]], dtype=np.float64)

    def __call__(self, data: Data, item: Dict[str, Any]) -> Data:
        global g_nusc
        sd0 = g_nusc.get("sample_data", data.sd_tokens[0])
        cs = g_nusc.get("calibrated_sensor", sd0["calibrated_sensor_token"])

        img0 = cv2.imread(data.img_paths[0], cv2.IMREAD_COLOR)
        if img0 is None:
            raise RuntimeError(f"读取失败: {data.img_paths[0]}")
        H, W = img0.shape[:2]
        data.ori_img_size = (H, W)

        key = (cs["token"], W, H)
        if key in self._cache:
            data.calib = self._cache[key]
            return data

        K = np.array(cs["camera_intrinsic"], dtype=np.float64)
        fu, fv = float(K[0, 0]), float(K[1, 1])
        cu, cv = float(K[0, 2]), float(K[1, 2])

        R_se = Quaternion(cs["rotation"]).rotation_matrix
        t_se = np.array(cs["translation"], dtype=np.float64)

        # 核心：cimg 的被动旋转（camera->world(ego)）
        R_passive = R_se @ self.R_fix
        yaw, pitch, roll = self._euler_zyx_from_R(R_passive)

        hfov = 2.0 * math.atan2(W * 0.5, fu)

        calib = {
            "world_x": float(t_se[0]),  # 世界=ego
            "world_y": float(t_se[1]),
            "world_z": float(t_se[2]),
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "image_width": W,
            "image_height": H,
            "focal_u": fu,
            "focal_v": fv,
            "center_u": cu,
            "center_v": cv,
            "pinhole_distort": (0, 0, 0, 0),
            "fov": float(hfov),
            "type": 0,
            "image_type": 'BGR',
        }
        data.calib = calib
        self._cache[key] = calib
        return data

    def _euler_zyx_from_R(self, R: np.ndarray) -> tuple[float, float, float]:
        """R ≈ Rz(yaw) @ Ry(pitch) @ Rx(roll)"""
        sy = -R[2, 0]
        cy = math.sqrt(max(0.0, 1.0 - sy * sy))
        if cy > 1e-6:
            yaw = math.atan2(R[1, 0], R[0, 0])
            pitch = math.asin(sy)
            roll = math.atan2(R[2, 1], R[2, 2])
        else:
            yaw = math.atan2(-R[0, 1], R[1, 1])
            pitch = math.asin(sy)
            roll = 0.0
        return float(yaw), float(pitch), float(roll)


# ========== 组件3：加载 3D 框（global→ego），并在 ego 中做过滤 ==========
class LoadDet3DNuScenes:
    """
    将 nuScenes 原始类别合并为 6 类，并在 ego 坐标系下做 BEV + HFOV 过滤。
    仅输出以下 6 类（背景类只作为占位，不产出框）：
        "Background":0, "Car":1, "Truck":2, "Bus":3, "Pedestrian":4, "Cyclist":5
    合并规则：
        - trailer, construction_* -> Truck
        - motorcycle, bicycle     -> Cyclist
        - traffic_cone, barrier   -> Background（跳过）
        - 其它未映射前缀          -> Background（跳过）
    """

    def __init__(self,
                 k_max: int = 80,
                 hfov_deg: float = 70.0,
                 x_range: Tuple[float, float] = (0.0, 80.0),
                 y_range: Tuple[float, float] = (-30.0, 30.0)):
        self.k_max = int(k_max)
        self.hfov = float(hfov_deg) * math.pi / 180.0
        self.xr = x_range
        self.yr = y_range
        self.cat_reduce_map = self._make_reduce_map()  # 原始前缀 -> 目标类ID（0..5）

    # ---------- 类别合并：nuScenes 前缀 -> 目标6类ID ----------
    def _make_reduce_map(self) -> Dict[str, int]:
        """
        返回一个“前缀到目标类ID”的映射。未命中时默认为 Background(0)。
        """
        m: Dict[str, int] = {}

        def add(prefix: str, target_name: str) -> None:
            m[prefix] = NAME2ID[target_name]

        add("vehicle.car", "Car")
        add("vehicle.truck", "Truck")
        add("vehicle.bus", "Bus")
        add("vehicle.trailer", "Truck")  # 合并为 Truck
        add("vehicle.construction", "Truck")  # 合并为 Truck
        add("vehicle.motorcycle", "Cyclist")  # 两轮统一为 Cyclist
        add("vehicle.bicycle", "Cyclist")
        add("human.pedestrian", "Pedestrian")
        add("movable_object.trafficcone", "Background")  # 背景：不产出框
        add("static_object.barrier", "Background")  # 背景：不产出框
        # 其它未覆盖的前缀 -> Background（在查找函数里兜底）
        return m

    def _reduce_id(self, cat: str) -> int:
        """
        输入 nuScenes 的 category_name（如 'vehicle.car'），输出目标类ID（0..5）。
        未匹配的前缀一律视为 Background(0)。
        """
        for prefix, cid in self.cat_reduce_map.items():
            if cat.startswith(prefix):
                return cid
        return NAME2ID["Background"]

    # ---------- 工具 ----------
    @staticmethod
    def _yaw_from_rotmat(R: np.ndarray) -> float:
        # ego 坐标下绕 z 的偏航角
        return math.atan2(R[1, 0], R[0, 0])

    @staticmethod
    def _invert(T: np.ndarray) -> np.ndarray:
        R = T[:3, :3]
        t = T[:3, 3]
        Ti = np.eye(4, dtype=T.dtype)
        Ti[:3, :3] = R.T
        Ti[:3, 3] = -R.T @ t
        return Ti

    # ---------- 主流程 ----------
    def __call__(self, data: Data, item: Dict[str, Any]) -> Data:
        global g_nusc

        T = len(data.sd_tokens)
        boxes_tensor = torch.zeros((T, self.k_max, 7), dtype=torch.float32)  # (cx,cy,cz,l,w,h,theta)
        names_tensor = torch.full((T, self.k_max), fill_value=-1, dtype=torch.int64)  # -1 为 padding

        tan_h = math.tan(self.hfov * 0.5)

        for t, sd_tok in enumerate(data.sd_tokens):
            sd = g_nusc.get("sample_data", sd_tok)
            sample = g_nusc.get("sample", sd["sample_token"])
            anns = sample.get("anns", [])
            if not isinstance(anns, list):
                # 严格规则：该帧 annos 缺失/非列表，valid=0，保持 padding
                continue

            ep = g_nusc.get("ego_pose", sd["ego_pose_token"])
            cs = g_nusc.get("calibrated_sensor", sd["calibrated_sensor_token"])

            # global -> ego
            T_ego_from_global = self._invert(_se3(ep["translation"], ep["rotation"]))
            R_se2ego = Quaternion(cs["rotation"]).rotation_matrix
            t_se2ego = np.array(cs["translation"], dtype=np.float64)

            # 相机在 ego 下的水平前向（nuScenes 相机 +z 朝前）
            f_e = R_se2ego @ np.array([0.0, 0.0, 1.0], dtype=np.float64)
            yaw_cam = math.atan2(f_e[1], f_e[0])
            cos_y, sin_y = math.cos(yaw_cam), math.sin(yaw_cam)

            keep_list: List[Tuple[np.ndarray, int]] = []

            for ann_tok in anns:
                ann = g_nusc.get("sample_annotation", ann_tok)

                # 1) 类别合并为 6 类
                cls_id = self._reduce_id(ann["category_name"])
                if cls_id == NAME2ID["Background"]:
                    # 背景/未映射的不产生框
                    continue

                # 2) box: global -> ego
                T_box_g = _se3(ann["translation"], ann["rotation"])
                T_box_e = T_ego_from_global @ T_box_g
                c_e = T_box_e[:3, 3]  # 中心（ego）
                R_e = T_box_e[:3, :3]  # 姿态（ego）

                # 尺寸 (w,l,h) -> (l,w,h)，yaw 取绕 z
                w, l, h = ann["size"]
                lwh = np.array([l, w, h], dtype=np.float32)
                yaw = self._yaw_from_rotmat(R_e)

                # 3) 过滤（ego 系）
                #    FOV：以相机朝向坐标（前向 v_fx、侧向 v_lat）判断
                v = c_e - t_se2ego
                v_fx = v[0] * cos_y + v[1] * sin_y
                v_lat = -v[0] * sin_y + v[1] * cos_y
                in_fov = (v_fx > 0.0) and (abs(v_lat) <= v_fx * tan_h)

                # BEV 距离/左右界
                in_range = (self.xr[0] <= c_e[0] <= self.xr[1]) and (self.yr[0] <= c_e[1] <= self.yr[1])

                if in_fov and in_range:
                    b7 = np.array([c_e[0], c_e[1], c_e[2], lwh[0], lwh[1], lwh[2], yaw], dtype=np.float32)
                    keep_list.append((b7, cls_id))

            if keep_list:
                K = min(len(keep_list), self.k_max)
                for k in range(K):
                    boxes_tensor[t, k] = torch.from_numpy(keep_list[k][0])
                    names_tensor[t, k] = int(keep_list[k][1])  # 1..5

        # 写回 Data
        data.det3d_box = boxes_tensor
        data.det3d_cls = names_tensor
        return data


# ========== 组件：地图（全部在类里实现，扇形为 +x 全角，坐标=ego）==========
class NuscMapExtractorComponent:
    """
    nuScenes 地图要素提取组件（统一输出到 ego 局部坐标：x 前、y 左、z 上）。
    - 依赖：data.ego_pose[t] 是 ego->global 的 4x4；data.scene_token 用于查 log.location。
    - 输出写回：
        data.map_divider[t]      : List[np.ndarray(Ni,2)]    车道/道路分隔线（ego）
        data.map_boundary[t]     : List[np.ndarray(Nj,2)]    可行驶区域边界（ego）
        data.map_ped_crossing[t] : List[np.ndarray(Nk,2)]    人行横道外轮廓（闭合，ego）
    - 过滤顺序（均在 ego 平面中进行）：
        先矩形裁剪（ego 系 (x_min,x_max,y_min,y_max)），再可选“围绕 +x 方向的扇形”裁剪（参数为**全角**）。
    - 关键修复：
        使用 3D→2D 的等效仿射：对 global 平面点 (x,y,0)，
          p_ego = R_eg2g^T @ (p_glb - t_ego)  ⇒
          [x',y'] = Rt[:2,:2]@[x,y] + ( -Rt[:2,:2]@[tx,ty] - Rt[:2,2]*tz )
        将 z 的影响项 Rt[:2,2]*tz 吸收到 2D 平移，避免“整体固定偏移”。
    """

    def __init__(self,
                 dataroot: str,
                 rect_bounds: Optional[Tuple[float, float, float, float]] = (-60.0, 60.0, -5.0, 60.0),
                 sector_full_angle_deg: Optional[float] = 150.0,  # 扇形全角（围绕 +x）
                 cache_xy_meter: float = 2.0,
                 cache_yaw_deg: float = 5.0):
        self.dataroot = dataroot
        self.rect_bounds = rect_bounds
        self.sector_full_angle_deg = sector_full_angle_deg
        self.cache_xy_meter = float(cache_xy_meter)
        self.cache_yaw_deg = float(cache_yaw_deg)

        # 预载 4 个城市地图
        self._maps: Dict[str, NuScenesMap] = {}
        self._explorers: Dict[str, NuScenesMapExplorer] = {}
        for loc in ("boston-seaport",
                    "singapore-hollandvillage",
                    "singapore-onenorth",
                    "singapore-queenstown"):
            nmap = NuScenesMap(dataroot=dataroot, map_name=loc)
            self._maps[loc] = nmap
            self._explorers[loc] = NuScenesMapExplorer(nmap)

        # 结果缓存：按 (location, qx, qy, qyaw) 量化
        self._cache: Dict[Tuple[str, int, int, int], Dict[str, List[np.ndarray]]] = {}

    # ================== 类内私有工具 ==================

    @staticmethod
    def _yaw_from_R_ego_to_global(R: np.ndarray) -> float:
        """从 ego->global 的 3x3 提取平面 yaw（弧度，CCW，x 前、y 左）。"""
        return math.atan2(R[1, 0], R[0, 0])

    def _quant_key(self, location: str, x: float, y: float, yaw_deg: float) -> Tuple[str, int, int, int]:
        qx = int(round(x / self.cache_xy_meter))
        qy = int(round(y / self.cache_xy_meter))
        qyaw = int(round(yaw_deg / self.cache_yaw_deg))
        return (location, qx, qy, qyaw)

    def _estimate_patch_len(self, rect_bounds: Optional[Tuple[float, float, float, float]],
                            default_xy: Tuple[float, float] = (200.0, 200.0)) -> Tuple[float, float]:
        """NuScenes patch 需要 (len_x,len_y)。若有 rect_bounds 则按范围扩 1m。"""
        if rect_bounds is None:
            return default_xy
        x_min, x_max, y_min, y_max = rect_bounds
        len_x = 2.0 * max(abs(x_min), abs(x_max))
        len_y = 2.0 * max(abs(y_min), abs(y_max))
        return (len_x + 1.0, len_y + 1.0)

    def _compose_clip_in_ego(self) -> Optional[Polygon]:
        """
        组合矩形与“围绕 +x 方向”的扇形（**全角**）为最终裁剪几何（都在 ego 局部系）。
        - 全角 A（度）→ 半角 half = A/2
        - 三角近似扇形：[(0,0), (r*cos(+half), r*sin(+half)), (r*cos(+half), -r*sin(+half))]
        """
        rect_geom = None
        radius = 200.0
        if self.rect_bounds is not None:
            x_min, x_max, y_min, y_max = self.rect_bounds
            rect_geom = box(x_min, y_min, x_max, y_max)
            radius = float(np.hypot(max(abs(x_min), abs(x_max)),
                                    max(abs(y_min), abs(y_max)))) + 1.0

        wedge_geom = None
        if self.sector_full_angle_deg is not None and self.sector_full_angle_deg > 0:
            half = math.radians(self.sector_full_angle_deg * 0.5)
            p0 = (0.0, 0.0)
            p1 = (radius * math.cos(half), radius * math.sin(half))
            p2 = (radius * math.cos(half), -radius * math.sin(half))
            wedge_geom = Polygon([p0, p1, p2])

        if rect_geom is None and wedge_geom is None:
            return None
        if rect_geom is None:
            return wedge_geom
        if wedge_geom is None:
            return rect_geom
        inter = rect_geom.intersection(wedge_geom)
        return inter if (inter is not None and not inter.is_empty) else None

    @staticmethod
    def _split_collections(g) -> List:
        """把 Multi/Collection 拆成基本几何并过滤空/非法。"""
        if g is None or getattr(g, "is_empty", True) or not getattr(g, "is_valid", False):
            return []
        gt = g.geom_type
        if "Multi" in gt or gt == "GeometryCollection":
            outs = []
            for x in getattr(g, "geoms", []):
                if x.is_valid and not x.is_empty:
                    outs.append(x)
            return outs
        return [g]

    @staticmethod
    def _clip_and_split(geom, clip_geom: Optional[Polygon]) -> List:
        """对几何做可选裁剪并拆分；合并线段碎片。"""
        if geom is None or geom.is_empty:
            return []
        g = geom if clip_geom is None else geom.intersection(clip_geom)
        if g is None or g.is_empty:
            return []
        if g.geom_type == "MultiLineString":
            g = ops.linemerge(g)
        return NuscMapExtractorComponent._split_collections(g)

    @staticmethod
    def _get_drivable_area_contour(drivable_areas: List[Polygon], clip_geom: Optional[Polygon]) -> List[LineString]:
        """从可行驶区域面提取边界线。为了避免边缘伪线，对 clip 做 -0.2m buffer。"""
        shrink = None
        if clip_geom is not None:
            shrink = clip_geom.buffer(-0.2)
            if shrink.is_empty:
                shrink = clip_geom

        exteriors: List[LinearRing] = []
        interiors: List[LinearRing] = []

        clipped_polys: List[Polygon] = []
        for poly in drivable_areas:
            for p in NuscMapExtractorComponent._clip_and_split(poly, clip_geom):
                if isinstance(p, Polygon):
                    clipped_polys.append(p)

        for poly in clipped_polys:
            exteriors.append(poly.exterior)
            interiors.extend(list(poly.interiors))

        out_lines: List[LineString] = []

        # 外环顺时针（右侧为可行驶）
        for ext in exteriors:
            if ext.is_ccw:
                ext = LinearRing(list(ext.coords)[::-1])
            lines = ext if shrink is None else ext.intersection(shrink)
            if getattr(lines, "geom_type", None) == "MultiLineString":
                lines = ops.linemerge(lines)
            if lines is not None:
                out_lines += [ls for ls in NuscMapExtractorComponent._split_collections(lines)
                              if isinstance(ls, LineString)]

        # 内环逆时针
        for inter in interiors:
            if not inter.is_ccw:
                inter = LinearRing(list(inter.coords)[::-1])
            lines = inter if shrink is None else inter.intersection(shrink)
            if getattr(lines, "geom_type", None) == "MultiLineString":
                lines = ops.linemerge(lines)
            if lines is not None:
                out_lines += [ls for ls in NuscMapExtractorComponent._split_collections(lines)
                              if isinstance(ls, LineString)]
        return out_lines

    @staticmethod
    def _get_ped_contour(poly: Polygon, clip_geom: Optional[Polygon]) -> Optional[LineString]:
        """人行横道外轮廓线（闭合）。"""
        if poly is None or poly.is_empty:
            return None
        ext = poly.exterior
        if not ext.is_ccw:
            ext = LinearRing(list(ext.coords)[::-1])
        if clip_geom is None:
            return ext
        clipped = ext.intersection(clip_geom)
        if clipped.is_empty:
            return None
        if clipped.geom_type == "GeometryCollection":
            lines = []
            for g in clipped.geoms:
                if g.geom_type in ("LineString", "LinearRing"):
                    lines.append(g)
                elif g.geom_type == "MultiLineString":
                    lines.extend(list(g.geoms))
            if not lines:
                return None
            merged = ops.linemerge(lines)
        else:
            merged = clipped
        if merged.geom_type == "MultiLineString":
            coords = []
            for l in merged.geoms:
                coords.extend(l.coords)
            merged = LineString(coords)
        if merged.geom_type not in ("LineString", "LinearRing"):
            return None
        if not merged.is_closed:
            c = list(merged.coords)
            if c and c[0] != c[-1]:
                c.append(c[0])
            merged = LineString(c)
        return merged

    @staticmethod
    def _union_ped(polys: List[Polygon]) -> List[Polygon]:
        """把相近且方向一致的人行横道 polygon 合并（兼容 shapely 1/2 的 STRtree 行为）。"""
        if not polys:
            return []
        tree = strtree.STRtree(polys)

        def get_dir(geom: Polygon):
            rect = geom.minimum_rotated_rectangle
            coords = np.asarray(rect.exterior.coords)
            if coords.shape[0] < 3:
                return np.array([1.0, 0.0], dtype=np.float64), 1.0
            v = coords[1:3] - coords[0:2]
            n = np.linalg.norm(v, axis=-1) + 1e-9
            i = n.argmax()
            return v[i], n[i]

        # 用 WKB 回索引（shapely 2 可能返回几何对象）
        wkb2idx: Dict[bytes, List[int]] = {}
        for i, g in enumerate(polys):
            try:
                wkb2idx.setdefault(g.wkb, []).append(i)
            except Exception:
                pass

        finals: List[Polygon] = []
        remain = set(range(len(polys)))
        for i in range(len(polys)):
            if i not in remain:
                continue
            base = polys[i]
            remain.remove(i)
            vi, ni = get_dir(base)
            merged = base

            cand = tree.query(base)
            cand_idx: List[int] = []
            # 可能返回 numpy 索引，也可能返回几何
            if hasattr(cand, "dtype"):
                cand_idx = [int(k) for k in cand.tolist()]
            else:
                for o in cand:
                    cand_idx += wkb2idx.get(getattr(o, "wkb", b""), [])

            for j in list(cand_idx):
                if j == i or j not in remain:
                    continue
                g = polys[j]
                vj, nj = get_dir(g)
                cosv = float(vi.dot(vj) / (ni * nj))
                if 1.0 - abs(cosv) < 0.01:  # 方向差 <~8°
                    try:
                        merged = merged.union(g)
                        remain.remove(j)
                    except Exception:
                        continue
            outs = NuscMapExtractorComponent._split_collections(merged)
            for p in outs:
                if isinstance(p, Polygon):
                    finals.append(p)

        return finals

    def _global_to_ego_affine_params(self,
                                     R_ego_to_global: np.ndarray,
                                     ego_xyz: Tuple[float, float, float]
                                     ) -> Tuple[float, float, float, float, float, float]:
        """
        构造 shapely.affinity.affine_transform 所需的 2D 仿射，把 global 平面点一次性变换到 ego 平面：
            p_ego = R_ego_to_global^T @ (p_glb - t_ego)，其中地图 z=0。
        返回 (a, b, d, e, xoff, yoff) 满足：
            x' = a*x + b*y + xoff
            y' = d*x + e*y + yoff
        关键：平移需包含 z 的影响项 —— -(Rt[:2,2] * tz)。
        """
        Rt = R_ego_to_global.T  # global->ego（3x3）
        x0, y0, z0 = float(ego_xyz[0]), float(ego_xyz[1]), float(ego_xyz[2])

        # 2x2 旋转
        a, b = float(Rt[0, 0]), float(Rt[0, 1])
        d, e = float(Rt[1, 0]), float(Rt[1, 1])

        # 2D 等效平移：-(Rt[:2,:2] @ [x0,y0] + Rt[:2,2] * z0)
        xoff = -(a * x0 + b * y0 + float(Rt[0, 2]) * z0)
        yoff = -(d * x0 + e * y0 + float(Rt[1, 2]) * z0)
        return a, b, d, e, xoff, yoff

    # ================== 每帧提取（修复后的矩阵版 global→ego） ==================
    def _extract_one(self,
                     location: str,
                     ego_xy: Tuple[float, float],
                     ego_R: np.ndarray,
                     ego_global_z: float) -> Dict[str, List[np.ndarray]]:
        """
        返回 {divider/boundary/ped_crossing: List[np.ndarray(N,3)]}，全在 ego 局部系。
        与官方 render_map_in_image 的 3D 变换一致性修复：
          在 yaw-only 的局部坐标基础上，加入 roll/pitch 引起的平面内常量偏移：
            off_xy = -c_z * [ R[2,0], R[2,1] ]  （R 为 ego->global 的 3x3）
        """
        yaw = self._yaw_from_R_ego_to_global(ego_R)  # rad
        yaw_deg = float(yaw / math.pi * 180.0)  # deg

        # —— 保持原有缓存（仅与 x/y/yaw 有关，未包含 z）——
        key = self._quant_key(location, ego_xy[0], ego_xy[1], yaw_deg)
        cached = self._cache.get(key, None)

        len_x, len_y = self._estimate_patch_len(self.rect_bounds)
        patch_box = (float(ego_xy[0]), float(ego_xy[1]), float(len_y), float(len_x))
        explorer = self._explorers[location]

        if cached is None:
            lane_dividers = explorer._get_layer_line(patch_box, yaw_deg, "lane_divider") or []
            road_dividers = explorer._get_layer_line(patch_box, yaw_deg, "road_divider") or []
            ped_polys = explorer._get_layer_polygon(patch_box, yaw_deg, "ped_crossing") or []
            road_segments = explorer._get_layer_polygon(patch_box, yaw_deg, "road_segment") or []
            lanes = explorer._get_layer_polygon(patch_box, yaw_deg, "lane") or []

            clip_geom = self._compose_clip_in_ego()

            # 线（yaw-only，本地坐标）
            dividers_2d: List[np.ndarray] = []
            for line in lane_dividers + road_dividers:
                for g in self._clip_and_split(line, clip_geom):
                    if isinstance(g, LineString):
                        dividers_2d.append(np.asarray(g.coords, dtype=np.float32))

            # 可行驶区域（yaw-only，本地坐标）
            union_roads = ops.unary_union(road_segments) if len(road_segments) else None
            union_lanes = ops.unary_union(lanes) if len(lanes) else None
            if union_roads is None and union_lanes is None:
                drivable_union = None
            elif union_roads is None:
                drivable_union = union_lanes
            elif union_lanes is None:
                drivable_union = union_roads
            else:
                drivable_union = ops.unary_union([union_roads, union_lanes])

            drivable_polys: List[Polygon] = []
            if (drivable_union is not None) and (not drivable_union.is_empty):
                for gg in self._split_collections(drivable_union):
                    for p in self._clip_and_split(gg, clip_geom):
                        if isinstance(p, Polygon):
                            drivable_polys.append(p)

            boundaries_2d: List[np.ndarray] = []
            for ls in self._get_drivable_area_contour(drivable_polys, clip_geom):
                boundaries_2d.append(np.asarray(ls.coords, dtype=np.float32))

            # 人行横道（yaw-only，本地坐标）
            ped_raw: List[Polygon] = []
            for p in ped_polys:
                ped_raw += [pg for pg in self._split_collections(p) if isinstance(pg, Polygon)]
            ped_merged = self._union_ped(ped_raw)

            ped_lines_2d: List[np.ndarray] = []
            for p in ped_merged:
                ls = self._get_ped_contour(p, clip_geom)
                if ls is not None and (not ls.is_empty):
                    ped_lines_2d.append(np.asarray(ls.coords, dtype=np.float32))

            cached = dict(divider=dividers_2d, boundary=boundaries_2d, ped_crossing=ped_lines_2d)
            self._cache[key] = cached

        # ====== 关键修复：加入 roll/pitch 导致的常量偏移 ======
        # Δ_xy = R^T @ [0,0,-c_z] 的前两维 = -c_z * [ R[2,0], R[2,1] ]
        off_x = -float(ego_global_z) * float(ego_R[2, 0])
        off_y = -float(ego_global_z) * float(ego_R[2, 1])

        def _apply_offset(arrs: List[np.ndarray]) -> List[np.ndarray]:
            if (off_x == 0.0) and (off_y == 0.0):
                return [a.astype(np.float32, copy=False) for a in arrs]
            off = np.array([off_x, off_y], dtype=np.float32)
            outs = []
            for a in arrs:
                if a is None or a.size == 0:
                    continue
                aa = a.astype(np.float32, copy=False)
                outs.append(aa + off[None, :])
            return outs

        div_2d = _apply_offset(cached["divider"])
        bnd_2d = _apply_offset(cached["boundary"])
        ped_2d = _apply_offset(cached["ped_crossing"])

        # —— 将 2D 提升为 3D，z 恒为 -ego_global_z —— #
        zc = -float(ego_global_z)
        out_3d = dict(
            divider=[self._to3d_with_const_z(arr, zc) for arr in div_2d],
            boundary=[self._to3d_with_const_z(arr, zc) for arr in bnd_2d],
            ped_crossing=[self._to3d_with_const_z(arr, zc) for arr in ped_2d],
        )
        return out_3d

    # ================== 对外入口（整段 scene） ==================
    def __call__(self, data) -> "data":
        """
        输入：
          - data.ego_pose[t] : (4,4) ego->global
          - data.scene_token : str
        写回：
          - data.map_divider[t] / data.map_boundary[t] / data.map_ped_crossing[t] : List[np.ndarray(N,2)]
        """
        global g_nusc

        scene = g_nusc.get("scene", data.scene_token)
        log = g_nusc.get("log", scene["log_token"])
        location = log["location"]  # e.g. 'singapore-onenorth'

        T = int(len(data.ego_pose))
        div_all = [None] * T
        bnd_all = [None] * T
        ped_all = [None] * T
        for t in range(T):
            T_eg2g = np.asarray(data.ego_pose[t], dtype=np.float64)
            R = T_eg2g[:3, :3]
            c = T_eg2g[:3, 3]
            ego_xyz = (float(c[0]), float(c[1]), float(c[2]))
            out = self._extract_one(location, ego_xyz, R)
            div_all[t] = out["divider"]
            bnd_all[t] = out["boundary"]
            ped_all[t] = out["ped_crossing"]

        data.map_divider = div_all
        data.map_boundary = bnd_all
        data.map_ped_crossing = ped_all
        return data


# ========== 组件4：CImgProcess（读图/裁剪/缩放/打包 cimg）==========
class CImgProcess:
    def __init__(self, keep_idx: int = 1, out_h: int = 256, out_w: int = 512):
        self.keep_idx = keep_idx
        self.out_h = out_h
        self.out_w = out_w

    def __call__(self, data: Data) -> Data:
        assert isinstance(data.calib, dict), "data.calib 为空，请先执行 BuildCalibForCImg"

        paths = data.img_paths
        T = len(paths)
        C, H, W = 3, self.out_h, self.out_w

        big_cimgs: List[CalibratedImage] = [None] * T
        ori_cimgs: List[CalibratedImage] = [None] * T

        for i, p in enumerate(paths):
            img = cv2.imread(p, cv2.IMREAD_COLOR)
            if img is None:
                raise RuntimeError(f"读取失败: {p}")

            cimg = CalibratedImage.load(img, data.calib)

            big_cf = ImageProcessor.crop(cimg, t=0.111, b=0.0, l=0., r=0.)
            big_cf = ImageProcessor.scale(big_cf, new_h=H, new_w=W)

            ori_cimgs[i] = cimg
            big_cimgs[i] = big_cf

            if i % self.keep_idx not in (0, self.keep_idx - 1):
                big_cimgs[i].image = None
                ori_cimgs[i].image = None

        data.big_cimg = big_cimgs
        data.ori_cimg = ori_cimgs

        return data


class CImgToTensor:
    """
    从 data.big_cimg 里取出每个 CalibratedImage.image：
    1. 假设每个 image 是 H x W x 3 的 BGR uint8
    2. 堆成 [T, H, W, 3] (THW3)
    3. 做 BGR -> RGB
    4. 改维度为 [T, 3, H, W]
    5. 转成 float32，并 /255
    最终得到 torch.Tensor，挂到 data.tensor 或 data.big_tensor 上
    """

    def __call__(self, data: "Data") -> "Data":
        big_cimgs: List["CalibratedImage"] = data.big_cimg
        assert isinstance(big_cimgs, list) and len(big_cimgs) > 0, "data.big_cimg 为空"

        imgs: List[np.ndarray] = []
        for i, cimg in enumerate(big_cimgs):
            img = getattr(cimg, "image", None)
            if img is None:
                raise RuntimeError(f"big_cimg[{i}].image 为 None，请检查前面是否被置空")
            if not isinstance(img, np.ndarray):
                raise RuntimeError(f"big_cimg[{i}].image 不是 numpy 数组")
            if img.ndim != 3 or img.shape[2] != 3:
                raise RuntimeError(f"big_cimg[{i}].image 形状非法: {img.shape}，期望 HxWx3")
            imgs.append(img)

        # [T, H, W, 3]，BGR
        stack = np.stack(imgs, axis=0)

        # BGR -> RGB
        stack = stack[..., ::-1]

        # 转 float32 并归一化
        stack = stack.astype(np.float32) / 255.0

        # [T, H, W, 3] -> [T, 3, H, W]
        stack = np.transpose(stack, (0, 3, 1, 2))

        # numpy -> torch，保证连续
        tensor = torch.from_numpy(stack).contiguous()  # shape: [T, 3, H, W], dtype=float32

        data.big_img_data = tensor
        return data


# ========== 组件X：由 3D 框生成 big_cimg 上的 2D 框（xyxy） ==========
class MakeDet2DFrom3DBig:
    def __init__(self, clamp: bool = True, min_side: float = 1.0):
        """
        clamp:  是否把边界裁到 [0,W-1]/[0,H-1]
        min_side: 宽或高小于该阈值的框视为无效（保持0）
        """
        self.clamp = bool(clamp)
        self.min_side = float(min_side)

    @staticmethod
    def _corners_from_b7(b7: np.ndarray) -> np.ndarray:
        """
        b7 = (cx,cy,cz,l,w,h,theta) —— ego系, z向上
        返回 (8,3) 八个角点（仅用于投影）
        """
        cx, cy, cz, l, w, h, theta = [float(x) for x in b7.tolist()]
        l2, w2, h2 = 0.5 * l, 0.5 * w, 0.5 * h
        pts = np.array([
            [l2, w2, -h2],
            [l2, -w2, -h2],
            [-l2, -w2, -h2],
            [-l2, w2, -h2],
            [l2, w2, h2],
            [l2, -w2, h2],
            [-l2, -w2, h2],
            [-l2, w2, h2],
        ], dtype=np.float32)
        c, s = math.cos(theta), math.sin(theta)
        Rz = np.array([[c, -s, 0.],
                       [s, c, 0.],
                       [0., 0., 1.]], dtype=np.float32)
        pts = (Rz @ pts.T).T
        pts += np.array([cx, cy, cz], dtype=np.float32)[None, :]
        return pts

    @staticmethod
    def _xyxy_from_uv(uv: np.ndarray, W: int, H: int, clamp: bool, min_side: float
                      ) -> tuple[float, float, float, float, bool]:
        """
        给像素点 (N,2) 求外接矩形；返回 (x1,y1,x2,y2, ok)。
        """
        if uv.size == 0:
            return 0., 0., 0., 0., False
        u, v = uv[:, 0], uv[:, 1]
        x1, y1 = float(u.min()), float(v.min())
        x2, y2 = float(u.max()), float(v.max())
        if clamp:
            x1 = max(0.0, min(x1, float(W - 1)))
            x2 = max(0.0, min(x2, float(W - 1)))
            y1 = max(0.0, min(y1, float(H - 1)))
            y2 = max(0.0, min(y2, float(H - 1)))
        ok = (x2 - x1) >= min_side and (y2 - y1) >= min_side
        return x1, y1, x2, y2, ok

    def __call__(self, data: Data) -> Data:
        """
        只处理 big_cimg：生成 data.det2d_box (T,K,4, xyxy)。
        要求：data.big_cimg / det3d_box / det3d_cls 已存在。
        """
        assert data.big_cimg is not None, "MakeDet2DFrom3DBig: 缺 big_cimg"
        assert data.det3d_box is not None and data.det3d_cls is not None, "MakeDet2DFrom3DBig: 缺 det3d_*"

        T, K = data.det3d_box.shape[:2]
        out = np.zeros((T, K, 4), dtype=np.float32)

        for t in range(T):
            cimg = data.big_cimg[t]
            H, W = int(cimg.image_height), int(cimg.image_width)

            b_t = data.det3d_box[t].detach().cpu().numpy()
            n_t = data.det3d_cls[t].detach().cpu().numpy()

            for k in range(K):
                if n_t[k] < 0:
                    continue  # padding
                b8 = b_t[k]
                if not np.any(np.abs(b8)):
                    continue  # 全0

                corners = self._corners_from_b7(b8)  # (8,3) ego
                uv = ImageProcessor.world_points_to_uv(cimg, corners).astype(np.float32)
                x1, y1, x2, y2, ok = self._xyxy_from_uv(uv, W, H, self.clamp, self.min_side)
                if ok:
                    out[t, k] = [x1, y1, x2, y2]  # 无效保持 0

        data.det2d_box = torch.from_numpy(out)
        return data


# ========== 组件5：对齐长度 ==========
class AlignLength:
    def __init__(self, fixed_length: int, max_random_offset: int = 0):
        self.fixed_length = fixed_length
        self.max_random_offset = max_random_offset

    def __call__(self, data: Data) -> Data | None:
        seqs: Dict[str, Any] = {
            k: v for k, v in vars(data).items()
            if isinstance(v, (list, np.ndarray, torch.Tensor))
        }
        if not seqs:
            return data

        min_len = min(len(v) for v in seqs.values())
        if min_len < self.fixed_length:
            logger.warning(f"{data.scene_name} 序列过短：需要 >= {self.fixed_length}，最短 {min_len}")
            return None

        allowed_max = min(len(v) - self.fixed_length for v in seqs.values())
        start_upper = min(self.max_random_offset, max(0, allowed_max))
        start = random.randint(0, start_upper)
        end = start + self.fixed_length

        for k, v in seqs.items():
            if isinstance(v, list):
                setattr(data, k, v[start:end])
            else:
                setattr(data, k, v[start:end])
        return data


# ========== 并行全局（直接使用常量配置） ==========
g_nusc = NuScenesCachedLoader(
    version="v1.0-trainval",
    dataroot="/opt_disk2/rd23442/Projects-SGS/R-AEB/data/all_nuScenes",
    use_gzip=True,
    cache_dir=None,
)

load_scene_front = LoadSceneCAMFront(
    cam_channel="CAM_FRONT",
    max_length=2000,
)

build_cimg_calib = BuildCalibForCImg(
    cam_channel="CAM_FRONT",
)

load_det3d_anno = LoadDet3DNuScenes(
    k_max=50,
    hfov_deg=70.0,  # 水平 FOV（度）
    x_range=(3.0, 80.0),  # ego 下前向距离范围
    y_range=(-30.0, 30.0),  # ego 下左右范围（左+）
)

map_extractor = NuscMapExtractorComponent(
    dataroot="/opt_disk2/rd23442/Projects-SGS/R-AEB/data/all_nuScenes",
    rect_bounds=(3.0, 60.0, -30.0, 30.0),  # ego 矩形裁剪
    sector_full_angle_deg=70.0,  # 扇形全角（围绕 +x）
    cache_xy_meter=1.0,
    cache_yaw_deg=3.0,
)

cimg_process = CImgProcess(
    keep_idx=1,
    out_h=320,
    out_w=640,
)

to_tensor = CImgToTensor()

# 2D 框组件
make_det2d_box = MakeDet2DFrom3DBig(
    clamp=True,
    min_side=1.0,
)

align_length = AlignLength(
    fixed_length=39,
    max_random_offset=0,
)


# ========== Dataset（sample级并行 + collate(t,B,...)）==========
class NuScenesSceneDataset(SamplePrefetchDataset):
    """
    基于 SamplePrefetchDataset 的 NuScenes 场景级数据集。

    - db: List[{"scene_token": str}]
    - db_fn: Callable[[Dict[str, str]], Data]
    """

    def __init__(self, split: str = "trainval", prefetch_size: int = 16):

        self.split = split

        # 1) 可选：在主进程尝试设置 fork，利于共享只读内存
        import multiprocessing as mp
        try:
            mp.get_start_method()
        except RuntimeError:
            try:
                mp.set_start_method("fork", force=True)
            except Exception:
                pass

        # 3) 构建 db：这里只关注「有哪些 scene，要怎么组织成列表」
        self.db = self.build_db(split)  # List[Dict[str, str]]

        # 5) 把 db 和 db_fn 交给父类，预取逻辑全部由 SamplePrefetchDataset 处理
        super().__init__(
            db=self.db,
            db_fn=self.db_fn,
            prefetch_size=prefetch_size,
        )

    @staticmethod
    def build_db(split: str) -> List[Dict[str, str]]:
        splits = create_splits_scenes()
        if split in splits:
            names = set(splits[split])
        elif split == "trainval":
            names = set(splits["train"]) | set(splits["val"])
        elif split in ("mini_train", "mini_val"):
            names = set(splits[split])
        else:
            logger.warning(f"未知 split={split}，默认使用 train。")
            names = set(splits.get("train", []))

        items = [{"scene_token": s["token"]} for s in g_nusc.scene if s["name"] in names]
        return items

    @staticmethod
    def db_fn(db: Dict[str, Any]) -> Dict[str, Any] | None:
        data = Data()

        data = load_scene_front(data, db)  # 路径/位姿/内参
        data = build_cimg_calib(data, db)  # 第一帧生成 calib（缓存），世界=ego
        data = load_det3d_anno(data, db)  # global→ego & ego中过滤（FOV+BEV）
        # data = map_extractor(data, g_nusc) # 提取地图（ego）
        data = cimg_process(data)  # 读图/裁剪/缩放/打包 cimg
        data = to_tensor(data)
        data = make_det2d_box(data)  # 由 3D 框生成 big_cimg 上的 2D 框
        data = align_length(data)  # 对齐长度

        return data

    def collate_fn(self, samples: List[Optional[Data]]) -> List[Dict[str, Any]]:
        # 1) 过滤 None；必要时回退到上一次可用样本
        valid_samples = [s for s in samples if s is not None]
        if not valid_samples:
            if self._last_good is not None:
                logger.warning("collate_fn: all None，回退上一个 batch 的样本。")
                valid_samples = [self._last_good]
            else:
                raise RuntimeError("all samples are None")
        self._last_good = valid_samples[0]
        samples = valid_samples  # 后续都用 samples 这个名字

        # 2) 基本维度/类型信息（假定各样本 T、图像尺寸一致）
        B = len(samples)
        T = len(samples[0].big_img_data)
        C, H, W = samples[0].big_img_data.shape[1:]

        K = samples[0].det3d_box.shape[1]

        # 3) 逐帧组织：返回一个长度为 T 的 list[dict]
        frames: List[Dict[str, Any]] = []

        for t in range(T):
            # 为该时刻 t 分配容器（不跨帧共享）
            big = torch.empty((B, C, H, W), dtype=torch.float32)

            det3d_box = torch.empty((B, K, 7), dtype=torch.float32)  # (B,K,7)  (ego)
            det3d_cls = torch.empty((B, K), dtype=torch.int32)  # (B,K)
            det2d_box = torch.empty((B, K, 4), dtype=torch.float32)  # (B,K,4) xyxy

            # cimgs：该时刻对应的 list[B]
            big_cimgs_t: List[Any] = []
            ori_cimgs_t: List[Any] = []

            # 元信息：与样本相关（非逐帧），这里每帧都带上与原实现保持可取性
            scene_name = [s.scene_name for s in samples]
            video_path = [s.video_path for s in samples]
            ori_img_size = [s.ori_img_size for s in samples]

            # 4) 填充该时刻的 batch 内容
            for b, s in enumerate(samples):
                # 图像
                big[b].copy_(s.big_img_data[t])

                # 3D/2D 标注
                det3d_box[b].copy_(s.det3d_box[t])  # (K,7)
                det3d_cls[b].copy_(s.det3d_cls[t])  # (K,)
                det2d_box[b].copy_(s.det2d_box[t])  # (K,4)

                # cimg 句柄/对象
                big_cimgs_t.append(s.big_cimg[t])
                ori_cimgs_t.append(s.ori_cimg[t])

            # 5) 组装该时刻的字典
            frame_dict: Dict[str, Any] = {
                "t_index": t,  # 可选：保留时序索引
                "scene_name": scene_name,  # list[B]
                "video_path": video_path,  # list[B]
                "ori_img_size": ori_img_size,  # list[B]

                "big_img_data": big,  # (B,C,H,W)

                "big_cimgs": big_cimgs_t,  # list[B]
                "ori_cimgs": ori_cimgs_t,  # list[B]

                "det3d_cls": det3d_cls,  # (B,K)
                "det3d_box": det3d_box,  # (B,K,7)  (ego)
                "det2d_box": det2d_box,  # (B,K,4)  xyxy
            }

            frames.append(frame_dict)

        return frames

    def collate_fn_bt(self, samples: list[Any | None]) -> dict[str, Any]:
        """
        将时间维 T 直接合并进 B，输出单个 batch 字典：
          big_img_data      : (B*T, C, H, W)
          det3d_box         : (B*T, K, 7)
          det3d_cls           : (B*T, K)
          det2d_box     : (B*T, K, 4)
          big_cimgs / ori_cimgs / scene_name / video_path / ori_img_size / t_index : 长度 B*T 的列表或张量

        假设各样本的 T、图像尺寸、K 一致；若本次全为 None，则回退到上次非 None 样本。
        """
        # 1) 过滤 None；必要时回退到上一次可用样本
        valid_samples = [s for s in samples if s is not None]
        if not valid_samples:
            if getattr(self, "_last_good", None) is not None:
                logger.warning("collate_fn: all None，回退上一个 batch 的样本。")
                valid_samples = [self._last_good]
            else:
                raise RuntimeError("all samples are None")
        self._last_good = valid_samples[0]
        samples = valid_samples

        # 2) 基本信息（假设 T、尺寸一致）
        B = len(samples)
        T = len(samples[0].big_img_data)
        C, H, W = samples[0].big_img_data.shape[1:]
        K = samples[0].det3d_box.shape[1]

        BT = B * T  # 展平后的 batch 维度

        # 3) 预分配展平后的容器
        big = torch.empty((BT, C, H, W), dtype=torch.float32)  # 图像
        det3d_box = torch.empty((BT, K, 7), dtype=torch.float32)  # 3D 框 (ego)
        det3d_cls = torch.empty((BT, K), dtype=torch.int32)  # 类别
        det2d_box = torch.empty((BT, K, 4), dtype=torch.float32)  # 2D 框 (xyxy)

        # 句柄/原图等（list，对齐 BT）
        big_cimgs_bt: list[Any] = []
        ori_cimgs_bt: list[Any] = []
        scene_name_bt: list[Any] = []
        video_path_bt: list[Any] = []
        ori_img_size_bt: list[Any] = []
        t_index_bt: list[int] = []

        # 4) 填充
        # 展平索引规则： idx = b*T + t
        for b, s in enumerate(samples):
            for t in range(T):
                idx = b * T + t

                # 图像
                big[idx].copy_(s.big_img_data[t])

                # 标注
                det3d_box[idx].copy_(s.det3d_box[t])  # (K,7)
                det3d_cls[idx].copy_(s.det3d_cls[t])  # (K,)
                det2d_box[idx].copy_(s.det2d_box[t])  # (K,4)

                # 句柄/原图
                big_cimgs_bt.append(s.big_cimg[t])
                ori_cimgs_bt.append(s.ori_cimg[t])

                # 元信息（按帧重复对齐）
                scene_name_bt.append(s.scene_name)
                video_path_bt.append(s.video_path)
                ori_img_size_bt.append(s.ori_img_size)
                t_index_bt.append(t)

        # 5) 组装成单个 batch 字典（T 已合并进 B）
        batch: dict[str, Any] = {
            "t_index": torch.tensor(t_index_bt, dtype=torch.int32),  # (BT,)
            "scene_name": scene_name_bt,  # list[BT]
            "video_path": video_path_bt,  # list[BT]
            "ori_img_size": ori_img_size_bt,  # list[BT]

            "big_img_data": big,  # (BT,C,H,W)
            "big_cimgs": big_cimgs_bt,  # list[BT]
            "ori_cimgs": ori_cimgs_bt,  # list[BT]

            "det3d_cls": det3d_cls,  # (BT,K)
            "det3d_box": det3d_box,  # (BT,K,7)
            "det2d_box": det2d_box,  # (BT,K,4)
        }
        return batch


# ========== Demo 可视化 ==========
if __name__ == "__main__":
    from torch.utils.data import DataLoader
    from visper.common.vis import boxes_to_corners3d, draw_3d_boxes, draw_2d_boxes

    ds = NuScenesSceneDataset(split="trainval", prefetch_size=4)

    loader = DataLoader(
        dataset=ds,
        batch_size=2,
        shuffle=True,
        num_workers=0,  # 样本级并行已预取
        prefetch_factor=None,
        collate_fn=ds.collate_fn
    )

    os.makedirs("nusc_output", exist_ok=True)
    fps = 10  # 可视化帧率
    b = 0
    for i, datas in enumerate(loader):
        T = len(datas)

        for t, data in enumerate(datas):
            scene_name = data['scene_name'][b]
            H, W = data['big_cimgs'][b].image.shape[:2]

            out_path = os.path.join("nusc_output", f"{scene_name}")

            vis_img = data['big_cimgs'][b].image.copy()

            # 读取 3D → 投影可视化（原有）
            gt_3dbox = data["det3d_box"][b].detach().cpu().numpy()  # (K, 8)  (cx,cy,cz,l,w,h,theta) —— ego
            gt_2dbox = data["det2d_box"][b].detach().cpu().numpy()  # (K, 4)
            gt_cls = data["det3d_cls"][b].detach().cpu().numpy()  # (K,)    int64，-1 为 padding

            valid_mask = (gt_cls >= 0) & (np.sum(np.abs(gt_3dbox), axis=1) > 0)
            if np.any(valid_mask):
                gt_3dbox = gt_3dbox[valid_mask].astype(np.float32, copy=False)
                gt_2dbox = gt_2dbox[valid_mask].astype(np.float32, copy=False)
                gt_cls = gt_cls[valid_mask].astype(np.int32, copy=False)

                corners_3d = boxes_to_corners3d(gt_3dbox)  # (N, 8, 3)  ego
                corners_uv = ImageProcessor.world_points_to_uv(data['big_cimgs'][b], corners_3d)

                K = gt_2dbox.shape[0]
                gt_objs6 = np.concatenate([gt_2dbox, np.ones((K, 1)), gt_cls.reshape(K, 1)], axis=1)  # (K,6) [x1,y1,x2,y2,1,cls]

                vis_img = draw_3d_boxes(vis_img, corners_uv, gt_objs6)
                # vis_img = draw_2d_boxes(vis_img, gt_objs6, ID2NAME)

                cv2.imwrite(f'{out_path}_{t}.jpg', vis_img)
                logger.info(f"saved: {out_path}_{t}.jpg")

    ds.shutdown()
