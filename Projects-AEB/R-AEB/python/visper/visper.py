from __future__ import annotations
import os
import threading
import time
from pathlib import Path
from typing import Any, Callable
import queue

import commentjson as cjson
import cv2
import numpy as np

from .common.logger import logger, DEBUG
from .cuboids import (
    cuboids_InitRAEB,
    cuboids_ProcRAEB,
    cuboids_VisCuboids,
)
from .infer import infer_InitRAEB, infer_InferRAEB
from .preproc import preproc_Init, preproc_Convert, preproc_ToData
from .postproc import postproc_InitRAEB, postproc_ProcRAEB, postproc_VisInstances
from .track import cuboid_tracker_Init, cuboid_tracker_TrackAndEstimate

HERE = Path(__file__).resolve().parent

g_TASKS: list[str] = []
g_FUNC_LOCK = threading.Lock()
g_RAEB_STAGEB_COND = threading.Condition(g_FUNC_LOCK)
g_RAEB_SYNC_RESULT = True
g_RAEB_SYNC_TIMEOUT_SEC = 1.0

g_RAEB_CALLBACK: Callable[[dict[str, Any]], None] | None = None
g_OP_CALLBACK: Callable[[dict[str, Any]], None] | None = None

# 全局上下文（尽量贴近 C++ 的字段：task/timestamp 等）
g_CTX: dict[str, Any] = {
    "img": None,
    "timestamp": None,
    # ------------------------------------
    "raeb_result": {
        "task": "RAEB",
        "timestamp": None,

        "objs": None,
        "masks": None,

        "track_info": None,
        "tracked_cuboids_raw": None,
        "tracked_cuboids": None,
        "tracked_cuboids_vel": None,
    },
    "raeb_vis": {
        "vis_img_2d": None,
        "vis_img_3d": None,
    },
    "raeb_extra": {
        "ego_timestamp": None,
        "ego_x": None,
        "ego_y": None,
        "ego_yaw": None,  # rad
    },
    # ------------------------------------
    "op_result": {
        "task": "OP",
        "timestamp": None,
        "preds": None,
    },
    "op_vis": {
        "vis_img": None,
    },
    "op_extra": {},
}

# RAEB 后处理线程（Stage B）相关
g_RAEB_STAGEB_THREAD: threading.Thread | None = None
g_RAEB_STAGEB_QUEUE: "queue.Queue[dict[str, Any] | None]" = queue.Queue(maxsize=1)


def _start_raeb_stageb_thread() -> None:
    global g_RAEB_STAGEB_THREAD

    # 已经启动过就不再建线程（简单判空即可，InitTask 只会在单线程上下文调用）
    if g_RAEB_STAGEB_THREAD is not None:
        return

    def worker() -> None:
        while True:
            stageA_out = g_RAEB_STAGEB_QUEUE.get()
            if stageA_out is None:
                break

            t = logger.timer()

            # Stage B：后处理 + 跟踪 + cuboids + 速度
            objs, masks = postproc_ProcRAEB(
                det_cat=stageA_out["det_cat"],
                proto=stageA_out["proto"],
                angle=stageA_out["angle"],
            )

            cuboids_raw_all = cuboids_ProcRAEB(
                objs=objs,
                grounding_z=0,
                masks=masks,
            )
            track_info, tracked_cuboids_raw, tracked_cuboids, tracked_cuboids_vel = cuboid_tracker_TrackAndEstimate(
                timestamp=stageA_out["timestamp"],
                cuboids=cuboids_raw_all,
                ego=stageA_out.get("raeb_extra", None),
                mode="ego",
            )
            track_info = np.asarray(track_info, dtype=np.int32)
            tracked_cuboids_raw = np.asarray(tracked_cuboids_raw, dtype=np.float32)
            tracked_cuboids = np.asarray(tracked_cuboids, dtype=np.float32)
            tracked_cuboids_vel = np.asarray(tracked_cuboids_vel, dtype=np.float32)

            t.done("RAEB StageB done")

            # 把结果写回 g_CTX
            with g_FUNC_LOCK:
                g_CTX["timestamp"] = stageA_out["timestamp"]
                g_CTX["img"] = stageA_out["img_rgb"]

                rr = g_CTX["raeb_result"]
                rr["timestamp"] = stageA_out["timestamp"]
                rr["objs"] = objs
                rr["masks"] = masks
                rr["track_info"] = track_info
                rr["tracked_cuboids_raw"] = tracked_cuboids_raw
                rr["tracked_cuboids"] = tracked_cuboids
                rr["tracked_cuboids_vel"] = tracked_cuboids_vel
                g_RAEB_STAGEB_COND.notify_all()

    g_RAEB_STAGEB_THREAD = threading.Thread(target=worker, daemon=True)
    g_RAEB_STAGEB_THREAD.start()


def VisPer_InitTask(task: str, config_path: str, model_path: str) -> None:
    global g_TASKS

    with g_FUNC_LOCK:
        first_task = len(g_TASKS) == 0
        already_inited = task in g_TASKS

        if first_task:
            logger.init_logger(level=DEBUG)

        if not already_inited:
            if task == "RAEB":
                try:
                    with open(config_path, "r", encoding="utf-8") as f:
                        cfg = cjson.load(f)
                except OSError as e:
                    raise RuntimeError(f"无法打开配置文件: {config_path}") from e

                # 这些初始化本身是线程安全的（只在这里调用）
                preproc_Init(
                    width=cfg["cyl"]["image_width"],
                    height=cfg["cyl"]["image_height"],
                    input_type=cfg["cyl"]["image_type"],
                    target_type=cfg["model"]["target_type"],
                )

                infer_InitRAEB(model_path=model_path)

                postproc_InitRAEB(
                    nc=cfg["model"]["num_cls"],
                    nm=cfg["model"]["num_mask"],
                    id2name=cfg["model"]["id2name"],
                    whitelist=cfg["model"]["whitelist"],
                    conf_thresh=cfg["model"]["conf_thresh"],
                    iou_thresh=cfg["model"]["iou_thresh"],
                    mask_thresh=cfg["model"]["mask_thresh"],
                    mask_up=cfg["model"]["mask_up"],
                    angle_postproc_cfg=cfg.get("angle_postproc", None),
                )

                cuboids_InitRAEB(
                    whitelist=cfg["model"]["whitelist"],
                    cyl_cam=cfg["cyl"],
                )

                cuboid_tracker_Init(
                    track_high_thresh=0.5,
                    track_low_thresh=0.3,
                    match_thresh=0.6,
                    new_track_thresh=0.6,
                    track_buffer=30,
                    frame_rate=30,
                    default_dt_ms=100.0,
                    timestamp_scale=1e-3,
                    center_gate_main_m=4.0,
                    center_gate_low_m=5.5,
                    weight_cls_main=0.55,
                    weight_center_main=0.35,
                    weight_size_main=0.10,
                    weight_cls_low=0.60,
                    weight_center_low=0.30,
                    weight_size_low=0.10,
                    sigma_a=2.0,
                    sigma_z=0.18,
                    vel_static_thresh=0.20,
                    vel_move_thresh=0.50,
                    stable_frames=1,
                    max_missed=30,
                    vel_clip_mps=8.0,
                    max_innovation_m=1.2,
                    vel_meas_blend=0.18,
                    vel_meas_blend_max=0.45,
                    vel_meas_innov_for_max=1.2,
                    pos_comp_sec=0.06,
                    pos_comp_min_speed=0.15,
                    pos_comp_max_m=0.60,
                    pos_comp_new_scale=0.60,
                    pos_comp_innov_for_zero=1.2,
                    pos_comp_dir_meas_base=0.20,
                    pos_comp_dir_meas_turn_gain=0.60,
                    pos_comp_reverse_cos=-0.20,
                    pos_comp_reverse_hold=3,
                    pos_comp_reverse_speed=0.40,
                    turn_recover_cos=0.30,
                    turn_recover_innov_m=0.10,
                    turn_recover_hold=6,
                    turn_recover_smooth_frames=3,
                    turn_recover_vel_blend=0.65,
                    turn_recover_pos_blend=0.70,
                    turn_recover_vel_blend_smooth=0.20,
                    turn_recover_pos_blend_smooth=0.25,
                    turn_recover_comp_boost=1.50,
                    turn_recover_min_speed=0.25,
                    turn_recover_comp_freeze=3,
                )

                _start_raeb_stageb_thread()

            elif task == "OP":
                pass
            else:
                logger.warning(f"未识别的 task: {task}")

        if task not in g_TASKS:
            g_TASKS.append(task)


def VisPer_RegCallback(task: str, cb: Callable[[dict[str, Any]], None]) -> None:
    global g_RAEB_CALLBACK, g_OP_CALLBACK

    with g_FUNC_LOCK:
        if task == "RAEB":
            g_RAEB_CALLBACK = cb
        elif task == "OP":
            g_OP_CALLBACK = cb
        else:
            logger.warning(f"VisPer_RegCallback: 未识别的 task: {task}")


def VisPer_PushExtraData(task: str, extra: dict[str, Any]) -> None:
    with g_FUNC_LOCK:
        if task == "RAEB":
            if "RAEB" not in g_TASKS:
                logger.warning("VisPer_PushExtraData: RAEB not initialized yet")
                return

            g_CTX["raeb_extra"] = dict(extra)

            e = g_CTX["raeb_extra"]
            logger.debug(
                f"VisPer_PushExtraData: RAEB updated "
                f"ego_timestamp={e.get('ego_timestamp')}, "
                f"ego_x={e.get('ego_x')}, ego_y={e.get('ego_y')}, ego_yaw={e.get('ego_yaw')}"
            )

        elif task == "OP":
            if "OP" not in g_TASKS:
                logger.warning("VisPer_PushExtraData: OP not initialized yet")
                return

            g_CTX["op_extra"] = dict(extra)

            logger.debug(f"VisPer_PushExtraData: OP updated keys={list(g_CTX['op_extra'].keys())}")

        else:
            logger.warning(f"VisPer_PushExtraData: unknown task: {task}")


def VisPer_RunInfer(img, timestamp: int) -> None:
    t = logger.timer()

    img_rgb = preproc_Convert(img=img, target_type="RGB")

    data = preproc_ToData(
        img=img_rgb,
        expand_axis=0,
        transpose_axes=(0, 3, 1, 2),
        out_dtype="float32",
    )
    t("Preproc")

    if "RAEB" in g_TASKS:
        # 拿一份 ego 的快照，避免后台线程再去碰 g_CTX.raeb_extra
        with g_FUNC_LOCK:
            raeb_extra_snapshot = dict(g_CTX["raeb_extra"])

        # det_cat, proto, angle = infer_InferRAEB(data=data, squeeze_batch=True)  //det_box", "det_cls", "det_mask
        det_box, det_cls,det_mask,proto, angle = infer_InferRAEB(data=data, squeeze_batch=True)
        det_cat = np.concatenate([det_box, det_cls, det_mask],axis=0)
        t("Infer")

        stageA_out = {
            "timestamp": timestamp,
            "img_rgb": img_rgb,
            "det_cat": det_cat,
            "proto": proto,
            "angle": angle,
            "raeb_extra": raeb_extra_snapshot,
        }
        g_RAEB_STAGEB_QUEUE.put(stageA_out)
        if g_RAEB_SYNC_RESULT:
            _wait_raeb_stageb_timestamp(target_timestamp=timestamp, timeout_sec=g_RAEB_SYNC_TIMEOUT_SEC)

        t.done("RAEB StageA done")

    if "OP" in g_TASKS:
        with g_FUNC_LOCK:
            g_CTX["op_result"]["timestamp"] = timestamp


def _wait_raeb_stageb_timestamp(target_timestamp: int, timeout_sec: float) -> None:
    deadline = time.monotonic() + max(0.0, float(timeout_sec))
    target = int(target_timestamp)
    with g_RAEB_STAGEB_COND:
        while True:
            done_ts = g_CTX["raeb_result"]["timestamp"]
            if done_ts is not None:
                try:
                    if int(done_ts) >= target:
                        return
                except (TypeError, ValueError):
                    if done_ts == target_timestamp:
                        return

            remain = deadline - time.monotonic()
            if remain <= 0.0:
                logger.warning(
                    f"VisPer_RunInfer stageB sync timeout: target_timestamp={target_timestamp}, "
                    f"done_timestamp={done_ts}"
                )
                return
            g_RAEB_STAGEB_COND.wait(timeout=min(0.01, remain))


def VisPer_GetResult(task: str) -> dict[str, Any] | None:
    with g_FUNC_LOCK:
        if task == "RAEB" and "RAEB" in g_TASKS:
            return dict(g_CTX["raeb_result"])
        if task == "OP" and "OP" in g_TASKS:
            return dict(g_CTX["op_result"])
        return None


def VisPer_GetVis(task: str, save_dir: str | None = None) -> dict[str, Any] | None:
    with g_FUNC_LOCK:
        ts = g_CTX["timestamp"]

        if task == "RAEB" and "RAEB" in g_TASKS:
            rr = g_CTX["raeb_result"]
            objs = rr["objs"]
            masks = rr["masks"]
            cuboids = rr["tracked_cuboids"]

            if objs is None:
                objs = np.empty((0, 7), dtype=np.float32)
            if masks is None:
                masks = np.zeros((0, 0, 0), dtype=np.uint8)
            if cuboids is None:
                cuboids = np.zeros((0, 9), dtype=np.float32)

            vis_img_2d = postproc_VisInstances(
                img=g_CTX["img"],
                objs=objs,
                masks=masks,
            )
            g_CTX["raeb_vis"]["vis_img_2d"] = vis_img_2d

            labels_3d: list[str] | None = None
            track_info = rr.get("track_info", None)
            if isinstance(track_info, np.ndarray) and track_info.ndim == 2 and track_info.shape[1] >= 3:
                ti = np.asarray(track_info, dtype=np.int32)
                n = int(min(cuboids.shape[0], ti.shape[0]))
                if n > 0:
                    state_map = {0: "N", 1: "T", 2: "L", 3: "R"}
                    labels_3d = [
                        f"ID {int(ti[i, 0])} {state_map.get(int(ti[i, 1]), str(int(ti[i, 1])))} A{int(ti[i, 2])}"
                        for i in range(n)
                    ]
                    if n < int(cuboids.shape[0]):
                        labels_3d.extend([""] * (int(cuboids.shape[0]) - n))

            vis_img_3d = cuboids_VisCuboids(
                cyl_img=vis_img_2d,
                cuboids=cuboids,
                labels=labels_3d,
            )
            g_CTX["raeb_vis"]["vis_img_3d"] = vis_img_3d

            if save_dir:
                os.makedirs(save_dir, exist_ok=True)
                path_3d = os.path.join(save_dir, f"raeb_vis_3d_{ts}.png")
                cv2.imwrite(path_3d, vis_img_3d)
                logger.info(f"Save image: {os.path.abspath(path_3d)}")

            return g_CTX["raeb_vis"]

        if task == "OP" and "OP" in g_TASKS:
            return g_CTX["op_vis"]

        return None


def VisPer_PrintResult(task: str, res: dict[str, Any] | None = None) -> None:
    def mat_to_text(m: Any) -> str:
        if m is None:
            return "empty"
        if not isinstance(m, np.ndarray):
            return f"non-ndarray({type(m)})"
        if m.ndim != 2:
            return f"non-2d(ndim={m.ndim})"

        rows, cols = m.shape[:2]
        c = 1 if m.ndim == 2 else m.shape[2]

        parts: list[str] = []
        parts.append(f"{rows}x{cols} C{c}\n[")
        for r in range(rows):
            row_items: list[str] = ["["]
            for cc in range(cols):
                if c == 1:
                    row_items.append(f"{m[r, cc]}")
                else:
                    vals = ", ".join(str(m[r, cc, ch]) for ch in range(c))
                    row_items.append(f"({vals})")
                row_items.append(",")
            row_items.append("],")
            parts.append("".join(row_items))
        parts.append("]")
        return "".join(parts)

    logger.debug("-------------------------------------")

    if res is None:
        with g_FUNC_LOCK:
            if task == "RAEB":
                res = g_CTX["raeb_result"]
                ts = g_CTX["timestamp"]
            elif task == "OP":
                res = g_CTX["op_result"]
                ts = g_CTX["timestamp"]
            else:
                logger.warning(f"VisPer_PrintResult: unknown task: {task}")
                logger.debug("-------------------------------------")
                return
    else:
        # res 已经是快照，这里只从中取 timestamp
        ts = res.get("timestamp", None)

    if task == "RAEB":
        logger.debug(f"REAB timestamp: {ts}")
        logger.debug(f"REAB track_info: {mat_to_text(res.get('track_info'))}")
        logger.debug(f"REAB tracked_cuboids_raw: {mat_to_text(res.get('tracked_cuboids_raw'))}")
        logger.debug(f"REAB tracked_cuboids: {mat_to_text(res.get('tracked_cuboids'))}")
        logger.debug(f"REAB tracked_cuboids_vel: {mat_to_text(res.get('tracked_cuboids_vel'))}")
    elif task == "OP":
        logger.debug(f"OP timestamp: {ts}")
        logger.debug(f"OP preds: {mat_to_text(res.get('preds'))}")
    else:
        logger.warning(f"VisPer_PrintResult: unknown task: {task}")

    logger.debug("-------------------------------------")


def VisPer_RunCallBack() -> None:
    def has_visl_trigger_file_in_cwd() -> bool:
        try:
            p = Path.cwd() / "visl"
            return p.exists() and p.is_file()
        except Exception:
            return False

    need_dump_vis = has_visl_trigger_file_in_cwd()
    save_dir = "vis" if need_dump_vis else None

    if "RAEB" in g_TASKS:
        if need_dump_vis:
            _ = VisPer_GetVis("RAEB", save_dir)

        raeb_res = VisPer_GetResult("RAEB")
        if raeb_res is not None:
            VisPer_PrintResult("RAEB", raeb_res)
            # 回调指针本身受 InitTask/RegCallback 保护，这里只读不写
            cb = None
            with g_FUNC_LOCK:
                cb = g_RAEB_CALLBACK
            if cb:
                cb(raeb_res)

    if "OP" in g_TASKS:
        op_res = VisPer_GetResult("OP")
        if op_res is not None:
            VisPer_PrintResult("OP", op_res)
            cb = None
            with g_FUNC_LOCK:
                cb = g_OP_CALLBACK
            if cb:
                cb(op_res)


def VisPer_CleanUp() -> None:
    """
    清理资源：关闭 RAEB 后台线程等。
    注意：不要在持有 g_FUNC_LOCK 的情况下 join 线程，避免死锁。
    """
    global g_RAEB_STAGEB_THREAD

    # 先在锁内取出线程句柄并置空全局引用
    with g_FUNC_LOCK:
        th = g_RAEB_STAGEB_THREAD
        g_RAEB_STAGEB_THREAD = None

    # 再在锁外发送退出信号并等待线程结束
    if th is not None:
        try:
            # 向队列发送 None 作为退出哨兵
            g_RAEB_STAGEB_QUEUE.put(None)
        except Exception:
            # 队列已被销毁之类的情况，直接忽略
            return

        th.join()
