from __future__ import annotations

from typing import Any

import numpy as np

from .postproc_angle import AnglePostProcConfig, AnglePostProcessor
from .postproc_angleseg_impl import AngleSegPostProcessor

__all__ = ["postproc_InitRAEB", "postproc_ProcRAEB", "postproc_VisInstances"]


g_segpost_raeb: AngleSegPostProcessor | None = None
g_anglepost_raeb: AnglePostProcessor | None = None


def postproc_InitRAEB(
    nc: int,
    nm: int,
    id2name: dict[int, str],
    whitelist: dict[int, dict],
    conf_thresh: float = 0.5,
    iou_thresh: float = 0.3,
    mask_thresh: float = 0.5,
    mask_up: int = 4,
    angle_postproc_cfg: dict[str, Any] | None = None,
) -> None:
    global g_segpost_raeb, g_anglepost_raeb

    g_segpost_raeb = AngleSegPostProcessor(
        nc=nc,
        nm=nm,
        id2name=id2name,
        whitelist=whitelist,
        conf_thresh=conf_thresh,
        iou_thresh=iou_thresh,
        mask_thresh=mask_thresh,
        mask_up=mask_up,
    )
    g_anglepost_raeb = AnglePostProcessor(AnglePostProcConfig.from_dict(angle_postproc_cfg))


def postproc_ProcRAEB(
    det_cat: np.ndarray,
    proto: np.ndarray,
    angle: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    if g_segpost_raeb is None:
        raise RuntimeError("SegPostProcessor is not initialized. Call postproc_InitRAEB(...) first.")
    if g_anglepost_raeb is None:
        raise RuntimeError("AnglePostProcessor is not initialized. Call postproc_InitRAEB(...) first.")

    angle_decoded = g_anglepost_raeb.decode_angle(angle)
    objs, masks = g_segpost_raeb.postprocess(
        det_cat=det_cat,
        proto=proto,
        angle=angle_decoded,
    )
    objs = g_anglepost_raeb.process(objs)
    return objs, masks


def postproc_VisInstances(
    img: np.ndarray,
    objs: np.ndarray,
    masks: np.ndarray,
    track_info: np.ndarray | None = None,
) -> np.ndarray:
    if g_segpost_raeb is None:
        raise RuntimeError("SegPostProcessor is not initialized. Call postproc_InitRAEB(...) first.")

    if track_info is None:
        return g_segpost_raeb.draw_angleins(
            img=img,
            objs=objs,
            masks=masks,
        )

    return g_segpost_raeb.draw_angleins_with_track_info(
        img=img,
        objs=objs,
        masks=masks,
        track_info=track_info,
    )
