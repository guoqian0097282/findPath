from __future__ import annotations

import numpy as np
from visper.cuboids.cuboids_cyl3dbox_impl import Cyl3DBoxEstimator

# 全局实例（初始化前为 None）
g_3dest_raeb: Cyl3DBoxEstimator | None = None

def cuboids_InitRAEB(
        whitelist: dict[str, dict],
        cyl_cam: dict[str, float],
) -> None:
    """
    初始化 2D 分割后处理与 3D 落点-尺寸估计器（单图版）。
    """
    global g_3dest_raeb

    g_3dest_raeb = Cyl3DBoxEstimator(
        cyl_cam=cyl_cam,
        whitelist=whitelist,
    )


def cuboids_ProcRAEB(
        objs: np.ndarray,
        grounding_z: float,
        masks: np.ndarray,
        track_ids: np.ndarray | None = None,
):

    # 3D：接地点→尺寸→盒（基于原图坐标的接地点像素进行投影）
    cuboids = g_3dest_raeb.cuboids_from_boxes(
        objs=objs,
        z=grounding_z,
        masks=masks,
        track_ids=track_ids,
    )

    return cuboids


def cuboids_VisCuboids(
        cyl_img: np.ndarray,
        cuboids: np.ndarray,
        labels: list[str] | None = None,
        show_info: bool = False,
):
    vis_img_3d = g_3dest_raeb.draw_3dboxes_on_cyl(
        cyl_img=cyl_img,
        cuboids=cuboids,
        labels=labels,
        show_info=show_info,
    )

    return vis_img_3d
