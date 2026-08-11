// raeb_postproc.cpp

#include <memory>
#include <stdexcept>

#include "postproc_api.h"
#include "postproc_angleseg_impl.hpp"

// ------------- 全局实例（对齐 Python: g_segpost_raeb）-------------
std::unique_ptr<AngleSegPostProcessor> g_segpost_raeb = nullptr;

// ------------- 初始化 -------------
// 对齐 Python: postproc_InitRAEB(nc, nm, id2name, whitelist, conf_thresh, iou_thresh, mask_thresh, mask_up)
void postproc_InitRAEB(
    int nc,
    int nm,
    const nlohmann::json::object_t& id2name,
    const nlohmann::json::object_t& whitelist,
    float conf_thresh,
    float iou_thresh,
    float mask_thresh,
    int mask_up,
    const nlohmann::json::object_t& angle_postproc
) {
    g_segpost_raeb = std::make_unique<AngleSegPostProcessor>(
        nc,
        nm,
        id2name,
        whitelist,
        conf_thresh,
        iou_thresh,
        mask_thresh,
        mask_up,
        angle_postproc
    );
}

// ------------- 单帧后处理（只做 2D）-------------
// 对齐 Python: postproc_ProcRAEB(det_cat, proto, angle) -> objs
cv::Mat postproc_ProcRAEB(
    const cv::Mat& det_cat,
    const cv::Mat& proto,
    const cv::Mat& angle,
    bool proto_chw
) {
    if (!g_segpost_raeb) {
        throw std::runtime_error("SegPostProcessor 未初始化：请先调用 postproc_InitRAEB(...)");
    }

    // AngleSegPostProcessor::postprocess 返回：
    // objs: Nx7 CV_32F
    return g_segpost_raeb->postprocess(det_cat, proto, angle, proto_chw);
}

RaebPostprocResult postproc_ProcRAEBWithMasks(
    const cv::Mat& det_cat,
    const cv::Mat& proto,
    const cv::Mat& angle,
    bool proto_chw
) {
    if (!g_segpost_raeb) {
        throw std::runtime_error("SegPostProcessor 未初始化：请先调用 postproc_InitRAEB(...)");
    }

    auto result = g_segpost_raeb->postprocess_with_masks(det_cat, proto, angle, proto_chw);
    return RaebPostprocResult{result.objs, result.masks};
}


// ------------- 实例可视化（不带 tracking）-------------
// 对齐 Python: postproc_VisInstances(img, objs, masks, track_info=None)
cv::Mat postproc_VisInstances(
    const cv::Mat& img,
    const cv::Mat& objs,
    const cv::Mat& masks
) {
    if (!g_segpost_raeb) {
        throw std::runtime_error("SegPostProcessor 未初始化：请先调用 postproc_InitRAEB(...)");
    }
    if (img.empty() || img.type() != CV_8UC3 || img.dims != 2) {
        throw std::runtime_error("img must be (H,W,3) CV_8UC3");
    }

    return g_segpost_raeb->draw_angleins(
        img,
        objs,
        masks
    );
}

// ------------- 实例可视化（带 tracking）-------------
// 对齐 Python: postproc_VisInstances(img, objs, masks, track_info)
cv::Mat postproc_VisInstances(
    const cv::Mat& img,
    const cv::Mat& objs,
    const cv::Mat& masks,
    const cv::Mat& track_info
) {
    if (!g_segpost_raeb) {
        throw std::runtime_error("SegPostProcessor 未初始化：请先调用 postproc_InitRAEB(...)");
    }

    if (track_info.empty()) {
        return g_segpost_raeb->draw_angleins(img, objs, masks);
    }

    return g_segpost_raeb->draw_angleins_with_track_info(
        img,
        /*objs=*/objs,
        /*masks=*/masks,
        /*track_info=*/track_info
    );
}
