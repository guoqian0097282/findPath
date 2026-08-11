// cuboids_raeb.cpp
#include "cuboids_api.h"
#include "cuboids_cyl3dbox_impl.hpp"

#include <memory>
#include <stdexcept>

// ------------- 全局实例定义 -------------
static std::unique_ptr<Cyl3DBoxEstimator> g_3dest_raeb;

// ------------- 初始化 -------------
void cuboids_InitRAEB(
    const nlohmann::json::object_t& whitelist,
    const nlohmann::json::object_t& cyl_cam
) {
    // 对应 Python:
    // g_3dest_raeb = Cyl3DBoxEstimator(cyl_cam=cyl_cam, whitelist=whitelist)
    g_3dest_raeb = std::make_unique<Cyl3DBoxEstimator>(
        cyl_cam,
        whitelist
    );
}

// ------------- 3D 盒计算 -------------
cv::Mat cuboids_ProcRAEB(
    const cv::Mat& objs,
    double grounding_z,
    const cv::Mat& masks
) {
    if (!g_3dest_raeb) {
        throw std::runtime_error("Cyl3DBoxEstimator 未初始化：请先调用 cuboids_InitRAEB(...)");
    }


    cv::Mat cuboids = g_3dest_raeb->cuboids_from_boxes(
        objs,           // (N,7) [x1,y1,x2,y2,conf,cls,theta_rel]
        grounding_z,    // z_world 先验 / fallback
        masks          // 3D (N,H,W) CV_8U
    );

    return cuboids;
}

// ------------- 3D 盒可视化 -------------
cv::Mat cuboids_VisCuboids(
    const cv::Mat& cyl_img_ori,
    const cv::Mat& cuboids,
    const std::vector<std::string>* labels,
    bool show_info
) {
    if (!g_3dest_raeb) {
        throw std::runtime_error("Cyl3DBoxEstimator 未初始化：请先调用 cuboids_InitRAEB(...)");
    }

    cv::Mat vis_img_3d = g_3dest_raeb->draw_3dboxes_on_cyl(
        cyl_img_ori,
        cuboids,
        1,
        labels,
        8,
        nullptr,
        show_info
    );

    return vis_img_3d;
}
