#pragma once

#include <opencv2/core.hpp>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

/**
 * @brief Initialize RAEB 3D cuboid estimator.
 *
 * @param whitelist JSON object (name -> { "id": int, "size": [l, w, h] }).
 * @param cyl_cam   JSON object (camera/cylindrical calibration parameters; fields depend on implementation).
 */
void cuboids_InitRAEB(
    const nlohmann::json::object_t& whitelist,
    const nlohmann::json::object_t& cyl_cam
);

/**
 * @brief Compute RAEB 3D cuboids.
 *
 * @param objs       cv::Mat, CV_32F, shape (N, 7):
 *                   [x1, y1, x2, y2, conf, cls, theta_rel]
 * @param grounding_z double, ground height prior (fallback).
 * @param masks      cv::Mat, CV_8U, shape (N, H, W), aligned with objs by index.
 *
 * @return cv::Mat, CV_32F, shape (N, 9):
 *         [cx, cy, cz, l, w, h, conf, cls, theta_abs]
 *         For failed cases: cx=cy=cz=0, l=w=h=-1.
 */
cv::Mat cuboids_ProcRAEB(
    const cv::Mat& objs,
    double grounding_z,
    const cv::Mat& masks
);

/**
 * @brief Compute cuboids from 2D detections and eight projected 3D-box corners.
 *
 * @param points3d CV_32F/CV_64F (M,18):
 *                 [p0.x,p0.y,...,p7.x,p7.y,conf,cls].
 *                 2D and 3D rows are matched by class and projected-box
 *                 IoU >= 0.70. Unmatched rows are kept from their source.
 */
cv::Mat cuboids_ProcRAEBAnd3D(
    const cv::Mat& objs,
    const cv::Mat& points3d,
    double grounding_z,
    const cv::Mat& masks
);

/**
 * @brief Compute cuboids using only 3D projected box points.
 *
 * @param points3d CV_32F/CV_64F, shape (M,18):
 *                 [p0.x,p0.y,...,p7.x,p7.y,conf,cls].
 */
cv::Mat cuboids_ProcRAEB3D(
    const cv::Mat& points3d,
    double grounding_z
);

/**
 * @brief Visualize 3D cuboids on cylindrical image.
 *
 * @param cyl_img_ori cv::Mat, CV_8UC3, shape (H, W, 3), BGR.
 * @param cuboids     cv::Mat, CV_32F, shape (N, 9):
 *                    [cx, cy, cz, l, w, h, conf, cls, theta_abs]
 *
 * @return cv::Mat, CV_8UC3, shape (H, W, 3), BGR.
 */
cv::Mat cuboids_VisCuboids(
    const cv::Mat& cyl_img_ori,
    const cv::Mat& cuboids,
    const std::vector<std::string>* labels = nullptr,
    bool show_info = false
);
