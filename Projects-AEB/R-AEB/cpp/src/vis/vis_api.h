#pragma once

#include <cstdint>

#include <opencv2/core.hpp>

// RAEB StageB 可视化子模块：
// 当 cwd 下存在文件 "visl" 时，自动将当前帧可视化结果写入 vis/ 目录。
bool vis_HasRaebStageBTrigger();

void vis_RunRaebStageBModule(
    std::int64_t timestamp,
    const cv::Mat& img_bgr,
    const cv::Mat& objs_in,
    const cv::Mat& masks_in,
    const cv::Mat& track_info,
    const cv::Mat& tracked_cuboids_in,
    const cv::Mat& tracked_cuboids_vel_in
);
