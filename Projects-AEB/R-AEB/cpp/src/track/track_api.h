#pragma once

#include <cstdint>
#include <string>
#include <tuple>

#include <opencv2/core.hpp>

void cuboid_tracker_Init(
    float track_high_thresh = 0.5f,
    float track_low_thresh = 0.3f,
    float match_thresh = 0.65f,
    float new_track_thresh = 0.6f,
    int track_buffer = 30,
    int frame_rate = 30,
    float default_dt_ms = 100.0f,
    float timestamp_scale = 1e-3f,
    float center_gate_main_m = 4.0f,
    float center_gate_low_m = 5.5f,
    float weight_cls_main = 0.55f,
    float weight_center_main = 0.35f,
    float weight_size_main = 0.10f,
    float weight_cls_low = 0.60f,
    float weight_center_low = 0.30f,
    float weight_size_low = 0.10f,
    float sigma_a = 2.0f,
    float sigma_z = 0.18f,
    float vel_static_thresh = 0.20f,
    float vel_move_thresh = 0.50f,
    int stable_frames = 1,
    int max_missed = 30,
    float vel_clip_mps = 8.0f,
    float max_innovation_m = 1.2f,
    float vel_meas_blend = 0.18f,
    float vel_meas_blend_max = 0.45f,
    float vel_meas_innov_for_max = 1.2f,
    float pos_comp_sec = 0.06f,
    float pos_comp_min_speed = 0.15f,
    float pos_comp_max_m = 0.60f,
    float pos_comp_new_scale = 0.60f,
    float pos_comp_innov_for_zero = 1.2f,
    float pos_comp_dir_meas_base = 0.20f,
    float pos_comp_dir_meas_turn_gain = 0.60f,
    float pos_comp_reverse_cos = -0.20f,
    int pos_comp_reverse_hold = 3,
    float pos_comp_reverse_speed = 0.40f,
    float turn_recover_cos = 0.30f,
    float turn_recover_innov_m = 0.10f,
    int turn_recover_hold = 6,
    int turn_recover_smooth_frames = 3,
    float turn_recover_vel_blend = 0.65f,
    float turn_recover_pos_blend = 0.70f,
    float turn_recover_vel_blend_smooth = 0.20f,
    float turn_recover_pos_blend_smooth = 0.25f,
    float turn_recover_comp_boost = 1.50f,
    float turn_recover_min_speed = 0.25f,
    int turn_recover_comp_freeze = 3
);

std::tuple<cv::Mat, cv::Mat> cuboid_tracker_Track(
    std::int64_t timestamp,
    const cv::Mat& cuboids
);

std::tuple<cv::Mat, cv::Mat, cv::Mat, cv::Mat> cuboid_tracker_TrackAndEstimate(
    std::int64_t timestamp,
    const cv::Mat& cuboids,
    double ego_yaw = 0.0,
    const std::string& mode = "ego"
);

void cuboid_tracker_Reset();
