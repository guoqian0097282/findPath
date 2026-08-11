#include "track_api.h"

#include <memory>
#include <stdexcept>

#include "track_impl.hpp"

namespace {
std::unique_ptr<cuboid_tracker_impl::CuboidTracker> g_cuboid_tracker;
}

void cuboid_tracker_Init(
    float track_high_thresh,
    float track_low_thresh,
    float match_thresh,
    float new_track_thresh,
    int track_buffer,
    int frame_rate,
    float default_dt_ms,
    float timestamp_scale,
    float center_gate_main_m,
    float center_gate_low_m,
    float weight_cls_main,
    float weight_center_main,
    float weight_size_main,
    float weight_cls_low,
    float weight_center_low,
    float weight_size_low,
    float sigma_a,
    float sigma_z,
    float vel_static_thresh,
    float vel_move_thresh,
    int stable_frames,
    int max_missed,
    float vel_clip_mps,
    float max_innovation_m,
    float vel_meas_blend,
    float vel_meas_blend_max,
    float vel_meas_innov_for_max,
    float pos_comp_sec,
    float pos_comp_min_speed,
    float pos_comp_max_m,
    float pos_comp_new_scale,
    float pos_comp_innov_for_zero,
    float pos_comp_dir_meas_base,
    float pos_comp_dir_meas_turn_gain,
    float pos_comp_reverse_cos,
    int pos_comp_reverse_hold,
    float pos_comp_reverse_speed,
    float turn_recover_cos,
    float turn_recover_innov_m,
    int turn_recover_hold,
    int turn_recover_smooth_frames,
    float turn_recover_vel_blend,
    float turn_recover_pos_blend,
    float turn_recover_vel_blend_smooth,
    float turn_recover_pos_blend_smooth,
    float turn_recover_comp_boost,
    float turn_recover_min_speed,
    int turn_recover_comp_freeze
) {
    g_cuboid_tracker = std::make_unique<cuboid_tracker_impl::CuboidTracker>(
        track_high_thresh,
        track_low_thresh,
        match_thresh,
        new_track_thresh,
        track_buffer,
        frame_rate,
        default_dt_ms,
        timestamp_scale,
        center_gate_main_m,
        center_gate_low_m,
        weight_cls_main,
        weight_center_main,
        weight_size_main,
        weight_cls_low,
        weight_center_low,
        weight_size_low,
        sigma_a,
        sigma_z,
        vel_static_thresh,
        vel_move_thresh,
        stable_frames,
        max_missed,
        vel_clip_mps,
        max_innovation_m,
        vel_meas_blend,
        vel_meas_blend_max,
        vel_meas_innov_for_max,
        pos_comp_sec,
        pos_comp_min_speed,
        pos_comp_max_m,
        pos_comp_new_scale,
        pos_comp_innov_for_zero,
        pos_comp_dir_meas_base,
        pos_comp_dir_meas_turn_gain,
        pos_comp_reverse_cos,
        pos_comp_reverse_hold,
        pos_comp_reverse_speed,
        turn_recover_cos,
        turn_recover_innov_m,
        turn_recover_hold,
        turn_recover_smooth_frames,
        turn_recover_vel_blend,
        turn_recover_pos_blend,
        turn_recover_vel_blend_smooth,
        turn_recover_pos_blend_smooth,
        turn_recover_comp_boost,
        turn_recover_min_speed,
        turn_recover_comp_freeze
    );
}

std::tuple<cv::Mat, cv::Mat> cuboid_tracker_Track(
    std::int64_t timestamp,
    const cv::Mat& cuboids
) {
    if (!g_cuboid_tracker) {
        throw std::runtime_error("CuboidTracker not initialized: call cuboid_tracker_Init first");
    }
    return g_cuboid_tracker->update(timestamp, cuboids);
}

std::tuple<cv::Mat, cv::Mat, cv::Mat, cv::Mat> cuboid_tracker_TrackAndEstimate(
    std::int64_t timestamp,
    const cv::Mat& cuboids,
    double ego_yaw,
    const std::string& mode
) {
    if (!g_cuboid_tracker) {
        throw std::runtime_error("CuboidTracker not initialized: call cuboid_tracker_Init first");
    }
    return g_cuboid_tracker->update_and_estimate(timestamp, cuboids, ego_yaw, mode);
}

void cuboid_tracker_Reset() {
    if (!g_cuboid_tracker) {
        throw std::runtime_error("CuboidTracker not initialized: call cuboid_tracker_Init first");
    }
    g_cuboid_tracker->reset();
}
