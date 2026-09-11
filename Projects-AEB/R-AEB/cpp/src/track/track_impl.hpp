// ============================================================
// CuboidTracker.hpp - 完整修复代码
// 包含Y轴速度方向一致性问题修复
// ============================================================

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

#include "track_utils/matching.hpp"

namespace cuboid_tracker_impl {

static constexpr int TRACK_NEW = 0;
static constexpr int TRACK_TRACKED = 1;
static constexpr int TRACK_LOST = 2;
static constexpr int TRACK_REMOVED = 3;

static constexpr int MOTION_UNKNOWN = 0;
static constexpr int MOTION_STATIC = 1;
static constexpr int MOTION_MOVING = 2;

struct CuboidTrack {
    int track_id = 0;
    cv::Vec<float, 9> cuboid{};
    int state = TRACK_NEW;
    int start_frame = 0;
    int frame_id = 0;
    int idx = -1;
    std::int64_t last_timestamp = 0;

    cv::Matx<double, 4, 1> x{};
    cv::Matx<double, 4, 4> P{};

    double last_meas_x = 0.0;
    double last_meas_y = 0.0;
    bool last_meas_valid = false;
    double last_innovation = 0.0;
    double meas_vx = 0.0;
    double meas_vy = 0.0;
    bool has_meas_velocity = false;
    double out_vx = 0.0;
    double out_vy = 0.0;
    bool out_vel_valid = false;
    int comp_freeze = 0;
    int maneuver_left = 0;

    int motion_state = MOTION_UNKNOWN;
    int stable_cnt = 0;
    int missed = 0;
    int hits = 0;

    // ===== Y轴速度平滑 =====
    double y_vel_smooth = 0.0;
    bool y_vel_smooth_valid = false;
    double y_vel_output = 0.0;
    bool y_vel_output_valid = false;
    
    // ===== Y轴速度历史 =====
    static constexpr int Y_HISTORY_SIZE = 20;
    double y_vel_history[Y_HISTORY_SIZE] = {0.0};
    int y_history_idx = 0;
    int y_history_count = 0;
    
    // ===== Y轴静止检测 =====
    double y_vel_abs_mean = 0.0;
    double y_vel_var = 0.0;
    int y_still_counter = 0;
    bool y_is_still = false;

    // ===== 异常值检测 =====
    double y_vel_predicted = 0.0;
    double y_vel_consistency = 1.0;
    double y_vel_confidence = 1.0;
    int y_outlier_counter = 0;
    bool y_is_outlier = false;
    double y_vel_median_history[5] = {0.0};
    int y_median_idx = 0;
    int y_median_count = 0;

    // ===== 方向反转检测 =====
    double y_vel_prev = 0.0;
    bool y_vel_prev_valid = false;
    int y_direction_stable_counter = 0;

    // ===== 新增：位置变化跟踪 =====
    double y_position_history[5] = {0.0};
    int y_pos_history_idx = 0;
    int y_pos_history_count = 0;
    double y_displacement_accum = 0.0;
    int y_consistent_motion_counter = 0;
    bool y_has_consistent_motion = false;
    double y_raw_measurement_vel = 0.0;
    bool y_raw_vel_valid = false;

    int age() const {
        return frame_id - start_frame + 1;
    }
};

class CuboidTracker {
public:
    using TrackPtr = std::shared_ptr<CuboidTrack>;

    CuboidTracker(
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

    void reset();

    std::tuple<cv::Mat, cv::Mat, cv::Mat, cv::Mat> update_and_estimate(
        std::int64_t timestamp,
        const cv::Mat& cuboids,
        double ego_yaw = 0.0,
        const std::string& mode = "ego"
    );

    std::tuple<cv::Mat, cv::Mat> update(std::int64_t timestamp, const cv::Mat& cuboids);

    void configureForLowSpeedScenario(double fps = 14.0, double max_speed_kmh = 20.0);

private:
    static double clip(double x, double lo, double hi);
    static cv::Vec<float, 9> row_as_vec9(const cv::Mat& m, int r);
    static cv::Mat gather_rows(const cv::Mat& src, const std::vector<int>& idx);
    static cv::Mat sanitize_cuboids(const cv::Mat& cuboids);
    static std::vector<char> det_valid_mask(const cv::Mat& cuboids);
    static int cls_to_int(float v);

    void init_kf_state(double px, double py, cv::Matx<double, 4, 1>& x, cv::Matx<double, 4, 4>& P) const;
    void kf_predict(
        const cv::Matx<double, 4, 1>& x,
        const cv::Matx<double, 4, 4>& P,
        double dt,
        cv::Matx<double, 4, 1>& x_pre,
        cv::Matx<double, 4, 4>& P_pre
    ) const;
    void kf_update(
        const cv::Matx<double, 4, 1>& x_pre,
        const cv::Matx<double, 4, 4>& P_pre,
        const cv::Matx<double, 2, 1>& z,
        cv::Matx<double, 4, 1>& x_post,
        cv::Matx<double, 4, 4>& P_post
    ) const;
    std::pair<double, double> clip_vel(double vx, double vy) const;
    cv::Matx<double, 4, 1> clip_speed(const cv::Matx<double, 4, 1>& x) const;
    cv::Matx<double, 4, 1> blend_measured_velocity(
        const TrackPtr& trk,
        const cv::Matx<double, 4, 1>& x_post,
        const cv::Matx<double, 2, 1>& z_used,
        double dt,
        double innovation_m
    ) const;

    std::pair<int, double> recovery_phase(const TrackPtr& trk) const;
    cv::Matx<double, 4, 1> apply_turn_recovery(
        const TrackPtr& trk,
        const cv::Matx<double, 4, 1>& x_post,
        const cv::Matx<double, 2, 1>& z_used,
        double vmx,
        double vmy,
        bool has_meas_velocity
    ) const;

    void update_motion_state(const TrackPtr& trk, double speed) const;
    void update_output_velocity(const TrackPtr& trk, double vx, double vy, bool has_measurement) const;
    double compute_dt(std::int64_t timestamp, std::int64_t last_timestamp) const;
    std::pair<double, double> predict_center(const TrackPtr& trk, std::int64_t timestamp) const;
    std::pair<double, double> position_comp_delta(
        const TrackPtr& trk,
        double vx,
        double vy,
        int track_state,
        int motion_state
    ) const;

    // ===== Y轴速度平滑核心方法 =====
    void updateYHistory(const TrackPtr& trk, double vy) const;
    void updateYPositionHistory(const TrackPtr& trk, double y_pos) const;
    void detectStillState(const TrackPtr& trk) const;
    
    // ===== 异常值检测与抑制方法 =====
    double detectAndSuppressOutlier(const TrackPtr& trk, double raw_vy, double dt) const;
    double computeVelocityConsistency(const TrackPtr& trk, double raw_vy) const;
    double medianFilterYVelocity(const TrackPtr& trk, double raw_vy) const;
    bool isPhysicalVelocityJump(const TrackPtr& trk, double raw_vy, double dt) const;
    
    // ===== 方向反转快速响应 =====
    double handleDirectionReversal(const TrackPtr& trk, double raw_vy, double dt) const;
    
    // ===== 新增：速度方向修复 =====
    double correctVelocityDirection(
        const TrackPtr& trk,
        double raw_vy,
        double dt,
        double current_y,
        double y_displacement
    ) const;
    bool hasConsistentMotion(const TrackPtr& trk) const;
    double computeRawMeasurementVelocity(const TrackPtr& trk, double current_y, double dt) const;
    
    double smoothYVelocity(
        const TrackPtr& trk,
        double raw_vy,
        double dt,
        double current_y
    ) const;
    double limitYVelocity(const TrackPtr& trk, double vy) const;

    static double size_cost(const cv::Vec<float, 9>& track_box, const cv::Vec<float, 9>& det_box, double eps = 1e-6);
    cv::Mat build_cost(
        const std::vector<TrackPtr>& tracks,
        const cv::Mat& detections,
        const std::vector<int>& det_indices,
        const std::optional<std::int64_t>& timestamp,
        double center_gate_m,
        double w_cls,
        double w_center,
        double w_size
    ) const;

    static std::tuple<std::vector<std::pair<int, int>>, std::vector<int>, std::vector<int>>
    linear_assignment(const cv::Mat& cost, float thresh);

    TrackPtr activate(const cv::Vec<float, 9>& det, int det_idx, std::int64_t timestamp);
    void update_track(const TrackPtr& trk, const cv::Vec<float, 9>& det, int det_idx, std::int64_t timestamp, int state) const;
    void predict_track_without_measure(const TrackPtr& trk, std::int64_t timestamp) const;

    static std::vector<TrackPtr> joint_tracks(const std::vector<TrackPtr>& a, const std::vector<TrackPtr>& b);
    static std::vector<TrackPtr> sub_tracks(const std::vector<TrackPtr>& a, const std::vector<TrackPtr>& b);
    std::pair<std::vector<TrackPtr>, std::vector<TrackPtr>> remove_duplicate(
        const std::vector<TrackPtr>& tracked,
        const std::vector<TrackPtr>& lost
    ) const;

    int next_id();

private:
    std::vector<TrackPtr> tracked_;
    std::vector<TrackPtr> lost_;
    std::vector<TrackPtr> removed_;

    int frame_id_ = 0;
    int track_count_ = 0;

    float track_high_thresh_ = 0.5f;
    float track_low_thresh_ = 0.3f;
    float match_thresh_ = 0.65f;
    float match_thresh_low_ = 0.80f;
    float match_thresh_new_ = 0.70f;
    float new_track_thresh_ = 0.6f;

    int track_buffer_ = 30;
    int max_time_lost_ = 30;

    double default_dt_sec_ = 0.1;
    double timestamp_scale_ = 1e-3;
    double min_dt_sec_ = 1e-6;
    double min_effective_dt_sec_ = 5e-3;
    double max_effective_dt_sec_ = 0.5;

    double center_gate_main_m_ = 4.0;
    double center_gate_low_m_ = 5.5;

    double weight_cls_main_ = 0.55;
    double weight_center_main_ = 0.35;
    double weight_size_main_ = 0.10;

    double weight_cls_low_ = 0.60;
    double weight_center_low_ = 0.30;
    double weight_size_low_ = 0.10;

    double sigma_a_ = 2.0;
    double sigma_z_ = 0.18;

    double vel_static_thresh_ = 0.20;
    double vel_move_thresh_ = 0.50;
    int stable_frames_ = 1;
    int max_missed_ = 30;
    double vel_clip_mps_ = 8.0;
    double max_innovation_m_ = 1.2;
    double vel_meas_blend_ = 0.18;
    double vel_meas_blend_max_ = 0.45;
    double vel_meas_innov_for_max_ = 1.2;
    double output_vel_alpha_ = 0.25;
    double output_vel_alpha_lost_ = 0.10;

    double pos_comp_sec_ = 0.06;
    double pos_comp_min_speed_ = 0.15;
    double pos_comp_max_m_ = 0.60;
    double pos_comp_new_scale_ = 0.60;
    double pos_comp_innov_for_zero_ = 1.2;
    double pos_comp_dir_meas_base_ = 0.20;
    double pos_comp_dir_meas_turn_gain_ = 0.60;
    double pos_comp_reverse_cos_ = -0.20;
    int pos_comp_reverse_hold_ = 3;
    double pos_comp_reverse_speed_ = 0.40;

    double turn_recover_cos_ = 0.30;
    double turn_recover_innov_m_ = 0.10;
    int turn_recover_hold_ = 6;
    int turn_recover_smooth_frames_ = 3;
    int turn_recover_pull_frames_ = 3;
    double turn_recover_vel_blend_ = 0.65;
    double turn_recover_pos_blend_ = 0.70;
    double turn_recover_vel_blend_smooth_ = 0.20;
    double turn_recover_pos_blend_smooth_ = 0.25;
    double turn_recover_comp_boost_ = 1.50;
    double turn_recover_min_speed_ = 0.25;
    int turn_recover_comp_freeze_ = 3;

    int mode_id_ego_ = 0;
    int mode_id_world_ = 1;

    double large_cost_ = 1e6;

    // ===== Y轴速度平滑参数 =====
    bool y_vel_smooth_enabled_ = true;
    
    double y_smooth_alpha_ = 0.30;
    double y_max_vel_ = 5.0;
    double y_vel_change_limit_ = 3.0;
    
    double y_still_thresh_ = 0.10;
    double y_still_var_thresh_ = 0.02;
    int y_still_confirm_frames_ = 3;
    
    // ===== 异常检测参数 =====
    double y_outlier_thresh_ = 0.5;
    double y_jump_thresh_ = 1.5;
    double y_outlier_decay_ = 0.3;
    int y_outlier_confirm_frames_ = 2;
    
    double actual_fps_ = 14.0;
    double actual_dt_ = 0.071;

    // ===== 新增：方向修复参数 =====
    double y_direction_correction_thresh_ = 0.008; // 位移阈值
    double y_consistent_motion_thresh_ = 0.03;     // 一致运动速度阈值
    int y_consistent_motion_frames_ = 3;           // 确认帧数
    double y_raw_vel_blend_ = 0.6;                  // 原始速度混合比例
};

// ============================================================
// 构造函数实现
// ============================================================
inline CuboidTracker::CuboidTracker(
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
    track_high_thresh_ = track_high_thresh;
    track_low_thresh_ = track_low_thresh;
    new_track_thresh_ = new_track_thresh;
    match_thresh_ = match_thresh;
    match_thresh_low_ = std::min(0.95f, match_thresh_ + 0.15f);
    match_thresh_new_ = std::min(0.95f, std::max(0.70f, match_thresh_));

    track_buffer_ = track_buffer;
    max_time_lost_ = static_cast<int>(frame_rate / 30.0f * static_cast<float>(track_buffer_));

    default_dt_sec_ = static_cast<double>(default_dt_ms) / 1000.0;
    timestamp_scale_ = static_cast<double>(timestamp_scale);

    center_gate_main_m_ = std::max(0.1, static_cast<double>(center_gate_main_m));
    center_gate_low_m_ = std::max(center_gate_main_m_, static_cast<double>(center_gate_low_m));

    weight_cls_main_ = weight_cls_main;
    weight_center_main_ = weight_center_main;
    weight_size_main_ = weight_size_main;
    weight_cls_low_ = weight_cls_low;
    weight_center_low_ = weight_center_low;
    weight_size_low_ = weight_size_low;

    sigma_a_ = std::max(1e-6, static_cast<double>(sigma_a));
    sigma_z_ = std::max(1e-6, static_cast<double>(sigma_z));
    vel_static_thresh_ = static_cast<double>(vel_static_thresh);
    vel_move_thresh_ = static_cast<double>(vel_move_thresh);
    stable_frames_ = std::max(1, stable_frames);
    max_missed_ = std::max(1, max_missed);
    vel_clip_mps_ = std::max(0.1, static_cast<double>(vel_clip_mps));
    max_innovation_m_ = std::max(0.1, static_cast<double>(max_innovation_m));
    vel_meas_blend_ = clip(static_cast<double>(vel_meas_blend), 0.0, 1.0);
    vel_meas_blend_max_ = clip(std::max(vel_meas_blend_, static_cast<double>(vel_meas_blend_max)), 0.0, 1.0);
    vel_meas_innov_for_max_ = std::max(1e-3, static_cast<double>(vel_meas_innov_for_max));

    pos_comp_sec_ = std::max(0.0, static_cast<double>(pos_comp_sec));
    pos_comp_min_speed_ = std::max(0.0, static_cast<double>(pos_comp_min_speed));
    pos_comp_max_m_ = std::max(0.0, static_cast<double>(pos_comp_max_m));
    pos_comp_new_scale_ = clip(static_cast<double>(pos_comp_new_scale), 0.0, 1.0);
    pos_comp_innov_for_zero_ = std::max(1e-3, static_cast<double>(pos_comp_innov_for_zero));
    pos_comp_dir_meas_base_ = clip(static_cast<double>(pos_comp_dir_meas_base), 0.0, 1.0);
    pos_comp_dir_meas_turn_gain_ = std::max(0.0, static_cast<double>(pos_comp_dir_meas_turn_gain));
    pos_comp_reverse_cos_ = clip(static_cast<double>(pos_comp_reverse_cos), -1.0, 1.0);
    pos_comp_reverse_hold_ = std::max(0, pos_comp_reverse_hold);
    pos_comp_reverse_speed_ = std::max(0.0, static_cast<double>(pos_comp_reverse_speed));

    turn_recover_cos_ = clip(static_cast<double>(turn_recover_cos), -1.0, 1.0);
    turn_recover_innov_m_ = std::max(0.0, static_cast<double>(turn_recover_innov_m));
    turn_recover_hold_ = std::max(0, turn_recover_hold);
    turn_recover_smooth_frames_ = std::max(0, std::min(turn_recover_smooth_frames, turn_recover_hold_));
    turn_recover_pull_frames_ = std::max(1, turn_recover_hold_ - turn_recover_smooth_frames_);
    turn_recover_vel_blend_ = clip(static_cast<double>(turn_recover_vel_blend), 0.0, 1.0);
    turn_recover_pos_blend_ = clip(static_cast<double>(turn_recover_pos_blend), 0.0, 1.0);
    turn_recover_vel_blend_smooth_ = clip(static_cast<double>(turn_recover_vel_blend_smooth), 0.0, 1.0);
    turn_recover_pos_blend_smooth_ = clip(static_cast<double>(turn_recover_pos_blend_smooth), 0.0, 1.0);
    turn_recover_comp_boost_ = std::max(0.0, static_cast<double>(turn_recover_comp_boost));
    turn_recover_min_speed_ = std::max(0.0, static_cast<double>(turn_recover_min_speed));
    turn_recover_comp_freeze_ = std::max(0, turn_recover_comp_freeze);

    configureForLowSpeedScenario(14.0, 20.0);
    reset();
}

// ============================================================
// 公共接口实现
// ============================================================
inline void CuboidTracker::reset() {
    tracked_.clear();
    lost_.clear();
    removed_.clear();
    frame_id_ = 0;
    track_count_ = 0;
}

inline std::tuple<cv::Mat, cv::Mat> CuboidTracker::update(std::int64_t timestamp, const cv::Mat& cuboids) {
    cv::Mat track_info;
    cv::Mat tracked_cuboids_raw;
    cv::Mat tracked_cuboids;
    cv::Mat tracked_cuboids_vel;
    std::tie(track_info, tracked_cuboids_raw, tracked_cuboids, tracked_cuboids_vel) =
        update_and_estimate(timestamp, cuboids, 0.0, "ego");
    (void)tracked_cuboids;
    (void)tracked_cuboids_vel;
    return {track_info, tracked_cuboids_raw};
}

// ============================================================
// 配置参数调整
// ============================================================
inline void CuboidTracker::configureForLowSpeedScenario(double fps, double max_speed_kmh) {
    actual_fps_ = std::max(5.0, std::min(30.0, fps));
    actual_dt_ = 1.0 / actual_fps_;
    
    y_smooth_alpha_ = 0.75;
    y_still_thresh_ = 0.02;
    y_still_var_thresh_ = 0.005;
    y_still_confirm_frames_ = 3;
    y_vel_change_limit_ = 12.0;
    y_max_vel_ = std::max(2.0, std::min(5.0, max_speed_kmh / 3.6 * 1.8));
    
    y_outlier_thresh_ = 0.15;
    y_jump_thresh_ = 6.0;
    y_outlier_decay_ = 0.35;
    y_outlier_confirm_frames_ = 5;
    
    output_vel_alpha_ = 0.5;
    output_vel_alpha_lost_ = 0.15;

    // 新增：方向修复参数
    y_direction_correction_thresh_ = 0.008;
    y_consistent_motion_thresh_ = 0.03;
    y_consistent_motion_frames_ = 3;
    y_raw_vel_blend_ = 0.6;
}

// ============================================================
// 静态辅助函数
// ============================================================
inline double CuboidTracker::clip(double x, double lo, double hi) {
    return std::max(lo, std::min(hi, x));
}

inline cv::Vec<float, 9> CuboidTracker::row_as_vec9(const cv::Mat& m, int r) {
    cv::Vec<float, 9> out{};
    const float* p = m.ptr<float>(r);
    for (int i = 0; i < 9; ++i) {
        out[i] = p[i];
    }
    return out;
}

inline cv::Mat CuboidTracker::gather_rows(const cv::Mat& src, const std::vector<int>& idx) {
    if (idx.empty()) {
        return cv::Mat(0, src.cols, src.type());
    }
    cv::Mat out(static_cast<int>(idx.size()), src.cols, src.type());
    for (int i = 0; i < static_cast<int>(idx.size()); ++i) {
        src.row(idx[i]).copyTo(out.row(i));
    }
    return out;
}

inline cv::Mat CuboidTracker::sanitize_cuboids(const cv::Mat& cuboids) {
    if (cuboids.empty()) {
        return cv::Mat(0, 9, CV_32F);
    }
    if (cuboids.dims != 2) {
        throw std::runtime_error("cuboids must be 2D");
    }
    if (cuboids.cols < 9) {
        throw std::runtime_error("cuboids shape[1] must be >=9");
    }

    cv::Mat arr;
    if (cuboids.type() != CV_32F) {
        cuboids.convertTo(arr, CV_32F);
    } else {
        arr = cuboids;
    }

    if (arr.cols > 9) {
        arr = arr.colRange(0, 9).clone();
    } else {
        arr = arr.clone();
    }
    return arr;
}

inline std::vector<char> CuboidTracker::det_valid_mask(const cv::Mat& cuboids) {
    std::vector<char> valid(cuboids.rows, 0);
    for (int i = 0; i < cuboids.rows; ++i) {
        const float* r = cuboids.ptr<float>(i);
        bool finite = true;
        for (int j = 0; j < cuboids.cols; ++j) {
            if (!std::isfinite(static_cast<double>(r[j]))) {
                finite = false;
                break;
            }
        }
        const bool size_ok = (r[3] > 0.0f) && (r[4] > 0.0f) && (r[5] > 0.0f);
        const bool conf_ok = std::isfinite(static_cast<double>(r[6])) && (r[6] > 0.0f);
        const bool cls_ok = std::isfinite(static_cast<double>(r[7]));
        valid[i] = static_cast<char>(finite && size_ok && conf_ok && cls_ok);
    }
    return valid;
}

inline int CuboidTracker::cls_to_int(float v) {
    return static_cast<int>(std::lround(static_cast<double>(v)));
}

// ============================================================
// KF相关函数
// ============================================================
inline void CuboidTracker::init_kf_state(
    double px,
    double py,
    cv::Matx<double, 4, 1>& x,
    cv::Matx<double, 4, 4>& P
) const {
    x = cv::Matx<double, 4, 1>(px, py, 0.0, 0.0);
    P = cv::Matx<double, 4, 4>::eye();
    P(0, 0) = 1.0;
    P(1, 1) = 1.0;
    P(2, 2) = 4.0;
    P(3, 3) = 4.0;
}

inline void CuboidTracker::kf_predict(
    const cv::Matx<double, 4, 1>& x,
    const cv::Matx<double, 4, 4>& P,
    double dt,
    cv::Matx<double, 4, 1>& x_pre,
    cv::Matx<double, 4, 4>& P_pre
) const {
    cv::Matx<double, 4, 4> F = cv::Matx<double, 4, 4>::eye();
    F(0, 2) = dt;
    F(1, 3) = dt;

    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt2 * dt2;
    const double sa2 = sigma_a_ * sigma_a_;

    const double q11 = 0.25 * dt4 * sa2;
    const double q12 = 0.5 * dt3 * sa2;
    const double q22 = dt2 * sa2;

    cv::Matx<double, 4, 4> Q = cv::Matx<double, 4, 4>::zeros();
    Q(0, 0) = q11;
    Q(0, 2) = q12;
    Q(2, 0) = q12;
    Q(2, 2) = q22;
    Q(1, 1) = q11;
    Q(1, 3) = q12;
    Q(3, 1) = q12;
    Q(3, 3) = q22;

    x_pre = F * x;
    P_pre = F * P * F.t() + Q;
}

inline void CuboidTracker::kf_update(
    const cv::Matx<double, 4, 1>& x_pre,
    const cv::Matx<double, 4, 4>& P_pre,
    const cv::Matx<double, 2, 1>& z,
    cv::Matx<double, 4, 1>& x_post,
    cv::Matx<double, 4, 4>& P_post
) const {
    cv::Matx<double, 2, 4> H = cv::Matx<double, 2, 4>::zeros();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;
    const cv::Matx<double, 2, 2> R = cv::Matx<double, 2, 2>::eye() * (sigma_z_ * sigma_z_);

    const cv::Matx<double, 2, 1> y = z - (H * x_pre);
    const cv::Matx<double, 2, 2> S = H * P_pre * H.t() + R;

    const double a = S(0, 0);
    const double b = S(0, 1);
    const double c = S(1, 0);
    const double d = S(1, 1);
    const double det = a * d - b * c;
    if (!std::isfinite(det) || std::abs(det) < 1e-18) {
        x_post = x_pre;
        P_post = P_pre;
        return;
    }

    cv::Matx<double, 2, 2> Sinv;
    const double inv_det = 1.0 / det;
    Sinv(0, 0) = d * inv_det;
    Sinv(0, 1) = -b * inv_det;
    Sinv(1, 0) = -c * inv_det;
    Sinv(1, 1) = a * inv_det;

    const cv::Matx<double, 4, 2> K = (P_pre * H.t()) * Sinv;
    x_post = x_pre + K * y;

    const cv::Matx<double, 4, 4> I = cv::Matx<double, 4, 4>::eye();
    const cv::Matx<double, 4, 4> IKH = I - K * H;
    P_post = IKH * P_pre * IKH.t() + K * R * K.t();
}

// ============================================================
// 速度相关函数
// ============================================================
inline std::pair<double, double> CuboidTracker::clip_vel(double vx, double vy) const {
    const double s = std::hypot(vx, vy);
    if (s <= vel_clip_mps_) {
        return {vx, vy};
    }
    const double scale = vel_clip_mps_ / std::max(s, 1e-9);
    return {vx * scale, vy * scale};
}

inline cv::Matx<double, 4, 1> CuboidTracker::clip_speed(const cv::Matx<double, 4, 1>& x) const {
    cv::Matx<double, 4, 1> x2 = x;
    double vx = x2(2, 0);
    double vy = x2(3, 0);
    std::tie(vx, vy) = clip_vel(vx, vy);
    x2(2, 0) = vx;
    x2(3, 0) = vy;
    return x2;
}

inline cv::Matx<double, 4, 1> CuboidTracker::blend_measured_velocity(
    const TrackPtr& trk,
    const cv::Matx<double, 4, 1>& x_post,
    const cv::Matx<double, 2, 1>& z_used,
    double dt,
    double innovation_m
) const {
    (void)trk;
    if (dt <= min_effective_dt_sec_) {
        return x_post;
    }

    double vmx = (z_used(0, 0) - trk->last_meas_x) / dt;
    double vmy = (z_used(1, 0) - trk->last_meas_y) / dt;
    std::tie(vmx, vmy) = clip_vel(vmx, vmy);

    const double ratio = clip(innovation_m / vel_meas_innov_for_max_, 0.0, 1.0);
    const double beta = vel_meas_blend_ + (vel_meas_blend_max_ - vel_meas_blend_) * ratio;

    cv::Matx<double, 4, 1> x2 = x_post;
    x2(2, 0) = (1.0 - beta) * x_post(2, 0) + beta * vmx;
    x2(3, 0) = (1.0 - beta) * x_post(3, 0) + beta * vmy;
    return x2;
}

// ============================================================
// Y轴位置历史更新
// ============================================================
inline void CuboidTracker::updateYPositionHistory(const TrackPtr& trk, double y_pos) const {
    const int idx = trk->y_pos_history_idx;
    const_cast<TrackPtr&>(trk)->y_position_history[idx] = y_pos;
    const_cast<TrackPtr&>(trk)->y_pos_history_idx = (idx + 1) % 5;
    const_cast<TrackPtr&>(trk)->y_pos_history_count = 
        std::min(trk->y_pos_history_count + 1, 5);
}

// ============================================================
// 方向一致性检测与修复（新增核心功能）
// ============================================================
inline bool CuboidTracker::hasConsistentMotion(const TrackPtr& trk) const {
    if (trk->y_pos_history_count < y_consistent_motion_frames_) {
        return false;
    }

    // 检查位置历史是否显示持续同向运动
    const int count = trk->y_pos_history_count;
    const int start_idx = (trk->y_pos_history_idx - count + 5) % 5;
    
    double sum_diff = 0.0;
    int valid_diffs = 0;
    
    for (int i = 0; i < count - 1; ++i) {
        const int idx1 = (start_idx + i) % 5;
        const int idx2 = (start_idx + i + 1) % 5;
        const double diff = trk->y_position_history[idx2] - trk->y_position_history[idx1];
        if (std::abs(diff) > 1e-6) {
            sum_diff += diff;
            valid_diffs++;
        }
    }
    
    if (valid_diffs < 2) {
        return false;
    }
    
    const double avg_diff = sum_diff / valid_diffs;
    const double speed = avg_diff / actual_dt_;
    
    // 有持续的同向运动且速度超过阈值
    if (std::abs(speed) > y_consistent_motion_thresh_) {
        const_cast<TrackPtr&>(trk)->y_consistent_motion_counter++;
        if (trk->y_consistent_motion_counter >= y_consistent_motion_frames_) {
            const_cast<TrackPtr&>(trk)->y_has_consistent_motion = true;
            return true;
        }
    } else {
        const_cast<TrackPtr&>(trk)->y_consistent_motion_counter = 
            std::max(0, trk->y_consistent_motion_counter - 1);
        if (trk->y_consistent_motion_counter < y_consistent_motion_frames_) {
            const_cast<TrackPtr&>(trk)->y_has_consistent_motion = false;
        }
    }
    
    return trk->y_has_consistent_motion;
}

inline double CuboidTracker::computeRawMeasurementVelocity(
    const TrackPtr& trk,
    double current_y,
    double dt
) const {
    if (!trk->last_meas_valid || dt < min_effective_dt_sec_) {
        return 0.0;
    }
    return (current_y - trk->last_meas_y) / dt;
}

inline double CuboidTracker::correctVelocityDirection(
    const TrackPtr& trk,
    double raw_vy,
    double dt,
    double current_y,
    double y_displacement
) const {
    // 如果没有位移或位移太小，不进行修正
    if (std::abs(y_displacement) < y_direction_correction_thresh_) {
        return raw_vy;
    }
    
    // 计算基于原始测量的速度
    double raw_meas_vy = computeRawMeasurementVelocity(trk, current_y, dt);
    
    // 检查是否有持续的一致运动
    bool has_consistent = hasConsistentMotion(trk);
    
    // 检查当前速度方向是否与位移方向一致
    bool direction_consistent = (raw_vy * y_displacement >= 0);
    
    // 如果方向不一致且有持续运动，强制修正
    if (!direction_consistent && has_consistent && std::abs(y_displacement) > 0.03) {
        // 使用原始测量速度的方向，混合当前速度的幅度
        double corrected_vy = std::copysign(
            std::max(std::abs(raw_meas_vy), std::abs(raw_vy) * 0.3),
            y_displacement
        );
        
        // 混合平滑值
        double blend = y_raw_vel_blend_;
        // 如果位移较大，增加混合比例
        if (std::abs(y_displacement) > 0.1) {
            blend = std::min(0.85, blend + 0.15);
        }
        
        double result = blend * corrected_vy + (1.0 - blend) * raw_vy;
        
        // 重置静止状态
        const_cast<TrackPtr&>(trk)->y_is_still = false;
        const_cast<TrackPtr&>(trk)->y_still_counter = 0;
        const_cast<TrackPtr&>(trk)->y_vel_smooth_valid = true;
        
        return result;
    }
    
    // 如果速度方向正确但幅度太小，而位移明显，增强速度
    if (direction_consistent && std::abs(y_displacement) > 0.05) {
        double expected_speed = y_displacement / dt;
        if (std::abs(raw_vy) < std::abs(expected_speed) * 0.3) {
            // 速度太小，增强
            double enhanced = raw_vy * 0.6 + expected_speed * 0.4;
            // 限制增强幅度
            if (std::abs(enhanced) > std::abs(expected_speed) * 1.5) {
                enhanced = std::copysign(std::abs(expected_speed) * 1.5, enhanced);
            }
            const_cast<TrackPtr&>(trk)->y_vel_smooth_valid = true;
            return enhanced;
        }
    }
    
    return raw_vy;
}

// ============================================================
// Y轴速度历史更新
// ============================================================
inline void CuboidTracker::updateYHistory(const TrackPtr& trk, double vy) const {
    const int idx = trk->y_history_idx;
    const_cast<TrackPtr&>(trk)->y_vel_history[idx] = vy;
    const_cast<TrackPtr&>(trk)->y_history_idx = (idx + 1) % CuboidTrack::Y_HISTORY_SIZE;
    const_cast<TrackPtr&>(trk)->y_history_count =
        std::min(trk->y_history_count + 1, CuboidTrack::Y_HISTORY_SIZE);
}

// ============================================================
// 改进的静止检测（包含位置信息）
// ============================================================
inline void CuboidTracker::detectStillState(const TrackPtr& trk) const {
    const int count = trk->y_history_count;
    if (count < 3) {
        const_cast<TrackPtr&>(trk)->y_is_still = false;
        const_cast<TrackPtr&>(trk)->y_still_counter = 0;
        return;
    }

    const int window_size = std::min(count, std::max(3, static_cast<int>(actual_fps_ * 0.5)));
    const int start_idx = (trk->y_history_idx - window_size + CuboidTrack::Y_HISTORY_SIZE) % CuboidTrack::Y_HISTORY_SIZE;
    
    double sum_abs = 0.0, sum2 = 0.0, sum_signed = 0.0;
    for (int i = 0; i < window_size; ++i) {
        const int idx = (start_idx + i) % CuboidTrack::Y_HISTORY_SIZE;
        const double v = trk->y_vel_history[idx];
        sum_abs += std::abs(v);
        sum2 += v * v;
        sum_signed += v;
    }
    const double n = static_cast<double>(window_size);
    const double abs_mean = sum_abs / n;
    const double mean = sum_signed / n;
    const double variance = std::max(0.0, (sum2 / n) - (mean * mean));

    const_cast<TrackPtr&>(trk)->y_vel_abs_mean = abs_mean;
    const_cast<TrackPtr&>(trk)->y_vel_var = variance;

    // 检查是否有有意义的运动
    bool has_meaningful_motion = false;
    const double min_meaningful_speed = 0.08;
    
    for (int i = 0; i < window_size; ++i) {
        const int idx = (start_idx + i) % CuboidTrack::Y_HISTORY_SIZE;
        if (std::abs(trk->y_vel_history[idx]) > min_meaningful_speed) {
            has_meaningful_motion = true;
            break;
        }
    }
    
    bool has_direction = std::abs(mean) > y_still_thresh_ * 0.5;
    
    // ===== 新增：检查位置是否在持续移动 =====
    bool has_position_motion = false;
    if (trk->y_pos_history_count >= 3) {
        const int pos_start = (trk->y_pos_history_idx - trk->y_pos_history_count + 5) % 5;
        double total_displacement = 0.0;
        int valid_pos = 0;
        for (int i = 0; i < trk->y_pos_history_count - 1; ++i) {
            const int idx1 = (pos_start + i) % 5;
            const int idx2 = (pos_start + i + 1) % 5;
            total_displacement += std::abs(trk->y_position_history[idx2] - trk->y_position_history[idx1]);
            valid_pos++;
        }
        if (valid_pos > 0) {
            double avg_displacement = total_displacement / valid_pos;
            // 如果平均位移超过阈值，认为有位置运动
            if (avg_displacement > 0.02) {
                has_position_motion = true;
            }
        }
    }

    // 判断静止 - 新增位置运动检查
    bool is_still = !has_meaningful_motion && 
                    (abs_mean < y_still_thresh_ && variance < y_still_var_thresh_) || 
                    (!has_direction && abs_mean < y_still_thresh_ * 1.5);
    
    // 如果有位置运动，不能判定为静止
    if (has_position_motion && is_still) {
        // 检查位置运动是否持续同向
        bool consistent_direction = false;
        if (trk->y_pos_history_count >= 3) {
            const int pos_start = (trk->y_pos_history_idx - trk->y_pos_history_count + 5) % 5;
            double first = trk->y_position_history[pos_start];
            double last = trk->y_position_history[(pos_start + trk->y_pos_history_count - 1) % 5];
            if (std::abs(last - first) > 0.05) {
                consistent_direction = true;
            }
        }
        if (consistent_direction) {
            is_still = false;
        }
    }
    
    if (is_still) {
        const_cast<TrackPtr&>(trk)->y_still_counter++;
        if (trk->y_still_counter >= y_still_confirm_frames_) {
            const_cast<TrackPtr&>(trk)->y_is_still = true;
        }
    } else {
        if (has_meaningful_motion || has_position_motion) {
            const_cast<TrackPtr&>(trk)->y_still_counter = 
                std::max(0, trk->y_still_counter - 2);
            if (trk->y_still_counter <= 0) {
                const_cast<TrackPtr&>(trk)->y_is_still = false;
            }
        } else {
            const_cast<TrackPtr&>(trk)->y_still_counter = 
                std::max(0, trk->y_still_counter - 1);
            if (trk->y_still_counter < y_still_confirm_frames_) {
                const_cast<TrackPtr&>(trk)->y_is_still = false;
            }
        }
    }
}

// ============================================================
// 异常值检测与抑制
// ============================================================
inline double CuboidTracker::medianFilterYVelocity(
    const TrackPtr& trk,
    double raw_vy
) const {
    const int idx = trk->y_median_idx;
    const_cast<TrackPtr&>(trk)->y_vel_median_history[idx] = raw_vy;
    const_cast<TrackPtr&>(trk)->y_median_idx = (idx + 1) % 5;
    const_cast<TrackPtr&>(trk)->y_median_count = 
        std::min(trk->y_median_count + 1, 5);
    
    if (trk->y_median_count < 3) {
        return raw_vy;
    }
    
    // A three-sample median rejects one-frame spikes while allowing a real
    // direction change to take effect after two consistent measurements.
    std::vector<double> vals;
    vals.reserve(3);
    const int count = std::min(trk->y_median_count, 3);
    const int start = (trk->y_median_idx - count + 5) % 5;
    for (int i = 0; i < count; ++i) {
        vals.push_back(trk->y_vel_median_history[(start + i) % 5]);
    }
    std::nth_element(vals.begin(), vals.begin() + vals.size() / 2, vals.end());
    return vals[vals.size() / 2];
}

inline double CuboidTracker::computeVelocityConsistency(
    const TrackPtr& trk,
    double raw_vy
) const {
    const int count = trk->y_history_count;
    if (count < 3) {
        return 0.8;
    }
    
    double sum = 0.0, sum_abs = 0.0;
    const int window = std::min(count, 5);
    const int start_idx = (trk->y_history_idx - window + CuboidTrack::Y_HISTORY_SIZE) % CuboidTrack::Y_HISTORY_SIZE;
    for (int i = 0; i < window; ++i) {
        const int idx = (start_idx + i) % CuboidTrack::Y_HISTORY_SIZE;
        sum += trk->y_vel_history[idx];
        sum_abs += std::abs(trk->y_vel_history[idx]);
    }
    const double mean = sum / window;
    const double abs_mean = sum_abs / window;
    
    const double diff = std::abs(raw_vy - mean);
    const double diff_ratio = diff / (abs_mean + 0.01);
    
    double trend = 0.0;
    if (count >= 4) {
        const int trend_start = (trk->y_history_idx - 4 + CuboidTrack::Y_HISTORY_SIZE) % CuboidTrack::Y_HISTORY_SIZE;
        double v0 = trk->y_vel_history[trend_start];
        double v1 = trk->y_vel_history[(trend_start + 2) % CuboidTrack::Y_HISTORY_SIZE];
        trend = (v1 - v0) / 2.0;
    }
    
    double predicted = mean + trend;
    const double pred_diff = std::abs(raw_vy - predicted);
    const double pred_diff_ratio = pred_diff / (abs_mean + 0.01);
    
    double consistency = 1.0 - std::min(1.0, diff_ratio / 2.0);
    consistency = consistency * 0.5 + (1.0 - std::min(1.0, pred_diff_ratio / 2.0)) * 0.5;
    
    if (trk->y_vel_smooth_valid && count >= 3) {
        const double sign_current = raw_vy > 0 ? 1.0 : -1.0;
        const double sign_history = trk->y_vel_smooth > 0 ? 1.0 : -1.0;
        if (sign_current != sign_history && std::abs(raw_vy) > 0.2) {
            consistency = std::max(consistency, 0.4);
        }
    }
    
    return clip(consistency, 0.0, 1.0);
}

inline bool CuboidTracker::isPhysicalVelocityJump(
    const TrackPtr& trk,
    double raw_vy,
    double dt
) const {
    if (!trk->y_vel_smooth_valid || dt < 1e-6) {
        return false;
    }
    
    const double diff = std::abs(raw_vy - trk->y_vel_smooth);
    if (diff <= y_jump_thresh_) {
        return false;
    }
    
    const double acceleration = diff / dt;
    const double accel_thresh = 10.0 + std::abs(trk->y_vel_smooth) * 2.0;
    if (acceleration > accel_thresh) {
        return true;
    }
    
    if (trk->y_history_count >= 3) {
        double max_abs = 0.0;
        const int check_count = std::min(trk->y_history_count, 10);
        for (int i = 0; i < check_count; ++i) {
            const int idx = (trk->y_history_idx - 1 - i + CuboidTrack::Y_HISTORY_SIZE) % CuboidTrack::Y_HISTORY_SIZE;
            max_abs = std::max(max_abs, std::abs(trk->y_vel_history[idx]));
        }
        if (max_abs > 0.1 && std::abs(raw_vy) > max_abs * 3.0) {
            return true;
        }
    }
    
    return false;
}

// ============================================================
// 方向反转检测
// ============================================================
inline double CuboidTracker::handleDirectionReversal(const TrackPtr& trk, double raw_vy, double dt) const {
    (void)dt;
    
    if (!trk->y_vel_smooth_valid || !trk->y_vel_prev_valid) {
        const_cast<TrackPtr&>(trk)->y_vel_prev = raw_vy;
        const_cast<TrackPtr&>(trk)->y_vel_prev_valid = true;
        const_cast<TrackPtr&>(trk)->y_direction_stable_counter = 0;
        return raw_vy;
    }

    const double prev_vy = trk->y_vel_prev;
    const bool direction_changed = (prev_vy * raw_vy < 0.0) && (std::abs(raw_vy) > 0.1);

    if (direction_changed) {
        const_cast<TrackPtr&>(trk)->y_direction_stable_counter = 0;
        const_cast<TrackPtr&>(trk)->y_is_outlier = false;
        const_cast<TrackPtr&>(trk)->y_outlier_counter = 0;
        
        const_cast<TrackPtr&>(trk)->y_vel_smooth = raw_vy * 0.7 + prev_vy * 0.3;
        const_cast<TrackPtr&>(trk)->y_vel_smooth_valid = true;
        const_cast<TrackPtr&>(trk)->y_vel_output = raw_vy * 0.7 + prev_vy * 0.3;
        const_cast<TrackPtr&>(trk)->y_vel_output_valid = true;
        
        const_cast<TrackPtr&>(trk)->y_is_still = false;
        const_cast<TrackPtr&>(trk)->y_still_counter = 0;
        
        const_cast<TrackPtr&>(trk)->y_vel_prev = raw_vy;
        return trk->y_vel_smooth;
    }

    if (prev_vy * raw_vy > 0) {
        const_cast<TrackPtr&>(trk)->y_direction_stable_counter++;
    }

    const_cast<TrackPtr&>(trk)->y_vel_prev = raw_vy;
    return raw_vy;
}

// ============================================================
// 异常值检测与抑制（主函数）
// ============================================================
inline double CuboidTracker::detectAndSuppressOutlier(
    const TrackPtr& trk,
    double raw_vy,
    double dt
) const {
    double median_vy = medianFilterYVelocity(trk, raw_vy);
    double adjusted_vy = handleDirectionReversal(trk, median_vy, dt);
    median_vy = adjusted_vy;
    
    double consistency = computeVelocityConsistency(trk, median_vy);
    const_cast<TrackPtr&>(trk)->y_vel_consistency = consistency;
    
    bool is_jump = isPhysicalVelocityJump(trk, median_vy, dt);
    
    bool is_outlier = false;
    double suppressed_vy = median_vy;
    
    bool direction_reversed = false;
    if (trk->y_vel_smooth_valid && trk->y_vel_prev_valid) {
        if (trk->y_vel_smooth * trk->y_vel_prev < 0 && std::abs(trk->y_vel_smooth) > 0.1) {
            direction_reversed = true;
        }
    }
    
    double effective_thresh = direction_reversed ? y_outlier_thresh_ * 0.5 : y_outlier_thresh_;
    
    if (consistency < effective_thresh || is_jump) {
        const_cast<TrackPtr&>(trk)->y_outlier_counter++;
        if (trk->y_outlier_counter >= y_outlier_confirm_frames_) {
            is_outlier = true;
            const_cast<TrackPtr&>(trk)->y_is_outlier = true;
        }
    } else {
        const_cast<TrackPtr&>(trk)->y_outlier_counter = 
            std::max(0, trk->y_outlier_counter - 1);
        if (trk->y_outlier_counter < y_outlier_confirm_frames_) {
            const_cast<TrackPtr&>(trk)->y_is_outlier = false;
        }
    }
    
    if (trk->y_is_outlier && !direction_reversed) {
        if (trk->y_vel_smooth_valid) {
            double blend = std::max(0.0, 1.0 - consistency / y_outlier_thresh_);
            blend = std::min(1.0, blend * 0.8 + 0.2);
            
            double predicted = trk->y_vel_smooth;
            if (trk->y_history_count >= 3) {
                const int latest_idx = (trk->y_history_idx - 1 + CuboidTrack::Y_HISTORY_SIZE) % CuboidTrack::Y_HISTORY_SIZE;
                const double latest_vel = trk->y_vel_history[latest_idx];
                const double trend = trk->y_vel_smooth - latest_vel;
                predicted += trend * 0.5;
            }
            
            suppressed_vy = (1.0 - blend) * median_vy + blend * predicted;
            suppressed_vy = suppressed_vy * (1.0 - y_outlier_decay_) + 
                           trk->y_vel_smooth * y_outlier_decay_;
        } else {
            suppressed_vy = median_vy * 0.5;
        }
        
        const_cast<TrackPtr&>(trk)->y_vel_confidence = 0.2;
    } else {
        const_cast<TrackPtr&>(trk)->y_vel_confidence = 
            std::min(1.0, trk->y_vel_confidence + 0.1);
    }
    
    return suppressed_vy;
}

// ============================================================
// Y轴速度平滑核心函数（包含方向修正）
// ============================================================
inline double CuboidTracker::smoothYVelocity(
    const TrackPtr& trk,
    double raw_vy,
    double dt,
    double current_y
) const {
    double clipped = clip(raw_vy, -y_max_vel_, y_max_vel_);

    // ===== 新增：方向修正 =====
    double y_displacement = 0.0;
    if (trk->last_meas_valid) {
        y_displacement = current_y - trk->last_meas_y;
    }
    
    // 更新位置历史
    updateYPositionHistory(trk, current_y);
    
    // 在异常检测之前进行方向修正
    double corrected_vy = correctVelocityDirection(trk, clipped, dt, current_y, y_displacement);

    double filtered = detectAndSuppressOutlier(trk, corrected_vy, dt);

    updateYHistory(trk, filtered);

    detectStillState(trk);

    // A measured displacement is the authoritative source for the Y sign and
    // magnitude.  Do not let the low-pass filter reduce a real displacement to
    // a stale, opposite-direction KF velocity.
    const bool has_measured_displacement =
        trk->last_meas_valid &&
        std::abs(y_displacement) >= y_direction_correction_thresh_ &&
        dt >= min_effective_dt_sec_;
    const double measured_vy = has_measured_displacement ? y_displacement / dt : 0.0;
    const bool measured_direction_confirmed =
        has_measured_displacement &&
        (filtered * measured_vy >= 0.0 || hasConsistentMotion(trk));

    // 如果静止但位置在移动，强制退出静止状态
    if (trk->y_is_still && hasConsistentMotion(trk)) {
        const_cast<TrackPtr&>(trk)->y_is_still = false;
        const_cast<TrackPtr&>(trk)->y_still_counter = 0;
    }

    if (trk->y_is_still) {
        double current_smooth = trk->y_vel_smooth;
        
        bool has_meaningful_motion = std::abs(raw_vy) > 0.05;
        bool same_direction = (raw_vy * current_smooth >= 0) || std::abs(current_smooth) < 0.01;
        
        const double decay = 0.92;
        double decayed = current_smooth * decay;
        
        // 如果有位置运动，不允许完全衰减到0
        if (std::abs(y_displacement) > 0.02) {
            double expected_direction = y_displacement > 0 ? 1.0 : -1.0;
            double min_speed = std::max(0.02, std::abs(y_displacement) / dt * 0.3);
            if (decayed * expected_direction < min_speed * 0.5) {
                decayed = expected_direction * min_speed * 0.5;
            }
        }
        
        if (has_meaningful_motion && same_direction && std::abs(raw_vy) > 0.1) {
            double blend_ratio = std::min(0.3, std::abs(raw_vy) * 0.05);
            decayed = (1.0 - blend_ratio) * decayed + blend_ratio * raw_vy;
        }
        
        if (has_meaningful_motion && !same_direction && std::abs(raw_vy) > 0.15) {
            decayed = raw_vy * 0.6 + current_smooth * 0.4;
            const_cast<TrackPtr&>(trk)->y_is_still = false;
            const_cast<TrackPtr&>(trk)->y_still_counter = 0;
        }
        
        if (std::abs(decayed) < 0.005) {
            decayed = 0.0;
        }
        
        const_cast<TrackPtr&>(trk)->y_vel_smooth = decayed;
        const_cast<TrackPtr&>(trk)->y_vel_output = decayed;
        
        if (std::abs(decayed) > 0.001) {
            const_cast<TrackPtr&>(trk)->y_vel_smooth_valid = true;
            const_cast<TrackPtr&>(trk)->y_vel_output_valid = true;
        }
        
        return decayed;
    }

    if (!trk->y_vel_smooth_valid) {
        const double initial_vy = has_measured_displacement
                                      ? std::copysign(
                                            std::max(std::abs(filtered), std::abs(measured_vy)),
                                            measured_vy
                                        )
                                      : filtered;
        const_cast<TrackPtr&>(trk)->y_vel_smooth = initial_vy;
        const_cast<TrackPtr&>(trk)->y_vel_smooth_valid = true;
        const_cast<TrackPtr&>(trk)->y_vel_output = initial_vy;
        const_cast<TrackPtr&>(trk)->y_vel_output_valid = true;
        return initial_vy;
    }

    double max_change = y_vel_change_limit_ * dt;
    
    if (trk->y_vel_smooth * filtered < 0 && std::abs(filtered) > 0.1) {
        max_change *= 5.0;
    }
    
    if (trk->y_is_outlier) {
        max_change *= 0.5;
    }
    
    if (std::abs(trk->y_vel_smooth) < 0.05 && std::abs(filtered) > 0.2) {
        max_change *= 5.0;
    }
    if (std::abs(filtered) > 0.5) {
        const double speed_factor = std::min(3.0, std::abs(filtered) * 0.5 + 0.5);
        max_change *= speed_factor;
    }

    if (measured_direction_confirmed) {
        // Measurements must be able to catch up within one frame.  This is
        // especially important for slow targets where two smoothing stages
        // otherwise make Y velocity appear much smaller than dy / dt.
        max_change = std::max(max_change, std::abs(measured_vy));
        if (trk->y_vel_smooth * measured_vy < 0.0) {
            max_change = std::max(max_change, std::abs(trk->y_vel_smooth) + std::abs(measured_vy));
        }
    }

    double diff = filtered - trk->y_vel_smooth;
    if (std::abs(diff) > max_change) {
        diff = std::copysign(max_change, diff);
    }
    double limited = trk->y_vel_smooth + diff;
    
    double alpha = y_smooth_alpha_;
    
    if (trk->y_vel_smooth * filtered < 0 && std::abs(filtered) > 0.1) {
        alpha = std::max(alpha, 0.85);
    }
    
    if (trk->y_is_outlier) {
        alpha = 0.15;
    } else if (std::abs(filtered) > 0.5) {
        alpha = std::min(0.6, y_smooth_alpha_ * 1.5);
    } else if (std::abs(trk->y_vel_smooth) < 0.05 && std::abs(filtered) > 0.2) {
        alpha = 0.6;
    } else if (trk->y_vel_smooth * filtered < 0 && std::abs(filtered) > 0.1) {
        alpha = std::max(alpha, 0.8);
    }

    if (measured_direction_confirmed) {
        alpha = std::max(alpha, 0.75);
    }

    double smoothed = alpha * limited + (1.0 - alpha) * trk->y_vel_smooth;

    if (measured_direction_confirmed) {
        // Preserve the measured direction and avoid attenuating the measured
        // speed below the displacement-derived speed.
        smoothed = std::copysign(
            std::max(std::abs(smoothed), std::abs(measured_vy) * 0.75),
            measured_vy
        );
    }

    const_cast<TrackPtr&>(trk)->y_vel_smooth = smoothed;
    const_cast<TrackPtr&>(trk)->y_vel_output = smoothed;

    return smoothed;
}

inline double CuboidTracker::limitYVelocity(const TrackPtr& trk, double vy) const {
    (void)trk;
    return clip(vy, -y_max_vel_, y_max_vel_);
}

// ============================================================
// 输出速度平滑
// ============================================================
inline void CuboidTracker::update_output_velocity(
    const TrackPtr& trk,
    double vx,
    double vy,
    bool has_measurement
) const {
    double alpha = has_measurement ? output_vel_alpha_ : output_vel_alpha_lost_;
    
    if (trk->out_vel_valid && trk->out_vy * vy < 0 && std::abs(vy) > 0.1) {
        alpha = std::max(alpha, 0.7);
    }
    
    if (!trk->out_vel_valid) {
        trk->out_vx = vx;
        trk->out_vy = vy;
        trk->out_vel_valid = true;
        return;
    }

    trk->out_vx = (1.0 - alpha) * trk->out_vx + alpha * vx;
    trk->out_vy = (1.0 - alpha) * trk->out_vy + alpha * vy;
}

// ============================================================
// 其他函数
// ============================================================
inline std::pair<int, double> CuboidTracker::recovery_phase(const TrackPtr& trk) const {
    if (turn_recover_hold_ <= 0) {
        return {0, 0.0};
    }
    const int left = trk->maneuver_left;
    if (left <= 0) {
        return {0, 0.0};
    }

    const int elapsed = std::max(0, turn_recover_hold_ - left);
    if (elapsed < turn_recover_smooth_frames_) {
        return {1, 0.0};
    }

    const int pull_idx = std::min(
        std::max(elapsed - turn_recover_smooth_frames_ + 1, 1),
        turn_recover_pull_frames_
    );
    const double pull_ratio = static_cast<double>(pull_idx) / static_cast<double>(std::max(1, turn_recover_pull_frames_));
    return {2, pull_ratio};
}

inline cv::Matx<double, 4, 1> CuboidTracker::apply_turn_recovery(
    const TrackPtr& trk,
    const cv::Matx<double, 4, 1>& x_post,
    const cv::Matx<double, 2, 1>& z_used,
    double vmx,
    double vmy,
    bool has_meas_velocity
) const {
    cv::Matx<double, 4, 1> x2 = x_post;
    int phase = 0;
    double pull_ratio = 0.0;
    std::tie(phase, pull_ratio) = recovery_phase(trk);
    if (phase == 0) {
        return clip_speed(x2);
    }

    double pos_alpha = turn_recover_pos_blend_smooth_;
    double vel_alpha = turn_recover_vel_blend_smooth_;
    if (phase == 2) {
        pos_alpha = turn_recover_pos_blend_smooth_ +
                    (turn_recover_pos_blend_ - turn_recover_pos_blend_smooth_) * pull_ratio;
        vel_alpha = turn_recover_vel_blend_smooth_ +
                    (turn_recover_vel_blend_ - turn_recover_vel_blend_smooth_) * pull_ratio;
    }

    x2(0, 0) = (1.0 - pos_alpha) * x2(0, 0) + pos_alpha * z_used(0, 0);
    x2(1, 0) = (1.0 - pos_alpha) * x2(1, 0) + pos_alpha * z_used(1, 0);

    if (has_meas_velocity) {
        x2(2, 0) = (1.0 - vel_alpha) * x2(2, 0) + vel_alpha * vmx;
        x2(3, 0) = (1.0 - vel_alpha) * x2(3, 0) + vel_alpha * vmy;
    }

    return clip_speed(x2);
}

inline void CuboidTracker::update_motion_state(const TrackPtr& trk, double speed) const {
    int target_state = -1;
    if (speed <= vel_static_thresh_) {
        target_state = MOTION_STATIC;
    } else if (speed >= vel_move_thresh_) {
        target_state = MOTION_MOVING;
    }

    if (target_state < 0) {
        trk->stable_cnt = 0;
        return;
    }

    if (trk->motion_state == target_state) {
        trk->stable_cnt = 0;
        return;
    }

    trk->stable_cnt += 1;
    if (trk->stable_cnt >= stable_frames_) {
        trk->motion_state = target_state;
        trk->stable_cnt = 0;
    }
}

inline double CuboidTracker::compute_dt(std::int64_t timestamp, std::int64_t last_timestamp) const {
    if (timestamp <= 0 || last_timestamp <= 0) {
        return default_dt_sec_;
    }

    const double dt = static_cast<double>(timestamp - last_timestamp) * timestamp_scale_;
    if ((!std::isfinite(dt)) || (dt <= min_dt_sec_)) {
        return default_dt_sec_;
    }
    if ((dt < min_effective_dt_sec_) || (dt > max_effective_dt_sec_)) {
        return default_dt_sec_;
    }
    return dt;
}

inline std::pair<double, double> CuboidTracker::predict_center(const TrackPtr& trk, std::int64_t timestamp) const {
    if (timestamp == trk->last_timestamp) {
        return {trk->x(0, 0), trk->x(1, 0)};
    }
    const double dt = compute_dt(timestamp, trk->last_timestamp);
    cv::Matx<double, 4, 1> x_pre{};
    cv::Matx<double, 4, 4> P_pre{};
    kf_predict(trk->x, trk->P, dt, x_pre, P_pre);
    return {x_pre(0, 0), x_pre(1, 0)};
}

inline std::pair<double, double> CuboidTracker::position_comp_delta(
    const TrackPtr& trk,
    double vx,
    double vy,
    int track_state,
    int motion_state
) const {
    if (pos_comp_sec_ <= 0.0) {
        return {0.0, 0.0};
    }

    int phase = 0;
    double pull_ratio = 0.0;
    std::tie(phase, pull_ratio) = recovery_phase(trk);
    if (phase == 1) {
        return {0.0, 0.0};
    }

    if (trk->comp_freeze > 0) {
        return {0.0, 0.0};
    }

    const double speed = std::hypot(vx, vy);
    if (speed < pos_comp_min_speed_) {
        return {0.0, 0.0};
    }
    if (motion_state == MOTION_STATIC) {
        return {0.0, 0.0};
    }

    const double state_scale = (track_state == TRACK_NEW) ? pos_comp_new_scale_ : 1.0;
    const double innov_scale = 1.0 - clip(trk->last_innovation / pos_comp_innov_for_zero_, 0.0, 1.0);
    if (innov_scale <= 0.0) {
        return {0.0, 0.0};
    }

    double dir_x = vx / std::max(speed, 1e-9);
    double dir_y = vy / std::max(speed, 1e-9);
    double speed_used = speed;
    double heading_scale = 1.0;

    if (trk->has_meas_velocity) {
        const double vmx = trk->meas_vx;
        const double vmy = trk->meas_vy;
        const double vm_speed = std::hypot(vmx, vmy);
        if (vm_speed > 1e-6) {
            const double kf_x = vx / std::max(speed, 1e-9);
            const double kf_y = vy / std::max(speed, 1e-9);
            const double vm_x = vmx / vm_speed;
            const double vm_y = vmy / vm_speed;
            const double cos_h = clip(kf_x * vm_x + kf_y * vm_y, -1.0, 1.0);

            heading_scale = std::max(0.0, 0.5 * (cos_h + 1.0));
            const double beta = clip(
                pos_comp_dir_meas_base_ + (1.0 - std::max(cos_h, 0.0)) * pos_comp_dir_meas_turn_gain_,
                0.0,
                1.0
            );

            const double mix_x = (1.0 - beta) * kf_x + beta * vm_x;
            const double mix_y = (1.0 - beta) * kf_y + beta * vm_y;
            const double mix_n = std::hypot(mix_x, mix_y);
            if (mix_n > 1e-9) {
                dir_x = mix_x / mix_n;
                dir_y = mix_y / mix_n;
            }
            speed_used = (1.0 - beta) * speed + beta * vm_speed;
        }
    }

    const double scale = state_scale * innov_scale * heading_scale;
    if (scale <= 0.0) {
        return {0.0, 0.0};
    }

    const double maneuver_gain = (phase == 2) ? (1.0 + turn_recover_comp_boost_ * pull_ratio) : 1.0;
    double dist = speed_used * pos_comp_sec_ * scale * maneuver_gain;
    if (dist > pos_comp_max_m_) {
        dist = pos_comp_max_m_;
    }
    if (dist <= 0.0) {
        return {0.0, 0.0};
    }

    return {dir_x * dist, dir_y * dist};
}

// ============================================================
// Cost计算
// ============================================================
inline double CuboidTracker::size_cost(const cv::Vec<float, 9>& track_box, const cv::Vec<float, 9>& det_box, double eps) {
    const double tl = track_box[3];
    const double tw = track_box[4];
    const double th = track_box[5];
    const double dl = det_box[3];
    const double dw = det_box[4];
    const double dh = det_box[5];
    const double c = (
        std::abs(std::log((tl + eps) / (dl + eps))) +
        std::abs(std::log((tw + eps) / (dw + eps))) +
        std::abs(std::log((th + eps) / (dh + eps)))
    ) / 3.0;
    return clip(c, 0.0, 1.5);
}

inline cv::Mat CuboidTracker::build_cost(
    const std::vector<TrackPtr>& tracks,
    const cv::Mat& detections,
    const std::vector<int>& det_indices,
    const std::optional<std::int64_t>& timestamp,
    double center_gate_m,
    double w_cls,
    double w_center,
    double w_size
) const {
    (void)det_indices;

    const int nt = static_cast<int>(tracks.size());
    const int nd = detections.rows;
    if (nt == 0 || nd == 0) {
        return cv::Mat(nt, nd, CV_32F, cv::Scalar(0));
    }

    const double ws = std::max(w_cls + w_center + w_size, 1e-6);
    cv::Mat cost(nt, nd, CV_32F, cv::Scalar(static_cast<float>(large_cost_)));

    std::vector<cv::Vec<float, 9>> det_rows(static_cast<size_t>(nd));
    std::vector<double> det_diag(static_cast<size_t>(nd), 1e-3);
    std::unordered_map<int, std::vector<int>> det_bins;
    det_bins.reserve(static_cast<size_t>(nd) * 2U + 1U);

    for (int j = 0; j < nd; ++j) {
        const cv::Vec<float, 9> det = row_as_vec9(detections, j);
        det_rows[static_cast<size_t>(j)] = det;

        const int dcls = cls_to_int(det[7]);
        det_diag[static_cast<size_t>(j)] = std::max(
            1e-3,
            std::hypot(static_cast<double>(det[3]), static_cast<double>(det[4]))
        );
        det_bins[dcls].push_back(j);
    }

    for (int i = 0; i < nt; ++i) {
        const auto& trk = tracks[i];
        double tcx = trk->cuboid[0];
        double tcy = trk->cuboid[1];
        if (timestamp.has_value()) {
            std::tie(tcx, tcy) = predict_center(trk, *timestamp);
        }

        const double tdiag = std::max(
            1e-3,
            std::hypot(static_cast<double>(trk->cuboid[3]), static_cast<double>(trk->cuboid[4]))
        );
        const int tcls = cls_to_int(trk->cuboid[7]);
        const auto it_bin = det_bins.find(tcls);
        if (it_bin == det_bins.end()) {
            continue;
        }

        for (const int j : it_bin->second) {
            const cv::Vec<float, 9>& det = det_rows[static_cast<size_t>(j)];

            const double dcx = det[0];
            const double dcy = det[1];
            const double ddiag = det_diag[static_cast<size_t>(j)];
            const double dist = std::hypot(tcx - dcx, tcy - dcy);
            if (dist > center_gate_m) {
                continue;
            }

            const double norm = std::max(1e-3, 0.5 * (tdiag + ddiag));
            const double center_cost = clip(dist / (norm * 2.0), 0.0, 1.5);
            const double cls_cost = 0.0;
            const double sz_cost = size_cost(trk->cuboid, det);

            const double c = (w_cls * cls_cost + w_center * center_cost + w_size * sz_cost) / ws;
            cost.at<float>(i, j) = static_cast<float>(c);
        }
    }

    return cost;
}

inline std::tuple<std::vector<std::pair<int, int>>, std::vector<int>, std::vector<int>>
CuboidTracker::linear_assignment(const cv::Mat& cost, float thresh) {
    const int nr = cost.rows;
    const int nc = cost.cols;
    if (nr == 0 || nc == 0) {
        std::vector<int> u_r(nr);
        std::vector<int> u_c(nc);
        std::iota(u_r.begin(), u_r.end(), 0);
        std::iota(u_c.begin(), u_c.end(), 0);
        return {{}, u_r, u_c};
    }

    LinearAssignmentResult r = matching::linear_assignment(cost, thresh);
    return {r.matches, r.unmatched_a, r.unmatched_b};
}

// ============================================================
// Track管理 - 激活新跟踪
// ============================================================
inline CuboidTracker::TrackPtr CuboidTracker::activate(const cv::Vec<float, 9>& det, int det_idx, std::int64_t timestamp) {
    TrackPtr trk = std::make_shared<CuboidTrack>();
    trk->track_id = next_id();
    trk->cuboid = det;
    trk->state = TRACK_NEW;
    trk->start_frame = frame_id_;
    trk->frame_id = frame_id_;
    trk->idx = det_idx;
    trk->last_timestamp = timestamp;
    init_kf_state(static_cast<double>(det[0]), static_cast<double>(det[1]), trk->x, trk->P);
    trk->last_meas_x = det[0];
    trk->last_meas_y = det[1];
    trk->last_meas_valid = true;
    trk->last_innovation = 0.0;
    trk->meas_vx = 0.0;
    trk->meas_vy = 0.0;
    trk->has_meas_velocity = false;
    trk->out_vx = 0.0;
    trk->out_vy = 0.0;
    trk->out_vel_valid = true;
    trk->comp_freeze = 0;
    trk->maneuver_left = 0;
    trk->motion_state = MOTION_UNKNOWN;
    trk->stable_cnt = 0;
    trk->missed = 0;
    trk->hits = 1;

    // Y轴速度相关变量初始化
    trk->y_vel_smooth = 0.0;
    trk->y_vel_smooth_valid = false;
    trk->y_vel_output = 0.0;
    trk->y_vel_output_valid = false;
    trk->y_history_count = 0;
    trk->y_history_idx = 0;
    trk->y_vel_abs_mean = 0.0;
    trk->y_vel_var = 0.0;
    trk->y_still_counter = 0;
    trk->y_is_still = false;
    
    trk->y_vel_predicted = 0.0;
    trk->y_vel_consistency = 1.0;
    trk->y_vel_confidence = 1.0;
    trk->y_outlier_counter = 0;
    trk->y_is_outlier = false;
    trk->y_median_idx = 0;
    trk->y_median_count = 0;
    
    trk->y_vel_prev = 0.0;
    trk->y_vel_prev_valid = false;
    trk->y_direction_stable_counter = 0;

    // 新增：位置历史和方向修复变量
    trk->y_pos_history_idx = 0;
    trk->y_pos_history_count = 0;
    for (int i = 0; i < 5; ++i) {
        trk->y_position_history[i] = det[1];
    }
    trk->y_displacement_accum = 0.0;
    trk->y_consistent_motion_counter = 0;
    trk->y_has_consistent_motion = false;
    trk->y_raw_measurement_vel = 0.0;
    trk->y_raw_vel_valid = false;

    return trk;
}

// ============================================================
// update_track函数（包含方向修复）
// ============================================================
inline void CuboidTracker::update_track(
    const TrackPtr& trk,
    const cv::Vec<float, 9>& det,
    int det_idx,
    std::int64_t timestamp,
    int state
) const {
    const double dt = compute_dt(timestamp, trk->last_timestamp);

    cv::Matx<double, 4, 1> x_pre{};
    cv::Matx<double, 4, 4> P_pre{};
    kf_predict(trk->x, trk->P, dt, x_pre, P_pre);

    if (trk->comp_freeze > 0) {
        trk->comp_freeze -= 1;
    }
    if (trk->maneuver_left > 0) {
        trk->maneuver_left -= 1;
    }

    // 使用原始检测位置计算测量速度（用于方向修正）
    const double raw_det_y = det[1];
    double raw_meas_vy = 0.0;
    if (trk->last_meas_valid && dt > min_effective_dt_sec_) {
        raw_meas_vy = (raw_det_y - trk->last_meas_y) / dt;
    }

    cv::Matx<double, 2, 1> z(static_cast<double>(det[0]), static_cast<double>(det[1]));

    const double dx = z(0, 0) - x_pre(0, 0);
    const double dy = z(1, 0) - x_pre(1, 0);
    const double innov = std::hypot(dx, dy);
    if (innov > max_innovation_m_) {
        const double scale = max_innovation_m_ / std::max(innov, 1e-9);
        z(0, 0) = x_pre(0, 0) + dx * scale;
        z(1, 0) = x_pre(1, 0) + dy * scale;
    }

    double vmx = 0.0;
    double vmy = 0.0;
    bool has_meas_velocity = false;
    const bool has_valid_measurement = trk->last_meas_valid && dt > min_effective_dt_sec_;
    
    if (has_valid_measurement) {
        vmx = (z(0, 0) - trk->last_meas_x) / dt;
        vmy = (z(1, 0) - trk->last_meas_y) / dt;
        std::tie(vmx, vmy) = clip_vel(vmx, vmy);
        has_meas_velocity = true;
    }

    const double innov_used = std::hypot(z(0, 0) - x_pre(0, 0), z(1, 0) - x_pre(1, 0));

    cv::Matx<double, 4, 1> x_post{};
    cv::Matx<double, 4, 4> P_post{};
    kf_update(x_pre, P_pre, z, x_post, P_post);

    double kf_vx = x_post(2, 0);
    double kf_vy = x_post(3, 0);

    if (has_meas_velocity) {
        const double ratio = clip(innov_used / vel_meas_innov_for_max_, 0.0, 1.0);
        const double beta = vel_meas_blend_ + (vel_meas_blend_max_ - vel_meas_blend_) * ratio;
        kf_vx = (1.0 - beta) * x_post(2, 0) + beta * vmx;
    }

    // ===== Y速度平滑（包含方向修复） =====
    if (y_vel_smooth_enabled_) {
        if (has_valid_measurement) {
            // 使用原始检测位置计算的速度进行方向修正
            double raw_vy_for_smooth = raw_meas_vy;
            
            // 如果原始测量速度有效，使用它；否则使用KF速度
            if (std::abs(raw_vy_for_smooth) > 1e-6) {
                kf_vy = smoothYVelocity(trk, raw_vy_for_smooth, dt, raw_det_y);
            } else {
                kf_vy = smoothYVelocity(trk, vmy, dt, raw_det_y);
            }
            
            // 额外检查：如果KF速度与位移方向不一致，强制修正
            double y_displacement = raw_det_y - trk->last_meas_y;
            if (std::abs(y_displacement) > 0.03 && kf_vy * y_displacement < 0) {
                // 使用原始测量速度修正
                double corrected_vy = std::copysign(
                    std::max(std::abs(raw_meas_vy), 0.05),
                    y_displacement
                );
                // 混合当前速度
                kf_vy = 0.5 * kf_vy + 0.5 * corrected_vy;
                const_cast<TrackPtr&>(trk)->y_is_still = false;
                const_cast<TrackPtr&>(trk)->y_still_counter = 0;
            }
        } else {
            double vy = clip(x_post(3, 0), -y_max_vel_, y_max_vel_);
            kf_vy = smoothYVelocity(trk, vy, dt, x_pre(1, 0));
        }
    } else {
        kf_vy = limitYVelocity(trk, kf_vy);
    }

    cv::Matx<double, 4, 1> x_final = x_post;
    x_final(2, 0) = kf_vx;
    x_final(3, 0) = kf_vy;
    // 在update_track内部，x_final(3,0)=kf_vy之后，增加
    //如果平滑后的 y 速度和**位置位移方向不一致，并且没有连续运动**，直接把 y 速度向 0 拉回，避免检测噪声导致符号翻转。
    if (y_vel_smooth_enabled_ && has_valid_measurement) {
        double y_disp = raw_det_y - trk->last_meas_y;
        const double measured_vy = y_disp / std::max(dt, min_effective_dt_sec_);
        const bool displacement_confirmed =
            std::abs(y_disp) >= y_direction_correction_thresh_ &&
            (std::abs(kf_vy) < 1e-6 || kf_vy * measured_vy >= 0.0 ||
             hasConsistentMotion(trk));
        if (displacement_confirmed && kf_vy * measured_vy < 0.0) {
            // Only correct a direction mismatch after the displacement has
            // survived the median/consistency checks.
            kf_vy = std::copysign(std::max(std::abs(kf_vy), std::abs(measured_vy) * 0.5), measured_vy);
            x_final(3, 0) = kf_vy;
        }
    }
    x_final = clip_speed(x_final);

    double speed = std::hypot(x_final(2, 0), x_final(3, 0));
    const double vm_speed = has_meas_velocity ? std::hypot(vmx, vmy) : 0.0;

    bool trigger_turn_recover = false;
    if (has_meas_velocity && vm_speed >= turn_recover_min_speed_) {
        if (innov_used >= turn_recover_innov_m_) {
            trigger_turn_recover = true;
        }
        if (speed >= turn_recover_min_speed_) {
            const double cos_h = clip((x_final(2, 0) * vmx + x_final(3, 0) * vmy) / std::max(speed * vm_speed, 1e-9), -1.0, 1.0);
            if (cos_h <= turn_recover_cos_) {
                trigger_turn_recover = true;
            }
        }
    }

    bool recover_active = trk->maneuver_left > 0;
    if (trigger_turn_recover && turn_recover_hold_ > 0) {
        if (!recover_active) {
            trk->maneuver_left = turn_recover_hold_;
            recover_active = true;
        } else {
            trk->maneuver_left = std::max(trk->maneuver_left, turn_recover_pull_frames_);
        }
        trk->comp_freeze = std::max(trk->comp_freeze, turn_recover_comp_freeze_);
    }

    if (recover_active) {
        x_final = apply_turn_recovery(trk, x_final, z, vmx, vmy, has_meas_velocity);
        if (y_vel_smooth_enabled_) {
            kf_vy = smoothYVelocity(trk, x_final(3, 0), dt, x_final(1, 0));
            x_final(3, 0) = kf_vy;
        }
        kf_vx = x_final(2, 0);
        kf_vy = x_final(3, 0);
        speed = std::hypot(kf_vx, kf_vy);
    }

    const double fx = x_final(0, 0);
    const double fy = x_final(1, 0);

    update_motion_state(trk, speed);
    update_output_velocity(trk, kf_vx, kf_vy, true);

    trk->x = x_final;
    
    cv::Matx<double, 4, 4> P_updated = P_post;
    if (y_vel_smooth_enabled_ && trk->y_vel_smooth_valid && 
        !trk->y_is_outlier && trk->y_history_count > 5) {
        P_updated(2, 2) *= 0.85;
        P_updated(3, 3) *= 0.85;
    }
    trk->P = P_updated;
    
    trk->last_timestamp = timestamp;
    trk->last_meas_x = z(0, 0);
    trk->last_meas_y = z(1, 0);
    trk->last_meas_valid = true;
    trk->last_innovation = innov_used;
    trk->meas_vx = vmx;
    trk->meas_vy = vmy;
    trk->has_meas_velocity = has_meas_velocity;
    trk->missed = 0;

    if (has_meas_velocity) {
        if (vm_speed >= pos_comp_reverse_speed_ && speed >= pos_comp_reverse_speed_) {
            const double cos_h = clip((kf_vx * vmx + kf_vy * vmy) / std::max(speed * vm_speed, 1e-9), -1.0, 1.0);
            if (cos_h <= pos_comp_reverse_cos_) {
                trk->comp_freeze = std::max(trk->comp_freeze, pos_comp_reverse_hold_);
            }
        }
    }

    trk->cuboid = det;
    trk->cuboid[0] = static_cast<float>(fx);
    trk->cuboid[1] = static_cast<float>(fy);
    trk->idx = det_idx;
    trk->state = state;
    trk->frame_id = frame_id_;
    trk->hits += 1;
}

// ============================================================
// 预测无测量时的跟踪
// ============================================================
inline void CuboidTracker::predict_track_without_measure(const TrackPtr& trk, std::int64_t timestamp) const {
    if (timestamp == trk->last_timestamp) {
        return;
    }

    const double dt = compute_dt(timestamp, trk->last_timestamp);
    cv::Matx<double, 4, 1> x_pre{};
    cv::Matx<double, 4, 4> P_pre{};
    kf_predict(trk->x, trk->P, dt, x_pre, P_pre);
    x_pre = clip_speed(x_pre);

    if (y_vel_smooth_enabled_) {
        double vy = clip(x_pre(3, 0), -y_max_vel_, y_max_vel_);
        if (trk->y_vel_smooth_valid) {
            double decayed = trk->y_vel_smooth * 0.98;
            if (std::abs(decayed) < 0.005) {
                decayed = 0.0;
            }
            const_cast<TrackPtr&>(trk)->y_vel_smooth = decayed;
            const_cast<TrackPtr&>(trk)->y_vel_output = decayed;
            vy = decayed;
        } else {
            vy = smoothYVelocity(trk, vy, dt, x_pre(1, 0));
        }
        x_pre(3, 0) = vy;
        const_cast<TrackPtr&>(trk)->y_vel_output = vy;
        const_cast<TrackPtr&>(trk)->y_vel_output_valid = true;
    }

    trk->x = x_pre;
    trk->P = P_pre;
    trk->last_timestamp = timestamp;
    trk->last_meas_valid = false;
    trk->last_innovation = pos_comp_innov_for_zero_;
    trk->has_meas_velocity = false;
    trk->meas_vx = 0.0;
    trk->meas_vy = 0.0;
    trk->missed += 1;
    if (trk->comp_freeze > 0) {
        trk->comp_freeze -= 1;
    }
    if (trk->maneuver_left > 0) {
        trk->maneuver_left -= 1;
    }

    trk->cuboid[0] = static_cast<float>(x_pre(0, 0));
    trk->cuboid[1] = static_cast<float>(x_pre(1, 0));

    const double speed = std::hypot(x_pre(2, 0), x_pre(3, 0));
    update_motion_state(trk, speed);
    update_output_velocity(trk, x_pre(2, 0), x_pre(3, 0), false);
}

// ============================================================
// Track列表操作
// ============================================================
inline std::vector<CuboidTracker::TrackPtr> CuboidTracker::joint_tracks(
    const std::vector<TrackPtr>& a,
    const std::vector<TrackPtr>& b
) {
    std::vector<TrackPtr> out;
    out.reserve(a.size() + b.size());
    std::unordered_set<int> seen;
    seen.reserve((a.size() + b.size()) * 2 + 1);

    for (const auto& t : a) {
        if (seen.insert(t->track_id).second) {
            out.push_back(t);
        }
    }
    for (const auto& t : b) {
        if (seen.insert(t->track_id).second) {
            out.push_back(t);
        }
    }
    return out;
}

inline std::vector<CuboidTracker::TrackPtr> CuboidTracker::sub_tracks(
    const std::vector<TrackPtr>& a,
    const std::vector<TrackPtr>& b
) {
    std::unordered_set<int> b_ids;
    b_ids.reserve(b.size() * 2 + 1);
    for (const auto& t : b) {
        b_ids.insert(t->track_id);
    }

    std::vector<TrackPtr> out;
    out.reserve(a.size());
    for (const auto& t : a) {
        if (b_ids.find(t->track_id) == b_ids.end()) {
            out.push_back(t);
        }
    }
    return out;
}

inline std::pair<std::vector<CuboidTracker::TrackPtr>, std::vector<CuboidTracker::TrackPtr>> CuboidTracker::remove_duplicate(
    const std::vector<TrackPtr>& tracked,
    const std::vector<TrackPtr>& lost
) const {
    if (tracked.empty() || lost.empty()) {
        return {tracked, lost};
    }

    cv::Mat lost_det(static_cast<int>(lost.size()), 9, CV_32F, cv::Scalar(0));
    std::vector<int> det_indices(lost.size(), -1);
    for (int i = 0; i < static_cast<int>(lost.size()); ++i) {
        float* row = lost_det.ptr<float>(i);
        for (int j = 0; j < 9; ++j) {
            row[j] = lost[i]->cuboid[j];
        }
        det_indices[i] = lost[i]->idx;
    }

    const cv::Mat cost = build_cost(
        tracked,
        lost_det,
        det_indices,
        std::nullopt,
        center_gate_main_m_,
        0.60,
        0.30,
        0.10
    );

    std::unordered_set<int> drop_t;
    std::unordered_set<int> drop_l;
    for (int i = 0; i < cost.rows; ++i) {
        for (int j = 0; j < cost.cols; ++j) {
            if (cost.at<float>(i, j) < 0.12f) {
                const int age_t = tracked[i]->age();
                const int age_l = lost[j]->age();
                if (age_t >= age_l) {
                    drop_l.insert(j);
                } else {
                    drop_t.insert(i);
                }
            }
        }
    }

    std::vector<TrackPtr> tracked2;
    tracked2.reserve(tracked.size());
    for (int i = 0; i < static_cast<int>(tracked.size()); ++i) {
        if (drop_t.find(i) == drop_t.end()) {
            tracked2.push_back(tracked[i]);
        }
    }

    std::vector<TrackPtr> lost2;
    lost2.reserve(lost.size());
    for (int i = 0; i < static_cast<int>(lost.size()); ++i) {
        if (drop_l.find(i) == drop_l.end()) {
            lost2.push_back(lost[i]);
        }
    }

    return {tracked2, lost2};
}

// ============================================================
// 主更新函数
// ============================================================
inline std::tuple<cv::Mat, cv::Mat, cv::Mat, cv::Mat> CuboidTracker::update_and_estimate(
    std::int64_t timestamp,
    const cv::Mat& cuboids,
    double ego_yaw,
    const std::string& mode
) {
    if (mode != "ego" && mode != "world") {
        throw std::runtime_error("velocity mode must be 'ego' or 'world'");
    }

    frame_id_ += 1;
    const int mode_id = (mode == "ego") ? mode_id_ego_ : mode_id_world_;

    const cv::Mat arr = sanitize_cuboids(cuboids);
    const int n_det = arr.rows;
    const std::vector<char> valid = det_valid_mask(arr);

    std::vector<int> high_idx;
    std::vector<int> low_idx;
    high_idx.reserve(n_det);
    low_idx.reserve(n_det);
    for (int i = 0; i < n_det; ++i) {
        if (!valid[i]) {
            continue;
        }
        const float score = arr.at<float>(i, 6);
        if (score >= track_high_thresh_) {
            high_idx.push_back(i);
        } else if (score > track_low_thresh_ && score < track_high_thresh_) {
            low_idx.push_back(i);
        }
    }

    const cv::Mat det_high = gather_rows(arr, high_idx);
    const cv::Mat det_low = gather_rows(arr, low_idx);

    std::vector<TrackPtr> activated;
    std::vector<TrackPtr> refind;
    std::vector<TrackPtr> lost_now;
    std::vector<TrackPtr> removed_now;

    std::vector<TrackPtr> unconfirmed;
    std::vector<TrackPtr> tracked;
    unconfirmed.reserve(tracked_.size());
    tracked.reserve(tracked_.size());
    for (const auto& t : tracked_) {
        if (t->state == TRACK_NEW) {
            unconfirmed.push_back(t);
        } else if (t->state == TRACK_TRACKED) {
            tracked.push_back(t);
        }
    }

    // 1) main match: (tracked + lost) x high-conf detections
    const std::vector<TrackPtr> pool = joint_tracks(tracked, lost_);
    const cv::Mat cost_main = build_cost(
        pool,
        det_high,
        high_idx,
        timestamp,
        center_gate_main_m_,
        weight_cls_main_,
        weight_center_main_,
        weight_size_main_
    );
    std::vector<std::pair<int, int>> matches;
    std::vector<int> u_pool;
    std::vector<int> u_high;
    std::tie(matches, u_pool, u_high) = linear_assignment(cost_main, match_thresh_);
    for (const auto& m : matches) {
        const int it = m.first;
        const int idet = m.second;
        const auto& trk = pool[it];
        const cv::Vec<float, 9> det = row_as_vec9(det_high, idet);
        const int det_idx = high_idx[idet];
        if (trk->state == TRACK_TRACKED) {
            update_track(trk, det, det_idx, timestamp, TRACK_TRACKED);
            activated.push_back(trk);
        } else {
            update_track(trk, det, det_idx, timestamp, TRACK_TRACKED);
            refind.push_back(trk);
        }
    }

    // 2) low-conf match for unmatched tracked-only
    std::vector<TrackPtr> rem_tracked;
    rem_tracked.reserve(u_pool.size());
    for (const int i : u_pool) {
        if (pool[i]->state == TRACK_TRACKED) {
            rem_tracked.push_back(pool[i]);
        }
    }

    const cv::Mat cost_low = build_cost(
        rem_tracked,
        det_low,
        low_idx,
        timestamp,
        center_gate_low_m_,
        weight_cls_low_,
        weight_center_low_,
        weight_size_low_
    );
    std::vector<std::pair<int, int>> matches2;
    std::vector<int> u_rt;
    std::tie(matches2, u_rt, std::ignore) = linear_assignment(cost_low, match_thresh_low_);
    for (const auto& m : matches2) {
        const int it = m.first;
        const int idet = m.second;
        const auto& trk = rem_tracked[it];
        const cv::Vec<float, 9> det = row_as_vec9(det_low, idet);
        const int det_idx = low_idx[idet];
        update_track(trk, det, det_idx, timestamp, TRACK_TRACKED);
        activated.push_back(trk);
    }
    for (const int i : u_rt) {
        const auto& trk = rem_tracked[i];
        trk->state = TRACK_LOST;
        trk->missed += 1;
        lost_now.push_back(trk);
    }

    // 3) unconfirmed x unmatched high detections
    std::vector<int> rem_high_idx;
    rem_high_idx.reserve(u_high.size());
    for (const int i : u_high) {
        rem_high_idx.push_back(high_idx[i]);
    }
    const cv::Mat rem_high = gather_rows(arr, rem_high_idx);
    const cv::Mat cost_new = build_cost(
        unconfirmed,
        rem_high,
        rem_high_idx,
        timestamp,
        center_gate_main_m_,
        weight_cls_main_,
        weight_center_main_,
        weight_size_main_
    );
    std::vector<std::pair<int, int>> matches3;
    std::vector<int> u_unconfirmed;
    std::vector<int> u_high2;
    std::tie(matches3, u_unconfirmed, u_high2) = linear_assignment(cost_new, match_thresh_new_);
    for (const auto& m : matches3) {
        const int it = m.first;
        const int idet = m.second;
        const auto& trk = unconfirmed[it];
        const cv::Vec<float, 9> det = row_as_vec9(rem_high, idet);
        const int det_idx = rem_high_idx[idet];
        update_track(trk, det, det_idx, timestamp, TRACK_TRACKED);
        activated.push_back(trk);
    }
    for (const int i : u_unconfirmed) {
        const auto& trk = unconfirmed[i];
        trk->state = TRACK_REMOVED;
        removed_now.push_back(trk);
    }

    // 4) spawn new tracks from unmatched high detections
    for (const int i : u_high2) {
        const cv::Vec<float, 9> det = row_as_vec9(rem_high, i);
        const int det_idx = rem_high_idx[i];
        if (det[6] < new_track_thresh_) {
            continue;
        }
        activated.push_back(activate(det, det_idx, timestamp));
    }

    // 5) age-out lost tracks
    std::unordered_set<int> refind_ids;
    refind_ids.reserve(refind.size() * 2 + 1);
    for (const auto& t : refind) {
        refind_ids.insert(t->track_id);
    }
    for (const auto& trk : lost_) {
        if (refind_ids.find(trk->track_id) == refind_ids.end()) {
            predict_track_without_measure(trk, timestamp);
        }
        if ((frame_id_ - trk->frame_id > max_time_lost_) || (trk->missed > max_missed_)) {
            trk->state = TRACK_REMOVED;
            removed_now.push_back(trk);
        }
    }

    // 6) merge lists
    std::vector<TrackPtr> tracked_kept;
    tracked_kept.reserve(tracked_.size());
    for (const auto& t : tracked_) {
        if (t->state == TRACK_NEW || t->state == TRACK_TRACKED) {
            tracked_kept.push_back(t);
        }
    }
    tracked_ = joint_tracks(tracked_kept, activated);
    tracked_ = joint_tracks(tracked_, refind);

    lost_ = sub_tracks(lost_, tracked_);
    lost_.insert(lost_.end(), lost_now.begin(), lost_now.end());
    lost_ = sub_tracks(lost_, removed_now);

    std::tie(tracked_, lost_) = remove_duplicate(tracked_, lost_);

    removed_.insert(removed_.end(), removed_now.begin(), removed_now.end());
    if (removed_.size() > 2000U) {
        removed_.erase(removed_.begin(), removed_.end() - 1000);
    }

    std::vector<TrackPtr> active;
    active.reserve(tracked_.size());
    for (const auto& t : tracked_) {
        if (t->state == TRACK_NEW || t->state == TRACK_TRACKED) {
            active.push_back(t);
        }
    }
    std::sort(active.begin(), active.end(), [](const TrackPtr& a, const TrackPtr& b) {
        return a->track_id < b->track_id;
    });

    const double yaw = std::isfinite(ego_yaw) ? ego_yaw : 0.0;
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);

    const int n = static_cast<int>(active.size());
    cv::Mat track_info(n, 4, CV_32S, cv::Scalar(0));
    cv::Mat tracked_cuboids_raw(n, 9, CV_32F, cv::Scalar(0));
    cv::Mat tracked_cuboids(n, 9, CV_32F, cv::Scalar(0));
    cv::Mat tracked_cuboids_vel(n, 4, CV_32F, cv::Scalar(0));

    for (int i = 0; i < n; ++i) {
        const auto& trk = active[i];

        track_info.at<int>(i, 0) = trk->track_id;
        track_info.at<int>(i, 1) = trk->state;
        track_info.at<int>(i, 2) = trk->age();
        track_info.at<int>(i, 3) = trk->idx;

        float* row_c = tracked_cuboids.ptr<float>(i);
        for (int j = 0; j < 9; ++j) {
            row_c[j] = trk->cuboid[j];
        }

        float* row_raw = tracked_cuboids_raw.ptr<float>(i);
        if (trk->idx >= 0 && trk->idx < n_det) {
            const float* src = arr.ptr<float>(trk->idx);
            for (int j = 0; j < 9; ++j) {
                row_raw[j] = src[j];
            }
        } else {
            for (int j = 0; j < 9; ++j) {
                row_raw[j] = trk->cuboid[j];
            }
        }

        const double vx_e = trk->x(2, 0);
        const double vy_e = trk->x(3, 0);
        double dx_comp = 0.0;
        double dy_comp = 0.0;
        std::tie(dx_comp, dy_comp) = position_comp_delta(
            trk,
            vx_e,
            vy_e,
            trk->state,
            trk->motion_state
        );
        tracked_cuboids.at<float>(i, 0) = static_cast<float>(tracked_cuboids.at<float>(i, 0) + dx_comp);
        tracked_cuboids.at<float>(i, 1) = static_cast<float>(tracked_cuboids.at<float>(i, 1) + dy_comp);

        double vx_out = trk->out_vel_valid ? trk->out_vx : vx_e;
        double vy_out = trk->out_vel_valid ? trk->out_vy : vy_e;
        
        if (y_vel_smooth_enabled_ && trk->y_vel_output_valid) {
            vy_out = trk->y_vel_output;
        }
        
        if (mode == "world") {
            const double vx_base = vx_out;
            const double vy_base = vy_out;
            vx_out = c * vx_base - s * vy_base;
            vy_out = s * vx_base + c * vy_base;
        }

        tracked_cuboids_vel.at<float>(i, 0) = static_cast<float>(mode_id);
        tracked_cuboids_vel.at<float>(i, 1) = static_cast<float>(trk->motion_state);
        tracked_cuboids_vel.at<float>(i, 2) = static_cast<float>(vx_out);
        tracked_cuboids_vel.at<float>(i, 3) = static_cast<float>(vy_out);
    }

    return {track_info, tracked_cuboids_raw, tracked_cuboids, tracked_cuboids_vel};
}

inline int CuboidTracker::next_id() {
    track_count_ += 1;
    return track_count_;
}

} // namespace cuboid_tracker_impl