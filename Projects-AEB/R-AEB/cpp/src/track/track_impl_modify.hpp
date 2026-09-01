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
#include <deque>

#include <opencv2/core.hpp>

#include "track_utils/matching.hpp"

// 在 CuboidTracker 类定义之前添加
struct DirectionSmoothParams {
    float max_movement_per_frame = 0.8f;      // 单帧最大位移 (m)
    float position_smooth_alpha = 0.4f;       // 位置EMA平滑系数
    int direction_median_window = 7;          // 方向中值窗口大小
    float direction_angle_threshold = 0.3f;   // 方向偏差阈值 (rad)
    float still_speed_threshold = 0.3f;       // 静止速度阈值 (m/s)
    float slow_speed_threshold = 1.0f;        // 慢速速度阈值 (m/s)
    int max_dir_history = 10;                 // 方向历史长度
    float stable_dir_threshold = 0.85f;       // 方向稳定阈值
    float min_speed_for_direction = 0.3f;     // 方向有效最小速度
    float measurement_jump_limit = 0.8f;      // 测量跳变限幅 (m)
};


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

    int age() const {
        return frame_id - start_frame + 1;
    }
    // ===== 方向平滑新增 =====
    // 位置历史 (x, y, timestamp)
    std::deque<std::tuple<float, float, std::int64_t>> pos_history;
    int max_history = 5;
    
    // 平滑位置
    float smooth_pos_x = 0.0f;
    float smooth_pos_y = 0.0f;
    bool has_smooth_pos = false;
    
    // 方向历史 (cos, sin)
    std::deque<std::pair<float, float>> dir_history;
    
    // 稳定方向
    float stable_dir_x = 1.0f;
    float stable_dir_y = 0.0f;
    int dir_stable_count = 0;
    
    // 角度历史（中值滤波）
    std::deque<float> angle_history;
    
    // 慢速平滑状态
    float slow_vx = 0.0f;
    float slow_vy = 0.0f;
    int still_counter = 0;
    
    // 速度置信度
    float last_confidence = 1.0f;
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
    // 新增：设置方向平滑参数
    void setDirectionSmoothParams(const DirectionSmoothParams& params);
    const DirectionSmoothParams& getDirectionSmoothParams() const;

    // 新增：内存优化方法
    void reserveMemory(int max_tracks);
    void trimMemory();
    size_t getMemoryUsage() const;
    std::tuple<cv::Mat, cv::Mat> update(std::int64_t timestamp, const cv::Mat& cuboids);

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
    // ===== 方向平滑方法 =====
    cv::Point2f stabilizeMeasurementPosition(
        TrackPtr& trk,
        const cv::Point2f& z,
        float max_movement = 0.8f);
    
    cv::Point2f smoothMeasurementPosition(
        TrackPtr& trk,
        const cv::Point2f& z,
        float alpha = 0.4f);
    
    cv::Point2f preprocessMeasurement(
        TrackPtr& trk,
        const cv::Point2f& z);

    cv::Point2f computeVelocityFromHistoryWeighted(
        const std::deque<std::tuple<float, float, std::int64_t>>& pos_history,
        float decay_factor = 0.8f);
    
    cv::Point2f computeSmoothVelocity(
        TrackPtr& trk,
        const cv::Point2f& z,
        std::int64_t timestamp);
    
    cv::Point2f smoothDirection(TrackPtr& trk, float vx, float vy);
    cv::Point2f medianFilterDirection(TrackPtr& trk, float vx, float vy);
    
    int classifyVelocityState(const TrackPtr& trk, float speed) const;
    cv::Point2f applyVelocitySmoothingStrategy(
        TrackPtr& trk,
        float raw_vx,
        float raw_vy);
    
    float computeVelocityConfidence(const TrackPtr& trk, float vx, float vy) const;
    cv::Point2f weightedVelocityOutput(
        TrackPtr& trk,
        float kf_vx,
        float kf_vy,
        float meas_vx,
        float meas_vy);
    
    float angleDiff(float a, float b) const;
    void clipVelocity(float& vx, float& vy, float max_speed) const;

    // ===== 内存优化相关 =====
    // 预分配的对象池
    std::vector<TrackPtr> track_pool_;
    std::unordered_set<int> track_id_pool_;
    
    // 内存统计
    struct MemoryStats {
        size_t active_tracks = 0;
        size_t lost_tracks = 0;
        size_t removed_tracks = 0;
        size_t history_bytes = 0;
    } mem_stats_;

    // 方向平滑参数
    DirectionSmoothParams dir_params_;
};

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

    // 初始化方向平滑参数（默认值）
    dir_params_.max_movement_per_frame = 0.8f;
    dir_params_.position_smooth_alpha = 0.4f;
    dir_params_.direction_median_window = 7;
    dir_params_.direction_angle_threshold = 0.3f;
    dir_params_.still_speed_threshold = 0.3f;
    dir_params_.slow_speed_threshold = 1.0f;
    dir_params_.max_dir_history = 10;
    dir_params_.stable_dir_threshold = 0.85f;
    dir_params_.min_speed_for_direction = 0.3f;
    dir_params_.measurement_jump_limit = 0.8f;

    // 预分配内存
    reserveMemory(100);
    reset();
}

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
    if ((!trk->last_meas_valid) || (dt <= min_effective_dt_sec_)) {
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

inline void CuboidTracker::update_output_velocity(
    const TrackPtr& trk,
    double vx,
    double vy,
    bool has_measurement
) const {
    const double alpha = has_measurement ? output_vel_alpha_ : output_vel_alpha_lost_;
    if (!trk->out_vel_valid) {
        trk->out_vx = vx;
        trk->out_vy = vy;
        trk->out_vel_valid = true;
        return;
    }

    trk->out_vx = (1.0 - alpha) * trk->out_vx + alpha * vx;
    trk->out_vy = (1.0 - alpha) * trk->out_vy + alpha * vy;
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
            // double weight = (tcls == 1) ? 0.5 : 1.0;
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
inline CuboidTracker::TrackPtr CuboidTracker::activate(const cv::Vec<float, 9>& det, int det_idx, std::int64_t timestamp) {
    TrackPtr trk = std::make_shared<CuboidTrack>();
    trk->track_id = next_id();
    trk->cuboid = det;
    trk->state = TRACK_NEW;
    trk->start_frame = frame_id_;
    trk->frame_id = frame_id_;
    trk->idx = det_idx;
    trk->last_timestamp = timestamp;
    trk->max_history = 5;
    
    // 预分配历史缓冲区
    trk->pos_history.clear();
    trk->dir_history.clear();
    trk->angle_history.clear();
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

    // 方向平滑初始化
    trk->smooth_pos_x = det[0];
    trk->smooth_pos_y = det[1];
    trk->has_smooth_pos = true;
    trk->stable_dir_x = 1.0f;
    trk->stable_dir_y = 0.0f;
    trk->dir_stable_count = 0;
    trk->slow_vx = 0.0f;
    trk->slow_vy = 0.0f;
    trk->still_counter = 0;
    trk->last_confidence = 1.0f;
    return trk;
}

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

    // cv::Matx<double, 2, 1> z(static_cast<double>(det[0]), static_cast<double>(det[1]));
    // ===== 修改：位置预处理 =====
    cv::Point2f z_raw(det[0], det[1]);
    cv::Point2f z_smooth = const_cast<CuboidTracker*>(this)->preprocessMeasurement(
        const_cast<TrackPtr&>(trk), z_raw);
    
    cv::Matx<double, 2, 1> z(z_smooth.x, z_smooth.y);
    // ===== 修改：多帧速度计算 =====
    cv::Point2f meas_vel = const_cast<CuboidTracker*>(this)->computeSmoothVelocity(
        const_cast<TrackPtr&>(trk), z_smooth, timestamp);
    bool has_meas_velocity = (std::hypot(meas_vel.x, meas_vel.y) > 0.01f);

    // ===== 修改：速度方向平滑 =====
    if (has_meas_velocity) {
        meas_vel = const_cast<CuboidTracker*>(this)->applyVelocitySmoothingStrategy(
            const_cast<TrackPtr&>(trk), meas_vel.x, meas_vel.y);
    }
    const double dx = z(0, 0) - x_pre(0, 0);
    const double dy = z(1, 0) - x_pre(1, 0);
    const double innov = std::hypot(dx, dy);
    if (innov > max_innovation_m_) {
        const double scale = max_innovation_m_ / std::max(innov, 1e-9);
        z(0, 0) = x_pre(0, 0) + dx * scale;
        z(1, 0) = x_pre(1, 0) + dy * scale;
    }

    double vmx = meas_vel.x;
    double vmy = meas_vel.y;
    if (!has_meas_velocity && trk->last_meas_valid && dt > min_effective_dt_sec_) {
        vmx = (z(0, 0) - trk->last_meas_x) / dt;
        vmy = (z(1, 0) - trk->last_meas_y) / dt;
        std::tie(vmx, vmy) = clip_vel(vmx, vmy);
        has_meas_velocity = true;
    }

    const double innov_used = std::hypot(z(0, 0) - x_pre(0, 0), z(1, 0) - x_pre(1, 0));

    cv::Matx<double, 4, 1> x_post{};
    cv::Matx<double, 4, 4> P_post{};
    kf_update(x_pre, P_pre, z, x_post, P_post);
    x_post = blend_measured_velocity(trk, x_post, z, dt, innov_used);
    x_post = clip_speed(x_post);
    // ===== 修改：使用置信度加权速度融合 =====
    if (has_meas_velocity) {
        cv::Point2f fused_vel = const_cast<CuboidTracker*>(this)->weightedVelocityOutput(
            const_cast<TrackPtr&>(trk),
            static_cast<float>(x_post(2, 0)),
            static_cast<float>(x_post(3, 0)),
            meas_vel.x, meas_vel.y);
        x_post(2, 0) = fused_vel.x;
        x_post(3, 0) = fused_vel.y;
    }

    double kf_vx = x_post(2, 0);
    double kf_vy = x_post(3, 0);
    double speed = std::hypot(kf_vx, kf_vy);
    const double vm_speed = has_meas_velocity ? std::hypot(vmx, vmy) : 0.0;

    bool trigger_turn_recover = false;
    if (has_meas_velocity && vm_speed >= turn_recover_min_speed_) {
        if (innov_used >= turn_recover_innov_m_) {
            trigger_turn_recover = true;
        }
        if (speed >= turn_recover_min_speed_) {
            const double cos_h = clip((kf_vx * vmx + kf_vy * vmy) / std::max(speed * vm_speed, 1e-9), -1.0, 1.0);
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
        x_post = apply_turn_recovery(trk, x_post, z, vmx, vmy, has_meas_velocity);
        kf_vx = x_post(2, 0);
        kf_vy = x_post(3, 0);
        speed = std::hypot(kf_vx, kf_vy);
    }

    const double fx = x_post(0, 0);
    const double fy = x_post(1, 0);
    update_motion_state(trk, speed);
    update_output_velocity(trk, kf_vx, kf_vy, true);

    trk->x = x_post;
    trk->P = P_post;
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

inline void CuboidTracker::predict_track_without_measure(const TrackPtr& trk, std::int64_t timestamp) const {
    if (timestamp == trk->last_timestamp) {
        return;
    }

    const double dt = compute_dt(timestamp, trk->last_timestamp);
    cv::Matx<double, 4, 1> x_pre{};
    cv::Matx<double, 4, 4> P_pre{};
    kf_predict(trk->x, trk->P, dt, x_pre, P_pre);
    x_pre = clip_speed(x_pre);

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

// ===== 方向平滑方法 =====
cv::Point2f CuboidTracker::stabilizeMeasurementPosition(
    TrackPtr& trk,
    const cv::Point2f& z,
    float max_movement)
{
    if (!trk->last_meas_valid) {
        return z;
    }

    float dx = z.x - static_cast<float>(trk->last_meas_x);
    float dy = z.y - static_cast<float>(trk->last_meas_y);
    float dist = std::hypot(dx, dy);

    if (dist > max_movement) {
        float scale = max_movement / dist;
        return cv::Point2f(
            static_cast<float>(trk->last_meas_x) + dx * scale,
            static_cast<float>(trk->last_meas_y) + dy * scale
        );
    }
    return z;
}

cv::Point2f CuboidTracker::smoothMeasurementPosition(
    TrackPtr& trk,
    const cv::Point2f& z,
    float alpha)
{
    if (!trk->has_smooth_pos) {
        trk->smooth_pos_x = z.x;
        trk->smooth_pos_y = z.y;
        trk->has_smooth_pos = true;
        return z;
    }

    trk->smooth_pos_x = alpha * z.x + (1.0f - alpha) * trk->smooth_pos_x;
    trk->smooth_pos_y = alpha * z.y + (1.0f - alpha) * trk->smooth_pos_y;
    return cv::Point2f(trk->smooth_pos_x, trk->smooth_pos_y);
}

cv::Point2f CuboidTracker::preprocessMeasurement(
    TrackPtr& trk,
    const cv::Point2f& z)
{
    cv::Point2f z_out = z;
    z_out = stabilizeMeasurementPosition(trk, z_out, dir_params_.measurement_jump_limit);
    z_out = smoothMeasurementPosition(trk, z_out, dir_params_.position_smooth_alpha);
    return z_out;
}

cv::Point2f CuboidTracker::computeVelocityFromHistoryWeighted(
    const std::deque<std::tuple<float, float, std::int64_t>>& pos_history,
    float decay_factor)
{
    if (pos_history.size() < 2) {
        return cv::Point2f(0.0f, 0.0f);
    }

    const size_t n = pos_history.size();
    std::vector<float> times(n), xs(n), ys(n), weights(n);
    
    const std::int64_t t0 = std::get<2>(pos_history[0]);
    float total_weight = 0.0f;

    for (size_t i = 0; i < n; ++i) {
        times[i] = static_cast<float>(std::get<2>(pos_history[i]) - t0);
        xs[i] = std::get<0>(pos_history[i]);
        ys[i] = std::get<1>(pos_history[i]);
        weights[i] = std::pow(decay_factor, static_cast<float>(n - 1 - i));
        total_weight += weights[i];
    }

    // 归一化权重
    for (auto& w : weights) {
        w /= total_weight;
    }

    // 加权最小二乘拟合：x = vx * t + bx
    float sum_tw = 0.0f, sum_xw = 0.0f, sum_yw = 0.0f;
    float sum_t2w = 0.0f, sum_txw = 0.0f, sum_tyw = 0.0f;

    for (size_t i = 0; i < n; ++i) {
        const float w = weights[i];
        sum_tw += times[i] * w;
        sum_xw += xs[i] * w;
        sum_yw += ys[i] * w;
        sum_t2w += times[i] * times[i] * w;
        sum_txw += times[i] * xs[i] * w;
        sum_tyw += times[i] * ys[i] * w;
    }

    const float denom = sum_t2w - sum_tw * sum_tw;
    if (std::abs(denom) < 1e-6f) {
        return cv::Point2f(0.0f, 0.0f);
    }

    const float vx = (sum_txw - sum_tw * sum_xw) / denom;
    const float vy = (sum_tyw - sum_tw * sum_yw) / denom;
    return cv::Point2f(vx, vy);
}

cv::Point2f CuboidTracker::computeSmoothVelocity(
    TrackPtr& trk,
    const cv::Point2f& z,
    std::int64_t timestamp)
{
    // 更新位置历史
    trk->pos_history.push_back(std::make_tuple(z.x, z.y, timestamp));
    if (trk->pos_history.size() > static_cast<size_t>(trk->max_history)) {
        trk->pos_history.pop_front();
    }

    // 使用历史拟合速度
    if (trk->pos_history.size() >= 3) {
        cv::Point2f vel = computeVelocityFromHistoryWeighted(trk->pos_history, 0.8f);
        const float speed = std::hypot(vel.x, vel.y);
        if (speed < 20.0f && std::isfinite(speed)) {
            return vel;
        }
    }

    // Fallback: 单帧差分
    if (trk->last_meas_valid) {
        const float dt = static_cast<float>(compute_dt(timestamp, trk->last_timestamp));
        if (dt > 0.005f) {
            float vx = (z.x - static_cast<float>(trk->last_meas_x)) / dt;
            float vy = (z.y - static_cast<float>(trk->last_meas_y)) / dt;
            clipVelocity(vx, vy, 20.0f);
            return cv::Point2f(vx, vy);
        }
    }

    return cv::Point2f(0.0f, 0.0f);
}

float CuboidTracker::angleDiff(float a, float b) const
{
    float diff = a - b;
    while (diff > M_PI) diff -= 2.0f * M_PI;
    while (diff < -M_PI) diff += 2.0f * M_PI;
    return diff;
}

cv::Point2f CuboidTracker::medianFilterDirection(
    TrackPtr& trk,
    float vx,
    float vy)
{
    const float speed = std::hypot(vx, vy);
    if (speed < dir_params_.min_speed_for_direction) {
        return cv::Point2f(vx, vy);
    }

    const float angle = std::atan2(vy, vx);
    trk->angle_history.push_back(angle);
    if (trk->angle_history.size() > static_cast<size_t>(dir_params_.direction_median_window)) {
        trk->angle_history.pop_front();
    }

    if (trk->angle_history.size() >= 3) {
        // 复制到vector并排序找中值
        std::vector<float> angles(trk->angle_history.begin(), trk->angle_history.end());
        std::nth_element(angles.begin(), angles.begin() + angles.size() / 2, angles.end());
        const float median_angle = angles[angles.size() / 2];

        const float diff = angleDiff(angle, median_angle);
        if (std::abs(diff) > dir_params_.direction_angle_threshold) {
            return cv::Point2f(std::cos(median_angle) * speed, std::sin(median_angle) * speed);
        }
    }

    return cv::Point2f(vx, vy);
}

cv::Point2f CuboidTracker::smoothDirection(TrackPtr& trk, float vx, float vy)
{
    const float speed = std::hypot(vx, vy);

    // 速度太小，使用历史稳定方向
    if (speed < dir_params_.min_speed_for_direction) {
        return cv::Point2f(trk->stable_dir_x * speed, trk->stable_dir_y * speed);
    }

    const float dir_x = vx / speed;
    const float dir_y = vy / speed;

    // 更新方向历史
    trk->dir_history.push_back(std::make_pair(dir_x, dir_y));
    if (trk->dir_history.size() > static_cast<size_t>(dir_params_.max_dir_history)) {
        trk->dir_history.pop_front();
    }

    // 方向一致性检查
    if (trk->dir_history.size() >= 3) {
        const size_t n = trk->dir_history.size();
        const size_t window = std::min(n, size_t(5));
        auto start = trk->dir_history.end() - window;

        float cos_sum = 0.0f, sin_sum = 0.0f;
        for (auto it = start; it != trk->dir_history.end(); ++it) {
            cos_sum += it->first;
            sin_sum += it->second;
        }

        const float avg_cos = cos_sum / window;
        const float avg_sin = sin_sum / window;
        const float avg_len = std::hypot(avg_cos, avg_sin);

        // 方向不稳定，使用稳定方向
        if (avg_len < 0.7f) {
            trk->dir_stable_count = 0;
            if (trk->age() < 5) {
                trk->stable_dir_x = dir_x;
                trk->stable_dir_y = dir_y;
            }
            return cv::Point2f(trk->stable_dir_x * speed, trk->stable_dir_y * speed);
        }

        // 方向稳定，更新稳定方向
        if (avg_len > dir_params_.stable_dir_threshold) {
            trk->dir_stable_count++;
            if (trk->dir_stable_count >= 2) {
                trk->stable_dir_x = avg_cos / avg_len;
                trk->stable_dir_y = avg_sin / avg_len;
            }
        }

        return cv::Point2f(trk->stable_dir_x * speed, trk->stable_dir_y * speed);
    }

    return cv::Point2f(vx, vy);
}

int CuboidTracker::classifyVelocityState(const TrackPtr& trk, float speed) const
{
    if (speed < dir_params_.still_speed_threshold) {
        return 0;  // 静止
    } else if (speed < dir_params_.slow_speed_threshold) {
        return 1;  // 慢速
    } else if (speed < 3.0f) {
        return 2;  // 中速
    } else {
        return 3;  // 快速
    }
}

cv::Point2f CuboidTracker::applyVelocitySmoothingStrategy(
    TrackPtr& trk,
    float raw_vx,
    float raw_vy)
{
    const float speed = std::hypot(raw_vx, raw_vy);
    const int state = classifyVelocityState(trk, speed);

    if (state == 0) {  // 静止
        const float alpha = 0.1f;
        trk->still_counter++;
        if (trk->still_counter > 3) {
            return cv::Point2f(0.0f, 0.0f);
        }
        return cv::Point2f(raw_vx * alpha, raw_vy * alpha);
    }

    if (state == 1) {  // 慢速：方向稳定性优先
        cv::Point2f v = medianFilterDirection(trk, raw_vx, raw_vy);
        v = smoothDirection(trk, v.x, v.y);

        const float alpha = 0.3f;
        const float vx = alpha * v.x + (1.0f - alpha) * trk->slow_vx;
        const float vy = alpha * v.y + (1.0f - alpha) * trk->slow_vy;
        trk->slow_vx = vx;
        trk->slow_vy = vy;
        return cv::Point2f(vx, vy);
    }

    if (state == 2) {  // 中速：中等平滑
        return smoothDirection(trk, raw_vx, raw_vy);
    }

    // 快速：最小平滑
    clipVelocity(raw_vx, raw_vy, 20.0f);
    return cv::Point2f(raw_vx, raw_vy);
}

float CuboidTracker::computeVelocityConfidence(
    const TrackPtr& trk,
    float vx,
    float vy) const
{
    const float speed = std::hypot(vx, vy);

    // 速度幅值置信度
    float speed_conf;
    if (speed < 0.3f) {
        speed_conf = 0.1f;
    } else if (speed < 1.0f) {
        speed_conf = 0.5f;
    } else {
        speed_conf = 0.9f;
    }

    // 方向一致性置信度
    float dir_conf = 1.0f;
    if (trk->dir_history.size() >= 5) {
        const size_t n = trk->dir_history.size();
        auto start = trk->dir_history.end() - std::min(n, size_t(5));
        float cos_sum = 0.0f, sin_sum = 0.0f;
        for (auto it = start; it != trk->dir_history.end(); ++it) {
            cos_sum += it->first;
            sin_sum += it->second;
        }
        dir_conf = std::hypot(cos_sum, sin_sum) / 5.0f;
    }

    // 跟踪状态置信度
    float state_conf = 1.0f;
    if (trk->state == TRACK_NEW) {
        state_conf = 0.3f;
    } else if (trk->missed > 0) {
        state_conf = 0.5f;
    }

    return clip(speed_conf * dir_conf * state_conf, 0.0f, 1.0f);
}

cv::Point2f CuboidTracker::weightedVelocityOutput(
    TrackPtr& trk,
    float kf_vx,
    float kf_vy,
    float meas_vx,
    float meas_vy)
{
    const float confidence = computeVelocityConfidence(trk, meas_vx, meas_vy);
    trk->last_confidence = confidence;

    // 低置信度时更多依赖卡尔曼滤波
    const float beta = 1.0f - confidence * 0.8f;  // 0.2 ~ 1.0

    float vx = (1.0f - beta) * kf_vx + beta * meas_vx;
    float vy = (1.0f - beta) * kf_vy + beta * meas_vy;

    // 置信度极低，使用KF
    if (confidence < 0.2f) {
        return cv::Point2f(kf_vx, kf_vy);
    }

    return cv::Point2f(vx, vy);
}

void CuboidTracker::clipVelocity(float& vx, float& vy, float max_speed) const
{
    const float speed = std::hypot(vx, vy);
    if (speed > max_speed) {
        const float scale = max_speed / speed;
        vx *= scale;
        vy *= scale;
    }
}

void CuboidTracker::setDirectionSmoothParams(const DirectionSmoothParams& params)
{
    dir_params_ = params;
}

const DirectionSmoothParams& CuboidTracker::getDirectionSmoothParams() const
{
    return dir_params_;
}

// 在 CuboidTracker.cpp 中添加

void CuboidTracker::reserveMemory(int max_tracks)
{
    // 预分配向量容量
    tracked_.reserve(max_tracks);
    lost_.reserve(max_tracks);
    removed_.reserve(max_tracks);
    
    // 预分配对象池
    track_pool_.reserve(max_tracks);
    track_id_pool_.reserve(max_tracks * 2 + 1);
}

void CuboidTracker::trimMemory()
{
    // 清理已移除的跟踪对象
    const size_t max_removed = 1000;
    if (removed_.size() > max_removed) {
        removed_.erase(removed_.begin(), removed_.end() - max_removed);
    }
    
    // 压缩向量容量（如果需要）
    if (tracked_.capacity() > tracked_.size() * 2 + 100) {
        std::vector<TrackPtr>(tracked_).swap(tracked_);
    }
    if (lost_.capacity() > lost_.size() * 2 + 100) {
        std::vector<TrackPtr>(lost_).swap(lost_);
    }
}

size_t CuboidTracker::getMemoryUsage() const
{
    size_t total = 0;
    
    // 计算向量内存
    total += tracked_.capacity() * sizeof(TrackPtr);
    total += lost_.capacity() * sizeof(TrackPtr);
    total += removed_.capacity() * sizeof(TrackPtr);
    
    // 计算每个 Track 的内存
    auto calc_track_memory = [](const TrackPtr& trk) -> size_t {
        size_t size = sizeof(CuboidTrack);
        size += trk->pos_history.size() * sizeof(std::tuple<float, float, std::int64_t>);
        size += trk->dir_history.size() * sizeof(std::pair<float, float>);
        size += trk->angle_history.size() * sizeof(float);
        return size;
    };
    
    for (const auto& trk : tracked_) {
        total += calc_track_memory(trk);
    }
    for (const auto& trk : lost_) {
        total += calc_track_memory(trk);
    }
    for (const auto& trk : removed_) {
        total += calc_track_memory(trk);
    }
    
    return total;
}

} // namespace cuboid_tracker_impl
