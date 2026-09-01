// tests/test_mathops_unittest.cpp
// 批量处理文件夹下的所有图片和对应的txt文件

#include <algorithm>
#include <array>
#include <any>
#include <chrono>
#include <cctype>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <regex>
#include <unordered_map>
#include <unordered_set>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "common/logger.hpp"
#include "VisPer.h"
#include "cuboids/cuboids_api.h"
#include "postproc/postproc_api.h"
#include <nlohmann/json.hpp>
#include <track/track_api.h>

namespace fs = std::filesystem;

// ---------- 常量配置（按你项目实际改路径） ----------
static const std::string kTask = "RAEB";
static const std::string kConfigPath = "/home/gq/guoqian/Projects-AEB/R-AEB/assets/RAEB/configFront_M112.jsonc";
static const std::string kModelPath = "/home/gq/guoqian/Projects-AEB/R-AEB/assets/RAEB/TI_lyl.onnx";

// 修改为文件夹路径
static const fs::path k3dImageDir = "/home/gq/guoqian/Projects-AEB/M112/test";
static const fs::path k3dDetectionDir = "/home/gq/guoqian/Projects-AEB/3Dmodle/out_onnx2/";
static const fs::path k3dOutputDir = "./vis/";

static const std::string kVis3dImagePrefix = "raeb_vis_3d_";
static const std::string kVis3dOriPrefix = "raeb_vis_ori3d_";
double timestamp = 0;
struct TrackVelocityPoint
{
    int frame;
    double vx;
    double vy;
    double vz;
};
auto MatToText = [](const cv::Mat &m) -> std::string
{
    if (m.empty())
        return "empty";
    if (m.dims != 2)
        return "non-2d";

    std::ostringstream oss;
    oss << m.rows << "x" << m.cols << " C" << m.channels() << "\n";

    const int C = m.channels();
    const int depth = m.type() & CV_MAT_DEPTH_MASK;

    auto append = [&](int r, int c, int ch)
    {
        switch (depth)
        {
        case CV_32S:
            oss << m.ptr<int>(r)[c * C + ch];
            break;
        case CV_32F:
            oss << m.ptr<float>(r)[c * C + ch];
            break;
        case CV_64F:
            oss << m.ptr<double>(r)[c * C + ch];
            break;
        case CV_8U:
            oss << static_cast<int>(m.ptr<uint8_t>(r)[c * C + ch]);
            break;
        case CV_16S:
            oss << m.ptr<int16_t>(r)[c * C + ch];
            break;
        case CV_16U:
            oss << m.ptr<uint16_t>(r)[c * C + ch];
            break;
        default:
            oss << "?";
            break;
        }
    };

    oss << "[";
    for (int r = 0; r < m.rows; ++r)
    {
        oss << "[";
        for (int c = 0; c < m.cols; ++c)
        {
            if (c)
                oss << " ";
            if (C == 1)
            {
                append(r, c, 0);
            }
            else
            {
                oss << "(";
                for (int ch = 0; ch < C; ++ch)
                {
                    if (ch)
                        oss << ", ";
                    append(r, c, ch);
                }
                oss << ")";
            }
            oss << ",";
        }
        oss << "],";
    }
    oss << "]";
    return oss.str();
};

static float CleanVelocityValue(float v)
{
    return std::isfinite(v) ? v : 0.0f;
}

static float Hypot2(float x, float y)
{
    return static_cast<float>(std::hypot(static_cast<double>(x), static_cast<double>(y)));
}
static float RaebVelocitySmoothDtSec(std::int64_t timestamp, std::int64_t last_timestamp)
{
    if (last_timestamp <= 0 || timestamp <= last_timestamp)
    {
        return 1.0f / 30.0f;
    }

    const double dt = static_cast<double>(timestamp - last_timestamp) * 1e-3;
    if (!std::isfinite(dt) || dt <= 0.0)
    {
        return 1.0f / 30.0f;
    }
    return static_cast<float>(std::clamp(dt, 1.0 / 120.0, 0.20));
}
static void LimitVelocityDelta(float prev_vx, float prev_vy, float max_delta, float &vx, float &vy)
{
    const float dvx = vx - prev_vx;
    const float dvy = vy - prev_vy;
    const float delta = Hypot2(dvx, dvy);
    if (delta <= max_delta || delta <= 1e-6f)
    {
        return;
    }

    const float scale = max_delta / delta;
    vx = prev_vx + dvx * scale;
    vy = prev_vy + dvy * scale;
}

static std::string FormatMatShape(const cv::Mat &m)
{
    std::ostringstream oss;
    oss << "dims=" << m.dims << ", shape=[";
    for (int i = 0; i < m.dims; ++i)
    {
        if (i != 0)
        {
            oss << ",";
        }
        oss << m.size[i];
    }
    oss << "], rows=" << m.rows
        << ", cols=" << m.cols
        << ", type=" << m.type()
        << ", channels=" << m.channels()
        << ", elemSize=" << m.elemSize()
        << ", continuous=" << (m.isContinuous() ? "true" : "false")
        << ", empty=" << (m.empty() ? "true" : "false");
    return oss.str();
}
using TrackVelMap = std::map<int, std::vector<TrackVelocityPoint>>;
static std::unordered_map<std::string, std::any> MakeRaebResultSnapshot(
    std::int64_t timestamp,
    const cv::Mat &objs,
    const cv::Mat &track_info,
    const cv::Mat &tracked_cuboids_raw,
    const cv::Mat &tracked_cuboids,
    const cv::Mat &tracked_cuboids_vel)
{
    std::unordered_map<std::string, std::any> rr;
    rr.reserve(7);
    rr.emplace("task", std::string("RAEB"));
    rr.emplace("timestamp", timestamp);
    rr.emplace("objs", objs);
    rr.emplace("track_info", track_info);
    rr.emplace("tracked_cuboids_raw", tracked_cuboids_raw);
    rr.emplace("tracked_cuboids", tracked_cuboids);
    rr.emplace("tracked_cuboids_vel", tracked_cuboids_vel);
    return rr;
}
struct RaebVelocitySmoothState
{
    float vx{0.0f};
    float vy{0.0f};
    int mode{-1};
    std::int64_t timestamp{0};
    bool valid{false};
};

static std::unordered_map<int, RaebVelocitySmoothState> g_RAEB_VEL_SMOOTH_STATES;
static void SmoothTrackedCuboidsVelocity(
    std::int64_t timestamp,
    const cv::Mat &track_info,
    cv::Mat &tracked_cuboids_vel)
{
    if (tracked_cuboids_vel.empty() || track_info.empty())
    {
        g_RAEB_VEL_SMOOTH_STATES.clear();
        return;
    }
    if (track_info.type() != CV_32S || tracked_cuboids_vel.type() != CV_32F ||
        track_info.cols < 4 || tracked_cuboids_vel.cols < 4 ||
        track_info.rows != tracked_cuboids_vel.rows)
    {
        LOG_WARNING(
            "SmoothTrackedCuboidsVelocity skipped: track_info=%s, tracked_cuboids_vel=%s",
            FormatMatShape(track_info).c_str(),
            FormatMatShape(tracked_cuboids_vel).c_str());
        return;
    }

    constexpr int kTrackIdCol = 0;
    constexpr int kTrackStateCol = 1;
    constexpr int kTrackAgeCol = 2;
    constexpr int kVelModeCol = 0;
    constexpr int kMotionStateCol = 1;
    constexpr int kVelXCol = 2;
    constexpr int kVelYCol = 3;

    constexpr int kTrackLost = 2;
    constexpr int kMotionStatic = 1;

    constexpr float kMovingAlpha = 0.30f;
    constexpr float kLostAlpha = 0.12f;
    constexpr float kStaticAlpha = 0.18f;
    constexpr float kStaticDeadbandMps = 0.25f;
    constexpr float kOutputZeroMps = 0.05f;
    constexpr float kMaxAccelMps2 = 6.0f;
    constexpr float kMinFrameDeltaMps = 0.18f;
    constexpr float kMaxFrameDeltaMps = 0.85f;

    std::unordered_set<int> live_ids;
    live_ids.reserve(static_cast<std::size_t>(track_info.rows));

    for (int i = 0; i < track_info.rows; ++i)
    {
        const int track_id = track_info.at<int>(i, kTrackIdCol);
        const int track_state = track_info.at<int>(i, kTrackStateCol);
        const int track_age = track_info.at<int>(i, kTrackAgeCol);
        const int mode = static_cast<int>(std::lround(tracked_cuboids_vel.at<float>(i, kVelModeCol)));
        const int motion_state = static_cast<int>(std::lround(tracked_cuboids_vel.at<float>(i, kMotionStateCol)));

        float raw_vx = CleanVelocityValue(tracked_cuboids_vel.at<float>(i, kVelXCol));
        float raw_vy = CleanVelocityValue(tracked_cuboids_vel.at<float>(i, kVelYCol));
        float target_vx = raw_vx;
        float target_vy = raw_vy;

        if (motion_state == kMotionStatic && Hypot2(raw_vx, raw_vy) < kStaticDeadbandMps)
        {
            target_vx = 0.0f;
            target_vy = 0.0f;
        }

        live_ids.insert(track_id);
        RaebVelocitySmoothState &state = g_RAEB_VEL_SMOOTH_STATES[track_id];
        const bool reset_state = !state.valid || track_age <= 1 || state.mode != mode;
        if (reset_state)
        {
            state.vx = target_vx;
            state.vy = target_vy;
            state.mode = mode;
            state.timestamp = timestamp;
            state.valid = true;
        }
        else
        {
            const float alpha = track_state == kTrackLost
                                    ? kLostAlpha
                                    : (motion_state == kMotionStatic ? kStaticAlpha : kMovingAlpha);
            float smooth_vx = state.vx + alpha * (target_vx - state.vx);
            float smooth_vy = state.vy + alpha * (target_vy - state.vy);

            const float dt_sec = RaebVelocitySmoothDtSec(timestamp, state.timestamp);
            const float max_delta = std::clamp(kMaxAccelMps2 * dt_sec, kMinFrameDeltaMps, kMaxFrameDeltaMps);
            LimitVelocityDelta(state.vx, state.vy, max_delta, smooth_vx, smooth_vy);

            if (Hypot2(smooth_vx, smooth_vy) < kOutputZeroMps)
            {
                smooth_vx = 0.0f;
                smooth_vy = 0.0f;
            }

            state.vx = smooth_vx;
            state.vy = smooth_vy;
            state.mode = mode;
            state.timestamp = timestamp;
            state.valid = true;
        }

        tracked_cuboids_vel.at<float>(i, kVelXCol) = state.vx;
        tracked_cuboids_vel.at<float>(i, kVelYCol) = state.vy;
    }

    for (auto it = g_RAEB_VEL_SMOOTH_STATES.begin(); it != g_RAEB_VEL_SMOOTH_STATES.end();)
    {
        if (live_ids.find(it->first) == live_ids.end())
        {
            it = g_RAEB_VEL_SMOOTH_STATES.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

// ---------- 清空目录函数 ----------
static void clear_directory(const fs::path &dir_path)
{
    if (!fs::exists(dir_path))
    {
        return;
    }

    if (!fs::is_directory(dir_path))
    {
        LOG_WARNING("Path exists but is not a directory: %s", dir_path.string().c_str());
        return;
    }

    try
    {
        // 遍历目录并删除所有内容
        for (const auto &entry : fs::directory_iterator(dir_path))
        {
            try
            {
                fs::remove_all(entry.path());
                LOG_INFO("Removed: %s", entry.path().filename().string().c_str());
            }
            catch (const fs::filesystem_error &e)
            {
                LOG_WARNING("Failed to remove %s: %s",
                            entry.path().string().c_str(), e.what());
            }
        }
        LOG_INFO("Cleared directory: %s", dir_path.string().c_str());
    }
    catch (const fs::filesystem_error &e)
    {
        LOG_ERROR("Failed to clear directory %s: %s",
                  dir_path.string().c_str(), e.what());
    }
}

// ---------- 从文件名中提取数字 ----------
static long long extract_number_from_filename(const std::string &filename)
{
    // 尝试从文件名中提取所有数字
    std::regex number_pattern(R"(\d+)");
    std::smatch match;
    std::string::const_iterator search_start(filename.cbegin());

    std::vector<long long> numbers;
    while (std::regex_search(search_start, filename.cend(), match, number_pattern))
    {
        try
        {
            numbers.push_back(std::stoll(match.str()));
        }
        catch (const std::exception &)
        {
            // 忽略转换失败的数字
        }
        search_start = match.suffix().first;
    }

    // 如果找到了数字，返回最后一个（通常是主要的序列号）
    if (!numbers.empty())
    {
        return numbers.back();
    }

    // 如果没有找到数字，返回0
    return 0;
}

// ---------- 自定义排序函数：按文件名中的数字排序 ----------
static bool compare_by_number(const fs::path &a, const fs::path &b)
{
    std::string name_a = a.stem().string();
    std::string name_b = b.stem().string();

    long long num_a = extract_number_from_filename(name_a);
    long long num_b = extract_number_from_filename(name_b);

    // 如果数字相同或都提取失败，按字母顺序排序
    if (num_a == num_b)
    {
        return name_a < name_b;
    }

    return num_a < num_b;
}
class TrackVelocityVisualizer
{
public:
    static void plotTrackVelocityComponents(
        const TrackVelMap &tracks_vel,
        const std::filesystem::path &out_vx_path,
        const std::filesystem::path &out_vy_path,
        const std::filesystem::path &out_vz_path,
        int width = 1280,
        int height = 720)
    {
        // collect track ids that have points
        std::vector<int> track_ids;
        for (const auto &kv : tracks_vel)
        {
            if (!kv.second.empty())
                track_ids.push_back(kv.first);
        }
        if (track_ids.empty())
            return;
        std::sort(track_ids.begin(), track_ids.end());

        // color map by track id using OpenCV colormap (COLORMAP_TURBO)
        std::map<int, cv::Scalar> color_by_tid;
        const size_t n_tracks = track_ids.size();
        if (n_tracks > 0)
        {
            // create a 1 x n_tracks single-channel image with values [0..255]
            cv::Mat idx(1, static_cast<int>(n_tracks), CV_8UC1);
            for (size_t i = 0; i < n_tracks; ++i)
            {
                int v = static_cast<int>(std::round(255.0 * (static_cast<double>(i) / std::max<size_t>(1, n_tracks - 1))));
                idx.at<unsigned char>(0, static_cast<int>(i)) = static_cast<unsigned char>(v);
            }
            cv::Mat color_map;
            cv::applyColorMap(idx, color_map, cv::COLORMAP_TURBO);
            for (size_t i = 0; i < n_tracks; ++i)
            {
                cv::Vec3b c = color_map.at<cv::Vec3b>(0, static_cast<int>(i));
                int tid = track_ids[i];
                color_by_tid[tid] = cv::Scalar(c[0], c[1], c[2]);
            }
        }

        auto plot_component = [&](int comp_idx, const std::string &title, const std::string &ylabel,
                                  const std::filesystem::path &out_path, int w, int h)
        {
            std::map<int, std::vector<double>> frames_map;
            std::map<int, std::vector<double>> vals_map;
            int min_frame = std::numeric_limits<int>::max();
            int max_frame = std::numeric_limits<int>::min();
            double min_val = std::numeric_limits<double>::infinity();
            double max_val = -std::numeric_limits<double>::infinity();
            bool any = false;

            for (int tid : track_ids)
            {
                const auto &pts = tracks_vel.at(tid);
                if (pts.empty())
                    continue;
                std::vector<TrackVelocityPoint> s = pts;
                std::sort(s.begin(), s.end(), [](const TrackVelocityPoint &a, const TrackVelocityPoint &b)
                          { return a.frame < b.frame; });
                for (const auto &p : s)
                {
                    double v = 0.0;
                    if (comp_idx == 1)
                        v = p.vx;
                    else if (comp_idx == 2)
                        v = p.vy;
                    else if (comp_idx == 3)
                        v = p.vz;
                    frames_map[tid].push_back(static_cast<double>(p.frame));
                    vals_map[tid].push_back(v);
                    min_frame = std::min(min_frame, p.frame);
                    max_frame = std::max(max_frame, p.frame);
                    min_val = std::min(min_val, v);
                    max_val = std::max(max_val, v);
                    any = true;
                }
            }
            if (!any)
                return;

            // dynamic vertical range (match matplotlib autoscale), add small padding
            if (!std::isfinite(min_val) || !std::isfinite(max_val))
                return;
            double val_span = max_val - min_val;
            if (val_span <= 1e-6)
            {
                // avoid zero span
                min_val -= 0.5;
                max_val += 0.5;
                val_span = max_val - min_val;
            }
            double pad = val_span * 0.05; // 5% padding
            min_val -= pad;
            max_val += pad;
            double val_range = max_val - min_val;

            // frame range
            int frame_range = max_frame - min_frame;
            if (frame_range <= 0)
                frame_range = 1;

            // margins and plot area
            const int margin_l = 80, margin_r = 60, margin_t = 50, margin_b = 70;
            int plot_w = w - margin_l - margin_r;
            int plot_h = h - margin_t - margin_b;

            cv::Mat img(h, w, CV_8UC3, cv::Scalar(255, 255, 255));

            // draw horizontal grid lines: choose step ~1 or nice division
            double approx_ticks = 10.0;
            double raw_step = val_range / approx_ticks;
            double step = std::pow(10.0, std::floor(std::log10(raw_step)));
            if (raw_step / step >= 5.0)
                step *= 5.0;
            else if (raw_step / step >= 2.0)
                step *= 2.0;
            // align start/end
            double start_v = std::floor(min_val / step) * step;
            double end_v = std::ceil(max_val / step) * step;
            for (double v = start_v; v <= end_v + 1e-9; v += step)
            {
                int y = margin_t + plot_h - static_cast<int>(((v - min_val) / val_range) * plot_h + 0.5);
                cv::line(img, cv::Point(margin_l, y), cv::Point(w - margin_r, y), cv::Scalar(230, 230, 230), 1);
                if (std::abs(v) < 1e-9)
                    cv::line(img, cv::Point(margin_l, y), cv::Point(w - margin_r, y), cv::Scalar(180, 180, 180), 2);
                std::string lab = std::to_string(static_cast<int>(std::round(v)));
                cv::putText(img, lab, cv::Point(10, y + 5), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 0, 0), 1);
            }

            // vertical grid lines: choose step 5 or 1 depending on frame_range
            int fstep = (frame_range > 20) ? 5 : 1;
            int first_f = ((min_frame + fstep - 1) / fstep) * fstep;
            for (int f = first_f; f <= max_frame; f += fstep)
            {
                int x = margin_l + static_cast<int>(((f - min_frame) / static_cast<double>(frame_range)) * plot_w + 0.5);
                cv::line(img, cv::Point(x, margin_t), cv::Point(x, h - margin_b), cv::Scalar(230, 230, 230), 1);
                std::string lab = std::to_string(f);
                cv::putText(img, lab, cv::Point(x - 10, h - margin_b + 18), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 0, 0), 1);
            }

            // border
            cv::rectangle(img, cv::Point(margin_l, margin_t), cv::Point(w - margin_r, h - margin_b), cv::Scalar(100, 100, 100), 1);

            // draw each track: marker+line
            for (int tid : track_ids)
            {
                auto itf = frames_map.find(tid);
                if (itf == frames_map.end())
                    continue;
                const auto &fr = itf->second;
                const auto &va = vals_map[tid];
                cv::Scalar color = color_by_tid.count(tid) ? color_by_tid[tid] : cv::Scalar(0, 0, 0);

                // polyline
                for (size_t i = 1; i < fr.size(); ++i)
                {
                    int x1 = margin_l + static_cast<int>(((fr[i - 1] - min_frame) / static_cast<double>(frame_range)) * plot_w + 0.5);
                    int x2 = margin_l + static_cast<int>(((fr[i] - min_frame) / static_cast<double>(frame_range)) * plot_w + 0.5);
                    int y1 = margin_t + plot_h - static_cast<int>(((va[i - 1] - min_val) / val_range) * plot_h + 0.5);
                    int y2 = margin_t + plot_h - static_cast<int>(((va[i] - min_val) / val_range) * plot_h + 0.5);
                    cv::line(img, cv::Point(x1, y1), cv::Point(x2, y2), color, 2);
                }
                // markers
                for (size_t i = 0; i < fr.size(); ++i)
                {
                    int x = margin_l + static_cast<int>(((fr[i] - min_frame) / static_cast<double>(frame_range)) * plot_w + 0.5);
                    int y = margin_t + plot_h - static_cast<int>(((va[i] - min_val) / val_range) * plot_h + 0.5);
                    cv::circle(img, cv::Point(x, y), 2, color, -1);
                }

                // trailing label
                if (!fr.empty())
                {
                    int x = margin_l + static_cast<int>(((fr.back() - min_frame) / static_cast<double>(frame_range)) * plot_w + 0.5);
                    double lastv = va.back();
                    int y = margin_t + plot_h - static_cast<int>(((lastv - min_val) / val_range) * plot_h + 0.5);
                    cv::putText(img, std::to_string(tid), cv::Point(x + 5, y + 5), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 3);
                    cv::putText(img, std::to_string(tid), cv::Point(x + 5, y + 5), cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
                }
            }

            // title and labels
            cv::putText(img, title, cv::Point(w / 2 - 100, 35), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 0), 2);
            cv::putText(img, "Frame", cv::Point(w / 2 - 30, h - 10), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 1);
            cv::putText(img, ylabel, cv::Point(15, h / 2), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 1);

            std::filesystem::create_directories(out_path.parent_path());
            cv::imwrite(std::string(out_path), img);
        };

        plot_component(1, "Track VX vs Frame", "VX (m/s)", out_vx_path, width, height);
        plot_component(2, "Track VY vs Frame", "VY (m/s)", out_vy_path, width, height);
        plot_component(3, "Track VZ vs Frame", "VZ (m/s)", out_vz_path, width, height);
    }
};

// ---------- 工具：判断图片后缀 ----------
static bool has_image_ext(const fs::path &p)
{
    static const std::set<std::string> exts = {
        ".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"};
    std::string e = p.extension().string();
    std::transform(e.begin(), e.end(), e.begin(),
                   [](unsigned char c)
                   { return (char)std::tolower(c); });
    return exts.count(e) > 0;
}

// ---------- 加载3D检测结果 ----------
static cv::Mat load_3d_detections(const fs::path &detection_path)
{
    std::ifstream file(detection_path);
    if (!file)
    {
        LOG_WARNING("Detection file not found: %s", detection_path.string().c_str());
        return cv::Mat();
    }

    std::vector<std::array<float, 18>> rows;
    std::string line;
    while (std::getline(file, line))
    {
        if (line.empty())
            continue;
        std::istringstream line_stream(line);
        float cls = 0.0f;
        float conf = 0.0f;
        std::array<float, 16> points{};
        bool valid = static_cast<bool>(line_stream >> cls >> conf);
        for (int j = 0; valid && j < 16; ++j)
        {
            if (!(line_stream >> points[j]))
            {
                valid = false;
            }
        }
        if (valid)
        {
            std::array<float, 18> row{};
            for (int j = 0; j < 16; ++j)
            {
                row[j] = points[j];
            }
            row[16] = conf;
            row[17] = cls;
            rows.push_back(row);
        }
    }

    cv::Mat detections(static_cast<int>(rows.size()), 18, CV_32F);
    for (int i = 0; i < detections.rows; ++i)
    {
        for (int j = 0; j < 18; ++j)
        {
            detections.at<float>(i, j) = rows[i][j];
        }
    }
    return detections;
}

// ---------- 批量处理函数 ----------
static int batch_process_3d_cases()
{
    // 创建输出目录（如果不存在）
    if (!fs::exists(k3dOutputDir))
    {
        fs::create_directories(k3dOutputDir);
        LOG_INFO("Created output directory: %s", k3dOutputDir.string().c_str());
    }
    else
    {
        // 如果目录已存在，清空所有内容
        LOG_INFO("Clearing output directory: %s", k3dOutputDir.string().c_str());
        clear_directory(k3dOutputDir);
        // 重新创建目录（确保空目录存在）
        fs::create_directories(k3dOutputDir);
    }

    // 加载配置
    std::ifstream config_file(kConfigPath);
    if (!config_file)
    {
        LOG_ERROR("Failed to open cuboid config: %s", kConfigPath.c_str());
        return 3;
    }
    const nlohmann::json config =
        nlohmann::json::parse(config_file, nullptr, true, true);
    cuboids_InitRAEB(
        config.at("model").at("whitelist").get<nlohmann::json::object_t>(),
        config.at("cyl").get<nlohmann::json::object_t>());
    cuboid_tracker_Init(
        0.5f,   // track_high_thresh
        0.3f,   // track_low_thresh
        0.6f,   // match_thresh
        0.6f,   // new_track_thresh
        30,     // track_buffer
        30,     // frame_rate
        100.0f, // default_dt_ms
        1e-3f,  // timestamp_scale
        4.0f,   // center_gate_main_m
        5.5f,   // center_gate_low_m
        0.55f,  // weight_cls_main
        0.35f,  // weight_center_main
        0.10f,  // weight_size_main
        0.60f,  // weight_cls_low
        0.30f,  // weight_center_low
        0.10f,  // weight_size_low
        2.0f,   // sigma_a
        0.18f,  // sigma_z
        0.20f,  // vel_static_thresh
        0.50f,  // vel_move_thresh
        1,      // stable_frames
        30,     // max_missed
        8.0f,   // vel_clip_mps
        1.2f,   // max_innovation_m
        0.18f,  // vel_meas_blend
        0.45f,  // vel_meas_blend_max
        1.2f,   // vel_meas_innov_for_max
        0.00f,  // pos_comp_sec
        0.15f,  // pos_comp_min_speed
        0.60f,  // pos_comp_max_m
        0.60f,  // pos_comp_new_scale
        1.2f,   // pos_comp_innov_for_zero
        0.20f,  // pos_comp_dir_meas_base
        0.60f,  // pos_comp_dir_meas_turn_gain
        -0.20f, // pos_comp_reverse_cos
        3,      // pos_comp_reverse_hold
        0.40f,  // pos_comp_reverse_speed
        0.30f,  // turn_recover_cos
        0.10f,  // turn_recover_innov_m
        6,      // turn_recover_hold
        3,      // turn_recover_smooth_frames
        0.65f,  // turn_recover_vel_blend
        0.70f,  // turn_recover_pos_blend
        0.20f,  // turn_recover_vel_blend_smooth
        0.25f,  // turn_recover_pos_blend_smooth
        1.50f,  // turn_recover_comp_boost
        0.25f,  // turn_recover_min_speed
        3       // turn_recover_comp_freeze
    );
    cuboid_tracker_Reset();
    // 收集所有图片
    std::vector<fs::path> image_paths;
    if (!fs::exists(k3dImageDir) || !fs::is_directory(k3dImageDir))
    {
        LOG_ERROR("Image directory not found: %s", k3dImageDir.string().c_str());
        return 4;
    }

    for (const auto &entry : fs::directory_iterator(k3dImageDir))
    {
        if (entry.is_regular_file() && has_image_ext(entry.path()))
        {
            image_paths.push_back(entry.path());
        }
    }

    if (image_paths.empty())
    {
        LOG_ERROR("No images found in directory: %s", k3dImageDir.string().c_str());
        return 5;
    }

    // 使用自定义排序：按文件名中的数字排序
    std::sort(image_paths.begin(), image_paths.end(), compare_by_number);

    LOG_INFO("Found %zu images to process (sorted by number)", image_paths.size());

    // 打印前10个文件用于调试
    int debug_count = 0;
    for (const auto &img_path : image_paths)
    {
        if (debug_count >= 10)
            break;
        LOG_INFO("  Image %d: %s (number: %lld)",
                 debug_count + 1,
                 img_path.filename().string().c_str(),
                 extract_number_from_filename(img_path.stem().string()));
        debug_count++;
    }
    if (image_paths.size() > 10)
    {
        LOG_INFO("  ... and %zu more files", image_paths.size() - 10);
    }

    int success_count = 0;
    int fail_count = 0;
    int skip_count = 0;

    // 用于画图的数据收集
    TrackVelMap tracks_vel;
    TrackVelMap tracks_position;
    int frame_index = 0;

    for (size_t idx = 0; idx < image_paths.size(); ++idx)
    {
        const auto &img_path = image_paths[idx];

        // 构建对应的txt文件路径
        std::string stem = img_path.stem().string();
        fs::path det_path = k3dDetectionDir / (stem + ".txt");

        // 构建输出路径
        fs::path output_3d_path = k3dOutputDir / (kVis3dImagePrefix + stem + ".jpg");
        fs::path output_ori_path = k3dOutputDir / (kVis3dOriPrefix + stem + ".jpg");

        LOG_INFO("[%zu/%zu] Processing: %s", idx + 1, image_paths.size(),
                 img_path.filename().string().c_str());

        // 读取图片
        cv::Mat image = cv::imread(img_path.string(), cv::IMREAD_COLOR);
        if (image.empty())
        {
            LOG_ERROR("  Failed to read image: %s", img_path.string().c_str());
            ++fail_count;
            continue;
        }

        // 加载检测结果
        cv::Mat detections = load_3d_detections(det_path);
        if (detections.empty())
        {
            LOG_WARNING("  No valid detections for: %s, skipping", det_path.string().c_str());
            // 即使没有检测结果，也生成一张原始图片
            if (!cv::imwrite(output_ori_path.string(), image))
            {
                LOG_ERROR("  Failed to save original image");
            }
            ++skip_count;
            continue;
        }

        // 绘制原始3D检测结果
        cv::Mat vis_img_ori = postproc_Vis3DOriBounding(image, detections);
        if (!cv::imwrite(output_ori_path.string(), vis_img_ori))
        {
            LOG_ERROR("  Failed to save original 3D result: %s", output_ori_path.string().c_str());
        }

        // 解算3D cuboids
        const double grounding_z = 0.0;
        const cv::Mat cuboids = cuboids_ProcRAEB3D(detections, grounding_z);

        // 绘制3D结果
        const cv::Mat output = cuboids_VisCuboids(image, cuboids, nullptr, true);
        if (!cv::imwrite(output_3d_path.string(), output))
        {
            LOG_ERROR("  Failed to save 3D result: %s", output_3d_path.string().c_str());
        }

        // Tracker: 该模块在原设计中为单线程 StageB 使用；最小化锁范围以降低争用
        cv::Mat track_info;
        cv::Mat tracked_cuboids_raw;
        cv::Mat tracked_cuboids;
        cv::Mat tracked_cuboids_vel;
        timestamp = static_cast<double>(frame_index) * 100.0; // 假设每帧间隔为100ms
        double ego_yaw = 0.0;

        std::tie(track_info, tracked_cuboids_raw, tracked_cuboids, tracked_cuboids_vel) =
            cuboid_tracker_TrackAndEstimate(timestamp, cuboids, ego_yaw, "ego");
        SmoothTrackedCuboidsVelocity(timestamp, track_info, tracked_cuboids_vel);
        {
            const std::string s = MatToText(track_info);
            LOG_INFO("REAB track_info: %s", s.c_str());
        }
        {
            const std::string s = MatToText(tracked_cuboids_raw);
            LOG_INFO("REAB tracked_cuboids_raw: %s", s.c_str());
        }
        {
            const std::string s = MatToText(tracked_cuboids);
            LOG_INFO("REAB tracked_cuboids: %s", s.c_str());
        }
        {
            const std::string s = MatToText(tracked_cuboids_vel);
            LOG_INFO("REAB tracked_cuboids_vel: %s", s.c_str());
        }
        auto raeb_res_snapshot = MakeRaebResultSnapshot(
            timestamp,
            detections,
            track_info,
            tracked_cuboids_raw,
            tracked_cuboids,
            tracked_cuboids_vel);

        auto result = raeb_res_snapshot;

        if (1)
        {
            // 在跟踪循环中收集数据
            const auto track_vel = result.find("tracked_cuboids_vel");
            cv::Mat g_track_vel = std::any_cast<const cv::Mat &>(track_vel->second);
            const auto track_cuboids = result.find("tracked_cuboids"); //[cx0, cy1, cz2, l3, w4, h5, conf6, cls7, theta_abs8]
            cv::Mat g_track_cuboids = std::any_cast<const cv::Mat &>(track_cuboids->second);
            const auto track_info = result.find("track_info"); //[TrackID, TrackState, TrackAge, Idx]
            cv::Mat g_track_info = std::any_cast<const cv::Mat &>(track_info->second);
            // Collect data for all tracked objects (use real track IDs)
            for (int r = 0; r < g_track_cuboids.rows; ++r)
            {
                int orig_track_id = g_track_info.at<int>(r, 0);

                TrackVelocityPoint point;
                // use current frame index 'i' as the frame number
                point.frame = static_cast<int>(frame_index);
                point.vx = g_track_vel.at<float>(r, 2);
                point.vy = g_track_vel.at<float>(r, 3);
                point.vz = 0.0; // if available
                tracks_vel[orig_track_id].push_back(point);

                // position
                TrackVelocityPoint pos_pt = point;
                pos_pt.vx = g_track_cuboids.at<float>(r, 0);
                pos_pt.vy = g_track_cuboids.at<float>(r, 1);
                tracks_position[orig_track_id].push_back(pos_pt);
            }
        }

        ++success_count;
        ++frame_index;
    }

    // 画图：速度分量和位置分量
    std::string OUT_XY_DIR = "/home/gq/guoqian/Projects-AEB/R-AEB/cpp/cmake-build-release-x86/";
    std::string vx_by_frame_path = OUT_XY_DIR + "frame_vs_vx_by_track.png";
    std::string vy_by_frame_path = OUT_XY_DIR + "frame_vs_vy_by_track.png";
    std::string vz_by_frame_path = OUT_XY_DIR + "frame_vs_vz_by_track.png";

    std::string x_by_frame_path = OUT_XY_DIR + "frame_vs_x_by_track.png";
    std::string y_by_frame_path = OUT_XY_DIR + "frame_vs_y_by_track.png";
    std::string z_by_frame_path = OUT_XY_DIR + "frame_vs_z_by_track.png";

    // 绘制速度分量图
    LOG_INFO("Drawing velocity component plots...");
    TrackVelocityVisualizer::plotTrackVelocityComponents(
        tracks_vel,
        vx_by_frame_path,
        vy_by_frame_path,
        vz_by_frame_path);
    LOG_INFO("Velocity plots saved to: %s", OUT_XY_DIR.c_str());

    // 绘制位置分量图
    LOG_INFO("Drawing position component plots...");
    TrackVelocityVisualizer::plotTrackVelocityComponents(
        tracks_position,
        x_by_frame_path,
        y_by_frame_path,
        z_by_frame_path);
    LOG_INFO("Position plots saved to: %s", OUT_XY_DIR.c_str());

    LOG_INFO("Batch processing completed:");
    LOG_INFO("  Total images: %zu", image_paths.size());
    LOG_INFO("  Success: %d", success_count);
    LOG_INFO("  Failed: %d", fail_count);
    LOG_INFO("  Skipped (no detections): %d", skip_count);

    return 0;
}

// ---------- BGR -> NV12 ----------
static std::vector<std::uint8_t> bgr_to_nv12(const cv::Mat &bgr)
{
    if (bgr.empty())
    {
        throw std::runtime_error("bgr_to_nv12: empty image");
    }
    if (bgr.type() != CV_8UC3)
    {
        throw std::runtime_error("bgr_to_nv12: expect CV_8UC3");
    }

    const int width = bgr.cols;
    const int height = bgr.rows;

    if ((width % 2) != 0 || (height % 2) != 0)
    {
        throw std::runtime_error("bgr_to_nv12: width and height must be even for NV12");
    }

    cv::Mat yuv_i420;
    cv::cvtColor(bgr, yuv_i420, cv::COLOR_BGR2YUV_I420);

    if (!yuv_i420.isContinuous())
    {
        yuv_i420 = yuv_i420.clone();
    }

    const std::size_t y_size = (std::size_t)width * (std::size_t)height;
    const std::size_t uv_size = y_size / 4;

    const std::uint8_t *src = yuv_i420.ptr<std::uint8_t>(0);
    const std::uint8_t *src_y = src;
    const std::uint8_t *src_u = src_y + y_size;
    const std::uint8_t *src_v = src_u + uv_size;

    std::vector<std::uint8_t> nv12;
    nv12.resize(y_size + 2 * uv_size);

    std::uint8_t *dst_y = nv12.data();
    std::uint8_t *dst_uv = dst_y + y_size;

    std::memcpy(dst_y, src_y, y_size);

    for (std::size_t i = 0; i < uv_size; ++i)
    {
        dst_uv[2 * i + 0] = src_u[i];
        dst_uv[2 * i + 1] = src_v[i];
    }

    return nv12;
}

// ---------- 主函数 ----------
int main()
{
    LOG_INFO("Start batch 3D cuboid processing with plotting");
    return batch_process_3d_cases();
}