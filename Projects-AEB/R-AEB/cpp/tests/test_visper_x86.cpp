// tests/test_mathops_unittest.cpp
// 遍历目录下所有图片，逐张转 NV12 并调用 VisPer 推理/回调。
// 可选：recursive 递归遍历；loop 无限循环跑。

#include <algorithm>
#include <any>
#include <chrono>
#include <cctype>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <exception>
#include <filesystem>
#include <iostream>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "common/logger.hpp"
#include "VisPer.h"

namespace fs = std::filesystem;

// ---------- 常量配置（按你项目实际改路径） ----------
static const std::string kTask = "RAEB";
static const std::string kConfigPath = "/home/gq/guoqian/Projects-AEB/R-AEB/assets/RAEB/configFront_M112.jsonc";
static const std::string kModelPath  = "/home/gq/guoqian/Projects-AEB/R-AEB/assets/RAEB/TI_lyl.onnx";
static const fs::path kDefaultInputDir = "/home/gq/guoqian/Projects-AEB/R-AEB/cpp/tests/vis-bike2";
static const fs::path kVisDir        = "vis";
static const fs::path kVis3dVideoMp4Path = kVisDir / "raeb_vis_3d_10fps.mp4";
static const fs::path kVis3dVideoAviPath = kVisDir / "raeb_vis_3d_10fps.avi";
static const std::string kVis3dImagePrefix = "raeb_vis_3d_";
static constexpr double kVis3dVideoFps = 10.0;
static constexpr std::int64_t kSyntheticStartTimestampMs = 1780392678989LL;
static constexpr std::int64_t kFrameIntervalMs = 100LL;


struct TrackVelocityPoint {
    int frame;
    double vx;
    double vy;
    double vz;
};

using TrackVelMap = std::map<int, std::vector<TrackVelocityPoint>>;

class TrackVelocityVisualizer {
public:
    static void plotTrackVelocityComponents(
        const TrackVelMap& tracks_vel,
        const std::filesystem::path& out_vx_path,
        const std::filesystem::path& out_vy_path,
        const std::filesystem::path& out_vz_path,
        int width = 1280,
        int height = 720
    ) {
        // collect track ids that have points
        std::vector<int> track_ids;
        for (const auto& kv : tracks_vel) {
            if (!kv.second.empty()) track_ids.push_back(kv.first);
        }
        if (track_ids.empty()) return;
        std::sort(track_ids.begin(), track_ids.end());

        // color map by track id using OpenCV colormap (COLORMAP_TURBO)
        std::map<int, cv::Scalar> color_by_tid;
        const size_t n_tracks = track_ids.size();
        if (n_tracks > 0) {
            // create a 1 x n_tracks single-channel image with values [0..255]
            cv::Mat idx(1, static_cast<int>(n_tracks), CV_8UC1);
            for (size_t i = 0; i < n_tracks; ++i) {
                int v = static_cast<int>(std::round(255.0 * (static_cast<double>(i) / std::max<size_t>(1, n_tracks - 1))));
                idx.at<unsigned char>(0, static_cast<int>(i)) = static_cast<unsigned char>(v);
            }
            cv::Mat color_map;
            cv::applyColorMap(idx, color_map, cv::COLORMAP_TURBO);
            for (size_t i = 0; i < n_tracks; ++i) {
                cv::Vec3b c = color_map.at<cv::Vec3b>(0, static_cast<int>(i));
                int tid = track_ids[i];
                color_by_tid[tid] = cv::Scalar(c[0], c[1], c[2]);
            }
        }



        auto plot_component = [&](int comp_idx, const std::string& title, const std::string& ylabel,
                                  const std::filesystem::path& out_path, int w, int h) {
            std::map<int, std::vector<double>> frames_map;
            std::map<int, std::vector<double>> vals_map;
            int min_frame = std::numeric_limits<int>::max();
            int max_frame = std::numeric_limits<int>::min();
            double min_val = std::numeric_limits<double>::infinity();
            double max_val = -std::numeric_limits<double>::infinity();
            bool any = false;

            for (int tid : track_ids) {
                const auto& pts = tracks_vel.at(tid);
                if (pts.empty()) continue;
                std::vector<TrackVelocityPoint> s = pts;
                std::sort(s.begin(), s.end(), [](const TrackVelocityPoint& a, const TrackVelocityPoint& b){ return a.frame < b.frame; });
                for (const auto& p : s) {
                    double v = 0.0;
                    if (comp_idx == 1) v = p.vx;
                    else if (comp_idx == 2) v = p.vy;
                    else if (comp_idx == 3) v = p.vz;
                    frames_map[tid].push_back(static_cast<double>(p.frame));
                    vals_map[tid].push_back(v);
                    min_frame = std::min(min_frame, p.frame);
                    max_frame = std::max(max_frame, p.frame);
                    min_val = std::min(min_val, v);
                    max_val = std::max(max_val, v);
                    any = true;
                }
            }
            if (!any) return;

            // dynamic vertical range (match matplotlib autoscale), add small padding
            if (!std::isfinite(min_val) || !std::isfinite(max_val)) return;
            double val_span = max_val - min_val;
            if (val_span <= 1e-6) {
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
            if (frame_range <= 0) frame_range = 1;

            // margins and plot area
            const int margin_l = 80, margin_r = 60, margin_t = 50, margin_b = 70;
            int plot_w = w - margin_l - margin_r;
            int plot_h = h - margin_t - margin_b;

            cv::Mat img(h, w, CV_8UC3, cv::Scalar(255,255,255));

            // draw horizontal grid lines: choose step ~1 or nice division
            double approx_ticks = 10.0;
            double raw_step = val_range / approx_ticks;
            double step = std::pow(10.0, std::floor(std::log10(raw_step)));
            if (raw_step / step >= 5.0) step *= 5.0;
            else if (raw_step / step >= 2.0) step *= 2.0;
            // align start/end
            double start_v = std::floor(min_val / step) * step;
            double end_v = std::ceil(max_val / step) * step;
            for (double v = start_v; v <= end_v + 1e-9; v += step) {
                int y = margin_t + plot_h - static_cast<int>(((v - min_val) / val_range) * plot_h + 0.5);
                cv::line(img, cv::Point(margin_l, y), cv::Point(w - margin_r, y), cv::Scalar(230,230,230), 1);
                if (std::abs(v) < 1e-9) cv::line(img, cv::Point(margin_l, y), cv::Point(w - margin_r, y), cv::Scalar(180,180,180), 2);
                std::string lab = std::to_string(static_cast<int>(std::round(v)));
                cv::putText(img, lab, cv::Point(10, y + 5), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0,0,0), 1);
            }

            // vertical grid lines: choose step 5 or 1 depending on frame_range
            int fstep = (frame_range > 20) ? 5 : 1;
            int first_f = ((min_frame + fstep - 1) / fstep) * fstep;
            for (int f = first_f; f <= max_frame; f += fstep) {
                int x = margin_l + static_cast<int>(((f - min_frame) / static_cast<double>(frame_range)) * plot_w + 0.5);
                cv::line(img, cv::Point(x, margin_t), cv::Point(x, h - margin_b), cv::Scalar(230,230,230), 1);
                std::string lab = std::to_string(f);
                cv::putText(img, lab, cv::Point(x - 10, h - margin_b + 18), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0,0,0), 1);
            }

            // border
            cv::rectangle(img, cv::Point(margin_l, margin_t), cv::Point(w - margin_r, h - margin_b), cv::Scalar(100,100,100), 1);

            // draw each track: marker+line
            for (int tid : track_ids) {
                auto itf = frames_map.find(tid);
                if (itf == frames_map.end()) continue;
                const auto& fr = itf->second;
                const auto& va = vals_map[tid];
                cv::Scalar color = color_by_tid.count(tid) ? color_by_tid[tid] : cv::Scalar(0,0,0);

                // polyline
                for (size_t i = 1; i < fr.size(); ++i) {
                    int x1 = margin_l + static_cast<int>(((fr[i-1] - min_frame) / static_cast<double>(frame_range)) * plot_w + 0.5);
                    int x2 = margin_l + static_cast<int>(((fr[i]   - min_frame) / static_cast<double>(frame_range)) * plot_w + 0.5);
                    int y1 = margin_t + plot_h - static_cast<int>(((va[i-1] - min_val) / val_range) * plot_h + 0.5);
                    int y2 = margin_t + plot_h - static_cast<int>(((va[i]   - min_val) / val_range) * plot_h + 0.5);
                    cv::line(img, cv::Point(x1,y1), cv::Point(x2,y2), color, 2);
                }
                // markers
                for (size_t i = 0; i < fr.size(); ++i) {
                    int x = margin_l + static_cast<int>(((fr[i] - min_frame) / static_cast<double>(frame_range)) * plot_w + 0.5);
                    int y = margin_t + plot_h - static_cast<int>(((va[i] - min_val) / val_range) * plot_h + 0.5);
                    cv::circle(img, cv::Point(x,y), 2, color, -1);
                }

                // trailing label
                if (!fr.empty()) {
                    int x = margin_l + static_cast<int>(((fr.back() - min_frame) / static_cast<double>(frame_range)) * plot_w + 0.5);
                    double lastv = va.back();
                    int y = margin_t + plot_h - static_cast<int>(((lastv - min_val) / val_range) * plot_h + 0.5);
                    cv::putText(img, std::to_string(tid), cv::Point(x + 5, y + 5), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255,255,255), 3);
                    cv::putText(img, std::to_string(tid), cv::Point(x + 5, y + 5), cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
                }
            }

            // title and labels
            cv::putText(img, title, cv::Point(w/2 - 100, 35), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,0,0), 2);
            cv::putText(img, "Frame", cv::Point(w/2 - 30, h - 10), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,0,0), 1);
            cv::putText(img, ylabel, cv::Point(15, h/2), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,0,0), 1);

            std::filesystem::create_directories(out_path.parent_path());
            cv::imwrite(std::string(out_path), img);
        };

        plot_component(1, "Track VX vs Frame", "VX (m/s)", out_vx_path, width, height);
        plot_component(2, "Track VY vs Frame", "VY (m/s)", out_vy_path, width, height);
        plot_component(3, "Track VZ vs Frame", "VZ (m/s)", out_vz_path, width, height);
    }
};

// 是否递归遍历子目录
static const bool kRecursive = false;
// 是否无限循环跑
static const bool kLoop = false;

// ---------- 工具：判断图片后缀 ----------
static bool has_image_ext(const fs::path& p) {
    static const std::set<std::string> exts = {
        ".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"
    };
    std::string e = p.extension().string();
    std::transform(e.begin(), e.end(), e.begin(),
                   [](unsigned char c) { return (char)std::tolower(c); });
    return exts.count(e) > 0;
}

// ---------- 真实时间戳（epoch ms） ----------
static std::int64_t now_ms_epoch() {
    using namespace std::chrono;
    return (std::int64_t)duration_cast<milliseconds>(
        system_clock::now().time_since_epoch()
    ).count();
}

static fs::path get_input_dir() {
    const char* env = std::getenv("VISPER_INPUT_DIR");
    if (env != nullptr && env[0] != '\0') {
        return fs::path(env);
    }
    return kDefaultInputDir;
}

// ---------- BGR -> NV12 ----------
static std::vector<std::uint8_t> bgr_to_nv12(const cv::Mat& bgr) {
    if (bgr.empty()) {
        throw std::runtime_error("bgr_to_nv12: empty image");
    }
    if (bgr.type() != CV_8UC3) {
        throw std::runtime_error("bgr_to_nv12: expect CV_8UC3");
    }

    const int width = bgr.cols;
    const int height = bgr.rows;

    if ((width % 2) != 0 || (height % 2) != 0) {
        throw std::runtime_error("bgr_to_nv12: width and height must be even for NV12");
    }

    cv::Mat yuv_i420;
    cv::cvtColor(bgr, yuv_i420, cv::COLOR_BGR2YUV_I420);

    if (!yuv_i420.isContinuous()) {
        yuv_i420 = yuv_i420.clone();
    }

    const std::size_t y_size = (std::size_t)width * (std::size_t)height;
    const std::size_t uv_size = y_size / 4;

    const std::uint8_t* src = yuv_i420.ptr<std::uint8_t>(0);
    const std::uint8_t* src_y = src;
    const std::uint8_t* src_u = src_y + y_size;
    const std::uint8_t* src_v = src_u + uv_size;

    std::vector<std::uint8_t> nv12;
    nv12.resize(y_size + 2 * uv_size);

    std::uint8_t* dst_y = nv12.data();
    std::uint8_t* dst_uv = dst_y + y_size;

    std::memcpy(dst_y, src_y, y_size);

    for (std::size_t i = 0; i < uv_size; ++i) {
        dst_uv[2 * i + 0] = src_u[i];
        dst_uv[2 * i + 1] = src_v[i];
    }

    return nv12;
}

// ---------- RAEB 回调 ----------
static void RaebCallback(const std::unordered_map<std::string, std::any>& res) {
    auto ts = std::any_cast<std::int64_t>(res.at("timestamp"));
    LOG_INFO("[callback] RAEB timestamp = %lld", (long long)ts);

    auto it_objs = res.find("objs");
    if (it_objs != res.end()) {
        LOG_INFO("[callback] objs exists");
    } else {
        LOG_WARNING("[callback] objs missing");
    }
}

// ---------- 收集图片 ----------
static std::vector<fs::path> collect_images(const fs::path& input_dir, bool recursive) {
    std::vector<fs::path> images;

    if (!fs::exists(input_dir)) {
        throw std::runtime_error("Input dir not found: " + input_dir.string());
    }
    if (!fs::is_directory(input_dir)) {
        throw std::runtime_error("Input path is not a directory: " + input_dir.string());
    }

    if (recursive) {
        for (auto it = fs::recursive_directory_iterator(input_dir);
             it != fs::recursive_directory_iterator(); ++it) {
            if (it->is_regular_file() && has_image_ext(it->path())) {
                images.push_back(it->path());
            }
        }
    } else {
        for (const auto& entry : fs::directory_iterator(input_dir)) {
            if (entry.is_regular_file() && has_image_ext(entry.path())) {
                images.push_back(entry.path());
            }
        }
    }

    std::sort(images.begin(), images.end());
    return images;
}

static bool parse_vis3d_timestamp(const fs::path& p, std::int64_t& timestamp) {
    const std::string stem = p.stem().string();
    if (stem.rfind(kVis3dImagePrefix, 0) != 0) {
        return false;
    }

    const std::string ts_text = stem.substr(kVis3dImagePrefix.size());
    if (ts_text.empty()) {
        return false;
    }
    const bool all_digits = std::all_of(
        ts_text.begin(),
        ts_text.end(),
        [](unsigned char c) { return std::isdigit(c) != 0; }
    );
    if (!all_digits) {
        return false;
    }

    try {
        timestamp = std::stoll(ts_text);
    } catch (const std::exception&) {
        return false;
    }
    return true;
}

static std::vector<fs::path> collect_vis3d_images(const fs::path& vis_dir) {
    std::vector<std::pair<std::int64_t, fs::path>> keyed_images;
    std::error_code ec;
    if (!fs::is_directory(vis_dir, ec)) {
        return {};
    }

    for (const auto& entry : fs::directory_iterator(vis_dir)) {
        if (!entry.is_regular_file() || !has_image_ext(entry.path())) {
            continue;
        }

        std::int64_t ts = 0;
        if (parse_vis3d_timestamp(entry.path(), ts)) {
            keyed_images.emplace_back(ts, entry.path());
        }
    }

    std::sort(
        keyed_images.begin(),
        keyed_images.end(),
        [](const auto& lhs, const auto& rhs) {
            if (lhs.first != rhs.first) {
                return lhs.first < rhs.first;
            }
            return lhs.second.string() < rhs.second.string();
        }
    );

    std::vector<fs::path> images;
    images.reserve(keyed_images.size());
    for (const auto& item : keyed_images) {
        images.push_back(item.second);
    }
    return images;
}

static bool build_vis3d_video() {
    const std::vector<fs::path> frames = collect_vis3d_images(kVisDir);
    if (frames.empty()) {
        LOG_WARNING("No 3D visualization images found in: %s", kVisDir.string().c_str());
        return false;
    }

    cv::Mat first = cv::imread(frames.front().string(), cv::IMREAD_COLOR);
    if (first.empty()) {
        LOG_WARNING("Failed to read first 3D visualization image: %s", frames.front().string().c_str());
        return false;
    }

    cv::VideoWriter writer;
    fs::path video_path = kVis3dVideoMp4Path;
    int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
    writer.open(video_path.string(), fourcc, kVis3dVideoFps, first.size(), true);
    if (!writer.isOpened()) {
        LOG_WARNING("Failed to open MP4 video writer, fallback to AVI: %s",
                    video_path.string().c_str());
        video_path = kVis3dVideoAviPath;
        fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
        writer.open(video_path.string(), fourcc, kVis3dVideoFps, first.size(), true);
    }
    if (!writer.isOpened()) {
        LOG_WARNING("Failed to open 3D visualization video writer: %s", video_path.string().c_str());
        return false;
    }

    std::size_t written = 0;
    for (const fs::path& frame_path : frames) {
        cv::Mat frame = cv::imread(frame_path.string(), cv::IMREAD_COLOR);
        if (frame.empty()) {
            LOG_WARNING("Skip unreadable 3D visualization image: %s", frame_path.string().c_str());
            continue;
        }
        if (frame.size() != first.size()) {
            cv::resize(frame, frame, first.size(), 0.0, 0.0, cv::INTER_LINEAR);
        }
        writer.write(frame);
        ++written;
    }
    writer.release();

    if (written == 0) {
        LOG_WARNING("No frames written to 3D visualization video: %s", video_path.string().c_str());
        return false;
    }

    std::error_code ec;
    const std::string abs_video = fs::absolute(video_path, ec).string();
    LOG_INFO("Save 3D visualization video: %s fps=%.1f frames=%zu",
             abs_video.c_str(), kVis3dVideoFps, written);
    return true;
}

static std::size_t run_once(const std::vector<fs::path>& images) {
    std::size_t ok_count = 0;
    TrackVelMap tracks_vel;
    TrackVelMap tracks_position;
    int frameP = 0;
    int frameB = 0;
    for (std::size_t i = 0; i < images.size(); ++i) {
        const fs::path& p = images[i];

        cv::Mat img = cv::imread(p.string(), cv::IMREAD_COLOR);
        if (img.empty()) {
            LOG_WARNING("Failed to read image: %s", p.string().c_str());
            continue;
        }

        // 直接调用，无 try/catch
        std::vector<std::uint8_t> nv12 = bgr_to_nv12(img);

        const std::uint8_t* data = nv12.data();
        const std::size_t length = nv12.size();

        // 使用确定性 10fps 时间戳，避免速度估计受本机运行耗时影响。
        const std::int64_t ts = kSyntheticStartTimestampMs +
                                static_cast<std::int64_t>(i) * kFrameIntervalMs;

        LOG_INFO("Infer: %s (w=%d h=%d) nv12=%zu ts=%lld",
                 p.filename().string().c_str(),
                 img.cols, img.rows, length, (long long)ts);

        // 推理 + 回调 + 获取结果
        VisPer_RunInfer(data, length, ts);

        auto result = VisPer_GetResult(kTask);
        if (!result.empty()) {
            ++ok_count;
        } else {
            LOG_WARNING("Empty result: %s", p.string().c_str());
        }
        
        if(1)
        { 
            // 在跟踪循环中收集数据
            const auto track_vel = result.find("tracked_cuboids_vel");
            cv::Mat g_track_vel = std::any_cast<const cv::Mat&>(track_vel->second);
            const auto track_cuboids = result.find("tracked_cuboids");//[cx0, cy1, cz2, l3, w4, h5, conf6, cls7, theta_abs8]
            cv::Mat g_track_cuboids = std::any_cast<const cv::Mat&>(track_cuboids->second);
            const auto track_info = result.find("track_info"); //[TrackID, TrackState, TrackAge, Idx]
            cv::Mat g_track_info = std::any_cast<const cv::Mat&>(track_info->second);
            // Collect data for all tracked objects (use real track IDs)
            for (int r = 0; r < g_track_cuboids.rows; ++r) {
                int orig_track_id = g_track_info.at<int>(r, 0);

                TrackVelocityPoint point;
                // use current frame index 'i' as the frame number
                point.frame = static_cast<int>(i);
                point.vx = g_track_vel.at<float>(r, 2);
                point.vy = g_track_vel.at<float>(r, 3);
                point.vz = 0.0;  // if available
                tracks_vel[orig_track_id].push_back(point);

                // position
                TrackVelocityPoint pos_pt = point;
                pos_pt.vx = g_track_cuboids.at<float>(r, 0);
                pos_pt.vy = g_track_cuboids.at<float>(r, 1);
                tracks_position[orig_track_id].push_back(pos_pt);
            }
        }
    }

        std::string OUT_XY_DIR = "/home/gq/guoqian/Projects-AEB/R-AEB/cpp/cmake-build-release-x86/";
        std::string vx_by_frame_path = OUT_XY_DIR + "frame_vs_vx_by_track.png";
        std::string vy_by_frame_path = OUT_XY_DIR + "frame_vs_vy_by_track.png";
        std::string vz_by_frame_path = OUT_XY_DIR + "frame_vs_vz_by_track.png";

        std::string x_by_frame_path = OUT_XY_DIR + "frame_vs_x_by_track.png";
        std::string y_by_frame_path = OUT_XY_DIR + "frame_vs_y_by_track.png";  
        std::string z_by_frame_path = OUT_XY_DIR + "frame_vs_z_by_track.png";
        // 绘制速度分量图
        TrackVelocityVisualizer::plotTrackVelocityComponents(
            tracks_vel,
            vx_by_frame_path,
            vy_by_frame_path,
            vz_by_frame_path
        );
        TrackVelocityVisualizer::plotTrackVelocityComponents(
            tracks_position,
            x_by_frame_path,
            y_by_frame_path,
            z_by_frame_path
        );

    return ok_count;
}

int main() {
    LOG_INFO("Start RAEB NV12 folder unittest");
    LOG_INFO("Init: task=%s config=%s model=%s",
             kTask.c_str(), kConfigPath.c_str(), kModelPath.c_str());

    VisPer_InitTask(kTask, kConfigPath, kModelPath);

    LOG_INFO("Register callback: task=%s", kTask.c_str());
    VisPer_RegCallback(kTask, RaebCallback);

    const fs::path input_dir = get_input_dir();

    auto images = collect_images(input_dir, kRecursive);
    if (images.empty()) {
        LOG_ERROR("No images found in: %s", input_dir.string().c_str());
        VisPer_CleanUp(); // 清理后台线程
        return 2;
    }

    LOG_INFO("Found %zu images in %s (recursive=%d loop=%d)",
             images.size(),
             input_dir.string().c_str(),
             kRecursive ? 1 : 0,
             kLoop ? 1 : 0);

    if (kLoop) {
        std::int64_t round = 0;
        while (true) {
            LOG_INFO("===== round=%lld start =====", (long long)round);
            std::size_t ok = run_once(images);
            LOG_INFO("===== round=%lld done: ok=%zu/%zu =====",
                     (long long)round, ok, images.size());
            ++round;
        }
        // 如果以后改成可退出循环，记得在退出前调用 VisPer_CleanUp()
        // VisPer_CleanUp();
        // return 0;
    } else {
        std::size_t ok = run_once(images);
        if (ok == 0) {
            LOG_ERROR("No successful results generated.");
            VisPer_CleanUp(); // 清理后台线程
            return 3;
        }
        LOG_INFO("Done: ok=%zu/%zu", ok, images.size());
    }
    
    VisPer_CleanUp(); // 正常结束清理后台线程
    (void)build_vis3d_video();
    return 0;
}
