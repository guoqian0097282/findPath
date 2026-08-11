// visper.cpp
#include "VisPer.h"

#include <mutex>
#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <tuple>
#include <fstream>
#include <stdexcept>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <chrono>
#include <any>
#include <functional>
#include <thread>
#include <filesystem>
#include <sstream>
#include <memory>
#include <limits>
#include <cstring>
#include <cctype>
#include <cstdlib>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <nlohmann/json.hpp>

// —— 按 Python 模块名对应的 include —— //
#include "common/logger.hpp"         // SET_LOG_LEVEL/LOG_WARNING/DEBUG
#include "common/queue.hpp"
#include "common/visper_internal_utils.hpp"
#include "infer/infer_api.h"         // infer_InitRAEB, infer_InferRAEB
#include "preproc/preproc_api.h"     // preproc_InitRAEB, preproc_ProcRAEB
#include "postproc/postproc_api.h"   // postproc_InitRAEB, postproc_ProcRAEB
#include "cuboids/cuboids_api.h"
#include "track/track_api.h"
#include "vis/vis_api.h"

struct MI_SYS_FrameData_s;

#if defined(VISPER_ARCH_SGS)
#include "mi_sys.h"
#include "mi_sys_datatype.h"

std::vector<cv::Mat> infer_InferRAEBPhys(std::uintptr_t phy_addr,
                                         std::size_t length,
                                         bool squeeze_batch);
std::vector<cv::Mat> infer_InferRAEBFrame(const MI_SYS_FrameData_s& frame,
                                          bool squeeze_batch);
#endif

// ----------------- 新的 g_CTX 结构 -----------------
struct VisPerContext {
    // 原始 / 预处理后的图，默认空图但类型固定为 8UC3，通道数=3
    cv::Mat img = cv::Mat(1, 1, CV_8UC3);
    std::int64_t timestamp{0};

    // ========= RAEB =========
    std::unordered_map<std::string, std::any> raeb_result{
        {"task", std::any{std::string("RAEB")}},
        {"timestamp", std::any{std::int64_t(0)}},

        {"objs", std::any{cv::Mat(0, 7, CV_32F)}},

        {"track_info", std::any{cv::Mat(0, 4, CV_32S)}},
        {"tracked_cuboids_raw", std::any{cv::Mat(0, 9, CV_32F)}},
        {"tracked_cuboids", std::any{cv::Mat(0, 9, CV_32F)}},
        {"tracked_cuboids_vel", std::any{cv::Mat(0, 4, CV_32F)}},
    };

    // 新增：RAEB 额外数据
    std::unordered_map<std::string, std::any> raeb_extra;

    // Python: "op_result": { "preds": ... }
    std::unordered_map<std::string, std::any> op_result{
        {"task", std::any{std::string("OP")}}, // 推荐：显式放一个 std::string
        {"timestamp", std::any{}}, // int64_t：该次推理的时间戳
        {"preds", std::any{}}, // cv::Mat 或自定义结构
    };

    // 新增：OP 额外数据
    std::unordered_map<std::string, std::any> op_extra;
};


// =========================== 全局状态 ===========================
static std::vector<std::string> g_TASKS;
static std::mutex g_FUNC_LOCK;
static VisPerContext g_CTX;
static nlohmann::json cfg;

// 回调类型 & 全局回调
static std::function<void(const std::unordered_map<std::string, std::any>&)> g_RAEB_CALLBACK;
static std::function<void(const std::unordered_map<std::string, std::any>&)> g_OP_CALLBACK;

// =========================== RAEB StageB 队列/线程 ===========================
// StageA 输出给 StageB 的数据包
struct RaebStageAOut {
    std::int64_t timestamp{0};
    cv::Mat img_bgr;
    cv::Mat det_cat;
    cv::Mat proto;
    cv::Mat angle;
    std::shared_ptr<void> infer_output_owner;
    std::unordered_map<std::string, std::any> raeb_extra; // ego 快照
};

static BoundedBlockingQueue<RaebStageAOut> g_RAEB_STAGEB_QUEUE(1);
static std::thread g_RAEB_STAGEB_THREAD;

void VisPer_PrintResult(const std::string& task);

#if defined(VISPER_ARCH_SGS)
class SgsMappedBuffer {
public:
    SgsMappedBuffer(MI_PHY phy_addr, MI_U32 size)
        : size_(size) {
        if (phy_addr == 0) {
            throw std::runtime_error("SGS vis mmap input physical address is zero.");
        }
        if (size_ == 0) {
            throw std::runtime_error("SGS vis mmap input size is zero.");
        }
        MI_S32 ret = MI_SYS_Mmap(phy_addr, size_, &vir_, TRUE);
        if (ret != MI_SUCCESS) {
            throw std::runtime_error("MI_SYS_Mmap SGS vis input failed, ret=" + std::to_string(ret));
        }
    }

    ~SgsMappedBuffer() {
        if (vir_ != nullptr && size_ != 0) {
            MI_SYS_Munmap(vir_, size_);
        }
    }

    SgsMappedBuffer(const SgsMappedBuffer&) = delete;
    SgsMappedBuffer& operator=(const SgsMappedBuffer&) = delete;

    void* data() const { return vir_; }
    MI_U32 size() const { return size_; }

private:
    void* vir_{nullptr};
    MI_U32 size_{0};
};

static MI_U32 CheckedMiU32Size(std::size_t size, const char* what) {
    if (size == 0 || size > static_cast<std::size_t>(std::numeric_limits<MI_U32>::max())) {
        throw std::runtime_error(std::string(what) + " size is invalid: " + std::to_string(size));
    }
    return static_cast<MI_U32>(size);
}

static void FlushInvCacheForVis(void* addr, MI_U32 bytes, const char* label) {
    if (addr == nullptr || bytes == 0) {
        return;
    }
    MI_S32 ret = MI_SYS_FlushInvCache(addr, bytes);
    if (ret != MI_SUCCESS) {
        throw std::runtime_error(
            std::string("MI_SYS_FlushInvCache SGS vis input failed, label=") +
            label +
            ", ret=" +
            std::to_string(ret)
        );
    }
}

static cv::Mat BuildBgrFromSgsPhysicalForVis(std::uintptr_t phy_addr, std::size_t length) {
    const std::size_t image_bytes = preproc_InputBytes();
    if (length < image_bytes) {
        throw std::runtime_error(
            "SGS physical vis input length " + std::to_string(length) +
            " is smaller than configured image bytes " + std::to_string(image_bytes)
        );
    }

    const MI_U32 map_size = CheckedMiU32Size(length, "SGS physical vis input");
    SgsMappedBuffer mapped(static_cast<MI_PHY>(phy_addr), map_size);
    FlushInvCacheForVis(mapped.data(), mapped.size(), "phys");

    const auto* input = static_cast<const std::uint8_t*>(mapped.data());
    cv::Mat raw_img = preproc_ToMat(input, image_bytes);
    return preproc_Convert(raw_img, "BGR");
}

static cv::Mat BuildBgrFromSgsFrameForVis(const MI_SYS_FrameData_s& frame) {
    if (frame.ePhylayoutType == REALTIME_FRAME_DATA ||
        frame.pVirAddr[0] == MI_SYS_REALTIME_MAGIC_VADDR ||
        frame.phyAddr[0] == MI_SYS_REALTIME_MAGIC_PADDR) {
        throw std::runtime_error("SGS vis does not support REALTIME_FRAME_DATA magic addresses.");
    }
    if (frame.pVirAddr[0] == nullptr) {
        throw std::runtime_error("SGS vis frame.pVirAddr[0] is null.");
    }
    if (frame.u16Width == 0 || frame.u16Height == 0) {
        throw std::runtime_error("SGS vis frame width/height is zero.");
    }

    const int width = static_cast<int>(frame.u16Width);
    const int height = static_cast<int>(frame.u16Height);

    if (frame.ePixelFormat == E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420 ||
        frame.ePixelFormat == E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420_NV21) {
        if ((width & 1) || (height & 1)) {
            throw std::runtime_error("SGS vis NV12/NV21 frame width/height must be even.");
        }

        const int y_stride = frame.u32Stride[0] != 0 ? static_cast<int>(frame.u32Stride[0]) : width;
        const int uv_stride = frame.u32Stride[1] != 0 ? static_cast<int>(frame.u32Stride[1]) : y_stride;
        if (y_stride < width || uv_stride < width) {
            throw std::runtime_error("SGS vis NV12/NV21 stride is smaller than width.");
        }

        const auto* y_plane = static_cast<const std::uint8_t*>(frame.pVirAddr[0]);
        const bool has_separate_uv =
            frame.pVirAddr[1] != nullptr &&
            frame.pVirAddr[1] != MI_SYS_REALTIME_MAGIC_VADDR;
        const auto* uv_plane = has_separate_uv
                                   ? static_cast<const std::uint8_t*>(frame.pVirAddr[1])
                                   : y_plane + static_cast<std::size_t>(y_stride) * static_cast<std::size_t>(height);

        const std::size_t y_bytes = static_cast<std::size_t>(y_stride) * static_cast<std::size_t>(height);
        const std::size_t uv_bytes = static_cast<std::size_t>(uv_stride) * static_cast<std::size_t>(height / 2);
        FlushInvCacheForVis(const_cast<std::uint8_t*>(y_plane), CheckedMiU32Size(y_bytes, "SGS frame vis Y plane"), "frame-Y");
        if (has_separate_uv) {
            FlushInvCacheForVis(const_cast<std::uint8_t*>(uv_plane), CheckedMiU32Size(uv_bytes, "SGS frame vis UV plane"), "frame-UV");
        } else {
            FlushInvCacheForVis(
                const_cast<std::uint8_t*>(uv_plane),
                CheckedMiU32Size(uv_bytes, "SGS frame vis UV plane"),
                "frame-UV-contiguous"
            );
        }

        cv::Mat yuv(height + height / 2, width, CV_8UC1);
        for (int r = 0; r < height; ++r) {
            std::memcpy(yuv.ptr<std::uint8_t>(r), y_plane + static_cast<std::size_t>(r) * y_stride, width);
        }
        for (int r = 0; r < height / 2; ++r) {
            std::memcpy(yuv.ptr<std::uint8_t>(height + r), uv_plane + static_cast<std::size_t>(r) * uv_stride, width);
        }

        cv::Mat bgr;
        const int code = frame.ePixelFormat == E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420_NV21
                             ? cv::COLOR_YUV2BGR_NV21
                             : cv::COLOR_YUV2BGR_NV12;
        cv::cvtColor(yuv, bgr, code);
        return bgr;
    }

    if (frame.ePixelFormat == E_MI_SYS_PIXEL_FRAME_BGR888 ||
        frame.ePixelFormat == E_MI_SYS_PIXEL_FRAME_RGB888) {
        const int stride = frame.u32Stride[0] != 0 ? static_cast<int>(frame.u32Stride[0]) : width * 3;
        if (stride < width * 3) {
            throw std::runtime_error("SGS vis RGB/BGR stride is smaller than width*3.");
        }

        const auto* src = static_cast<const std::uint8_t*>(frame.pVirAddr[0]);
        const std::size_t bytes = static_cast<std::size_t>(stride) * static_cast<std::size_t>(height);
        FlushInvCacheForVis(const_cast<std::uint8_t*>(src), CheckedMiU32Size(bytes, "SGS frame vis RGB/BGR plane"), "frame-RGBBGR");

        cv::Mat packed(height, width, CV_8UC3);
        for (int r = 0; r < height; ++r) {
            std::memcpy(packed.ptr<std::uint8_t>(r), src + static_cast<std::size_t>(r) * stride, static_cast<std::size_t>(width) * 3);
        }

        if (frame.ePixelFormat == E_MI_SYS_PIXEL_FRAME_BGR888) {
            return packed;
        }

        cv::Mat bgr;
        cv::cvtColor(packed, bgr, cv::COLOR_RGB2BGR);
        return bgr;
    }

    throw std::runtime_error(
        "SGS vis unsupported frame pixel format: " + std::to_string(static_cast<int>(frame.ePixelFormat))
    );
}
#endif

static std::string FormatTaskList(const std::vector<std::string>& tasks) {
    std::ostringstream oss;
    oss << "[";
    for (std::size_t i = 0; i < tasks.size(); ++i) {
        if (i != 0) {
            oss << ", ";
        }
        oss << '"' << tasks[i] << '"';
    }
    oss << "]";
    return oss.str();
}

static std::string FormatMatShape(const cv::Mat& m) {
    std::ostringstream oss;
    oss << "dims=" << m.dims << ", shape=[";
    for (int i = 0; i < m.dims; ++i) {
        if (i != 0) {
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

static std::string FormatMatVectorShapes(const std::vector<cv::Mat>& mats) {
    std::ostringstream oss;
    oss << "count=" << mats.size();
    for (std::size_t i = 0; i < mats.size(); ++i) {
        oss << "; [" << i << "]{" << FormatMatShape(mats[i]) << "}";
    }
    return oss.str();
}

struct RaebInferOutputs {
    cv::Mat det_cat;
    cv::Mat proto;
    cv::Mat angle;
};

static std::vector<int> ShapeWithoutLeadingSingletons(const cv::Mat& m) {
    std::vector<int> shape;
    shape.reserve(static_cast<std::size_t>(std::max(m.dims, 0)));
    for (int i = 0; i < m.dims; ++i) {
        shape.push_back(m.size[i]);
    }
    while (shape.size() > 1U && shape.front() == 1) {
        shape.erase(shape.begin());
    }
    return shape;
}

static cv::Mat ReshapeTo(const cv::Mat& src, const std::vector<int>& shape) {
    cv::Mat base = src.isContinuous() ? src : src.clone();
    return base.reshape(1, static_cast<int>(shape.size()), shape.data());
}

static RaebInferOutputs NormalizeRaebInferOutputs(const std::vector<cv::Mat>& preds,
                                                  const char* source) {
    if (preds.size() < 3U) {
        throw std::runtime_error(std::string(source) + " returned fewer than 3 tensors.");
    }

    int proto_idx = -1;
    int det_idx = -1;
    int angle_idx = -1;
    int det_rows = -1;
    int det_cols = -1;

    std::vector<std::vector<int>> shapes;
    shapes.reserve(preds.size());
    for (const auto& pred : preds) {
        shapes.push_back(ShapeWithoutLeadingSingletons(pred));
    }

    for (std::size_t i = 0; i < preds.size(); ++i) {
        if (preds[i].type() != CV_32F) {
            continue;
        }
        if (shapes[i].size() == 3U && proto_idx < 0) {
            proto_idx = static_cast<int>(i);
        } else if (shapes[i].size() == 2U) {
            const int rows = shapes[i][0];
            const int cols = shapes[i][1];
            if (rows > det_rows) {
                det_idx = static_cast<int>(i);
                det_rows = rows;
                det_cols = cols;
            }
        }
    }

    for (std::size_t i = 0; i < preds.size(); ++i) {
        if (static_cast<int>(i) == det_idx || preds[i].type() != CV_32F || shapes[i].size() != 2U) {
            continue;
        }
        if (angle_idx < 0 || shapes[i][1] == det_cols) {
            angle_idx = static_cast<int>(i);
            if (shapes[i][1] == det_cols) {
                break;
            }
        }
    }

    if (det_idx < 0 || proto_idx < 0 || angle_idx < 0) {
        throw std::runtime_error(
            std::string(source) +
            " output layout not recognized. Expected det_cat [C,L], proto 3D, angle [A,L]. Got " +
            FormatMatVectorShapes(preds)
        );
    }

    RaebInferOutputs out;
    out.det_cat = ReshapeTo(preds[det_idx], shapes[det_idx]);
    out.proto = ReshapeTo(preds[proto_idx], shapes[proto_idx]);
    out.angle = ReshapeTo(preds[angle_idx], shapes[angle_idx]);

    if (!(out.det_cat.type() == CV_32F && out.det_cat.dims == 2)) {
        throw std::runtime_error("det_cat must be [C,L] CV_32F");
    }

    return out;
}

static std::string FormatAnyMapKeys(const std::unordered_map<std::string, std::any>& m) {
    std::ostringstream oss;
    oss << "{";
    bool first = true;
    for (const auto& kv : m) {
        if (!first) {
            oss << ", ";
        }
        first = false;
        oss << kv.first << ":";
        if (kv.second.has_value()) {
            oss << kv.second.type().name();
        } else {
            oss << "empty";
        }
    }
    oss << "}";
    return oss.str();
}

static std::unordered_map<std::string, std::any> MakeRaebResultSnapshot(
    std::int64_t timestamp,
    const cv::Mat& objs,
    const cv::Mat& track_info,
    const cv::Mat& tracked_cuboids_raw,
    const cv::Mat& tracked_cuboids,
    const cv::Mat& tracked_cuboids_vel
) {
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

struct RaebVelocitySmoothState {
    float vx{0.0f};
    float vy{0.0f};
    int mode{-1};
    std::int64_t timestamp{0};
    bool valid{false};
};

static std::unordered_map<int, RaebVelocitySmoothState> g_RAEB_VEL_SMOOTH_STATES;

static float CleanVelocityValue(float v) {
    return std::isfinite(v) ? v : 0.0f;
}

static float Hypot2(float x, float y) {
    return static_cast<float>(std::hypot(static_cast<double>(x), static_cast<double>(y)));
}

static float RaebVelocitySmoothDtSec(std::int64_t timestamp, std::int64_t last_timestamp) {
    if (last_timestamp <= 0 || timestamp <= last_timestamp) {
        return 1.0f / 30.0f;
    }

    const double dt = static_cast<double>(timestamp - last_timestamp) * 1e-3;
    if (!std::isfinite(dt) || dt <= 0.0) {
        return 1.0f / 30.0f;
    }
    return static_cast<float>(std::clamp(dt, 1.0 / 120.0, 0.20));
}

static void LimitVelocityDelta(float prev_vx, float prev_vy, float max_delta, float& vx, float& vy) {
    const float dvx = vx - prev_vx;
    const float dvy = vy - prev_vy;
    const float delta = Hypot2(dvx, dvy);
    if (delta <= max_delta || delta <= 1e-6f) {
        return;
    }

    const float scale = max_delta / delta;
    vx = prev_vx + dvx * scale;
    vy = prev_vy + dvy * scale;
}

static void SmoothTrackedCuboidsVelocity(
    std::int64_t timestamp,
    const cv::Mat& track_info,
    cv::Mat& tracked_cuboids_vel
) {
    if (tracked_cuboids_vel.empty() || track_info.empty()) {
        g_RAEB_VEL_SMOOTH_STATES.clear();
        return;
    }
    if (track_info.type() != CV_32S || tracked_cuboids_vel.type() != CV_32F ||
        track_info.cols < 4 || tracked_cuboids_vel.cols < 4 ||
        track_info.rows != tracked_cuboids_vel.rows) {
        LOG_WARNING(
            "SmoothTrackedCuboidsVelocity skipped: track_info=%s, tracked_cuboids_vel=%s",
            FormatMatShape(track_info).c_str(),
            FormatMatShape(tracked_cuboids_vel).c_str()
        );
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

    for (int i = 0; i < track_info.rows; ++i) {
        const int track_id = track_info.at<int>(i, kTrackIdCol);
        const int track_state = track_info.at<int>(i, kTrackStateCol);
        const int track_age = track_info.at<int>(i, kTrackAgeCol);
        const int mode = static_cast<int>(std::lround(tracked_cuboids_vel.at<float>(i, kVelModeCol)));
        const int motion_state = static_cast<int>(std::lround(tracked_cuboids_vel.at<float>(i, kMotionStateCol)));

        float raw_vx = CleanVelocityValue(tracked_cuboids_vel.at<float>(i, kVelXCol));
        float raw_vy = CleanVelocityValue(tracked_cuboids_vel.at<float>(i, kVelYCol));
        float target_vx = raw_vx;
        float target_vy = raw_vy;

        if (motion_state == kMotionStatic && Hypot2(raw_vx, raw_vy) < kStaticDeadbandMps) {
            target_vx = 0.0f;
            target_vy = 0.0f;
        }

        live_ids.insert(track_id);
        RaebVelocitySmoothState& state = g_RAEB_VEL_SMOOTH_STATES[track_id];
        const bool reset_state = !state.valid || track_age <= 1 || state.mode != mode;
        if (reset_state) {
            state.vx = target_vx;
            state.vy = target_vy;
            state.mode = mode;
            state.timestamp = timestamp;
            state.valid = true;
        } else {
            const float alpha = track_state == kTrackLost
                                    ? kLostAlpha
                                    : (motion_state == kMotionStatic ? kStaticAlpha : kMovingAlpha);
            float smooth_vx = state.vx + alpha * (target_vx - state.vx);
            float smooth_vy = state.vy + alpha * (target_vy - state.vy);

            const float dt_sec = RaebVelocitySmoothDtSec(timestamp, state.timestamp);
            const float max_delta = std::clamp(kMaxAccelMps2 * dt_sec, kMinFrameDeltaMps, kMaxFrameDeltaMps);
            LimitVelocityDelta(state.vx, state.vy, max_delta, smooth_vx, smooth_vy);

            if (Hypot2(smooth_vx, smooth_vy) < kOutputZeroMps) {
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

    for (auto it = g_RAEB_VEL_SMOOTH_STATES.begin(); it != g_RAEB_VEL_SMOOTH_STATES.end();) {
        if (live_ids.find(it->first) == live_ids.end()) {
            it = g_RAEB_VEL_SMOOTH_STATES.erase(it);
        } else {
            ++it;
        }
    }
}


// 真正干活的函数（这里放 LOG_TIMER）
static void RaebStageBWorker() {
    for (;;) {
        RaebStageAOut job;
        if (!g_RAEB_STAGEB_QUEUE.pop(job)) {
            LOG_INFO("RaebStageBWorker queue closed and empty; worker exit.");
            break;
        }

        LOG_TIMER_CREATE(t_stageb,"RAEB StageB start");

        cv::Mat objs;
        cv::Mat masks;
        if (!job.img_bgr.empty()) {
            RaebPostprocResult postproc_result =
                postproc_ProcRAEBWithMasks(job.det_cat, job.proto, job.angle);
            objs = std::move(postproc_result.objs);
            masks = std::move(postproc_result.masks);
        } else {
            objs = postproc_ProcRAEB(job.det_cat, job.proto, job.angle);
        }
        LOG_TIMER(t_stageb, "B Postproc");

        cv::Mat cuboids_raw_all = cuboids_ProcRAEB(objs, 0.0, masks);

        const double ego_yaw = visper::internal::GetEgoYawFromExtra(job.raeb_extra);

        cv::Mat track_info;
        cv::Mat tracked_cuboids_raw;
        cv::Mat tracked_cuboids;
        cv::Mat tracked_cuboids_vel;
        std::tie(track_info, tracked_cuboids_raw, tracked_cuboids, tracked_cuboids_vel) =
            cuboid_tracker_TrackAndEstimate(
                job.timestamp,
                cuboids_raw_all,
                ego_yaw,
                "ego"
            );
        SmoothTrackedCuboidsVelocity(job.timestamp, track_info, tracked_cuboids_vel);
        LOG_TIMER(t_stageb, "B Tracker");

        // StageB 可视化模块：仅当 cwd 下存在 visl 文件时自动落盘。
        if (!job.img_bgr.empty()) {
            try {
                vis_RunRaebStageBModule(
                    job.timestamp,
                    job.img_bgr,
                    objs,
                    masks,
                    track_info,
                    tracked_cuboids,
                    tracked_cuboids_vel
                );
            } catch (const std::exception& e) {
                LOG_WARNING(std::string("RAEB StageB vis module exception: ") + e.what());
            } catch (...) {
                LOG_WARNING("RAEB StageB vis module exception: unknown");
            }
        }

        auto raeb_res_snapshot = MakeRaebResultSnapshot(
            job.timestamp,
            objs,
            track_info,
            tracked_cuboids_raw,
            tracked_cuboids,
            tracked_cuboids_vel
        );

        std::function<void(const std::unordered_map<std::string, std::any>&)> raeb_cb;
        {
            std::lock_guard<std::mutex> lk(g_FUNC_LOCK);
            g_CTX.timestamp = job.timestamp;
            g_CTX.img = job.img_bgr;
            g_CTX.raeb_result = raeb_res_snapshot;
            raeb_cb = g_RAEB_CALLBACK;
        }

        VisPer_PrintResult("RAEB");

        if (raeb_cb) {
            // 回调作为 StageB 的最后一步执行（在锁外执行，避免阻塞/死锁）。
            try {
                raeb_cb(raeb_res_snapshot);
            } catch (const std::exception& e) {
                LOG_WARNING(std::string("RAEB StageB callback exception: ") + e.what());
            } catch (...) {
                LOG_WARNING("RAEB StageB callback exception: unknown");
            }
        }

        LOG_TIMER_DONE(t_stageb, "RAEB StageB done");
    }
}

static std::string BuildLogSavePathIfTriggered() {
    namespace fs = std::filesystem;
    std::error_code ec;
    const fs::path cwd = fs::current_path(ec);
    if (ec) {
        return "";
    }

    const fs::path marker = cwd / "savel";
    const bool exists = fs::exists(marker, ec);
    if (ec || !exists) {
        return "";
    }

    const bool is_file = fs::is_regular_file(marker, ec);
    if (ec || !is_file) {
        return "";
    }

    return (cwd / "log" / "visper.log").string();
}

static LogLevel ParseLogLevelOrDefault(const char* value, LogLevel fallback) {
    if (value == nullptr || value[0] == '\0') {
        return fallback;
    }
    std::string level(value);
    std::transform(level.begin(), level.end(), level.begin(), [](unsigned char ch) {
        return static_cast<char>(std::toupper(ch));
    });

    if (level == "DEBUG") return LogLevel::DEBUG;
    if (level == "INFO") return LogLevel::INFO;
    if (level == "WARNING" || level == "WARN") return LogLevel::WARNING;
    if (level == "ERROR") return LogLevel::ERROR;
    if (level == "CRITICAL" || level == "FATAL") return LogLevel::CRITICAL;
    return fallback;
}

static LogLevel DefaultLogLevelForBuild() {
#if defined(VISPER_ARCH_TI)
    return LogLevel::ERROR;
#else
    return LogLevel::INFO;
#endif
}

static LogLevel ResolveInitialLogLevel() {
    return ParseLogLevelOrDefault(std::getenv("VISPER_LOG_LEVEL"), DefaultLogLevelForBuild());
}

static std::string TrimEnvValue(const char* value) {
    if (value == nullptr) {
        return {};
    }
    std::string text(value);
    std::size_t begin = 0;
    while (begin < text.size() && std::isspace(static_cast<unsigned char>(text[begin])) != 0) {
        ++begin;
    }

    std::size_t end = text.size();
    while (end > begin && std::isspace(static_cast<unsigned char>(text[end - 1])) != 0) {
        --end;
    }

    return text.substr(begin, end - begin);
}

static bool EnvFlagEnabled(const char* name, bool fallback) {
    const char* env = std::getenv(name);
    if (env == nullptr || env[0] == '\0') {
        return fallback;
    }

    const std::string value = TrimEnvValue(env);
    return value == "1" || value == "true" || value == "TRUE" || value == "on" || value == "ON" ||
           value == "yes" || value == "YES";
}

static bool TiStageAOnlyEnabled() {
#if defined(VISPER_ARCH_TI)
    return EnvFlagEnabled("VISPER_TI_STAGEA_ONLY", false);
#else
    return false;
#endif
}

// =========================== API 实现 ===========================
void VisPer_InitTask(const std::string& task,
                     const std::string& config_path,
                     const std::string& model_path,
                     const std::string& ti_target);


void VisPer_InitTask(const std::string& task,
                     const std::string& config_path,
                     const std::string& model_path) {
    VisPer_InitTask(task, config_path, model_path, "");
}

void VisPer_InitTask(const std::string& task,
                     const std::string& config_path,
                     const std::string& model_path,
                     const std::string& ti_target) {
    std::lock_guard<std::mutex> lk(g_FUNC_LOCK);

    // 第一次进来才设日志
    if (g_TASKS.empty()) {
        INIT_LOG("", BuildLogSavePathIfTriggered(), ResolveInitialLogLevel());
    }

    const bool already_inited =
        std::find(g_TASKS.begin(), g_TASKS.end(), task) != g_TASKS.end();

    LOG_INFO(
        "VisPer_InitTask enter task=\"%s\", config=\"%s\", model=\"%s\", already_inited=%d, tasks_before=%s",
        task.c_str(),
        config_path.c_str(),
        model_path.c_str(),
        already_inited ? 1 : 0,
        FormatTaskList(g_TASKS).c_str()
    );

    if (!already_inited) {
        if (task == "RAEB") {
            std::ifstream ifs(config_path);
            if (!ifs) {
                LOG_WARNING("VisPer_InitTask RAEB config open failed: %s", config_path.c_str());
                throw std::runtime_error("无法打开配置文件: " + config_path);
            }
            cfg = nlohmann::json::parse(ifs, nullptr, true, true);

            preproc_Init(
                cfg["cyl"]["image_width"].get<int>(),
                cfg["cyl"]["image_height"].get<int>(),
                cfg["cyl"]["image_type"].get<std::string>(),
                cfg["model"]["target_type"].get<std::string>()
            );

            infer_InitRAEB(
                model_path,
                ti_target
            );

            postproc_InitRAEB(
                cfg["model"]["num_cls"].get<int>(),
                cfg["model"]["num_mask"].get<int>(),
                cfg["model"]["id2name"].get<nlohmann::json::object_t>(),
                cfg["model"]["whitelist"].get<nlohmann::json::object_t>(),
                cfg["model"]["conf_thresh"].get<float>(),
                cfg["model"]["iou_thresh"].get<float>(),
                cfg["model"]["mask_thresh"].get<float>(),
                cfg["model"]["mask_up"].get<int>(),
                (cfg.contains("angle_postproc") && cfg["angle_postproc"].is_object())
                    ? cfg["angle_postproc"].get<nlohmann::json::object_t>()
                    : nlohmann::json::object_t{}
            );

            cuboids_InitRAEB(
                cfg["model"]["whitelist"].get<nlohmann::json::object_t>(),
                cfg["cyl"].get<nlohmann::json::object_t>()
            );

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
            // 启动 RAEB StageB 后台线程
            if (!g_RAEB_STAGEB_THREAD.joinable()) {
                g_RAEB_STAGEB_THREAD = std::thread(RaebStageBWorker);
            } else {
                LOG_INFO("VisPer_InitTask RAEB StageB thread already joinable");
            }
        } else if (task == "OP") {
            // 以后补 OP 的初始化
            LOG_WARNING("VisPer_InitTask OP requested but OP initialization is not implemented");
        } else {
            LOG_WARNING(std::string("未识别的 task: ") + task);
            LOG_WARNING("VisPer_InitTask unknown task will not be registered: %s", task.c_str());
            LOG_INFO("VisPer_InitTask exit task=\"%s\", tasks_after=%s", task.c_str(), FormatTaskList(g_TASKS).c_str());
            return;
        }
    }

    if (std::find(g_TASKS.begin(), g_TASKS.end(), task) == g_TASKS.end()) {
        g_TASKS.push_back(task);
        LOG_INFO("VisPer_InitTask pushed task=\"%s\"", task.c_str());
    } else {
        LOG_INFO("VisPer_InitTask task already present after init: \"%s\"", task.c_str());
    }
    LOG_INFO("VisPer_InitTask exit task=\"%s\", tasks_after=%s", task.c_str(), FormatTaskList(g_TASKS).c_str());
}


void VisPer_RegCallback(const std::string& task,
                        const std::function<void(const std::unordered_map<std::string, std::any>&)>& cb) {
    std::lock_guard<std::mutex> lk(g_FUNC_LOCK);
    if (task == "RAEB") {
        g_RAEB_CALLBACK = cb;
    } else if (task == "OP") {
        g_OP_CALLBACK = cb;
    } else {
        LOG_WARNING(std::string("VisPer_SetCallback: 未识别的 task: ") + task);
    }
}


void VisPer_PushExtraData(const std::string& task, const std::unordered_map<std::string, std::any>& extra) {
    std::lock_guard<std::mutex> lk(g_FUNC_LOCK);

    if (task == "RAEB") {
        if (std::find(g_TASKS.begin(), g_TASKS.end(), "RAEB") == g_TASKS.end()) {
            LOG_WARNING("VisPer_PushExtraData: RAEB not initialized yet");
            return;
        }
        g_CTX.raeb_extra = extra;
    } else if (task == "OP") {
        if (std::find(g_TASKS.begin(), g_TASKS.end(), "OP") == g_TASKS.end()) {
            LOG_WARNING("VisPer_PushExtraData: OP not initialized yet");
            return;
        }
        g_CTX.op_extra = extra;
    } else {
        LOG_WARNING(std::string("VisPer_PushExtraData: unknown task: ") + task);
    }
}


// StageA：预处理 + 推理，把结果丢给 StageB 队列
void VisPer_RunInfer(const cv::Mat& img, std::int64_t timestamp) {
    LOG_DEBUG("=====================================");
    const std::int64_t now_ts = static_cast<std::int64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count()
    );
    LOG_INFO(
        "VisPer_RunInfer now_ts=%lld, arg_ts=%lld, delta=%lld ms",
        static_cast<long long>(now_ts),
        static_cast<long long>(timestamp),
        static_cast<long long>(now_ts - timestamp)
    );
    LOG_TIMER_CREATE(t,"RAEB StageA start");

    // 1) 预处理
    const bool need_stageb_vis = vis_HasRaebStageBTrigger();
    cv::Mat img_bgr;
    if (need_stageb_vis) {
        img_bgr = preproc_Convert(img, "BGR");
    }

#if defined(VISPER_ARCH_SGS)
    cv::Mat data_storage;
    const cv::Mat* data_ptr = &img;
    if (!(img.dims == 2 && img.type() == CV_8UC1)) {
        data_storage = preproc_ToData(img, 0, std::vector<int>{0, 1, 2}, CV_8U);
        data_ptr = &data_storage;
    }
    const cv::Mat& data = *data_ptr;
#elif defined(VISPER_ARCH_TI) || defined(VISPER_ARCH_TI_X86)
    cv::Mat img_rgb = preproc_Convert(img, "RGB");
    cv::Mat data = preproc_ToData(img_rgb, 0, std::vector<int>{0, 3, 1, 2}, CV_32F);
#elif defined(VISPER_ARCH_X86)
    cv::Mat img_rgb = preproc_Convert(img, "RGB");
    cv::Mat data = preproc_ToData(img_rgb, 0, std::vector<int>{0, 3, 1, 2}, CV_32F);
#endif

    LOG_TIMER(t, "A PrepareInput");

    bool has_raeb = false;
    bool has_op = false;
    std::string tasks_snapshot;

    {
        std::lock_guard<std::mutex> lk(g_FUNC_LOCK);
        has_raeb = std::find(g_TASKS.begin(), g_TASKS.end(), "RAEB") != g_TASKS.end();
        has_op = std::find(g_TASKS.begin(), g_TASKS.end(), "OP") != g_TASKS.end();
        tasks_snapshot = FormatTaskList(g_TASKS);
    }
    if (!has_raeb && !has_op) {
        LOG_WARNING("VisPer_RunInfer(Mat) no initialized task matched; returning without inference.");
        throw std::runtime_error("VisPer_RunInfer(Mat): no initialized task matched; tasks=" + tasks_snapshot);
    }

    // RAEB：StageA（infer）+ 入队给 StageB
    if (has_raeb) {
        // 拿一份 raeb_extra 快照
        std::unordered_map<std::string, std::any> raeb_extra_snapshot;
        {
            std::lock_guard<std::mutex> lk(g_FUNC_LOCK);
            raeb_extra_snapshot = g_CTX.raeb_extra;
        }

        std::vector<cv::Mat> preds;
        std::shared_ptr<void> infer_output_owner;
        try {
#if defined(VISPER_ARCH_TI)
            auto infer_result = infer_InferRAEBWithOwner(data, /*squeeze_batch*/ true);
            preds = std::move(infer_result.outputs);
            infer_output_owner = std::move(infer_result.owner);
#else
            preds = infer_InferRAEB(data, /*squeeze_batch*/ true);
#endif
        } catch (const std::exception& e) {
            LOG_WARNING(std::string("VisPer_RunInfer(Mat) infer_InferRAEB exception: ") + e.what());
            throw;
        } catch (...) {
            LOG_WARNING("VisPer_RunInfer(Mat) infer_InferRAEB exception: unknown");
            throw;
        }
        if (preds.size() < 3) {
            LOG_WARNING("VisPer_RunInfer(Mat) infer_InferRAEB returned fewer than 3 tensors.");
            throw std::runtime_error("infer_InferRAEB returned fewer than 3 tensors.");
        }
        LOG_TIMER(t, "A Infer");

        if (TiStageAOnlyEnabled()) {
            LOG_TIMER_DONE(t, "RAEB StageA-only done");
            return;
        }

        RaebInferOutputs raeb_outputs = NormalizeRaebInferOutputs(preds, "infer_InferRAEB");
        
        RaebStageAOut job;
        job.timestamp = timestamp;
        job.img_bgr = img_bgr;
        job.det_cat = raeb_outputs.det_cat;
        job.proto = raeb_outputs.proto;
        job.angle = raeb_outputs.angle;
        job.infer_output_owner = std::move(infer_output_owner);
        job.raeb_extra = std::move(raeb_extra_snapshot);

        g_RAEB_STAGEB_QUEUE.push(std::move(job));
        LOG_TIMER_DONE(t, "RAEB StageA done");
    } else {
        LOG_WARNING(
            "VisPer_RunInfer(Mat) RAEB branch skipped timestamp=%lld, tasks=%s; throwing instead of silent return",
            static_cast<long long>(timestamp),
            tasks_snapshot.c_str()
        );
        throw std::runtime_error("VisPer_RunInfer(Mat): RAEB task is not initialized; tasks=" + tasks_snapshot);
    }

    // OP 任务：目前仅记录时间戳
    if (has_op) {
        LOG_WARNING("VisPer_RunInfer(Mat) OP task is enabled but OP inference is not implemented.");
        std::lock_guard<std::mutex> lk(g_FUNC_LOCK);
    }
    LOG_INFO("VisPer_RunInfer(Mat) exit timestamp=%lld", static_cast<long long>(timestamp));
}


void VisPer_PrintResult(const std::string& task) {
    auto MatToText = [](const cv::Mat& m) -> std::string {
        if (m.empty()) return "empty";
        if (m.dims != 2) return "non-2d";

        std::ostringstream oss;
        oss << m.rows << "x" << m.cols << " C" << m.channels() << "\n";

        const int C = m.channels();
        const int depth = m.type() & CV_MAT_DEPTH_MASK;

        auto append = [&](int r, int c, int ch) {
            switch (depth) {
            case CV_32S: oss << m.ptr<int>(r)[c * C + ch];
                break;
            case CV_32F: oss << m.ptr<float>(r)[c * C + ch];
                break;
            case CV_64F: oss << m.ptr<double>(r)[c * C + ch];
                break;
            case CV_8U: oss << static_cast<int>(m.ptr<uint8_t>(r)[c * C + ch]);
                break;
            case CV_16S: oss << m.ptr<int16_t>(r)[c * C + ch];
                break;
            case CV_16U: oss << m.ptr<uint16_t>(r)[c * C + ch];
                break;
            default: oss << "?";
                break;
            }
        };

        oss << "[";
        for (int r = 0; r < m.rows; ++r) {
            oss << "[";
            for (int c = 0; c < m.cols; ++c) {
                if (c) oss << " ";
                if (C == 1) {
                    append(r, c, 0);
                } else {
                    oss << "(";
                    for (int ch = 0; ch < C; ++ch) {
                        if (ch) oss << ", ";
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

    LOG_INFO("-------------------------------------");

    std::int64_t ts = 0;
    cv::Mat objs;
    cv::Mat track_info;
    cv::Mat tracked_cuboids_raw;
    cv::Mat tracked_cuboids;
    cv::Mat tracked_cuboids_vel;
    std::unordered_map<std::string, std::any> orr;

    {
        std::lock_guard<std::mutex> lk(g_FUNC_LOCK);
        ts = g_CTX.timestamp;
        if (task == "RAEB") {
            auto& rr = g_CTX.raeb_result;
            objs = std::any_cast<const cv::Mat&>(rr.at("objs"));
            track_info = std::any_cast<const cv::Mat&>(rr.at("track_info"));
            tracked_cuboids_raw = std::any_cast<const cv::Mat&>(rr.at("tracked_cuboids_raw"));
            tracked_cuboids = std::any_cast<const cv::Mat&>(rr.at("tracked_cuboids"));
            tracked_cuboids_vel = std::any_cast<const cv::Mat&>(rr.at("tracked_cuboids_vel"));
        } else if (task == "OP") {
            orr = g_CTX.op_result;
        } else {
            LOG_WARNING(std::string("VisPer_PrintResult: unknown task: ") + task);
            LOG_INFO("-------------------------------------");
            return;
        }
    }

    const std::int64_t now_ts = static_cast<std::int64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count()
    );
    LOG_INFO(
        "VisPer_PrintResult now_ts=%lld, ctx_ts=%lld, delta=%lld ms",
        static_cast<long long>(now_ts),
        static_cast<long long>(ts),
        static_cast<long long>(now_ts - ts)
    );

    if (task == "RAEB") {
        try {
            LOG_INFO("REAB timestamp: %lld", static_cast<long long>(ts));

            {
                const std::string s = MatToText(objs);
                LOG_INFO("REAB objs: %s", s.c_str());
            }
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
        } catch (const std::exception& e) {
            LOG_WARNING(std::string("VisPer_PrintResult(RAEB) failed: ") + e.what());
        }
    } else if (task == "OP") {
        try {
            LOG_INFO("OP timestamp: %lld", static_cast<long long>(ts));
            if (auto it = orr.find("preds"); it != orr.end() && it->second.has_value()) {
                if (it->second.type() == typeid(cv::Mat)) {
                    const cv::Mat& preds = std::any_cast<const cv::Mat&>(it->second);
                    LOG_INFO("OP preds: %s", MatToText(preds).c_str());
                } else {
                    LOG_INFO("OP preds: (non-cv::Mat)");
                }
            } else {
                LOG_INFO("OP preds: empty");
            }
        } catch (const std::exception& e) {
            LOG_WARNING(std::string("VisPer_PrintResult(OP) failed: ") + e.what());
        }
    }
    LOG_INFO("-------------------------------------");
}


// 裸指针输入兼容接口。
void VisPer_RunInfer(const uint8_t* data, std::size_t length, std::int64_t timestamp) {
#if defined(VISPER_ARCH_SGS)
    if (data == nullptr) {
        throw std::runtime_error("VisPer_RunInfer(data): SGS hardware address is null.");
    }

    const std::uintptr_t phy_addr = reinterpret_cast<std::uintptr_t>(data);
    const bool need_stageb_vis = vis_HasRaebStageBTrigger();

    LOG_DEBUG("=====================================");
    const std::int64_t now_ts = static_cast<std::int64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count()
    );
    LOG_INFO(
        "VisPer_RunInfer hardware now_ts=%lld, arg_ts=%lld, delta=%lld ms, phy=0x%llx, length=%zu",
        static_cast<long long>(now_ts),
        static_cast<long long>(timestamp),
        static_cast<long long>(now_ts - timestamp),
        static_cast<unsigned long long>(phy_addr),
        length
    );
    LOG_TIMER_CREATE(t,"RAEB StageA start");

    LOG_TIMER(t, "A PrepareInput");

    bool has_raeb = false;
    bool has_op = false;
    std::string tasks_snapshot;
    bool stageb_joinable = false;

    {
        std::lock_guard<std::mutex> lk(g_FUNC_LOCK);
        has_raeb = std::find(g_TASKS.begin(), g_TASKS.end(), "RAEB") != g_TASKS.end();
        has_op = std::find(g_TASKS.begin(), g_TASKS.end(), "OP") != g_TASKS.end();
        tasks_snapshot = FormatTaskList(g_TASKS);
        stageb_joinable = g_RAEB_STAGEB_THREAD.joinable();
    }
    LOG_INFO(
        "VisPer_RunInfer(Phys) dispatch timestamp=%lld, has_raeb=%d, has_op=%d, tasks=%s, stageb_joinable=%d, phy=0x%llx, length=%zu",
        static_cast<long long>(timestamp),
        has_raeb ? 1 : 0,
        has_op ? 1 : 0,
        tasks_snapshot.c_str(),
        stageb_joinable ? 1 : 0,
        static_cast<unsigned long long>(phy_addr),
        length
    );
    if (!has_raeb && !has_op) {
        LOG_WARNING("VisPer_RunInfer(Phys) no initialized task matched; returning without inference.");
        throw std::runtime_error("VisPer_RunInfer(Phys): no initialized task matched; tasks=" + tasks_snapshot);
    }

    if (has_raeb) {
        std::unordered_map<std::string, std::any> raeb_extra_snapshot;
        {
            std::lock_guard<std::mutex> lk(g_FUNC_LOCK);
            raeb_extra_snapshot = g_CTX.raeb_extra;
        }
        LOG_INFO(
            "VisPer_RunInfer(Phys) RAEB branch enter timestamp=%lld, extra_keys=%s, phy=0x%llx, length=%zu",
            static_cast<long long>(timestamp),
            FormatAnyMapKeys(raeb_extra_snapshot).c_str(),
            static_cast<unsigned long long>(phy_addr),
            length
        );

        std::vector<cv::Mat> preds;
        try {
            LOG_INFO(
                "VisPer_RunInfer(Phys) infer_InferRAEBPhys begin timestamp=%lld, phy=0x%llx, length=%zu",
                static_cast<long long>(timestamp),
                static_cast<unsigned long long>(phy_addr),
                length
            );
            preds = infer_InferRAEBPhys(phy_addr, length, /*squeeze_batch*/ true);
            LOG_INFO(
                "VisPer_RunInfer(Phys) infer_InferRAEBPhys done timestamp=%lld, preds=%s",
                static_cast<long long>(timestamp),
                FormatMatVectorShapes(preds).c_str()
            );
        } catch (const std::exception& e) {
            LOG_WARNING(std::string("VisPer_RunInfer(Phys) infer_InferRAEBPhys exception: ") + e.what());
            throw;
        } catch (...) {
            LOG_WARNING("VisPer_RunInfer(Phys) infer_InferRAEBPhys exception: unknown");
            throw;
        }
        if (preds.size() < 3) {
            LOG_WARNING("VisPer_RunInfer(Phys) infer_InferRAEBPhys returned fewer than 3 tensors.");
            throw std::runtime_error("infer_InferRAEBPhys returned fewer than 3 tensors.");
        }
        RaebInferOutputs raeb_outputs = NormalizeRaebInferOutputs(preds, "infer_InferRAEBPhys");
        LOG_TIMER(t, "A Infer");

        cv::Mat img_bgr;
        if (need_stageb_vis) {
            try {
                img_bgr = BuildBgrFromSgsPhysicalForVis(phy_addr, length);
            } catch (const std::exception& e) {
                LOG_WARNING(std::string("VisPer_RunInfer(Phys) build StageB visualization image failed: ") + e.what());
            } catch (...) {
                LOG_WARNING("VisPer_RunInfer(Phys) build StageB visualization image failed: unknown");
            }
        }

        RaebStageAOut job;
        job.timestamp = timestamp;
        job.img_bgr = std::move(img_bgr);
        job.det_cat = raeb_outputs.det_cat;
        job.proto = raeb_outputs.proto;
        job.angle = raeb_outputs.angle;
        job.raeb_extra = std::move(raeb_extra_snapshot);

        LOG_INFO("VisPer_RunInfer(Phys) RAEB queue push begin timestamp=%lld", static_cast<long long>(timestamp));
        g_RAEB_STAGEB_QUEUE.push(std::move(job));
        LOG_INFO("VisPer_RunInfer(Phys) RAEB queue push done timestamp=%lld", static_cast<long long>(timestamp));
        LOG_TIMER_DONE(t, "RAEB StageA done");
    } else {
        LOG_WARNING(
            "VisPer_RunInfer(Phys) RAEB branch skipped timestamp=%lld, tasks=%s; throwing instead of silent return",
            static_cast<long long>(timestamp),
            tasks_snapshot.c_str()
        );
        throw std::runtime_error("VisPer_RunInfer(Phys): RAEB task is not initialized; tasks=" + tasks_snapshot);
    }

    if (has_op) {
        LOG_WARNING("VisPer_RunInfer(Phys) OP task is enabled but OP inference is not implemented.");
        std::lock_guard<std::mutex> lk(g_FUNC_LOCK);
    }
    LOG_INFO("VisPer_RunInfer(Phys) exit timestamp=%lld", static_cast<long long>(timestamp));
#else
    cv::Mat raw_img = preproc_ToMat(data, length);
    VisPer_RunInfer(raw_img, timestamp);
#endif
}

bool VisPer_GetTiPerfHandles(VisPerTiPerfHandles* out_handles) {
    if (out_handles != nullptr) {
        *out_handles = {};
    }
    if (out_handles == nullptr) {
        return false;
    }

    InferTiPerfHandles infer_handles;
    if (!infer_GetRAEBTiPerfHandles(&infer_handles)) {
        return false;
    }

    out_handles->graph = infer_handles.graph;
    out_handles->total_perf = infer_handles.total_perf;
    out_handles->graph_perf = infer_handles.graph_perf;
    return out_handles->graph != nullptr &&
        out_handles->total_perf != nullptr &&
        out_handles->graph_perf != nullptr;
}

void VisPer_RunInfer(const MI_SYS_FrameData_s& frame, std::int64_t timestamp) {
#if defined(VISPER_ARCH_SGS)
    LOG_TIMER_CREATE(t, "RAEB StageA start");
    const bool need_stageb_vis = vis_HasRaebStageBTrigger();

    LOG_TIMER(t, "A PrepareInput");

    bool has_raeb = false;
    bool has_op = false;
    std::string tasks_snapshot;

    {
        std::lock_guard<std::mutex> lk(g_FUNC_LOCK);
        has_raeb = std::find(g_TASKS.begin(), g_TASKS.end(), "RAEB") != g_TASKS.end();
        has_op = std::find(g_TASKS.begin(), g_TASKS.end(), "OP") != g_TASKS.end();
        tasks_snapshot = FormatTaskList(g_TASKS);
    }
    if (!has_raeb && !has_op) {
        LOG_WARNING("VisPer_RunInfer(SgsFrame) no initialized task matched; returning without inference.");
        throw std::runtime_error("VisPer_RunInfer(SgsFrame): no initialized task matched; tasks=" + tasks_snapshot);
    }

    if (has_raeb) {
        std::unordered_map<std::string, std::any> raeb_extra_snapshot;
        {
            std::lock_guard<std::mutex> lk(g_FUNC_LOCK);
            raeb_extra_snapshot = g_CTX.raeb_extra;
        }
        InferRAEBOutputBundle infer_result;
        try {
            infer_result = infer_InferRAEBFrameWithOwner(frame, /*squeeze_batch*/ true);
        } catch (const std::exception& e) {
            LOG_WARNING(std::string("VisPer_RunInfer(SgsFrame) infer_InferRAEBFrameWithOwner exception: ") + e.what());
            throw;
        } catch (...) {
            LOG_WARNING("VisPer_RunInfer(SgsFrame) infer_InferRAEBFrameWithOwner exception: unknown");
            throw;
        }
        const std::vector<cv::Mat>& preds = infer_result.outputs;
        if (preds.size() < 3) {
            LOG_WARNING("VisPer_RunInfer(SgsFrame) infer_InferRAEBFrameWithOwner returned fewer than 3 tensors.");
            throw std::runtime_error("infer_InferRAEBFrameWithOwner returned fewer than 3 tensors.");
        }
        RaebInferOutputs raeb_outputs = NormalizeRaebInferOutputs(preds, "infer_InferRAEBFrameWithOwner");
        LOG_TIMER(t, "A Infer");

        cv::Mat img_bgr;
        if (need_stageb_vis) {
            try {
                img_bgr = BuildBgrFromSgsFrameForVis(frame);
            } catch (const std::exception& e) {
                LOG_WARNING(std::string("VisPer_RunInfer(SgsFrame) build StageB visualization image failed: ") + e.what());
            } catch (...) {
                LOG_WARNING("VisPer_RunInfer(SgsFrame) build StageB visualization image failed: unknown");
            }
        }

        RaebStageAOut job;
        job.timestamp = timestamp;
        job.img_bgr = std::move(img_bgr);
        job.det_cat = raeb_outputs.det_cat;
        job.proto = raeb_outputs.proto;
        job.angle = raeb_outputs.angle;
        job.infer_output_owner = std::move(infer_result.owner);
        job.raeb_extra = std::move(raeb_extra_snapshot);

        g_RAEB_STAGEB_QUEUE.push(std::move(job));
        LOG_TIMER_DONE(t, "RAEB StageA done");
    } else {
        LOG_WARNING(
            "VisPer_RunInfer(SgsFrame) RAEB branch skipped timestamp=%lld, tasks=%s; throwing instead of silent return",
            static_cast<long long>(timestamp),
            tasks_snapshot.c_str()
        );
        throw std::runtime_error("VisPer_RunInfer(SgsFrame): RAEB task is not initialized; tasks=" + tasks_snapshot);
    }

    if (has_op) {
        LOG_WARNING("VisPer_RunInfer(SgsFrame) OP task is enabled but OP inference is not implemented.");
        std::lock_guard<std::mutex> lk(g_FUNC_LOCK);
    }
#else
    (void)frame;
    (void)timestamp;
    throw std::runtime_error("VisPer_RunInfer(MI_SYS_FrameData_s): only supported on SGS builds.");
#endif
}


// 只负责“拿结果”，不做推理/后处理
std::unordered_map<std::string, std::any> VisPer_GetResult(const std::string& task) {
    if (task == "RAEB") {
        std::int64_t ts = 0;
        cv::Mat objs;
        cv::Mat track_info;
        cv::Mat tracked_cuboids_raw;
        cv::Mat tracked_cuboids;
        cv::Mat tracked_cuboids_vel;
        {
            std::lock_guard<std::mutex> lk(g_FUNC_LOCK);
            if (std::find(g_TASKS.begin(), g_TASKS.end(), "RAEB") == g_TASKS.end()) {
                return {};
            }
            ts = g_CTX.timestamp;
            auto& rr = g_CTX.raeb_result;
            objs = std::any_cast<const cv::Mat&>(rr.at("objs"));
            track_info = std::any_cast<const cv::Mat&>(rr.at("track_info"));
            tracked_cuboids_raw = std::any_cast<const cv::Mat&>(rr.at("tracked_cuboids_raw"));
            tracked_cuboids = std::any_cast<const cv::Mat&>(rr.at("tracked_cuboids"));
            tracked_cuboids_vel = std::any_cast<const cv::Mat&>(rr.at("tracked_cuboids_vel"));
        }
        return MakeRaebResultSnapshot(
            ts,
            objs,
            track_info,
            tracked_cuboids_raw,
            tracked_cuboids,
            tracked_cuboids_vel
        );
    }

    std::lock_guard<std::mutex> lk(g_FUNC_LOCK);
    if (task == "OP" &&
        std::find(g_TASKS.begin(), g_TASKS.end(), "OP") != g_TASKS.end()) {
        return g_CTX.op_result;
    }

    return {};
}



// 清理资源：关闭 RAEB StageB 线程
void VisPer_CleanUp() {
    std::thread th;
    {
        std::lock_guard<std::mutex> lk(g_FUNC_LOCK);
        if (g_RAEB_STAGEB_THREAD.joinable()) {
            th = std::move(g_RAEB_STAGEB_THREAD);
        }
    }

    g_RAEB_STAGEB_QUEUE.close();

    if (th.joinable()) {
        th.join();
    }
    g_RAEB_VEL_SMOOTH_STATES.clear();

    // 必须在线程退出后再重置，否则 StageB 末尾保存会再次 clear 目录。
    visper::internal::ResetPreparedSaveDirs();
}


// 裸指针输入兼容接口。
std::unordered_map<std::string, std::any> PostProcessRAEB(const uint8_t* data, std::size_t length, std::int64_t timestamp,const TensorInfo* infos, std::size_t count) 
{

    cv::Mat raw_img = preproc_ToMat(data, length);
    // 1) 预处理
    const bool need_stageb_vis = vis_HasRaebStageBTrigger();
    cv::Mat img_bgr;
    if (need_stageb_vis) {
        img_bgr = preproc_Convert(raw_img, "BGR");
    }

    std::vector<cv::Mat> result;
    
    if (!infos || count == 0) {
        printf("Warning: Invalid tensor infos (NULL or count=0)\n");
    }
    
    result.reserve(count);
    printf("Converting %zu tensors to cv::Mat...\n", count);
    
    for (size_t i = 0; i < count; ++i) {
        const TensorInfo* info = &infos[i];
        
        if (!info || !info->output_data || info->output_size == 0) {
            printf("Warning: Tensor %zu has invalid data, skipping\n", i);
            continue;
        }
        
        // 根据形状创建 cv::Mat
        cv::Mat mat;
        
        if (info->shape_length == 0) {
            // 如果形状为空，创建一个1D数组
            mat = cv::Mat(1, static_cast<int>(info->output_size), CV_32F, info->output_data);
        } else if (info->shape_length == 1) {
            // 1D: [N]
            mat = cv::Mat(1, static_cast<int>(info->shape_data[0]), CV_32F, info->output_data);
        } else if (info->shape_length == 2) {
            // 2D: [H, W]
            mat = cv::Mat(static_cast<int>(info->shape_data[0]), 
                          static_cast<int>(info->shape_data[1]), 
                          CV_32F, info->output_data);
        } else if (info->shape_length == 3) {
            // 3D: [H, W, C] 或 [C, H, W]
            // 假设是 [H, W, C] 格式（OpenCV常用）
            int height = static_cast<int>(info->shape_data[0]);
            int width = static_cast<int>(info->shape_data[1]);
            int channels = static_cast<int>(info->shape_data[2]);
            
            // 检查数据大小是否匹配
            if (height * width * channels != static_cast<int>(info->output_size)) {
                printf("Warning: Shape mismatch for tensor %zu: [%d,%d,%d] vs size %zu\n",
                       i, height, width, channels, info->output_size);
            }
            
            // 创建多通道Mat
            mat = cv::Mat(height, width, CV_32FC(channels), info->output_data);
        } else if (info->shape_length == 4) {
            // 4D: [N, H, W, C] 或 [N, C, H, W]
            // 通常取第一个batch
            int batch = static_cast<int>(info->shape_data[0]);
            int height = static_cast<int>(info->shape_data[1]);
            int width = static_cast<int>(info->shape_data[2]);
            int channels = static_cast<int>(info->shape_data[3]);
            
            // 只取第一个batch
            if (batch > 1) {
                printf("Info: Tensor %zu has batch size %d, taking first batch\n", i, batch);
            }
            
            mat = cv::Mat(height, width, CV_32FC(channels), info->output_data);
        } else {
            // 更高维度，当作1D处理
            printf("Warning: Tensor %zu has %zu dimensions, treating as 1D\n", 
                   i, info->shape_length);
            mat = cv::Mat(1, static_cast<int>(info->output_size), CV_32F, info->output_data);
        }
        
        // 克隆数据（确保独立副本，避免原数据被释放）
        cv::Mat cloned_mat = mat.clone();
        result.push_back(cloned_mat);
        
        printf("  Tensor %zu converted to cv::Mat: [%d x %d], type=CV_32F, channels=%d\n",
               i, cloned_mat.rows, cloned_mat.cols, cloned_mat.channels());
    }
    
    printf("Successfully converted %zu tensors to cv::Mat\n", result.size());


    std::vector<cv::Mat> preds = result;
    RaebInferOutputs raeb_outputs = NormalizeRaebInferOutputs(preds, "infer_InferRAEB");

    RaebStageAOut job;
    job.timestamp = timestamp;
    job.img_bgr = img_bgr;
    job.det_cat = raeb_outputs.det_cat;
    job.proto = raeb_outputs.proto;
    job.angle = raeb_outputs.angle;    

    LOG_TIMER_CREATE(t_stageb,"RAEB StageB start");

    cv::Mat objs;
    cv::Mat masks;
    if (!img_bgr.empty()) {
        RaebPostprocResult postproc_result =
            postproc_ProcRAEBWithMasks(job.det_cat, job.proto, job.angle);
        objs = std::move(postproc_result.objs);
        masks = std::move(postproc_result.masks);
    } else {
        objs = postproc_ProcRAEB(job.det_cat, job.proto, job.angle);
    }
    LOG_TIMER(t_stageb, "B Postproc");

    cv::Mat cuboids_raw_all = cuboids_ProcRAEB(objs, 0.0, masks);

    const double ego_yaw = 0; //visper::internal::GetEgoYawFromExtra(job.raeb_extra);

    cv::Mat track_info;
    cv::Mat tracked_cuboids_raw;
    cv::Mat tracked_cuboids;
    cv::Mat tracked_cuboids_vel;
    std::tie(track_info, tracked_cuboids_raw, tracked_cuboids, tracked_cuboids_vel) =
        cuboid_tracker_TrackAndEstimate(
            job.timestamp,
            cuboids_raw_all,
            ego_yaw,
            "ego"
        );
    SmoothTrackedCuboidsVelocity(job.timestamp, track_info, tracked_cuboids_vel);
    LOG_TIMER(t_stageb, "B Tracker");

    // StageB 可视化模块：仅当 cwd 下存在 visl 文件时自动落盘。
    if (!img_bgr.empty()) {
        try {
            vis_RunRaebStageBModule(
                timestamp,
                img_bgr,
                objs,
                masks,
                track_info,
                tracked_cuboids,
                tracked_cuboids_vel
            );
        } catch (const std::exception& e) {
            LOG_WARNING(std::string("RAEB StageB vis module exception: ") + e.what());
        } catch (...) {
            LOG_WARNING("RAEB StageB vis module exception: unknown");
        }
    }
    LOG_TIMER_DONE(t_stageb, "RAEB StageB done");
    return MakeRaebResultSnapshot(
        job.timestamp,
        objs,
        track_info,
        tracked_cuboids_raw,
        tracked_cuboids,
        tracked_cuboids_vel
    );
}
