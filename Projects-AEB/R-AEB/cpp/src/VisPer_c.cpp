#include "VisPer_c.h"

#include "VisPer.h"

#include <any>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include <opencv2/core.hpp>

namespace {

std::mutex g_result_lock;
cv::Mat g_objs;
cv::Mat g_track_info;
cv::Mat g_tracked_cuboids;
cv::Mat g_tracked_cuboids_vel;

const cv::Mat& requireMat(const std::unordered_map<std::string, std::any>& result,
                          const char* key)
{
    const auto it = result.find(key);
    if (it == result.end()) {
        throw std::runtime_error(std::string("VisPer_GetResult_C missing key: ") + key);
    }
    return std::any_cast<const cv::Mat&>(it->second);
}

int64_t requireTimestamp(const std::unordered_map<std::string, std::any>& result)
{
    const auto it = result.find("timestamp");
    if (it == result.end()) {
        throw std::runtime_error("VisPer_GetResult_C missing key: timestamp");
    }
    return std::any_cast<std::int64_t>(it->second);
}

cv::Mat clone2DMat(const std::unordered_map<std::string, std::any>& result,
                   const char* key,
                   int type,
                   int cols)
{
    const cv::Mat& mat = requireMat(result, key);
    if (mat.dims != 2 || mat.type() != type || mat.cols != cols) {
        throw std::runtime_error(std::string("VisPer_GetResult_C unexpected shape/type for key: ") + key);
    }
    return mat.clone();
}

} // namespace

extern "C" {

void VisPer_InitTask_C(const char* task,
                       const char* config_path,
                       const char* model_path,
                       const char* ti_target)
{
    if (task == nullptr || config_path == nullptr || model_path == nullptr) {
        return;
    }

    if (ti_target == nullptr || ti_target[0] == '\0') {
        VisPer_InitTask(std::string(task), std::string(config_path), std::string(model_path));
        return;
    }

    VisPer_InitTask(std::string(task),
                    std::string(config_path),
                    std::string(model_path),
                    std::string(ti_target));
}

void VisPer_RunInfer_C(const uint8_t* data,
                       size_t length,
                       int64_t timestamp)
{
    if (data == nullptr || length == 0) {
        return;
    }

    VisPer_RunInfer(data, length, static_cast<std::int64_t>(timestamp));
}

void PostProcessRAEB_C(const uint8_t* data,size_t length,int64_t timestamp,const TensorInfo* infos, size_t count,VisPerRaebResult_C* out_result)
{
    auto result = PostProcessRAEB(data, length, static_cast<std::int64_t>(timestamp),infos,count);
    if (result.empty()) {
        return;
    }

    std::lock_guard<std::mutex> lk(g_result_lock);
    g_objs = clone2DMat(result, "objs", CV_32F, 7);
    g_track_info = clone2DMat(result, "track_info", CV_32S, 4);
    g_tracked_cuboids = clone2DMat(result, "tracked_cuboids", CV_32F, 9);
    g_tracked_cuboids_vel = clone2DMat(result, "tracked_cuboids_vel", CV_32F, 4);

    out_result->timestamp = requireTimestamp(result);
    out_result->objs_rows = g_objs.rows;
    out_result->objs = g_objs.empty() ? nullptr : reinterpret_cast<const float (*)[7]>(g_objs.data);
    out_result->track_info_rows = g_track_info.rows;
    out_result->track_info = g_track_info.empty() ? nullptr : reinterpret_cast<const int32_t (*)[4]>(g_track_info.data);
    out_result->tracked_cuboids_rows = g_tracked_cuboids.rows;
    out_result->tracked_cuboids = g_tracked_cuboids.empty() ? nullptr : reinterpret_cast<const float (*)[9]>(g_tracked_cuboids.data);
    out_result->tracked_cuboids_vel_rows = g_tracked_cuboids_vel.rows;
    out_result->tracked_cuboids_vel = g_tracked_cuboids_vel.empty() ? nullptr : reinterpret_cast<const float (*)[4]>(g_tracked_cuboids_vel.data);
}


void VisPer_GetResult_C(const char* task,
                        VisPerRaebResult_C* out_result)
{
    if (task == nullptr || out_result == nullptr) {
        return;
    }

    std::memset(out_result, 0, sizeof(*out_result));
    if (std::strcmp(task, "RAEB") != 0) {
        return;
    }

    auto result = VisPer_GetResult("RAEB");
    if (result.empty()) {
        return;
    }

    std::lock_guard<std::mutex> lk(g_result_lock);
    g_objs = clone2DMat(result, "objs", CV_32F, 7);
    g_track_info = clone2DMat(result, "track_info", CV_32S, 4);
    g_tracked_cuboids = clone2DMat(result, "tracked_cuboids", CV_32F, 9);
    g_tracked_cuboids_vel = clone2DMat(result, "tracked_cuboids_vel", CV_32F, 4);

    out_result->timestamp = requireTimestamp(result);
    out_result->objs_rows = g_objs.rows;
    out_result->objs = g_objs.empty() ? nullptr : reinterpret_cast<const float (*)[7]>(g_objs.data);
    out_result->track_info_rows = g_track_info.rows;
    out_result->track_info = g_track_info.empty() ? nullptr : reinterpret_cast<const int32_t (*)[4]>(g_track_info.data);
    out_result->tracked_cuboids_rows = g_tracked_cuboids.rows;
    out_result->tracked_cuboids = g_tracked_cuboids.empty() ? nullptr : reinterpret_cast<const float (*)[9]>(g_tracked_cuboids.data);
    out_result->tracked_cuboids_vel_rows = g_tracked_cuboids_vel.rows;
    out_result->tracked_cuboids_vel = g_tracked_cuboids_vel.empty() ? nullptr : reinterpret_cast<const float (*)[4]>(g_tracked_cuboids_vel.data);
}

int VisPer_GetTiPerfHandles_C(VisPerTiPerfHandles_C* out_handles)
{
    if (out_handles == nullptr) {
        return 0;
    }

    std::memset(out_handles, 0, sizeof(*out_handles));
    VisPerTiPerfHandles handles;
    if (!VisPer_GetTiPerfHandles(&handles)) {
        return 0;
    }

    out_handles->graph = handles.graph;
    out_handles->total_perf = handles.total_perf;
    out_handles->graph_perf = handles.graph_perf;
    return 1;
}

}
