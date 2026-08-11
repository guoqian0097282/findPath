#pragma once

#include <cstddef>
#include <memory>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

/**
 * @brief RAEB 模型：全局实例的 C++ 版 API（对应 Python 的 g_model_raeb +
 * 两个函数）
 *
 * - infer_InitRAEB(model_path)
 * - infer_InferRAEB(data, squeeze_batch=false)
 *
 * 说明：
 *  - ti_target 参数仅 TI 后端使用，用于选择 TIDL OpenVX 节点目标核；x86/SGS 后端忽略。
 *  - 输入输出均使用 OpenCV 的 cv::Mat（多维），type=CV_32F。
 */

// 初始化对象检测模型；失败抛异常
void infer_InitRAEB(const std::string& model_path);
void infer_InitRAEB(const std::string& model_path, const std::string& ti_target);

// 释放全局模型实例；可重复调用
void infer_DeinitRAEB();

// 推理；未初始化将抛出 std::runtime_error
std::vector<cv::Mat> infer_InferRAEB(const cv::Mat& data, bool squeeze_batch = false);

void infer_SmokeTestRAEB(int nTimes);

struct InferRAEBOutputBundle {
    std::vector<cv::Mat> outputs;
    std::shared_ptr<void> owner;
};

InferRAEBOutputBundle infer_InferRAEBWithOwner(const cv::Mat& data,
                                               bool squeeze_batch = false);

struct InferTiPerfHandles {
    void* graph = nullptr;
    void* total_perf = nullptr;
    void* graph_perf = nullptr;
};

bool infer_GetRAEBTiPerfHandles(InferTiPerfHandles* out_handles);

struct InferTiPipelineStressResult {
    std::size_t submitted = 0;
    std::size_t completed = 0;
    double preprocess_ms = 0.0;
    double write_input_ms = 0.0;
    double schedule_ms = 0.0;
    double wait_graph_ms = 0.0;
    double read_output_ms = 0.0;
    double elapsed_ms = 0.0;
    double fps = 0.0;
};

InferTiPipelineStressResult infer_RunRAEBTiPipelineStress(
    const std::vector<cv::Mat>& inputs,
    std::size_t total_tasks,
    std::size_t pipeline_depth,
    bool read_outputs = false);

#if defined(VISPER_ARCH_SGS)
struct MI_SYS_FrameData_s;

InferRAEBOutputBundle infer_InferRAEBFrameWithOwner(const MI_SYS_FrameData_s& frame,
                                                    bool squeeze_batch);
#endif
