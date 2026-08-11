#include <cstddef>
#include <cstdint>

#include "infer_api.h"

#if defined(VISPER_ARCH_X86)

#include "infer_ort_impl.hpp" // 只在 x86 编译进来

#include <chrono>
#include <iostream>
#include <memory>
#include <stdexcept>

// -------------------- 全局模型实例 --------------------
static std::unique_ptr<ONNXModel> g_model_raeb; // 对应 Python 的 g_model_raeb: ONNXModel | None

// -------------------- API 实现 --------------------
void infer_InitRAEB(const std::string& model_path) {
    infer_InitRAEB(model_path, "");
}

void infer_InitRAEB(const std::string& model_path, const std::string&) {
    // 每次初始化前先释放旧实例，避免重复 init 时资源残留
    g_model_raeb.reset();

    // 直接构造；构造失败会抛异常
    g_model_raeb = std::make_unique<ONNXModel>(model_path, "cpu");
}

void infer_DeinitRAEB() {
    g_model_raeb.reset();
}
bool HasAllOutputs(const std::vector<std::string>& names,
                   const std::vector<std::string>& required) {
    for (const std::string& name : required) {
        if (std::find(names.begin(), names.end(), name) == names.end()) {
            return false;
        }
    }
    return true;
}

int FindOutputIndex(const std::vector<std::string>& names, const std::string& wanted) {
    const auto it = std::find(names.begin(), names.end(), wanted);
    if (it == names.end()) {
        return -1;
    }
    return static_cast<int>(std::distance(names.begin(), it));
}

cv::Mat Ensure2dFloatOutput(const cv::Mat& tensor, const std::string& name) {
    if (tensor.type() != CV_32F) {
        throw std::runtime_error(name + " must be CV_32F.");
    }
    cv::Mat continuous = tensor.isContinuous() ? tensor : tensor.clone();
    std::vector<int> shape;
    shape.reserve(static_cast<std::size_t>(continuous.dims));
    for (int i = 0; i < continuous.dims; ++i) {
        shape.push_back(continuous.size[i]);
    }
    while (shape.size() > 2U && shape.front() == 1) {
        shape.erase(shape.begin());
    }
    if (shape.size() == 3U && shape[2] == 1) {
        shape.pop_back();
    }
    if (shape.size() != 2U) {
        throw std::runtime_error(name + " must be a 2D tensor after squeezing singleton dims.");
    }
    return continuous.reshape(1, static_cast<int>(shape.size()), shape.data());
}

cv::Mat MergeSplitDetCat(const cv::Mat& det_box,
                         const cv::Mat& det_cls,
                         const cv::Mat& det_mask) {
    cv::Mat box = Ensure2dFloatOutput(det_box, "det_box");
    cv::Mat cls = Ensure2dFloatOutput(det_cls, "det_cls");
    cv::Mat mask = Ensure2dFloatOutput(det_mask, "det_mask");
    if (box.rows != 4) {
        throw std::runtime_error("det_box must have 4 rows.");
    }
    if (box.cols != cls.cols || box.cols != mask.cols) {
        throw std::runtime_error("Split RAEB outputs have inconsistent prediction counts.");
    }
    std::vector<cv::Mat> rows = {box, cls, mask};
    cv::Mat det_cat;
    cv::vconcat(rows, det_cat);
    return det_cat;
}

std::vector<cv::Mat> NormalizeTiRaebOutputs(const std::vector<std::string>& names,
                                            std::vector<cv::Mat> outputs) {
    if (HasAllOutputs(names, {"det_cat", "proto", "angle"})) {
        return outputs;
    }

    if (!HasAllOutputs(names, {"det_box", "det_cls", "det_mask", "proto", "angle"})) {
        return outputs;
    }

    const int box_idx = FindOutputIndex(names, "det_box");
    const int cls_idx = FindOutputIndex(names, "det_cls");
    const int mask_idx = FindOutputIndex(names, "det_mask");
    const int proto_idx = FindOutputIndex(names, "proto");
    const int angle_idx = FindOutputIndex(names, "angle");
    if (box_idx < 0 || cls_idx < 0 || mask_idx < 0 || proto_idx < 0 || angle_idx < 0) {
        throw std::runtime_error("Internal error while resolving split RAEB output indices.");
    }

    std::vector<cv::Mat> normalized;
    normalized.reserve(3);
    normalized.push_back(MergeSplitDetCat(outputs[static_cast<std::size_t>(box_idx)],
                                          outputs[static_cast<std::size_t>(cls_idx)],
                                          outputs[static_cast<std::size_t>(mask_idx)]));
    normalized.push_back(outputs[static_cast<std::size_t>(proto_idx)]);
    normalized.push_back(outputs[static_cast<std::size_t>(angle_idx)]);
    return normalized;
}
std::vector<cv::Mat> infer_InferRAEB(const cv::Mat& data, bool squeeze_batch) {
    if (!g_model_raeb) {
        throw std::runtime_error("RAEB 模型未初始化，请先调用 infer_InitRAEB");
    }

    // return g_model_raeb->infer(data, /*squeeze_batch=*/squeeze_batch);
    return NormalizeTiRaebOutputs({"det_box", "det_cls", "det_mask", "proto", "angle"},
                    g_model_raeb->infer(data, /*squeeze_batch=*/squeeze_batch));
}

InferRAEBOutputBundle infer_InferRAEBWithOwner(const cv::Mat& data, bool squeeze_batch) {
    InferRAEBOutputBundle result;
    result.outputs = infer_InferRAEB(data, squeeze_batch);
    return result;
}

bool infer_GetRAEBTiPerfHandles(InferTiPerfHandles* out_handles) {
    if (out_handles != nullptr) {
        *out_handles = {};
    }
    return false;
}

InferTiPipelineStressResult infer_RunRAEBTiPipelineStress(
    const std::vector<cv::Mat>&,
    std::size_t,
    std::size_t,
    bool) {
    throw std::runtime_error("infer_RunRAEBTiPipelineStress is only supported on TI target build.");
}

#elif defined(VISPER_ARCH_SGS)

#include "infer_sgs_impl.hpp" // 内含 IPUModel
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

// -------------------- 全局模型实例 --------------------
static std::unique_ptr<IPUModel> g_model_raeb; // 与 x86 保持同名，全局唯一实例

namespace {

std::string InferApiMatShape(const cv::Mat& m) {
    std::ostringstream oss;
    oss << "dims=" << m.dims << ", shape=[";
    for (int i = 0; i < m.dims; ++i) {
        if (i != 0) {
            oss << ",";
        }
        oss << m.size[i];
    }
    oss << "], type=" << m.type()
        << ", channels=" << m.channels()
        << ", elemSize=" << m.elemSize()
        << ", continuous=" << (m.isContinuous() ? "true" : "false")
        << ", empty=" << (m.empty() ? "true" : "false");
    return oss.str();
}

std::string InferApiMatVectorShapes(const std::vector<cv::Mat>& mats) {
    std::ostringstream oss;
    oss << "count=" << mats.size();
    for (std::size_t i = 0; i < mats.size(); ++i) {
        oss << "; [" << i << "]{" << InferApiMatShape(mats[i]) << "}";
    }
    return oss.str();
}

} // namespace

// -------------------- API 实现（签名与 x86 完全一致） --------------------
void infer_InitRAEB(const std::string& model_path) {
    infer_InitRAEB(model_path, "");
}

void infer_InitRAEB(const std::string& model_path, const std::string&) {
    LOG_INFO("infer_InitRAEB(SGS) enter model_path=\"%s\", old_model=%p", model_path.c_str(), g_model_raeb.get());
    // 每次初始化前先释放旧实例，避免重复 init 时资源残留
    g_model_raeb.reset();
    LOG_INFO("infer_InitRAEB(SGS) old model reset done");

    // 构造失败会抛异常；第二参数可省略，默认 "ipu"
    g_model_raeb = std::make_unique<IPUModel>(model_path);
    LOG_INFO("infer_InitRAEB(SGS) done new_model=%p", g_model_raeb.get());
}

void infer_DeinitRAEB() {
    LOG_INFO("infer_DeinitRAEB(SGS) enter model=%p", g_model_raeb.get());
    g_model_raeb.reset();
    LOG_INFO("infer_DeinitRAEB(SGS) done");
}

std::vector<cv::Mat> infer_InferRAEB(const cv::Mat& data, bool squeeze_batch) {
    if (!g_model_raeb) {
        LOG_WARNING("infer_InferRAEB(SGS) called before model initialization");
        throw std::runtime_error("RAEB 模型未初始化，请先调用 infer_InitRAEB");
    }
    LOG_INFO(
        "infer_InferRAEB(SGS) begin model=%p, squeeze_batch=%d, input={%s}",
        g_model_raeb.get(),
        squeeze_batch ? 1 : 0,
        InferApiMatShape(data).c_str()
    );
    try {
        auto outs = g_model_raeb->infer(data, /*squeeze_batch=*/squeeze_batch);
        LOG_INFO("infer_InferRAEB(SGS) done outputs=%s", InferApiMatVectorShapes(outs).c_str());
        return outs;
    } catch (const std::exception& e) {
        LOG_WARNING(std::string("infer_InferRAEB(SGS) exception: ") + e.what());
        throw;
    } catch (...) {
        LOG_WARNING("infer_InferRAEB(SGS) exception: unknown");
        throw;
    }
}

InferRAEBOutputBundle infer_InferRAEBWithOwner(const cv::Mat& data, bool squeeze_batch) {
    InferRAEBOutputBundle result;
    result.outputs = infer_InferRAEB(data, squeeze_batch);
    return result;
}

bool infer_GetRAEBTiPerfHandles(InferTiPerfHandles* out_handles) {
    if (out_handles != nullptr) {
        *out_handles = {};
    }
    return false;
}

InferTiPipelineStressResult infer_RunRAEBTiPipelineStress(
    const std::vector<cv::Mat>&,
    std::size_t,
    std::size_t,
    bool) {
    throw std::runtime_error("infer_RunRAEBTiPipelineStress is only supported on TI target build.");
}

std::vector<cv::Mat> infer_InferRAEBPhys(std::uintptr_t phy_addr,
                                         std::size_t length,
                                         bool squeeze_batch) {
    if (!g_model_raeb) {
        LOG_WARNING("infer_InferRAEBPhys(SGS) called before model initialization");
        throw std::runtime_error("RAEB 模型未初始化，请先调用 infer_InitRAEB");
    }
    LOG_INFO(
        "infer_InferRAEBPhys(SGS) begin model=%p, phy=0x%llx, length=%zu, squeeze_batch=%d",
        g_model_raeb.get(),
        static_cast<unsigned long long>(phy_addr),
        length,
        squeeze_batch ? 1 : 0
    );
    try {
        auto outs = g_model_raeb->inferPhysical(static_cast<MI_PHY>(phy_addr),
                                                length,
                                                /*squeeze_batch=*/squeeze_batch);
        LOG_INFO("infer_InferRAEBPhys(SGS) done outputs=%s", InferApiMatVectorShapes(outs).c_str());
        return outs;
    } catch (const std::exception& e) {
        LOG_WARNING(std::string("infer_InferRAEBPhys(SGS) exception: ") + e.what());
        throw;
    } catch (...) {
        LOG_WARNING("infer_InferRAEBPhys(SGS) exception: unknown");
        throw;
    }
}

std::vector<cv::Mat> infer_InferRAEBFrame(const MI_SYS_FrameData_s& frame,
                                          bool squeeze_batch) {
    if (!g_model_raeb) {
        LOG_WARNING("infer_InferRAEBFrame(SGS) called before model initialization");
        throw std::runtime_error("RAEB 模型未初始化，请先调用 infer_InitRAEB");
    }
    LOG_INFO(
        "infer_InferRAEBFrame(SGS) begin model=%p, size=%ux%u, pixfmt=%d, layout=%d, buf=%u, vir0=%p, vir1=%p, phy0=0x%llx, phy1=0x%llx, squeeze_batch=%d",
        g_model_raeb.get(),
        static_cast<unsigned>(frame.u16Width),
        static_cast<unsigned>(frame.u16Height),
        static_cast<int>(frame.ePixelFormat),
        static_cast<int>(frame.ePhylayoutType),
        static_cast<unsigned>(frame.u32BufSize),
        frame.pVirAddr[0],
        frame.pVirAddr[1],
        static_cast<unsigned long long>(frame.phyAddr[0]),
        static_cast<unsigned long long>(frame.phyAddr[1]),
        squeeze_batch ? 1 : 0
    );
    try {
        auto outs = g_model_raeb->inferFrameData(frame, /*squeeze_batch=*/squeeze_batch);
        LOG_INFO("infer_InferRAEBFrame(SGS) done outputs=%s", InferApiMatVectorShapes(outs).c_str());
        return outs;
    } catch (const std::exception& e) {
        LOG_WARNING(std::string("infer_InferRAEBFrame(SGS) exception: ") + e.what());
        throw;
    } catch (...) {
        LOG_WARNING("infer_InferRAEBFrame(SGS) exception: unknown");
        throw;
    }
}

InferRAEBOutputBundle infer_InferRAEBFrameWithOwner(const MI_SYS_FrameData_s& frame,
                                                    bool squeeze_batch) {
    if (!g_model_raeb) {
        LOG_WARNING("infer_InferRAEBFrameWithOwner(SGS) called before model initialization");
        throw std::runtime_error("RAEB 模型未初始化，请先调用 infer_InitRAEB");
    }
    try {
        auto frameResult = g_model_raeb->inferFrameDataWithOwner(frame, /*squeeze_batch=*/squeeze_batch);
        InferRAEBOutputBundle result;
        result.outputs = std::move(frameResult.outputs);
        result.owner = std::move(frameResult.owner);
        return result;
    } catch (const std::exception& e) {
        LOG_WARNING(std::string("infer_InferRAEBFrameWithOwner(SGS) exception: ") + e.what());
        throw;
    } catch (...) {
        LOG_WARNING("infer_InferRAEBFrameWithOwner(SGS) exception: unknown");
        throw;
    }
}

// 冒烟测试：调用类内的 smokeTest(nTimes)，做 n 次推理
void infer_SmokeTestRAEB(int nTimes) {
    if (!g_model_raeb) {
        LOG_WARNING("infer_SmokeTestRAEB(SGS) called before model initialization");
        throw std::runtime_error("RAEB 模型未初始化，请先调用 infer_InitRAEB");
    }
    LOG_INFO("infer_SmokeTestRAEB(SGS) begin nTimes=%d", nTimes);
    g_model_raeb->smokeTest(nTimes);
    LOG_INFO("infer_SmokeTestRAEB(SGS) done nTimes=%d", nTimes);
}
#elif defined(VISPER_ARCH_TI_X86)

#include "infer_ti_ort_impl.hpp"

#include <algorithm>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

static std::unique_ptr<TidlOnnxModel> g_model_raeb;

namespace {

std::string JoinOutputNamesTiX86(const std::vector<std::string>& names) {
    std::ostringstream oss;
    for (std::size_t i = 0; i < names.size(); ++i) {
        if (i != 0) {
            oss << ", ";
        }
        oss << names[i];
    }
    return oss.str();
}

void ValidateRaebOutputsTiX86(const TidlOnnxModel& model, const std::string& model_path) {
    const std::vector<std::string>& names = model.outputNames();
    const bool has_merged_outputs =
        std::find(names.begin(), names.end(), "det_cat") != names.end() &&
        std::find(names.begin(), names.end(), "proto") != names.end() &&
        std::find(names.begin(), names.end(), "angle") != names.end();
    const bool has_split_outputs =
        std::find(names.begin(), names.end(), "det_box") != names.end() &&
        std::find(names.begin(), names.end(), "det_cls") != names.end() &&
        std::find(names.begin(), names.end(), "det_mask") != names.end() &&
        std::find(names.begin(), names.end(), "proto") != names.end() &&
        std::find(names.begin(), names.end(), "angle") != names.end();

    if (!has_merged_outputs && !has_split_outputs) {
        throw std::runtime_error(
            "TI x86 RAEB model output mismatch for model_path=" + model_path +
            ". Expected merged outputs [det_cat, proto, angle] or split outputs "
            "[det_box, det_cls, det_mask, proto, angle], actual outputs=[" +
            JoinOutputNamesTiX86(names) + "].");
    }
}

} // namespace

void infer_InitRAEB(const std::string& model_path) {
    infer_InitRAEB(model_path, "");
}

void infer_InitRAEB(const std::string& model_path, const std::string& artifacts_or_ti_target) {
    g_model_raeb.reset();
    std::string artifacts_dir = artifacts_or_ti_target;
    if (artifacts_dir == "DSP_C7-1" || artifacts_dir == "DSP_C7-2") {
        artifacts_dir.clear();
    }
    g_model_raeb = std::make_unique<TidlOnnxModel>(model_path, artifacts_dir);
    ValidateRaebOutputsTiX86(*g_model_raeb, model_path);
}

void infer_DeinitRAEB() {
    g_model_raeb.reset();
}

std::vector<cv::Mat> infer_InferRAEB(const cv::Mat& data, bool squeeze_batch) {
    if (!g_model_raeb) {
        throw std::runtime_error("RAEB 模型未初始化，请先调用 infer_InitRAEB");
    }
    return g_model_raeb->infer(data, /*squeeze_batch=*/squeeze_batch);
}

InferRAEBOutputBundle infer_InferRAEBWithOwner(const cv::Mat& data, bool squeeze_batch) {
    InferRAEBOutputBundle result;
    result.outputs = infer_InferRAEB(data, squeeze_batch);
    return result;
}

bool infer_GetRAEBTiPerfHandles(InferTiPerfHandles* out_handles) {
    if (out_handles != nullptr) {
        *out_handles = {};
    }
    return false;
}

InferTiPipelineStressResult infer_RunRAEBTiPipelineStress(
    const std::vector<cv::Mat>&,
    std::size_t,
    std::size_t,
    bool) {
    throw std::runtime_error("infer_RunRAEBTiPipelineStress is only supported on TI target build.");
}

void infer_SmokeTestRAEB(int nTimes) {
    if (!g_model_raeb) {
        throw std::runtime_error("RAEB 模型未初始化，请先调用 infer_InitRAEB");
    }
    g_model_raeb->smokeTest(nTimes);
}

#elif defined(VISPER_ARCH_TI)

#include "infer_ti_impl.hpp" // 内含 TIDLRTModel
#include <algorithm>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

// -------------------- 全局模型实例 --------------------
static std::unique_ptr<TIDLModel> g_model_raeb; // 与 x86 保持同名，全局唯一实例

namespace {

std::string JoinOutputNames(const std::vector<std::string>& names) {
    std::ostringstream oss;
    for (std::size_t i = 0; i < names.size(); ++i) {
        if (i != 0) {
            oss << ", ";
        }
        oss << names[i];
    }
    return oss.str();
}

bool HasAllOutputs(const std::vector<std::string>& names,
                   const std::vector<std::string>& required) {
    for (const std::string& name : required) {
        if (std::find(names.begin(), names.end(), name) == names.end()) {
            return false;
        }
    }
    return true;
}

int FindOutputIndex(const std::vector<std::string>& names, const std::string& wanted) {
    const auto it = std::find(names.begin(), names.end(), wanted);
    if (it == names.end()) {
        return -1;
    }
    return static_cast<int>(std::distance(names.begin(), it));
}

cv::Mat Ensure2dFloatOutput(const cv::Mat& tensor, const std::string& name) {
    if (tensor.type() != CV_32F) {
        throw std::runtime_error(name + " must be CV_32F.");
    }
    cv::Mat continuous = tensor.isContinuous() ? tensor : tensor.clone();
    std::vector<int> shape;
    shape.reserve(static_cast<std::size_t>(continuous.dims));
    for (int i = 0; i < continuous.dims; ++i) {
        shape.push_back(continuous.size[i]);
    }
    while (shape.size() > 2U && shape.front() == 1) {
        shape.erase(shape.begin());
    }
    if (shape.size() == 3U && shape[2] == 1) {
        shape.pop_back();
    }
    if (shape.size() != 2U) {
        throw std::runtime_error(name + " must be a 2D tensor after squeezing singleton dims.");
    }
    return continuous.reshape(1, static_cast<int>(shape.size()), shape.data());
}

cv::Mat MergeSplitDetCat(const cv::Mat& det_box,
                         const cv::Mat& det_cls,
                         const cv::Mat& det_mask) {
    cv::Mat box = Ensure2dFloatOutput(det_box, "det_box");
    cv::Mat cls = Ensure2dFloatOutput(det_cls, "det_cls");
    cv::Mat mask = Ensure2dFloatOutput(det_mask, "det_mask");
    if (box.rows != 4) {
        throw std::runtime_error("det_box must have 4 rows.");
    }
    if (box.cols != cls.cols || box.cols != mask.cols) {
        throw std::runtime_error("Split RAEB outputs have inconsistent prediction counts.");
    }
    std::vector<cv::Mat> rows = {box, cls, mask};
    cv::Mat det_cat;
    cv::vconcat(rows, det_cat);
    return det_cat;
}

std::vector<cv::Mat> NormalizeTiRaebOutputs(const std::vector<std::string>& names,
                                            std::vector<cv::Mat> outputs) {
    if (HasAllOutputs(names, {"det_cat", "proto", "angle"})) {
        return outputs;
    }

    if (!HasAllOutputs(names, {"det_box", "det_cls", "det_mask", "proto", "angle"})) {
        return outputs;
    }

    const int box_idx = FindOutputIndex(names, "det_box");
    const int cls_idx = FindOutputIndex(names, "det_cls");
    const int mask_idx = FindOutputIndex(names, "det_mask");
    const int proto_idx = FindOutputIndex(names, "proto");
    const int angle_idx = FindOutputIndex(names, "angle");
    if (box_idx < 0 || cls_idx < 0 || mask_idx < 0 || proto_idx < 0 || angle_idx < 0) {
        throw std::runtime_error("Internal error while resolving split RAEB output indices.");
    }

    std::vector<cv::Mat> normalized;
    normalized.reserve(3);
    normalized.push_back(MergeSplitDetCat(outputs[static_cast<std::size_t>(box_idx)],
                                          outputs[static_cast<std::size_t>(cls_idx)],
                                          outputs[static_cast<std::size_t>(mask_idx)]));
    normalized.push_back(outputs[static_cast<std::size_t>(proto_idx)]);
    normalized.push_back(outputs[static_cast<std::size_t>(angle_idx)]);
    return normalized;
}

void ValidateRaebOutputs(const TIDLModel& model, const std::string& model_path) {
    const std::vector<std::string> names = model.outputNames();
    const bool has_merged_outputs = HasAllOutputs(names, {"det_cat", "proto", "angle"});
    const bool has_split_outputs = HasAllOutputs(names, {"det_box", "det_cls", "det_mask", "proto", "angle"});

    if (!has_merged_outputs && !has_split_outputs) {
        throw std::runtime_error(
            "TI RAEB model output mismatch for model_path=" + model_path +
            ". Expected merged outputs [det_cat, proto, angle] or split outputs "
            "[det_box, det_cls, det_mask, proto, angle], actual outputs=[" +
            JoinOutputNames(names) + "].");
    }
}

} // namespace

// -------------------- API 实现（签名与 x86 完全一致） --------------------
void infer_InitRAEB(const std::string& model_path) {
    infer_InitRAEB(model_path, "");
}

void infer_InitRAEB(const std::string& model_path, const std::string& ti_target) {
    // 每次初始化前先释放旧实例，避免重复 init 时资源残留
    g_model_raeb.reset();

    // 构造失败会抛异常；TI target 为空时由 TIDLModel 使用默认 DSP_C7-2。
    g_model_raeb = std::make_unique<TIDLModel>(model_path, ti_target);
    ValidateRaebOutputs(*g_model_raeb, model_path);
}

void infer_DeinitRAEB() {
    g_model_raeb.reset();
}

std::vector<cv::Mat> infer_InferRAEB(const cv::Mat& data, bool squeeze_batch) {
    if (!g_model_raeb) {
        throw std::runtime_error("RAEB 模型未初始化，请先调用 infer_InitRAEB");
    }
    return NormalizeTiRaebOutputs(g_model_raeb->outputNames(),
                                  g_model_raeb->infer(data, /*squeeze_batch=*/squeeze_batch));
}

InferRAEBOutputBundle infer_InferRAEBWithOwner(const cv::Mat& data, bool squeeze_batch) {
    if (!g_model_raeb) {
        throw std::runtime_error("RAEB 模型未初始化，请先调用 infer_InitRAEB");
    }
    InferRAEBOutputBundle result;
    result.outputs = NormalizeTiRaebOutputs(
        g_model_raeb->outputNames(),
        g_model_raeb->inferWithOutputOwner(data, /*squeeze_batch=*/squeeze_batch, result.owner));
    return result;
}

bool infer_GetRAEBTiPerfHandles(InferTiPerfHandles* out_handles) {
    if (out_handles != nullptr) {
        *out_handles = {};
    }
    if (!g_model_raeb || out_handles == nullptr) {
        return false;
    }
    return g_model_raeb->perfHandles(
        &out_handles->graph,
        &out_handles->total_perf,
        &out_handles->graph_perf);
}

InferTiPipelineStressResult infer_RunRAEBTiPipelineStress(
    const std::vector<cv::Mat>& inputs,
    std::size_t total_tasks,
    std::size_t pipeline_depth,
    bool read_outputs) {
    if (!g_model_raeb) {
        throw std::runtime_error("RAEB 模型未初始化，请先调用 infer_InitRAEB");
    }
    const auto stress = g_model_raeb->pipelineStress(
        inputs,
        total_tasks,
        pipeline_depth,
        read_outputs);
    InferTiPipelineStressResult result;
    result.submitted = stress.submitted;
    result.completed = stress.completed;
    result.write_input_ms = stress.write_input_ms;
    result.schedule_ms = stress.schedule_ms;
    result.wait_graph_ms = stress.wait_graph_ms;
    result.read_output_ms = stress.read_output_ms;
    result.elapsed_ms = stress.elapsed_ms;
    result.fps = stress.fps;
    return result;
}

// 冒烟测试：调用类内的 smokeTest(nTimes)，做 n 次推理
void infer_SmokeTestRAEB(int nTimes) {
    if (!g_model_raeb) {
        throw std::runtime_error("RAEB 模型未初始化，请先调用 infer_InitRAEB");
    }
    g_model_raeb->smokeTest(nTimes);
}


#else

#error "Unknown architecture"

#endif
