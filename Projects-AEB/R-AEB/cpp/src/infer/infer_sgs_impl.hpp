#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <cstring>
#include <sys/types.h>

#include <string.h>
#include <fstream>
#include <iostream>
#include <dirent.h>
#include <getopt.h>
#include <opencv2/core/core.hpp>
#include <sys/time.h>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <random>
#include <sstream>
#include <unordered_map>
#include <vector>

#include "mi_common_datatype.h"
#include "mi_sys_datatype.h"
#include "mi_ipu.h"
#include "mi_sys.h"

#include "common/logger.hpp"

// —— 若小函数定义在别处源文件，这里先声明它们 ——
// （如果就在同一 .cpp 里实现，则不需要 extern 声明）
MI_S32 IPUCreateDevice(MI_U32 u32VarBufSize) {
    MI_S32 s32Ret = MI_SUCCESS;
    MI_IPU_DevAttr_t stDevAttr = MI_IPU_DevAttr_t();
    stDevAttr.u32MaxVariableBufSize = u32VarBufSize;
    s32Ret = MI_IPU_CreateDevice(&stDevAttr, NULL, NULL, 0);
    return s32Ret;
}


MI_S32 IPUCreateChannel(MI_U32* u32Channel, char* pModelImage) {
    MI_IPUChnAttr_t stChnAttr = MI_IPUChnAttr_t();

    //create channel
    memset(&stChnAttr, 0, sizeof(stChnAttr));
    stChnAttr.u32InputBufDepth = 1;
    stChnAttr.u32OutputBufDepth = 1;
    return MI_IPU_CreateCHN(u32Channel, &stChnAttr, NULL, pModelImage);
}

struct NetInfo {
    MI_IPU_OfflineModelStaticInfo_t OfflineModelInfo{};
    MI_IPU_SubNet_InputOutputDesc_t desc{};
};


class IPUModel {
public:
    // 构造：完成系统/设备/通道/张量获取；始终打印模型 I/O 信息
    // - modelFile: 模型路径
    IPUModel(const std::string& modelFile)
        : modelFile_(modelFile) {
        const char* dumpEnv = std::getenv("DUMP_INPUT_BIN");
        dumpInputBin_ = (dumpEnv != nullptr);

        // 1) 初始化系统
        MI_S32 ret = MI_SYS_Init(0);
        if (ret != MI_SUCCESS) {
            throw std::runtime_error("MI_SYS_Init failed, code=" + std::to_string(ret));
        }
        sysInited_ = true;

        // 2) 获取离线模型静态信息（拿 variable buffer size）
        ret = MI_IPU_GetOfflineModeStaticInfo(nullptr,
                                              const_cast<char*>(modelFile_.c_str()),
                                              &net_.OfflineModelInfo);
        if (ret != MI_SUCCESS) {
            cleanup();
            throw std::runtime_error("GetOfflineModeStaticInfo failed, code=" + std::to_string(ret));
        }

        // 3) 创建设备
        ret = IPUCreateDevice(net_.OfflineModelInfo.u32VariableBufferSize);
        if (ret != MI_SUCCESS) {
            cleanup();
            throw std::runtime_error("IPUCreateDevice failed, code=" + std::to_string(ret));
        }
        devCreated_ = true;

        // 4) 创建通道并加载模型
        ret = IPUCreateChannel(&u32ChannelID_, const_cast<char*>(modelFile_.c_str()));
        if (ret != MI_SUCCESS) {
            cleanup();
            throw std::runtime_error("IPUCreateChannel failed, code=" + std::to_string(ret));
        }
        chnCreated_ = true;

        // 5) 获取 I/O 描述。I/O tensor buffer 改为按路径懒加载：
        //    普通 cv::Mat 路径首次 infer 时获取 SDK buffer；
        //    外部物理地址路径使用 Invoke2 用户 buffer，避免额外 input memcpy。
        MI_IPU_GetInOutTensorDesc(u32ChannelID_, &net_.desc);

        // 始终打印模型 I/O 信息
        printModelIOInfo(net_.desc);
    }

    ~IPUModel() {
        cleanup();
    }

    struct FrameDataOutputBundle {
        std::vector<cv::Mat> outputs;
        std::shared_ptr<void> owner;
    };


    // 放在 class IPUModel 的 public 区域
    std::vector<cv::Mat> infer(const cv::Mat& input,
                               bool squeeze_batch = false,
                               long long* invokeUs = nullptr) {
        std::lock_guard<std::mutex> inferLock(inferMutex_);
        const auto totalT0 = std::chrono::high_resolution_clock::now();
        // 1) 只支持单输入模型
        if (net_.desc.u32InputTensorCount != 1) {
            throw std::runtime_error("infer() only supports single-input models.");
        }

        auto& inDesc = net_.desc.astMI_InputTensorDescs[0];

        // 通用检查：元素必须是 U8
        if (input.type() != CV_8U) {
            throw std::runtime_error("infer() requires input type CV_8U.");
        }

        // 根据模型输入的 eElmFormat 区分两种情况
        size_t inputBytes = 0;

        if (inDesc.eElmFormat == MI_IPU_FORMAT_U8 &&
            inDesc.eLayoutType == E_IPU_LAYOUT_TYPE_NHWC) {
            // ===== 原来那条 RGB NHWC 路径 =====
            if (input.dims != 4) {
                throw std::runtime_error("For NHWC U8 input, infer() requires 4D Mat [N,H,W,C].");
            }

            const int* sz = input.size.p;
            const int N = sz[0], H = sz[1], W = sz[2], C = sz[3];
            if (C != 3) {
                throw std::runtime_error("For NHWC U8 input, last dim C must be 3 (RGB).");
            }

            // 检查张量描述的 shape
            if (inDesc.u32TensorDim != 4 ||
                static_cast<int>(inDesc.u32TensorShape[0]) != N ||
                static_cast<int>(inDesc.u32TensorShape[1]) != H ||
                static_cast<int>(inDesc.u32TensorShape[2]) != W ||
                static_cast<int>(inDesc.u32TensorShape[3]) != C) {
                throw std::runtime_error("Input shape does not match model NHWC input tensor shape.");
            }

            inputBytes = static_cast<size_t>(N) * H * W * C;
            if (inputBytes != static_cast<size_t>(inDesc.u32BufSize)) {
                throw std::runtime_error("Input bytes do not match model NHWC input buffer size.");
            }
        }
        else if (inDesc.eElmFormat == MI_IPU_FORMAT_NV12) {
            // ===== 新增：NV12 路径 =====
            // 这里不强制 layoutType，只要字节数对得上即可；
            // 支持原始 2D NV12 Mat [H*3/2,W]，也兼容旧的 3D [1,H*3/2,W]。
            if (input.dims == 2) {
                inputBytes = input.total() * input.elemSize();
            }
            else if (input.dims == 3) {
                const int* sz = input.size.p;
                const int N = sz[0];
                const int H3 = sz[1]; // H * 3/2
                const int W = sz[2];

                if (N != 1) {
                    throw std::runtime_error("For NV12 input, first dim must be 1.");
                }

                inputBytes = static_cast<size_t>(N) * H3 * W;
            }
            else {
                throw std::runtime_error("For NV12 input, infer() expects 2D Mat [H*3/2,W] or 3D Mat [1,H*3/2,W].");
            }

            if (inputBytes != static_cast<size_t>(inDesc.u32BufSize)) {
                throw std::runtime_error("NV12 input bytes do not match model NV12 input buffer size.");
            }
        }
        else {
            throw std::runtime_error("Unsupported input element format: only U8(NHWC) and NV12 are supported.");
        }
        const auto validateT1 = std::chrono::high_resolution_clock::now();
        const long long validate_elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(validateT1 - totalT0).count();

        // 2) 拷贝到 IPU 输入缓冲区并刷 cache（两种格式共用逻辑）
        const auto ensureT0 = std::chrono::high_resolution_clock::now();
        ensureLegacyTensors();
        const auto ensureT1 = std::chrono::high_resolution_clock::now();
        const long long ensure_elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(ensureT1 - ensureT0).count();

        void* dst = input_.astArrayTensors[0].ptTensorData[0];

        const auto copyT0 = std::chrono::high_resolution_clock::now();
        if (!input.isContinuous()) {
            cv::Mat contiguous = input.clone();
            std::memcpy(dst, contiguous.data, inputBytes);
        }
        else {
            std::memcpy(dst, input.data, inputBytes);
        }
        const auto copyT1 = std::chrono::high_resolution_clock::now();
        const long long copy_elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(copyT1 - copyT0).count();

        const auto flushT0 = std::chrono::high_resolution_clock::now();
        MI_S32 flushRet = MI_SYS_FlushInvCache(dst, inDesc.u32BufSize);
        const auto flushT1 = std::chrono::high_resolution_clock::now();
        const long long flush_elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(flushT1 - flushT0).count();

        // 3) 调用推理
        const auto invokeT0 = std::chrono::high_resolution_clock::now();
        MI_S32 ret = MI_IPU_Invoke(u32ChannelID_, &input_, &output_);
        const auto invokeT1 = std::chrono::high_resolution_clock::now();
        if (invokeUs != nullptr) {
            *invokeUs = std::chrono::duration_cast<std::chrono::microseconds>(
                            invokeT1 - invokeT0)
                            .count();
        }
        if (ret != MI_SUCCESS) {
            throw std::runtime_error("MI_IPU_Invoke failed, ret=" + std::to_string(ret));
        }

        const auto readT0 = std::chrono::high_resolution_clock::now();
        auto outs = readOutputTensors(output_.astArrayTensors, squeeze_batch);
        const auto readT1 = std::chrono::high_resolution_clock::now();
        const long long read_elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(readT1 - readT0).count();
        const auto totalT1 = std::chrono::high_resolution_clock::now();
        const long long total_elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(totalT1 - totalT0).count();

        LOG_INFO(
            "IPUModel::infer breakdown_us validate=%lld, ensure_tensors=%lld, copy_input=%lld, flush_cache=%lld, invoke=%lld, read_outputs=%lld, total=%lld, input_bytes=%zu, continuous=%d, flush_ret=%d",
            validate_elapsed_us,
            ensure_elapsed_us,
            copy_elapsed_us,
            flush_elapsed_us,
            std::chrono::duration_cast<std::chrono::microseconds>(invokeT1 - invokeT0).count(),
            read_elapsed_us,
            total_elapsed_us,
            inputBytes,
            input.isContinuous() ? 1 : 0,
            static_cast<int>(flushRet)
        );

        return outs;
    }

    std::vector<cv::Mat> inferPhysical(MI_PHY inputPhyAddr,
                                       std::size_t inputLength,
                                       bool squeeze_batch = false,
                                       long long* invokeUs = nullptr) {
        std::lock_guard<std::mutex> inferLock(inferMutex_);
        LOG_INFO(
            "IPUModel::inferPhysical enter channel=%u, phy=0x%llx, length=%zu, squeeze_batch=%d, input_count=%u, output_count=%u",
            static_cast<unsigned>(u32ChannelID_),
            static_cast<unsigned long long>(inputPhyAddr),
            inputLength,
            squeeze_batch ? 1 : 0,
            static_cast<unsigned>(net_.desc.u32InputTensorCount),
            static_cast<unsigned>(net_.desc.u32OutputTensorCount)
        );
        if (net_.desc.u32InputTensorCount != 1) {
            LOG_WARNING("IPUModel::inferPhysical reject: input tensor count is %u", static_cast<unsigned>(net_.desc.u32InputTensorCount));
            throw std::runtime_error("inferPhysical() only supports single-input models.");
        }

        const auto& inDesc = net_.desc.astMI_InputTensorDescs[0];
        LOG_INFO(
            "IPUModel::inferPhysical input desc name=%s, fmt=%s, layout=%u, dim=%u, shape=%s, buf=%u, aligned=%d",
            inDesc.name,
            typeName(inDesc.eElmFormat),
            static_cast<unsigned>(inDesc.eLayoutType),
            static_cast<unsigned>(inDesc.u32TensorDim),
            tensorShapeText(inDesc).c_str(),
            static_cast<unsigned>(inDesc.u32BufSize),
            static_cast<int>(inDesc.s32AlignedBufSize)
        );
        if (!((inDesc.eElmFormat == MI_IPU_FORMAT_U8 &&
               inDesc.eLayoutType == E_IPU_LAYOUT_TYPE_NHWC) ||
              inDesc.eElmFormat == MI_IPU_FORMAT_NV12)) {
            LOG_WARNING(
                "IPUModel::inferPhysical reject: unsupported input fmt=%s layout=%u",
                typeName(inDesc.eElmFormat),
                static_cast<unsigned>(inDesc.eLayoutType)
            );
            throw std::runtime_error("inferPhysical() only supports U8(NHWC) and NV12 input tensors.");
        }

        const MI_U32 inputBytes = invoke2TensorBufferSize(inDesc);
        LOG_INFO(
            "IPUModel::inferPhysical input bytes required=%u, provided=%zu",
            static_cast<unsigned>(inputBytes),
            inputLength
        );
        if (inputLength < static_cast<std::size_t>(inputBytes)) {
            LOG_WARNING(
                "IPUModel::inferPhysical reject: input length %zu < required %u",
                inputLength,
                static_cast<unsigned>(inputBytes)
            );
            throw std::runtime_error("inferPhysical() input buffer is smaller than IPU input tensor buffer.");
        }

        LOG_INFO("IPUModel::inferPhysical ensureInvoke2Resources begin");
        ensureInvoke2Resources();
        LOG_INFO("IPUModel::inferPhysical ensureInvoke2Resources done");
        auto outputLease = acquireInvoke2OutputSlot();
        bindInvoke2OutputSlot(outputLease->slotIndex);
        LOG_INFO(
            "IPUModel::inferPhysical output slot acquired index=%zu",
            outputLease->slotIndex
        );
        LOG_INFO(
            "IPUModel::inferPhysical mapExternalInput begin phy=0x%llx, bytes=%u",
            static_cast<unsigned long long>(inputPhyAddr),
            static_cast<unsigned>(inputBytes)
        );
        void* inputVirAddr = mapExternalInput(inputPhyAddr, inputBytes);
        LOG_INFO(
            "IPUModel::inferPhysical mapExternalInput done phy=0x%llx, vir=%p",
            static_cast<unsigned long long>(inputPhyAddr),
            inputVirAddr
        );

        invoke2_.astArrayTensors[0].ptTensorData[0] = inputVirAddr;
        invoke2_.astArrayTensors[0].phyTensorAddr[0] = inputPhyAddr;

        LOG_INFO("IPUModel::inferPhysical FlushInvCache begin vir=%p, bytes=%u", inputVirAddr, static_cast<unsigned>(inputBytes));
        MI_S32 ret = MI_SYS_FlushInvCache(inputVirAddr, inputBytes);
        if (ret != MI_SUCCESS) {
            LOG_WARNING("IPUModel::inferPhysical FlushInvCache failed ret=%d", static_cast<int>(ret));
            throw std::runtime_error("MI_SYS_FlushInvCache external input failed, ret=" + std::to_string(ret));
        }
        LOG_INFO("IPUModel::inferPhysical FlushInvCache done");

        std::memset(&runtime2_, 0, sizeof(runtime2_));
        const auto invokeT0 = std::chrono::high_resolution_clock::now();
        LOG_INFO(
            "IPUModel::inferPhysical Invoke2 begin batch_mode=%d, use_custom=%d",
            static_cast<int>(net_.OfflineModelInfo.eBatchMode),
            net_.OfflineModelInfo.eBatchMode == E_IPU_BATCH_ONE_BUF_MODE ? 1 : 0
        );
        if (net_.OfflineModelInfo.eBatchMode == E_IPU_BATCH_ONE_BUF_MODE) {
            ret = MI_IPU_Invoke2Custom(u32ChannelID_, &invoke2_, &runtime2_);
        }
        else {
            ret = MI_IPU_Invoke2(u32ChannelID_, &invoke2_, &runtime2_);
        }
        const auto invokeT1 = std::chrono::high_resolution_clock::now();
        if (invokeUs != nullptr) {
            *invokeUs = std::chrono::duration_cast<std::chrono::microseconds>(
                            invokeT1 - invokeT0)
                            .count();
        }
        const long long invoke_elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
                                                invokeT1 - invokeT0)
                                                .count();
        LOG_INFO("IPUModel::inferPhysical Invoke2 done ret=%d, elapsed_us=%lld", static_cast<int>(ret), invoke_elapsed_us);
        if (ret != MI_SUCCESS) {
            throw std::runtime_error("MI_IPU_Invoke2 external input failed, ret=" + std::to_string(ret));
        }

        const MI_U32 outputBase = net_.desc.u32InputTensorCount;
        LOG_INFO("IPUModel::inferPhysical readOutputTensors begin output_base=%u", static_cast<unsigned>(outputBase));
        auto outs = readOutputTensors(invoke2OutputSlots_[outputLease->slotIndex].tensors.data(), squeeze_batch);
        LOG_INFO("IPUModel::inferPhysical readOutputTensors done count=%zu", outs.size());
        return outs;
    }

    std::vector<cv::Mat> inferFrameData(const MI_SYS_FrameData_s& frame,
                                        bool squeeze_batch = false,
                                        long long* invokeUs = nullptr) {
        auto result = inferFrameDataImpl(frame, squeeze_batch, invokeUs, false);
        return std::move(result.outputs);
    }

    FrameDataOutputBundle inferFrameDataWithOwner(const MI_SYS_FrameData_s& frame,
                                                  bool squeeze_batch = false,
                                                  long long* invokeUs = nullptr) {
        return inferFrameDataImpl(frame, squeeze_batch, invokeUs, true);
    }

    FrameDataOutputBundle inferFrameDataImpl(const MI_SYS_FrameData_s& frame,
                                             bool squeeze_batch,
                                             long long* invokeUs,
                                             bool zero_copy_outputs) {
        std::lock_guard<std::mutex> inferLock(inferMutex_);
        const auto totalT0 = std::chrono::high_resolution_clock::now();
        if (net_.desc.u32InputTensorCount != 1) {
            LOG_WARNING("IPUModel::inferFrameData reject: input tensor count is %u", static_cast<unsigned>(net_.desc.u32InputTensorCount));
            throw std::runtime_error("inferFrameData() only supports single-input models.");
        }
        if (frame.ePhylayoutType == REALTIME_FRAME_DATA ||
            frame.pVirAddr[0] == MI_SYS_REALTIME_MAGIC_VADDR ||
            frame.phyAddr[0] == MI_SYS_REALTIME_MAGIC_PADDR) {
            LOG_WARNING("IPUModel::inferFrameData reject: REALTIME_FRAME_DATA/magic address cannot be bound as a normal external tensor.");
            throw std::runtime_error("inferFrameData() does not support REALTIME_FRAME_DATA magic addresses.");
        }
        if (frame.pVirAddr[0] == nullptr) {
            LOG_WARNING("IPUModel::inferFrameData reject: frame.pVirAddr[0] is null");
            throw std::runtime_error("inferFrameData() frame.pVirAddr[0] is null.");
        }
        if (frame.phyAddr[0] == 0) {
            LOG_WARNING("IPUModel::inferFrameData reject: frame.phyAddr[0] is zero");
            throw std::runtime_error("inferFrameData() frame.phyAddr[0] is zero.");
        }

        const auto& inDesc = net_.desc.astMI_InputTensorDescs[0];
        if (!((inDesc.eElmFormat == MI_IPU_FORMAT_U8 &&
               inDesc.eLayoutType == E_IPU_LAYOUT_TYPE_NHWC) ||
              inDesc.eElmFormat == MI_IPU_FORMAT_NV12)) {
            LOG_WARNING(
                "IPUModel::inferFrameData reject: unsupported input fmt=%s layout=%u",
                typeName(inDesc.eElmFormat),
                static_cast<unsigned>(inDesc.eLayoutType)
            );
            throw std::runtime_error("inferFrameData() only supports U8(NHWC) and NV12 input tensors.");
        }

        const MI_U32 inputBytes = invoke2TensorBufferSize(inDesc);
        if (frame.u32BufSize != 0 && frame.u32BufSize < inputBytes) {
            LOG_WARNING(
                "IPUModel::inferFrameData reject: frame buffer %u < required %u",
                static_cast<unsigned>(frame.u32BufSize),
                static_cast<unsigned>(inputBytes)
            );
            throw std::runtime_error("inferFrameData() frame buffer is smaller than IPU input tensor buffer.");
        }
        if (frame.u32BufSize == 0) {
            LOG_WARNING("IPUModel::inferFrameData frame.u32BufSize is zero; relying on model input size for cache flush.");
        }

        ensureInvoke2Resources();

        auto outputLease = acquireInvoke2OutputSlot();
        bindInvoke2OutputSlot(outputLease->slotIndex);
        const auto slotT1 = std::chrono::high_resolution_clock::now();

        invoke2_.astArrayTensors[0].ptTensorData[0] = frame.pVirAddr[0];
        invoke2_.astArrayTensors[0].phyTensorAddr[0] = frame.phyAddr[0];
        invoke2_.astArrayTensors[0].ptTensorData[1] = nullptr;
        invoke2_.astArrayTensors[0].phyTensorAddr[1] = 0;

        const bool hasSecondPlane = inDesc.eElmFormat == MI_IPU_FORMAT_NV12 &&
                                    frame.pVirAddr[1] != nullptr &&
                                    frame.phyAddr[1] != 0 &&
                                    frame.pVirAddr[1] != MI_SYS_REALTIME_MAGIC_VADDR &&
                                    frame.phyAddr[1] != MI_SYS_REALTIME_MAGIC_PADDR;
        if (hasSecondPlane) {
            invoke2_.astArrayTensors[0].ptTensorData[1] = frame.pVirAddr[1];
            invoke2_.astArrayTensors[0].phyTensorAddr[1] = frame.phyAddr[1];
        }

        auto flushCache = [](void* addr, MI_U32 bytes, const char* label) {
            if (bytes == 0) {
                return;
            }
            MI_S32 ret = MI_SYS_FlushInvCache(addr, bytes);
            if (ret != MI_SUCCESS) {
                LOG_WARNING("IPUModel::inferFrameData FlushInvCache failed label=%s, ret=%d", label, static_cast<int>(ret));
                throw std::runtime_error(std::string("MI_SYS_FlushInvCache frame input failed, label=") + label + ", ret=" + std::to_string(ret));
            }
        };

        bool flushed = false;
        if (hasSecondPlane) {
            if (frame.u16Height == 0 || frame.u32Stride[0] == 0) {
                LOG_WARNING("IPUModel::inferFrameData reject: NV12 second plane exists but height/stride[0] cannot split cache range.");
                throw std::runtime_error("inferFrameData() cannot split NV12 cache flush without height and stride[0].");
            }
            const std::uint64_t yBytes64 = static_cast<std::uint64_t>(frame.u32Stride[0]) *
                                           static_cast<std::uint64_t>(frame.u16Height);
            if (yBytes64 == 0 || yBytes64 >= inputBytes || yBytes64 > std::numeric_limits<MI_U32>::max()) {
                LOG_WARNING(
                    "IPUModel::inferFrameData reject: invalid NV12 Y plane bytes=%llu, inputBytes=%u",
                    static_cast<unsigned long long>(yBytes64),
                    static_cast<unsigned>(inputBytes)
                );
                throw std::runtime_error("inferFrameData() invalid NV12 Y plane size for cache flush.");
            }
            const MI_U32 yBytes = static_cast<MI_U32>(yBytes64);
            const MI_U32 uvBytes = inputBytes - yBytes;
            const auto vir0 = reinterpret_cast<std::uintptr_t>(frame.pVirAddr[0]);
            const auto vir1 = reinterpret_cast<std::uintptr_t>(frame.pVirAddr[1]);
            const bool virContiguous = (vir1 == vir0 + yBytes);
            const bool phyContiguous = (frame.phyAddr[1] == frame.phyAddr[0] + yBytes);
            if (!virContiguous || !phyContiguous) {
                flushCache(frame.pVirAddr[0], yBytes, "NV12-Y");
                flushCache(frame.pVirAddr[1], uvBytes, "NV12-UV");
                flushed = true;
            }
        }
        if (!flushed) {
            flushCache(frame.pVirAddr[0], inputBytes, "input");
        }
        const auto flushT1 = std::chrono::high_resolution_clock::now();

        std::memset(&runtime2_, 0, sizeof(runtime2_));
        const auto invokeT0 = std::chrono::high_resolution_clock::now();
        MI_S32 ret = MI_SUCCESS;
        const bool useCustomInvoke = net_.OfflineModelInfo.eBatchMode == E_IPU_BATCH_ONE_BUF_MODE;
        if (useCustomInvoke) {
            ret = MI_IPU_Invoke2Custom(u32ChannelID_, &invoke2_, &runtime2_);
        }
        else {
            ret = MI_IPU_Invoke2(u32ChannelID_, &invoke2_, &runtime2_);
        }
        const auto invokeT1 = std::chrono::high_resolution_clock::now();
        if (invokeUs != nullptr) {
            *invokeUs = std::chrono::duration_cast<std::chrono::microseconds>(
                            invokeT1 - invokeT0)
                            .count();
        }
        const long long invoke_elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
                                                invokeT1 - invokeT0)
                                                .count();
        if (ret != MI_SUCCESS) {
            throw std::runtime_error("MI_IPU_Invoke2 frame input failed, ret=" + std::to_string(ret));
        }

        const auto outputT0 = std::chrono::high_resolution_clock::now();
        FrameDataOutputBundle result;
        if (zero_copy_outputs) {
            result.outputs = viewOutputTensors(invoke2OutputSlots_[outputLease->slotIndex].tensors.data(), squeeze_batch);
            result.owner = outputLease;
        } else {
            result.outputs = readOutputTensors(invoke2OutputSlots_[outputLease->slotIndex].tensors.data(), squeeze_batch);
        }
        const auto outputT1 = std::chrono::high_resolution_clock::now();
        const auto totalT1 = std::chrono::high_resolution_clock::now();
        const long long flush_elapsed_us =
            std::chrono::duration_cast<std::chrono::microseconds>(flushT1 - slotT1).count();
        const long long output_elapsed_us =
            std::chrono::duration_cast<std::chrono::microseconds>(outputT1 - outputT0).count();
        const long long total_elapsed_us =
            std::chrono::duration_cast<std::chrono::microseconds>(totalT1 - totalT0).count();
        const long long sdk_ipu_us = static_cast<long long>(runtime2_.u64IpuTime);
        const long long invoke_overhead_us = invoke_elapsed_us - sdk_ipu_us;

        LOG_INFO(
            "IPUModel::inferFrameData invoke_profile wall_us=%lld, sdk_ipu_us=%lld, overhead_us=%lld, bw=%llu, bw_read=%llu, bw_write=%llu, flush_us=%lld, output_us=%lld, total_us=%lld, input_bytes=%u, api=%s, output_mode=%s, slot=%zu",
            invoke_elapsed_us,
            sdk_ipu_us,
            invoke_overhead_us,
            static_cast<unsigned long long>(runtime2_.u64BandWidth),
            static_cast<unsigned long long>(runtime2_.u64BandWidthRead),
            static_cast<unsigned long long>(runtime2_.u64BandWidthWrite),
            flush_elapsed_us,
            output_elapsed_us,
            total_elapsed_us,
            static_cast<unsigned>(inputBytes),
            useCustomInvoke ? "Invoke2Custom" : "Invoke2",
            zero_copy_outputs ? "view" : "copy",
            outputLease->slotIndex
        );
        return result;
    }


    void smokeTest(int nTimes) {
        if (nTimes <= 0) {
            std::cout << "[SmokeTest] nTimes <= 0, nothing to do.\n";
            return;
        }

        if (net_.desc.u32InputTensorCount == 0) {
            throw std::runtime_error("SmokeTest: model has no input tensors.");
        }

        const auto& inDesc = net_.desc.astMI_InputTensorDescs[0];

        // 简单工具：打印 shape
        auto printShape = [](const cv::Mat& m, const std::string& name) {
            std::cout << name << " dims=" << m.dims << " [";
            for (int i = 0; i < m.dims; ++i) {
                std::cout << m.size[i] << (i + 1 == m.dims ? "" : ",");
            }
            std::cout << "], type=" << m.type() << "\n";
        };

        // 简单工具：打印前几个元素
        auto printHead = [](const cv::Mat& m, int maxCount, const std::string& name) {
            int total = 1;
            for (int i = 0; i < m.dims; ++i) {
                total *= m.size[i];
            }
            maxCount = std::min(maxCount, total);

            std::cout << name << " first " << maxCount << " elements: ";

            if (m.type() == CV_32F) {
                const float* p = reinterpret_cast<const float*>(m.data);
                for (int i = 0; i < maxCount; ++i) {
                    std::cout << p[i] << (i + 1 == maxCount ? "" : ", ");
                }
            }
            else if (m.type() == CV_32S) {
                const int32_t* p = reinterpret_cast<const int32_t*>(m.data);
                for (int i = 0; i < maxCount; ++i) {
                    std::cout << p[i] << (i + 1 == maxCount ? "" : ", ");
                }
            }
            else if (m.type() == CV_16S) {
                const int16_t* p = reinterpret_cast<const int16_t*>(m.data);
                for (int i = 0; i < maxCount; ++i) {
                    std::cout << p[i] << (i + 1 == maxCount ? "" : ", ");
                }
            }
            else if (m.type() == CV_8S) {
                const int8_t* p = reinterpret_cast<const int8_t*>(m.data);
                for (int i = 0; i < maxCount; ++i) {
                    std::cout << static_cast<int>(p[i]) << (i + 1 == maxCount ? "" : ", ");
                }
            }
            else if (m.type() == CV_8U) {
                const uint8_t* p = reinterpret_cast<const uint8_t*>(m.data);
                for (int i = 0; i < maxCount; ++i) {
                    std::cout << static_cast<int>(p[i]) << (i + 1 == maxCount ? "" : ", ");
                }
            }
            else {
                std::cout << "(unknown type, skip print)";
            }
            std::cout << "\n";
        };

        // 1. 根据模型输入描述，构造一次随机输入（只用第一个 input）
        cv::Mat input;
        std::mt19937 rng(1234);
        std::uniform_int_distribution<int> distU8(0, 255);

        if (inDesc.eElmFormat == MI_IPU_FORMAT_U8 &&
            inDesc.eLayoutType == E_IPU_LAYOUT_TYPE_NHWC &&
            inDesc.u32TensorDim == 4) {
            // NHWC: [N,H,W,C]
            int sizes[4];
            for (int i = 0; i < 4; ++i) {
                sizes[i] = static_cast<int>(inDesc.u32TensorShape[i]);
            }

            input.create(4, sizes, CV_8U);

            int total = sizes[0] * sizes[1] * sizes[2] * sizes[3];
            uint8_t* p = input.data;
            for (int i = 0; i < total; ++i) {
                p[i] = static_cast<uint8_t>(distU8(rng));
            }

            std::cout << "[SmokeTest] Created random NHWC input.\n";
            printShape(input, "Input");
        }
        else if (inDesc.eElmFormat == MI_IPU_FORMAT_NV12) {
            // NV12：infer() 要求 3D Mat [1, H3, W]，只要总字节数匹配 u32BufSize 即可
            // 这里做一个稳定又简单的实现：dims = [1, 1, bufSize]
            int sizes[3];
            sizes[0] = 1;
            sizes[1] = 1;
            sizes[2] = static_cast<int>(inDesc.u32BufSize);

            input.create(3, sizes, CV_8U);

            int total = sizes[0] * sizes[1] * sizes[2];
            uint8_t* p = input.data;
            for (int i = 0; i < total; ++i) {
                p[i] = static_cast<uint8_t>(distU8(rng));
            }

            std::cout << "[SmokeTest] Created random NV12 input (1 x 1 x bufSize).\n";
            printShape(input, "Input");
        }
        else {
            throw std::runtime_error("SmokeTest: unsupported input format (only U8 NHWC or NV12).");
        }

        // 计时统计
        long long totalUs = 0;
        long long minUs = std::numeric_limits<long long>::max();
        long long maxUs = 0;

        // 2. 连续做 n 次推理
        for (int i = 0; i < nTimes; ++i) {
            long long us = 0;
            auto outs = infer(input, true, &us); // 压掉 batch 维度，方便查看
            totalUs += us;
            if (us < minUs) minUs = us;
            if (us > maxUs) maxUs = us;

            double ms = static_cast<double>(us) / 1000.0;

            std::cout << "[SmokeTest] Iter " << i
                << " done, MI_IPU_Invoke time = " << ms << " ms"
                << ", output tensor count = " << outs.size() << "\n";

            // 只在第 0 次的时候稍微打印一下输出 shape 和部分数值
            if (i == 0) {
                for (size_t k = 0; k < outs.size(); ++k) {
                    std::string name = "Output[" + std::to_string(k) + "]";
                    printShape(outs[k], name);
                    printHead(outs[k], 10, name);
                }
            }
        }

        double avgMs = static_cast<double>(totalUs) / nTimes / 1000.0;
        double minMs = static_cast<double>(minUs) / 1000.0;
        double maxMs = static_cast<double>(maxUs) / 1000.0;

        std::cout << "[SmokeTest] All " << nTimes << " iterations finished.\n";
        std::cout << "[SmokeTest] MI_IPU_Invoke Avg = " << avgMs
            << " ms, Min = " << minMs
            << " ms, Max = " << maxMs << " ms\n";
    }

    // Getter
    MI_U32 channel() const { return u32ChannelID_; }
    const MI_IPU_SubNet_InputOutputDesc_t& desc() const { return net_.desc; }
    bool dumpInputEnabled() const { return dumpInputBin_; }

    // I/O 张量（外部预处理与 MI_IPU_Invoke 使用）
    MI_IPU_TensorVector_t& inputTensors() {
        ensureLegacyTensors();
        return input_;
    }
    MI_IPU_TensorVector_t& outputTensors() {
        ensureLegacyTensors();
        return output_;
    }

private:
    static std::string tensorShapeText(const MI_IPU_TensorDesc_t& desc) {
        std::ostringstream oss;
        oss << "[";
        for (MI_U32 i = 0; i < desc.u32TensorDim; ++i) {
            if (i != 0) {
                oss << ",";
            }
            oss << static_cast<unsigned>(desc.u32TensorShape[i]);
        }
        oss << "]";
        return oss.str();
    }

    static const char* typeName(MI_IPU_ELEMENT_FORMAT fmt) {
        switch (fmt) {
        case MI_IPU_FORMAT_U8: return "U8";
        case MI_IPU_FORMAT_NV12: return "NV12";
        case MI_IPU_FORMAT_INT16: return "INT16";
        case MI_IPU_FORMAT_INT32: return "INT32";
        case MI_IPU_FORMAT_INT8: return "INT8";
        case MI_IPU_FORMAT_FP32: return "FP32";
        case MI_IPU_FORMAT_ARGB8888: return "ARGB8888";
        case MI_IPU_FORMAT_ABGR8888: return "ABGR8888";
        case MI_IPU_FORMAT_GRAY: return "GRAY";
        default: return "UNKNOWN";
        }
    }

    struct ExternalInputMap {
        void* virAddr{nullptr};
        MI_U32 size{0};
    };

    static constexpr std::size_t kInvoke2OutputPoolSize = 3;

    struct Invoke2OutputSlot {
        std::vector<MI_IPU_Tensor_t> tensors;
        std::vector<MI_U32> sizes;
        bool inUse{false};
    };

    struct Invoke2OutputLease {
        IPUModel* model{nullptr};
        std::size_t slotIndex{0};

        ~Invoke2OutputLease() {
            if (model != nullptr) {
                model->releaseInvoke2OutputSlot(slotIndex);
            }
        }
    };

    static bool elemTypeAndSize(MI_IPU_ELEMENT_FORMAT fmt, int& cvType, size_t& elemSize) {
        switch (fmt) {
        case MI_IPU_FORMAT_FP32:
            cvType = CV_32F;
            elemSize = sizeof(float);
            return true;
        case MI_IPU_FORMAT_INT32:
            cvType = CV_32S;
            elemSize = sizeof(int32_t);
            return true;
        case MI_IPU_FORMAT_INT16:
            cvType = CV_16S;
            elemSize = sizeof(int16_t);
            return true;
        case MI_IPU_FORMAT_INT8:
            cvType = CV_8S;
            elemSize = sizeof(int8_t);
            return true;
        case MI_IPU_FORMAT_U8:
            cvType = CV_8U;
            elemSize = sizeof(uint8_t);
            return true;
        default:
            return false;
        }
    }

    static MI_U32 invoke2TensorBufferSize(const MI_IPU_TensorDesc_t& desc) {
        if (desc.s32AlignedBufSize > 0) {
            return static_cast<MI_U32>(desc.s32AlignedBufSize);
        }
        return desc.u32BufSize;
    }

    void ensureLegacyTensors() {
        if (tensorsAcquired_) {
            return;
        }

        MI_S32 ret = MI_IPU_GetInputTensors(u32ChannelID_, &input_);
        if (ret != MI_SUCCESS) {
            throw std::runtime_error("MI_IPU_GetInputTensors failed, ret=" + std::to_string(ret));
        }
        ret = MI_IPU_GetOutputTensors(u32ChannelID_, &output_);
        if (ret != MI_SUCCESS) {
            MI_IPU_PutInputTensors(u32ChannelID_, &input_);
            throw std::runtime_error("MI_IPU_GetOutputTensors failed, ret=" + std::to_string(ret));
        }
        tensorsAcquired_ = true;
    }

    void allocTensorBuffer(MI_IPU_Tensor_t& tensor, MI_U32 size) {
        if (size == 0) {
            throw std::runtime_error("allocTensorBuffer() got zero size.");
        }

        MI_PHY phyAddr = 0;
        void* virAddr = nullptr;
        MI_S32 ret = MI_SYS_MMA_Alloc(0, NULL, size, &phyAddr);
        if (ret != MI_SUCCESS) {
            throw std::runtime_error("MI_SYS_MMA_Alloc tensor buffer failed, ret=" + std::to_string(ret));
        }

        ret = MI_SYS_Mmap(phyAddr, size, &virAddr, TRUE);
        if (ret != MI_SUCCESS) {
            MI_SYS_MMA_Free(0, phyAddr);
            throw std::runtime_error("MI_SYS_Mmap tensor buffer failed, ret=" + std::to_string(ret));
        }

        tensor.ptTensorData[0] = virAddr;
        tensor.phyTensorAddr[0] = phyAddr;
    }

    void freeTensorBuffer(MI_IPU_Tensor_t& tensor, MI_U32 size) noexcept {
        if (tensor.ptTensorData[0] != nullptr && size != 0) {
            MI_SYS_Munmap(tensor.ptTensorData[0], size);
        }
        if (tensor.phyTensorAddr[0] != 0) {
            MI_SYS_MMA_Free(0, tensor.phyTensorAddr[0]);
        }
        tensor.ptTensorData[0] = nullptr;
        tensor.phyTensorAddr[0] = 0;
    }

    void ensureInvoke2Resources() {
        if (invoke2ResourcesReady_) {
            return;
        }

        LOG_INFO(
            "IPUModel::ensureInvoke2Resources begin inputs=%u, outputs=%u, var_buf_size=%u",
            static_cast<unsigned>(net_.desc.u32InputTensorCount),
            static_cast<unsigned>(net_.desc.u32OutputTensorCount),
            static_cast<unsigned>(net_.OfflineModelInfo.u32VariableBufferSize)
        );
        std::memset(&invoke2_, 0, sizeof(invoke2_));
        std::memset(&runtime2_, 0, sizeof(runtime2_));
        invoke2_.u32BatchN = 1;
        // SDK 示例要求使用外部 user memory 时绑定 IPU core。
        invoke2_.u32IpuAffinity = IPU_DEV_0;
        invoke2_.u32VarBufSize = net_.OfflineModelInfo.u32VariableBufferSize;

        MI_S32 ret = MI_SUCCESS;
        if (invoke2_.u32VarBufSize != 0) {
            ret = MI_SYS_MMA_Alloc(0, NULL, invoke2_.u32VarBufSize, &invoke2VarPhyAddr_);
            if (ret != MI_SUCCESS) {
                LOG_WARNING("IPUModel::ensureInvoke2Resources alloc variable buffer failed ret=%d", static_cast<int>(ret));
                throw std::runtime_error("MI_SYS_MMA_Alloc Invoke2 variable buffer failed, ret=" + std::to_string(ret));
            }
            invoke2_.u64VarBufPhyAddr = invoke2VarPhyAddr_;
        }

        invoke2OutputSizes_.assign(net_.desc.u32OutputTensorCount, 0);
        std::uint64_t outputBytesPerSlot = 0;
        try {
            for (MI_U32 i = 0; i < net_.desc.u32OutputTensorCount; ++i) {
                invoke2OutputSizes_[i] = invoke2TensorBufferSize(net_.desc.astMI_OutputTensorDescs[i]);
                outputBytesPerSlot += invoke2OutputSizes_[i];
            }

            invoke2OutputSlots_.clear();
            invoke2OutputSlots_.resize(kInvoke2OutputPoolSize);
            for (std::size_t slotIndex = 0; slotIndex < invoke2OutputSlots_.size(); ++slotIndex) {
                auto& slot = invoke2OutputSlots_[slotIndex];
                slot.tensors.assign(net_.desc.u32OutputTensorCount, MI_IPU_Tensor_t());
                slot.sizes = invoke2OutputSizes_;
                slot.inUse = false;

                for (MI_U32 i = 0; i < net_.desc.u32OutputTensorCount; ++i) {
                    const MI_U32 size = invoke2OutputSizes_[i];
                    allocTensorBuffer(slot.tensors[i], size);
                }
            }
        } catch (...) {
            LOG_WARNING("IPUModel::ensureInvoke2Resources failed; releasing partial resources");
            releaseInvoke2Resources();
            throw;
        }

        invoke2ResourcesReady_ = true;
        LOG_INFO(
            "IPUModel::ensureInvoke2Resources done output_pool=%zu, outputs=%u, output_bytes_per_slot=%llu, var_buf_size=%u",
            invoke2OutputSlots_.size(),
            static_cast<unsigned>(net_.desc.u32OutputTensorCount),
            static_cast<unsigned long long>(outputBytesPerSlot),
            static_cast<unsigned>(invoke2_.u32VarBufSize)
        );
    }

    void releaseInvoke2Resources() noexcept {
        waitInvoke2OutputSlotsIdle();
        for (auto& slot : invoke2OutputSlots_) {
            for (std::size_t i = 0; i < slot.tensors.size(); ++i) {
                const MI_U32 size = i < slot.sizes.size() ? slot.sizes[i] : 0;
                freeTensorBuffer(slot.tensors[i], size);
            }
            slot.tensors.clear();
            slot.sizes.clear();
            slot.inUse = false;
        }
        invoke2OutputSlots_.clear();
        invoke2OutputSizes_.clear();

        if (invoke2VarPhyAddr_ != 0 && invoke2_.u32VarBufSize != 0) {
            MI_SYS_MMA_Free(0, invoke2VarPhyAddr_);
        }
        invoke2VarPhyAddr_ = 0;
        invoke2ResourcesReady_ = false;
        std::memset(&invoke2_, 0, sizeof(invoke2_));
        std::memset(&runtime2_, 0, sizeof(runtime2_));
    }

    void waitInvoke2OutputSlotsIdle() noexcept {
        std::unique_lock<std::mutex> lk(invoke2OutputMutex_);
        invoke2OutputCv_.wait(lk, [&] {
            for (const auto& slot : invoke2OutputSlots_) {
                if (slot.inUse) {
                    return false;
                }
            }
            return true;
        });
    }

    void releaseInvoke2OutputSlot(std::size_t slotIndex) noexcept {
        {
            std::lock_guard<std::mutex> lk(invoke2OutputMutex_);
            if (slotIndex < invoke2OutputSlots_.size()) {
                invoke2OutputSlots_[slotIndex].inUse = false;
            }
        }
        invoke2OutputCv_.notify_all();
    }

    std::shared_ptr<Invoke2OutputLease> acquireInvoke2OutputSlot() {
        std::unique_lock<std::mutex> lk(invoke2OutputMutex_);
        if (invoke2OutputSlots_.empty()) {
            throw std::runtime_error("Invoke2 output slot pool is not initialized.");
        }

        invoke2OutputCv_.wait(lk, [&] {
            for (const auto& slot : invoke2OutputSlots_) {
                if (!slot.inUse) {
                    return true;
                }
            }
            return false;
        });

        for (std::size_t i = 0; i < invoke2OutputSlots_.size(); ++i) {
            if (!invoke2OutputSlots_[i].inUse) {
                invoke2OutputSlots_[i].inUse = true;
                auto lease = std::make_shared<Invoke2OutputLease>();
                lease->model = this;
                lease->slotIndex = i;
                return lease;
            }
        }

        throw std::runtime_error("Invoke2 output slot wait finished without a free slot.");
    }

    void bindInvoke2OutputSlot(std::size_t slotIndex) {
        if (slotIndex >= invoke2OutputSlots_.size()) {
            throw std::runtime_error("Invalid Invoke2 output slot index.");
        }

        const auto& slot = invoke2OutputSlots_[slotIndex];
        if (slot.tensors.size() < net_.desc.u32OutputTensorCount) {
            throw std::runtime_error("Invoke2 output slot tensor count is smaller than model output count.");
        }

        const MI_U32 outputBase = net_.desc.u32InputTensorCount;
        for (MI_U32 i = 0; i < net_.desc.u32OutputTensorCount; ++i) {
            invoke2_.astArrayTensors[outputBase + i] = slot.tensors[i];
        }
    }

    void* mapExternalInput(MI_PHY phyAddr, MI_U32 size) {
        auto it = externalInputMaps_.find(phyAddr);
        if (it != externalInputMaps_.end()) {
            if (it->second.size >= size) {
                LOG_INFO(
                    "IPUModel::mapExternalInput cache hit phy=0x%llx, vir=%p, mapped_size=%u, requested=%u",
                    static_cast<unsigned long long>(phyAddr),
                    it->second.virAddr,
                    static_cast<unsigned>(it->second.size),
                    static_cast<unsigned>(size)
                );
                return it->second.virAddr;
            }
            LOG_INFO(
                "IPUModel::mapExternalInput remap phy=0x%llx because mapped_size=%u < requested=%u",
                static_cast<unsigned long long>(phyAddr),
                static_cast<unsigned>(it->second.size),
                static_cast<unsigned>(size)
            );
            MI_SYS_Munmap(it->second.virAddr, it->second.size);
            externalInputMaps_.erase(it);
        }

        void* virAddr = nullptr;
        LOG_INFO(
            "IPUModel::mapExternalInput mmap begin phy=0x%llx, size=%u",
            static_cast<unsigned long long>(phyAddr),
            static_cast<unsigned>(size)
        );
        MI_S32 ret = MI_SYS_Mmap(phyAddr, size, &virAddr, TRUE);
        if (ret != MI_SUCCESS) {
            LOG_WARNING("IPUModel::mapExternalInput mmap failed ret=%d", static_cast<int>(ret));
            throw std::runtime_error("MI_SYS_Mmap external input failed, ret=" + std::to_string(ret));
        }

        externalInputMaps_.emplace(phyAddr, ExternalInputMap{virAddr, size});
        LOG_INFO("IPUModel::mapExternalInput mmap done phy=0x%llx, vir=%p", static_cast<unsigned long long>(phyAddr), virAddr);
        return virAddr;
    }

    void releaseExternalInputMaps() noexcept {
        for (auto& item : externalInputMaps_) {
            if (item.second.virAddr != nullptr && item.second.size != 0) {
                MI_SYS_Munmap(item.second.virAddr, item.second.size);
            }
        }
        externalInputMaps_.clear();
    }

    std::vector<cv::Mat> readOutputTensors(const MI_IPU_Tensor_t* outputTensors,
                                           bool squeeze_batch) const {
        std::vector<cv::Mat> outs;
        outs.reserve(net_.desc.u32OutputTensorCount);

        for (MI_U32 i = 0; i < net_.desc.u32OutputTensorCount; ++i) {
            const auto& od = net_.desc.astMI_OutputTensorDescs[i];

            int cvType = 0;
            size_t elemSize = 0;
            if (!elemTypeAndSize(od.eElmFormat, cvType, elemSize)) {
                const int rawBytes = od.s32AlignedBufSize > 0
                                         ? od.s32AlignedBufSize
                                         : static_cast<int>(od.u32BufSize);
                cv::Mat raw(1, rawBytes, CV_8U);
                std::memcpy(raw.data,
                            outputTensors[i].ptTensorData[0],
                            static_cast<size_t>(rawBytes));
                outs.emplace_back(std::move(raw));
                continue;
            }

            int nd = static_cast<int>(od.u32TensorDim);
            std::vector<int> dims;
            dims.reserve(nd);
            for (int k = 0; k < nd; ++k) {
                dims.push_back(static_cast<int>(od.u32TensorShape[k]));
            }

            if (squeeze_batch && nd >= 1 && !dims.empty() && dims[0] == 1) {
                dims.erase(dims.begin());
                nd -= 1;
            }
            if (nd >= 1 && !dims.empty() && dims.back() == 1) {
                dims.pop_back();
                nd -= 1;
            }
            if (nd <= 0) {
                dims.clear();
                dims.push_back(1);
                nd = 1;
            }

            size_t logicalCount = 1;
            for (int d : dims) {
                logicalCount *= static_cast<size_t>(d);
            }
            const size_t logicalBytes = logicalCount * elemSize;

            cv::Mat outMat(nd, dims.data(), cvType);
            std::memcpy(outMat.data,
                        outputTensors[i].ptTensorData[0],
                        logicalBytes);

            outs.emplace_back(std::move(outMat));
        }

        return outs;
    }

    std::vector<cv::Mat> viewOutputTensors(const MI_IPU_Tensor_t* outputTensors,
                                           bool squeeze_batch) const {
        std::vector<cv::Mat> outs;
        outs.reserve(net_.desc.u32OutputTensorCount);

        for (MI_U32 i = 0; i < net_.desc.u32OutputTensorCount; ++i) {
            const auto& od = net_.desc.astMI_OutputTensorDescs[i];
            void* data = outputTensors[i].ptTensorData[0];
            if (data == nullptr) {
                throw std::runtime_error("viewOutputTensors() got null output tensor data.");
            }

            int cvType = 0;
            size_t elemSize = 0;
            if (!elemTypeAndSize(od.eElmFormat, cvType, elemSize)) {
                const int rawBytes = od.s32AlignedBufSize > 0
                                         ? od.s32AlignedBufSize
                                         : static_cast<int>(od.u32BufSize);
                outs.emplace_back(1, rawBytes, CV_8U, data);
                continue;
            }

            int nd = static_cast<int>(od.u32TensorDim);
            std::vector<int> dims;
            dims.reserve(nd);
            for (int k = 0; k < nd; ++k) {
                dims.push_back(static_cast<int>(od.u32TensorShape[k]));
            }

            if (squeeze_batch && nd >= 1 && !dims.empty() && dims[0] == 1) {
                dims.erase(dims.begin());
                nd -= 1;
            }
            if (nd >= 1 && !dims.empty() && dims.back() == 1) {
                dims.pop_back();
                nd -= 1;
            }
            if (nd <= 0) {
                dims.clear();
                dims.push_back(1);
                nd = 1;
            }

            outs.emplace_back(nd, dims.data(), cvType, data);
        }

        return outs;
    }

    static void printModelIOInfo(const MI_IPU_SubNet_InputOutputDesc_t& d) {
        LOG_INFO("=================== Model IO ===================");

        for (MI_U32 i = 0; i < d.u32InputTensorCount; ++i) {
            const auto& t = d.astMI_InputTensorDescs[i];

            std::string line;
            line.reserve(256);
            line += "Input[";
            line += std::to_string((unsigned)i);
            line += "] name=";
            line += t.name;
            line += ", fmt=";
            line += typeName(t.eElmFormat);
            line += ", dim=";
            line += std::to_string((unsigned)t.u32TensorDim);
            line += ", shape=[";

            for (MI_U32 k = 0; k < t.u32TensorDim; ++k) {
                line += std::to_string((unsigned)t.u32TensorShape[k]);
                if (k + 1 != t.u32TensorDim) line += ",";
            }
            line += "]";

            LOG_INFO("%s", line.c_str());
        }

        for (MI_U32 i = 0; i < d.u32OutputTensorCount; ++i) {
            const auto& t = d.astMI_OutputTensorDescs[i];

            std::string line;
            line.reserve(256);
            line += "Output[";
            line += std::to_string((unsigned)i);
            line += "] name=";
            line += t.name;
            line += ", fmt=";
            line += typeName(t.eElmFormat);
            line += ", dim=";
            line += std::to_string((unsigned)t.u32TensorDim);
            line += ", shape=[";

            for (MI_U32 k = 0; k < t.u32TensorDim; ++k) {
                line += std::to_string((unsigned)t.u32TensorShape[k]);
                if (k + 1 != t.u32TensorDim) line += ",";
            }
            line += "]";

            LOG_INFO("%s", line.c_str());
        }

        LOG_INFO("================================================");
    }

    void cleanup() noexcept {
        releaseInvoke2Resources();
        releaseExternalInputMaps();
        if (tensorsAcquired_) {
            MI_IPU_PutInputTensors(u32ChannelID_, &input_);
            MI_IPU_PutOutputTensors(u32ChannelID_, &output_);
            tensorsAcquired_ = false;
        }
        if (chnCreated_) {
            MI_IPU_DestroyCHN(u32ChannelID_);
            chnCreated_ = false;
        }
        if (devCreated_) {
            MI_IPU_DestroyDevice();
            devCreated_ = false;
        }
        if (sysInited_) {
            MI_SYS_Exit(0);
            sysInited_ = false;
        }
    }

    // 基本参数
    std::string modelFile_;
    bool dumpInputBin_{false};

    // IPU 资源
    MI_U32 u32ChannelID_{0};
    NetInfo net_{};
    MI_IPU_TensorVector_t input_{};
    MI_IPU_TensorVector_t output_{};
    MI_IPU_BatchInvokeParam_t invoke2_{};
    MI_IPU_RuntimeInfo_t runtime2_{};
    MI_PHY invoke2VarPhyAddr_{0};
    std::vector<MI_U32> invoke2OutputSizes_;
    std::vector<Invoke2OutputSlot> invoke2OutputSlots_;
    std::unordered_map<MI_PHY, ExternalInputMap> externalInputMaps_;
    std::mutex inferMutex_;
    std::mutex invoke2OutputMutex_;
    std::condition_variable invoke2OutputCv_;

    // 状态标记
    bool sysInited_{false};
    bool devCreated_{false};
    bool chnCreated_{false};
    bool tensorsAcquired_{false};
    bool invoke2ResourcesReady_{false};
};
