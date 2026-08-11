#pragma once

#include <TI/dl_kernels.h>
#include <TI/j7_tidl.h>
#include <TI/tivx.h>
#include <opencv2/core.hpp>
#include <tivx_utils_graph_perf.h>
#include <utils/app_init/include/app_init.h>
#include <utils/perf_stats/include/app_perf_stats.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

#include "common/logger.hpp"
#include "itidl_ti.h"

class TIDLModel
{
public:
    explicit TIDLModel(const std::string& model_path,
                       const std::string& target_name = std::string())
        : model_path_(model_path),
          target_name_(normalizeTargetName(target_name))
    {
        auto resolved = resolveModelPaths(model_path_);
        io_path_ = resolved.io_path;
        network_path_ = resolved.network_path;
        debugLog("ctor: resolved artifacts");
        debugLog("ctor: io_path=" + io_path_);
        debugLog("ctor: network_path=" + network_path_);
        debugLog("ctor: target_name=" + target_name_);

        debugLog("ctor: appInit begin");
        const int init_status = appInit();
        if (init_status != 0)
        {
            debugLog("ctor: appInit failed code=" + std::to_string(init_status));
            throw std::runtime_error("appInit failed, code=" + std::to_string(init_status));
        }
        app_inited_ = true;
        debugLog("ctor: appInit done");

        try
        {
            debugLog("ctor: initOpenVX begin");
            initOpenVX();
            debugLog("ctor: initOpenVX done");
            debugLog("ctor: printModelIOInfo begin");
            printModelIOInfo();
            debugLog("ctor: printModelIOInfo done");
        }
        catch (const std::exception& e)
        {
            debugLog(std::string("ctor: exception before cleanup: ") + e.what());
            cleanup();
            throw;
        }
        catch (...)
        {
            debugLog("ctor: unknown exception before cleanup");
            cleanup();
            throw;
        }
    }

    ~TIDLModel()
    {
        cleanup();
    }

    std::vector<cv::Mat> infer(const cv::Mat& input, bool squeeze_batch = false)
    {
        if (input.empty())
        {
            throw std::invalid_argument("input 为空。");
        }
        if (input.type() != CV_32F)
        {
            throw std::invalid_argument("input 必须为 CV_32F（float32）。");
        }

        const cv::Mat src = input.isContinuous() ? input : input.clone();
        auto lease = acquireGraphSlot();
        auto& slot = graph_slots_[lease->slotIndex];

        std::lock_guard<std::mutex> process_lock(graph_process_mutex_);
        appPerfPointBegin(&slot.total_perf);
        const auto total_t0 = Clock::now();
        const auto write_t0 = Clock::now();
        writeInputTensor(slot, src);
        const auto write_t1 = Clock::now();

        appPerfPointBegin(&slot.graph_perf);
        const auto graph_t0 = Clock::now();
        const vx_status status = vxProcessGraph(slot.graph);
        const auto graph_t1 = Clock::now();
        appPerfPointEnd(&slot.graph_perf);
        if (status != VX_SUCCESS)
        {
            appPerfPointEnd(&slot.total_perf);
            throw std::runtime_error("vxProcessGraph failed, status=" + std::to_string(status));
        }

        const auto read_t0 = Clock::now();
        auto outputs = readOutputTensors(slot, squeeze_batch);
        const auto read_t1 = Clock::now();
        appPerfPointEnd(&slot.total_perf);
        const auto total_t1 = Clock::now();

        printTiPerf(slot, write_t0, write_t1, graph_t0, graph_t1, read_t0, read_t1, total_t0, total_t1);
        return outputs;
    }

    std::vector<cv::Mat> inferWithOutputOwner(const cv::Mat& input,
                                              bool squeeze_batch,
                                              std::shared_ptr<void>& owner)
    {
        owner.reset();
        if (input.empty())
        {
            throw std::invalid_argument("input 为空。");
        }
        if (input.type() != CV_32F)
        {
            throw std::invalid_argument("input 必须为 CV_32F（float32）。");
        }

        const cv::Mat src = input.isContinuous() ? input : input.clone();
        auto lease = acquireGraphSlot();
        auto& slot = graph_slots_[lease->slotIndex];

        std::lock_guard<std::mutex> process_lock(graph_process_mutex_);
        appPerfPointBegin(&slot.total_perf);
        const auto total_t0 = Clock::now();
        const auto write_t0 = Clock::now();
        writeInputTensor(slot, src);
        const auto write_t1 = Clock::now();

        appPerfPointBegin(&slot.graph_perf);
        const auto graph_t0 = Clock::now();
        const vx_status status = vxProcessGraph(slot.graph);
        const auto graph_t1 = Clock::now();
        appPerfPointEnd(&slot.graph_perf);
        if (status != VX_SUCCESS)
        {
            appPerfPointEnd(&slot.total_perf);
            throw std::runtime_error("vxProcessGraph failed, status=" + std::to_string(status));
        }

        const auto read_t0 = Clock::now();
        auto outputs = readOutputTensors(slot, squeeze_batch);
        const auto read_t1 = Clock::now();
        appPerfPointEnd(&slot.total_perf);
        const auto total_t1 = Clock::now();

        printTiPerf(slot, write_t0, write_t1, graph_t0, graph_t1, read_t0, read_t1, total_t0, total_t1);
        owner.reset();
        return outputs;
    }

    struct PipelineStressResult
    {
        std::size_t submitted = 0;
        std::size_t completed = 0;
        double write_input_ms = 0.0;
        double schedule_ms = 0.0;
        double wait_graph_ms = 0.0;
        double read_output_ms = 0.0;
        double elapsed_ms = 0.0;
        double fps = 0.0;
    };

    PipelineStressResult pipelineStress(const std::vector<cv::Mat>& inputs,
                                        std::size_t total_tasks,
                                        std::size_t pipeline_depth,
                                        bool read_outputs)
    {
        if (inputs.empty())
        {
            throw std::invalid_argument("pipelineStress inputs is empty.");
        }
        if (total_tasks == 0U)
        {
            return {};
        }

        pipeline_depth = std::max<std::size_t>(1U, pipeline_depth);
        pipeline_depth = std::min<std::size_t>(pipeline_depth, graph_slots_.size());

        std::vector<std::shared_ptr<GraphSlotLease>> leases;
        leases.reserve(pipeline_depth);
        for (std::size_t i = 0; i < pipeline_depth; ++i)
        {
            leases.push_back(acquireGraphSlot());
        }

        std::lock_guard<std::mutex> process_lock(graph_process_mutex_);
        struct ActiveSlot
        {
            std::size_t lease_index = 0;
            std::size_t task_index = 0;
        };

        std::vector<ActiveSlot> active;
        active.reserve(pipeline_depth);

        std::size_t submitted = 0;
        std::size_t completed = 0;
        long long write_input_us = 0;
        long long schedule_us = 0;
        long long wait_graph_us = 0;
        long long read_output_us = 0;
        const bool print_load = true;
        appPerfPointReset(&stress_wall_perf_);
        for (const auto& lease : leases)
        {
            appPerfPointReset(&graph_slots_[lease->slotIndex].stress_frame_perf);
        }
        if (print_load)
        {
            appPerfStatsResetAll();
            std::fprintf(stderr, "[MIN_DET_TIDL][LOAD] stress window reset\n");
            std::fflush(stderr);
        }
        const auto begin = Clock::now();
        appPerfPointBegin(&stress_wall_perf_);

        auto submit = [&](std::size_t lease_index) {
            const cv::Mat& input = inputs[submitted % inputs.size()];
            if (input.empty())
            {
                throw std::invalid_argument("pipelineStress input contains empty cv::Mat.");
            }
            if (input.type() != CV_32F)
            {
                throw std::invalid_argument("pipelineStress input must be CV_32F.");
            }
            const cv::Mat src = input.isContinuous() ? input : input.clone();
            TiGraphSlot& slot = graph_slots_[leases[lease_index]->slotIndex];
            appPerfPointBegin(&slot.stress_frame_perf);
            const auto write_begin = Clock::now();
            writeInputTensor(slot, src);
            const auto write_end = Clock::now();
            write_input_us += durationUs(write_begin, write_end);
            const auto schedule_begin = Clock::now();
            const vx_status status = vxScheduleGraph(slot.graph);
            const auto schedule_end = Clock::now();
            schedule_us += durationUs(schedule_begin, schedule_end);
            if (status != VX_SUCCESS)
            {
                throw std::runtime_error("vxScheduleGraph failed, status=" + std::to_string(status));
            }
            active.push_back(ActiveSlot{lease_index, submitted});
            ++submitted;
        };

        while (submitted < total_tasks && active.size() < pipeline_depth)
        {
            submit(active.size());
        }

        while (!active.empty())
        {
            const ActiveSlot done = active.front();
            active.erase(active.begin());

            TiGraphSlot& slot = graph_slots_[leases[done.lease_index]->slotIndex];
            const auto wait_begin = Clock::now();
            const vx_status wait_status = vxWaitGraph(slot.graph);
            const auto wait_end = Clock::now();
            wait_graph_us += durationUs(wait_begin, wait_end);
            if (wait_status != VX_SUCCESS)
            {
                throw std::runtime_error("vxWaitGraph failed, status=" + std::to_string(wait_status));
            }

            if (read_outputs)
            {
                const auto read_begin = Clock::now();
                auto outputs = readOutputTensors(slot, /*squeeze_batch=*/true);
                const auto read_end = Clock::now();
                read_output_us += durationUs(read_begin, read_end);
                (void)outputs;
            }
            ++completed;
            appPerfPointEnd(&slot.stress_frame_perf);
            appPerfPointEnd(&stress_wall_perf_);
            if (completed < total_tasks)
            {
                appPerfPointBegin(&stress_wall_perf_);
            }

            if (submitted < total_tasks)
            {
                submit(done.lease_index);
            }
        }

        const auto end = Clock::now();
        const double elapsed_ms = static_cast<double>(durationUs(begin, end)) / 1000.0;
        PipelineStressResult result;
        result.submitted = submitted;
        result.completed = completed;
        result.write_input_ms = static_cast<double>(write_input_us) / 1000.0;
        result.schedule_ms = static_cast<double>(schedule_us) / 1000.0;
        result.wait_graph_ms = static_cast<double>(wait_graph_us) / 1000.0;
        result.read_output_ms = static_cast<double>(read_output_us) / 1000.0;
        result.elapsed_ms = elapsed_ms;
        result.fps = elapsed_ms > 0.0
            ? static_cast<double>(completed) * 1000.0 / elapsed_ms
            : 0.0;
        if (print_load)
        {
            std::fprintf(stderr, "[MIN_DET_TIDL][LOAD] appPerfStatsPrintAll begin\n");
            (void)appPerfStatsPrintAll();
            std::fprintf(stderr, "[MIN_DET_TIDL][LOAD] appPerfStatsPrintAll end\n");

            std::fprintf(stderr, "[MIN_DET_TIDL][OFFICIAL] appPerfPointPrint begin\n");
            appPerfPointPrint(&stress_wall_perf_);
            appPerfPointPrintFPS(&stress_wall_perf_);
            for (const auto& lease : leases)
            {
                TiGraphSlot& slot = graph_slots_[lease->slotIndex];
                std::fprintf(stderr,
                             "[MIN_DET_TIDL][OFFICIAL] slot=%zu appPerfPointPrint begin\n",
                             lease->slotIndex);
                appPerfPointPrint(&slot.stress_frame_perf);
                appPerfPointPrintFPS(&slot.stress_frame_perf);
            }
            std::fprintf(stderr, "[MIN_DET_TIDL][OFFICIAL] appPerfPointPrint end\n");

            for (const auto& lease : leases)
            {
                TiGraphSlot& slot = graph_slots_[lease->slotIndex];
                std::fprintf(stderr,
                             "[MIN_DET_TIDL][OFFICIAL] slot=%zu tivx_utils_graph_perf_print begin\n",
                             lease->slotIndex);
                const vx_status graph_perf_status = tivx_utils_graph_perf_print(slot.graph);
                std::fprintf(stderr,
                             "[MIN_DET_TIDL][OFFICIAL] slot=%zu tivx_utils_graph_perf_print status=%d\n",
                             lease->slotIndex,
                             static_cast<int>(graph_perf_status));
            }
            std::fflush(stderr);
        }
        return result;
    }

    void smokeTest(int nTimes)
    {
        if (nTimes <= 0)
        {
            LOG_INFO("[TIDLModel::SmokeTest] nTimes <= 0, nothing to do.");
            return;
        }

        const auto input_shape = expectedInputShape();
        cv::Mat input(static_cast<int>(input_shape.size()), input_shape.data(), CV_32F);

        std::mt19937 rng(1234);
        std::uniform_real_distribution<float> dist(0.0f, 255.0f);
        float* data = reinterpret_cast<float*>(input.data);
        for (size_t i = 0; i < input.total(); ++i)
        {
            data[i] = dist(rng);
        }

        LOG_INFO("[TIDLModel::SmokeTest] Created random float32 input.");

        long long totalUs = 0;
        long long minUs = std::numeric_limits<long long>::max();
        long long maxUs = 0;

        for (int i = 0; i < nTimes; ++i)
        {
            const auto t0 = std::chrono::high_resolution_clock::now();
            auto outs = infer(input, true);
            const auto t1 = std::chrono::high_resolution_clock::now();

            const long long us =
                std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

            totalUs += us;
            minUs = std::min(minUs, us);
            maxUs = std::max(maxUs, us);

            LOG_INFO("[TIDLModel::SmokeTest] Iter %d done, time = %.3f ms, output tensor count = %d",
                     i,
                     static_cast<double>(us) / 1000.0,
                     static_cast<int>(outs.size()));

        }

        LOG_INFO("[TIDLModel::SmokeTest] All %d iterations finished.", nTimes);
        LOG_INFO("[TIDLModel::SmokeTest] Avg = %.3f ms, Min = %.3f ms, Max = %.3f ms",
                 static_cast<double>(totalUs) / static_cast<double>(nTimes) / 1000.0,
                 static_cast<double>(minUs) / 1000.0,
                 static_cast<double>(maxUs) / 1000.0);
    }

    std::vector<std::string> outputNames() const
    {
        std::vector<std::string> names;
        names.reserve(num_outputs_);
        for (uint32_t i = 0; i < num_outputs_; ++i)
        {
            names.push_back(outputName(i));
        }
        return names;
    }

    bool perfHandles(void** graph, void** total_perf, void** graph_perf)
    {
        if (graph == nullptr || total_perf == nullptr || graph_perf == nullptr)
        {
            return false;
        }

        std::lock_guard<std::mutex> lk(graph_slot_mutex_);
        if (graph_slots_.empty() || graph_slots_.front().graph == nullptr)
        {
            *graph = nullptr;
            *total_perf = nullptr;
            *graph_perf = nullptr;
            return false;
        }

        *graph = reinterpret_cast<void*>(graph_slots_.front().graph);
        *total_perf = reinterpret_cast<void*>(&graph_slots_.front().total_perf);
        *graph_perf = reinterpret_cast<void*>(&graph_slots_.front().graph_perf);
        return true;
    }

private:
    using Clock = std::chrono::high_resolution_clock;

    static void debugLog(const std::string& message)
    {
        if (!envFlagEnabled("VISPER_TI_DEBUG_LOG", false))
        {
            return;
        }
        std::fprintf(stderr, "[MIN_DET_TIDL] %s\n", message.c_str());
        std::fflush(stderr);
    }

    static void debugLogStatus(const std::string& stage, vx_status status)
    {
        debugLog(stage + " status=" + std::to_string(static_cast<int>(status)));
    }

    struct ResolvedModelPaths
    {
        std::string io_path;
        std::string network_path;
    };

    enum class TensorOrder
    {
        NCHW,
        NHWC
    };

    static constexpr uint32_t kMaxParams = 16U;
    static constexpr std::size_t kDefaultGraphSlotPoolSize = 1U;
    static constexpr std::size_t kMaxGraphSlotPoolSize = 3U;

    static std::string trimString(const std::string& value)
    {
        std::size_t begin = 0;
        while (begin < value.size() &&
               std::isspace(static_cast<unsigned char>(value[begin])) != 0)
        {
            ++begin;
        }

        std::size_t end = value.size();
        while (end > begin &&
               std::isspace(static_cast<unsigned char>(value[end - 1])) != 0)
        {
            --end;
        }

        return value.substr(begin, end - begin);
    }

    static std::string normalizeTargetName(const std::string& target_name)
    {
        const std::string trimmed = trimString(target_name);
        if (trimmed.empty())
        {
            return TIVX_TARGET_DSP_C7_2;
        }

        if (trimmed == "DSP_C7-1")
        {
            return TIVX_TARGET_DSP_C7_1;
        }

        if (trimmed == "DSP_C7-2")
        {
            return TIVX_TARGET_DSP_C7_2;
        }

        throw std::invalid_argument(
            "Unsupported TI TIDL target: " + target_name +
            ". Supported values: DSP_C7-1, DSP_C7-2.");
    }

    static bool shouldPrintTiPerf()
    {
        return envFlagEnabled("VISPER_TI_PRINT_PERF", false);
    }

    static bool shouldPrintTiLoad()
    {
        return envFlagEnabled("VISPER_TI_PRINT_LOAD", false);
    }

    static uint64_t tiPerfPrintEvery()
    {
        const char* env = std::getenv("VISPER_TI_PRINT_PERF_EVERY");
        if (env == nullptr || env[0] == '\0')
        {
            return 1U;
        }

        const std::string value = trimString(env);
        char* end = nullptr;
        const unsigned long long parsed = std::strtoull(value.c_str(), &end, 10);
        if (end == value.c_str() || parsed == 0U)
        {
            return 1U;
        }
        return static_cast<uint64_t>(parsed);
    }

    static std::size_t graphSlotPoolSize()
    {
        const char* env = std::getenv("VISPER_TI_GRAPH_SLOTS");
        if (env == nullptr || env[0] == '\0')
        {
            return kDefaultGraphSlotPoolSize;
        }

        const std::string value = trimString(env);
        char* end = nullptr;
        const unsigned long long parsed = std::strtoull(value.c_str(), &end, 10);
        if (end == value.c_str() || parsed == 0U)
        {
            return kDefaultGraphSlotPoolSize;
        }
        return std::min<std::size_t>(kMaxGraphSlotPoolSize, static_cast<std::size_t>(parsed));
    }

    static bool envFlagEnabled(const char* name, bool fallback)
    {
        const char* env = std::getenv(name);
        if (env == nullptr || env[0] == '\0')
        {
            return fallback;
        }
        const std::string value = trimString(env);
        return value == "1" || value == "true" || value == "TRUE" || value == "on" || value == "ON" ||
               value == "yes" || value == "YES";
    }

    static long long durationUs(Clock::time_point begin, Clock::time_point end)
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
    }

    struct TiGraphSlot
    {
        vx_graph graph{nullptr};
        vx_node tidl_node{nullptr};
        vx_user_data_object create_params{nullptr};
        vx_user_data_object in_args{nullptr};
        vx_user_data_object out_args{nullptr};
        std::vector<vx_tensor> input_tensors;
        std::vector<vx_tensor> output_tensors;
        app_perf_point_t total_perf{};
        app_perf_point_t graph_perf{};
        app_perf_point_t stress_frame_perf{};
        bool in_use{false};
    };

    struct GraphSlotLease
    {
        TIDLModel* model{nullptr};
        std::size_t slotIndex{0};

        ~GraphSlotLease()
        {
            if (model != nullptr)
            {
                model->releaseGraphSlot(slotIndex);
            }
        }
    };

    static vx_size getOpenVXType(int32_t tidl_type)
    {
        if (tidl_type == TIDL_UnsignedChar) return VX_TYPE_UINT8;
        if (tidl_type == TIDL_SignedChar) return VX_TYPE_INT8;
        if (tidl_type == TIDL_UnsignedShort) return VX_TYPE_UINT16;
        if (tidl_type == TIDL_SignedShort) return VX_TYPE_INT16;
        if (tidl_type == TIDL_UnsignedWord) return VX_TYPE_UINT32;
        if (tidl_type == TIDL_SignedWord) return VX_TYPE_INT32;
        if (tidl_type == TIDL_SinglePrecFloat) return VX_TYPE_FLOAT32;
        return VX_TYPE_INVALID;
    }

    static size_t elementSize(vx_size data_type)
    {
        switch (data_type)
        {
        case VX_TYPE_INT8:
        case VX_TYPE_UINT8:
            return sizeof(vx_int8);
        case VX_TYPE_INT16:
        case VX_TYPE_UINT16:
            return sizeof(vx_int16);
        case VX_TYPE_INT32:
        case VX_TYPE_UINT32:
            return sizeof(vx_int32);
        case VX_TYPE_FLOAT32:
            return sizeof(vx_float32);
        default:
            return 0U;
        }
    }

    static bool containsToken(const std::filesystem::path& path, const std::string& token)
    {
        return path.filename().string().find(token) != std::string::npos;
    }

    static bool isNetworkArtifact(const std::filesystem::path& path)
    {
        return containsToken(path, "tidl_net") || containsToken(path, "_net");
    }

    static bool isIoArtifact(const std::filesystem::path& path)
    {
        return containsToken(path, "tidl_io") || containsToken(path, "_io");
    }

    static std::vector<std::filesystem::path> pairedArtifactCandidates(
        const std::filesystem::path& path,
        bool input_is_network)
    {
        namespace fs = std::filesystem;

        const std::string filename = path.filename().string();
        const fs::path parent = path.has_parent_path() ? path.parent_path() : fs::current_path();

        const std::vector<std::pair<std::string, std::string>> replacements = input_is_network
            ? std::vector<std::pair<std::string, std::string>>{
                {"tidl_net", "tidl_io"},
                {"tidl_net", "tidl_io_1"},
                {"_net", "_io"}}
            : std::vector<std::pair<std::string, std::string>>{
                {"tidl_io_1", "tidl_net"},
                {"tidl_io", "tidl_net"},
                {"_io", "_net"}};

        std::vector<fs::path> candidates;
        for (const auto& [from, to] : replacements)
        {
            const std::size_t pos = filename.find(from);
            if (pos == std::string::npos)
            {
                continue;
            }

            std::string paired = filename;
            paired.replace(pos, from.size(), to);
            candidates.push_back(parent / paired);
        }

        std::sort(candidates.begin(), candidates.end());
        candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());
        return candidates;
    }

    static std::filesystem::path findExistingPairedArtifact(const std::filesystem::path& path,
                                                            bool input_is_network)
    {
        for (const auto& candidate : pairedArtifactCandidates(path, input_is_network))
        {
            if (std::filesystem::is_regular_file(candidate))
            {
                return candidate;
            }
        }
        return {};
    }

    static std::string joinCandidates(const std::vector<std::filesystem::path>& paths)
    {
        std::ostringstream oss;
        for (size_t i = 0; i < paths.size(); ++i)
        {
            oss << paths[i].filename().string();
            if (i + 1 != paths.size()) oss << ", ";
        }
        return oss.str();
    }

    static std::vector<std::filesystem::path> collectModelBins(const std::filesystem::path& dir,
                                                               const std::string& token)
    {
        std::vector<std::filesystem::path> out;
        if (!std::filesystem::exists(dir) || !std::filesystem::is_directory(dir))
        {
            return out;
        }

        for (const auto& entry : std::filesystem::directory_iterator(dir))
        {
            if (!entry.is_regular_file()) continue;
            const auto& path = entry.path();
            if (path.extension() == ".bin" && containsToken(path, token))
            {
                out.push_back(path);
            }
        }
        std::sort(out.begin(), out.end());
        return out;
    }

    static std::vector<ResolvedModelPaths> collectModelPairs(const std::filesystem::path& dir)
    {
        std::vector<ResolvedModelPaths> out;
        if (!std::filesystem::exists(dir) || !std::filesystem::is_directory(dir))
        {
            return out;
        }

        for (const auto& entry : std::filesystem::directory_iterator(dir))
        {
            if (!entry.is_regular_file()) continue;

            const auto& path = entry.path();
            if (path.extension() != ".bin" || !isNetworkArtifact(path))
            {
                continue;
            }

            const auto io_path = findExistingPairedArtifact(path, true);
            if (!io_path.empty())
            {
                out.push_back({io_path.string(), path.string()});
            }
        }

        std::sort(out.begin(), out.end(), [](const auto& a, const auto& b) {
            return a.network_path < b.network_path;
        });
        return out;
    }

    static std::string joinPairs(const std::vector<ResolvedModelPaths>& pairs)
    {
        std::ostringstream oss;
        for (size_t i = 0; i < pairs.size(); ++i)
        {
            oss << "(" << std::filesystem::path(pairs[i].io_path).filename().string()
                << ", " << std::filesystem::path(pairs[i].network_path).filename().string() << ")";
            if (i + 1 != pairs.size()) oss << ", ";
        }
        return oss.str();
    }

    static ResolvedModelPaths resolveModelPaths(const std::string& model_path)
    {
        namespace fs = std::filesystem;

        const fs::path input_path(model_path);
        const fs::path dir = fs::is_directory(input_path)
            ? input_path
            : (input_path.has_parent_path() ? input_path.parent_path() : fs::current_path());

        fs::path io_path;
        fs::path network_path;

        if (fs::is_regular_file(input_path))
        {
            const bool input_is_io = isIoArtifact(input_path);
            const bool input_is_network = isNetworkArtifact(input_path);

            if (!input_is_io && !input_is_network)
            {
                throw std::runtime_error(
                    "Explicit TI artifact file must be named like *_net.bin, *_io.bin, "
                    "*tidl_net*.bin, or *tidl_io*.bin: " + input_path.string());
            }

            if (input_is_network)
            {
                network_path = input_path;
                io_path = findExistingPairedArtifact(input_path, true);
            }
            else
            {
                io_path = input_path;
                network_path = findExistingPairedArtifact(input_path, false);
            }

            if (io_path.empty() || network_path.empty())
            {
                const auto expected = pairedArtifactCandidates(input_path, input_is_network);
                std::ostringstream oss;
                oss << "Failed to resolve paired TI artifact for explicit path: "
                    << input_path.string() << ". Expected candidates=["
                    << joinCandidates(expected) << "]";
                throw std::runtime_error(oss.str());
            }

            return {io_path.string(), network_path.string()};
        }

        const auto pairs = collectModelPairs(dir);
        const auto io_candidates = collectModelBins(dir, "_io");
        const auto net_candidates = collectModelBins(dir, "_net");

        if (pairs.size() == 1U && io_candidates.size() == 1U && net_candidates.size() == 1U)
        {
            return pairs.front();
        }

        if (pairs.size() > 1U)
        {
            std::ostringstream oss;
            oss << "Multiple TI model artifact pairs found in directory: " << dir.string()
                << ". Please pass an explicit *_net.bin or *_io.bin path. pairs=["
                << joinPairs(pairs) << "]";
            throw std::runtime_error(oss.str());
        }

        if (pairs.size() == 1U)
        {
            std::ostringstream oss;
            oss << "Directory contains one complete TI model artifact pair plus extra TI artifacts: "
                << dir.string()
                << ". Please pass an explicit *_net.bin or *_io.bin path. pairs=["
                << joinPairs(pairs) << "], io candidates=["
                << joinCandidates(io_candidates) << "], net candidates=["
                << joinCandidates(net_candidates) << "]";
            throw std::runtime_error(oss.str());
        }

        if (io_path.empty() || network_path.empty())
        {
            std::ostringstream oss;
            oss << "Failed to resolve TIDL model artifacts from path: " << model_path
                << ". Directory searched: " << dir.string()
                << ". io candidates=[" << joinCandidates(io_candidates)
                << "], net candidates=[" << joinCandidates(net_candidates) << "]";
            throw std::runtime_error(oss.str());
        }

        return {io_path.string(), network_path.string()};
    }

    static std::vector<uint8_t> readBinaryFile(const std::string& path)
    {
        std::ifstream ifs(path, std::ios::binary);
        if (!ifs)
        {
            throw std::runtime_error("无法打开二进制文件: " + path);
        }

        ifs.seekg(0, std::ios::end);
        const std::streamsize size = ifs.tellg();
        if (size < 0)
        {
            throw std::runtime_error("读取文件大小失败: " + path);
        }
        ifs.seekg(0, std::ios::beg);

        std::vector<uint8_t> data(static_cast<size_t>(size));
        if (!ifs.read(reinterpret_cast<char*>(data.data()), size))
        {
            throw std::runtime_error("读取二进制文件失败: " + path);
        }
        return data;
    }

    static std::string shapeToString(const std::vector<int>& shape)
    {
        std::ostringstream oss;
        oss << "[";
        for (size_t i = 0; i < shape.size(); ++i)
        {
            oss << shape[i];
            if (i + 1 != shape.size()) oss << ",";
        }
        oss << "]";
        return oss.str();
    }

    static const char* layoutName(int32_t layout)
    {
        return layout == TIDL_LT_NHWC ? "NHWC" : "NCHW";
    }

    static const char* typeName(int32_t type)
    {
        switch (type)
        {
        case TIDL_UnsignedChar: return "U8";
        case TIDL_SignedChar: return "S8";
        case TIDL_UnsignedShort: return "U16";
        case TIDL_SignedShort: return "S16";
        case TIDL_UnsignedWord: return "U32";
        case TIDL_SignedWord: return "S32";
        case TIDL_SinglePrecFloat: return "F32";
        default: return "UNKNOWN";
        }
    }

    static std::string tidlNameToString(const int8_t* name, size_t max_len)
    {
        std::string out;
        for (size_t i = 0; i < max_len; ++i)
        {
            const int8_t ch = name[i];
            if (ch == 0) break;
            out.push_back((ch >= 32 && ch <= 126) ? static_cast<char>(ch) : '?');
        }
        return out.empty() ? "-" : out;
    }

    template <typename T>
    static T clampCast(long long value)
    {
        const long long lo = static_cast<long long>(std::numeric_limits<T>::lowest());
        const long long hi = static_cast<long long>(std::numeric_limits<T>::max());
        return static_cast<T>(std::min(std::max(value, lo), hi));
    }

    template <typename T>
    static T quantizeValue(float value, float scale, int32_t zero_point)
    {
        if constexpr (std::is_same_v<T, float>)
        {
            return value;
        }
        else
        {
            const float scaled = (scale == 0.0f)
                ? value
                : value * scale + static_cast<float>(zero_point);
            return clampCast<T>(static_cast<long long>(std::llround(scaled)));
        }
    }

    template <typename T>
    static float dequantizeValue(T value, float scale, int32_t zero_point)
    {
        if constexpr (std::is_same_v<T, float>)
        {
            return value;
        }
        else
        {
            if (scale == 0.0f)
            {
                return static_cast<float>(value);
            }
            return (static_cast<float>(value) - static_cast<float>(zero_point)) / scale;
        }
    }

    static bool isIdentityTensorScale(float scale, int32_t zero_point)
    {
        return zero_point == 0 && std::abs(scale - 1.0f) <= 1e-6f;
    }

    void initOpenVX()
    {
        debugLog("initOpenVX: vxCreateContext begin");
        context_ = vxCreateContext();
        const vx_status context_status = vxGetStatus(reinterpret_cast<vx_reference>(context_));
        debugLogStatus("initOpenVX: vxCreateContext", context_status);
        if (context_status != VX_SUCCESS)
        {
            throw std::runtime_error("vxCreateContext failed.");
        }

        debugLog("initOpenVX: readConfig begin");
        readConfig();
        debugLog("initOpenVX: readConfig done");
        debugLog("initOpenVX: readNetwork begin");
        readNetwork();
        debugLog("initOpenVX: readNetwork done");

        debugLog("initOpenVX: ensureSupportedModel begin");
        ensureSupportedModel();
        debugLog("initOpenVX: ensureSupportedModel done");

        debugLog("initOpenVX: tivxTIDLLoadKernels begin");
        tivxTIDLLoadKernels(context_);
        tidl_kernels_loaded_ = true;
        debugLog("initOpenVX: tivxTIDLLoadKernels done");

        debugLog("initOpenVX: tivxAddKernelTIDL begin, inputs=" +
                 std::to_string(num_inputs_) + ", outputs=" + std::to_string(num_outputs_));
        tidl_kernel_ = tivxAddKernelTIDL(context_, num_inputs_, num_outputs_);
        const vx_status kernel_status = vxGetStatus(reinterpret_cast<vx_reference>(tidl_kernel_));
        debugLogStatus("initOpenVX: tivxAddKernelTIDL", kernel_status);
        if (kernel_status != VX_SUCCESS)
        {
            throw std::runtime_error("tivxAddKernelTIDL failed.");
        }

        debugLog("initOpenVX: createGraphSlots begin");
        createGraphSlots();
        debugLog("initOpenVX: createGraphSlots done");

        appPerfStatsResetAll();
        for (auto& slot : graph_slots_)
        {
            appPerfPointSetName(&slot.total_perf, "VISPER_TOTAL");
            appPerfPointSetName(&slot.graph_perf, "VISPER_TIDL_GRAPH");
            appPerfPointSetName(&slot.stress_frame_perf, "VISPER_STR_FRAME");
        }
        appPerfPointSetName(&stress_wall_perf_, "VISPER_STR_FPS");
    }

    void readConfig()
    {
        debugLog("readConfig: readBinaryFile begin path=" + io_path_);
        const auto data = readBinaryFile(io_path_);
        debugLog("readConfig: readBinaryFile done bytes=" + std::to_string(data.size()) +
                 ", expected=" + std::to_string(sizeof(sTIDL_IOBufDesc_t)));
        if (data.size() != sizeof(sTIDL_IOBufDesc_t))
        {
            throw std::runtime_error("Unexpected TIDL IO config size: " + io_path_);
        }

        debugLog("readConfig: vxCreateUserDataObject(tivxTIDLJ7Params) begin");
        config_ = vxCreateUserDataObject(
            context_, "tivxTIDLJ7Params", sizeof(tivxTIDLJ7Params), nullptr);
        const vx_status config_status = vxGetStatus(reinterpret_cast<vx_reference>(config_));
        debugLogStatus("readConfig: vxCreateUserDataObject(tivxTIDLJ7Params)", config_status);
        if (config_status != VX_SUCCESS)
        {
            config_ = nullptr;
            throw std::runtime_error("vxCreateUserDataObject(tivxTIDLJ7Params) failed.");
        }

        vx_map_id map_id = 0;
        tivxTIDLJ7Params* params = nullptr;
        const vx_status status = vxMapUserDataObject(
            config_,
            0,
            sizeof(tivxTIDLJ7Params),
            &map_id,
            reinterpret_cast<void**>(&params),
            VX_WRITE_ONLY,
            VX_MEMORY_TYPE_HOST,
            0);
        debugLogStatus("readConfig: vxMapUserDataObject(config)", status);
        if (status != VX_SUCCESS || params == nullptr)
        {
            throw std::runtime_error("vxMapUserDataObject(config) failed.");
        }

        tivx_tidl_j7_params_init(params);
        params->optimize_ivision_activation = 1;
        std::memcpy(&params->ioBufDesc, data.data(), data.size());
        std::memcpy(&io_desc_, &params->ioBufDesc, sizeof(io_desc_));
        num_inputs_ = static_cast<uint32_t>(io_desc_.numInputBuf);
        num_outputs_ = static_cast<uint32_t>(io_desc_.numOutputBuf);
        debugLog("readConfig: parsed num_inputs=" + std::to_string(num_inputs_) +
                 ", num_outputs=" + std::to_string(num_outputs_));

        debugLog("readConfig: vxUnmapUserDataObject(config) begin");
        vxUnmapUserDataObject(config_, map_id);
        debugLog("readConfig: vxUnmapUserDataObject(config) done");
    }

    void readNetwork()
    {
        debugLog("readNetwork: readBinaryFile begin path=" + network_path_);
        const auto data = readBinaryFile(network_path_);
        debugLog("readNetwork: readBinaryFile done bytes=" + std::to_string(data.size()));

        debugLog("readNetwork: vxCreateUserDataObject(TIDL_network) begin");
        network_ = vxCreateUserDataObject(
            context_, "TIDL_network", static_cast<vx_size>(data.size()), nullptr);

        const vx_status network_status = vxGetStatus(reinterpret_cast<vx_reference>(network_));
        debugLogStatus("readNetwork: vxCreateUserDataObject(TIDL_network)", network_status);
        if (network_status != VX_SUCCESS)
        {
            network_ = nullptr;
            throw std::runtime_error("vxCreateUserDataObject(TIDL_network) failed.");
        }

        vx_map_id map_id = 0;
        void* buffer = nullptr;
        const vx_status status = vxMapUserDataObject(
            network_,
            0,
            static_cast<vx_size>(data.size()),
            &map_id,
            &buffer,
            VX_WRITE_ONLY,
            VX_MEMORY_TYPE_HOST,
            0);
        debugLogStatus("readNetwork: vxMapUserDataObject(network)", status);
        if (status != VX_SUCCESS || buffer == nullptr)
        {
            throw std::runtime_error("vxMapUserDataObject(network) failed.");
        }

        std::memcpy(buffer, data.data(), data.size());
        debugLog("readNetwork: vxUnmapUserDataObject(network) begin");
        vxUnmapUserDataObject(network_, map_id);
        debugLog("readNetwork: vxUnmapUserDataObject(network) done");
    }

    vx_user_data_object createTidlCreateParams()
    {
        vx_user_data_object obj = vxCreateUserDataObject(
            context_, "TIDL_CreateParams", sizeof(TIDL_CreateParams), nullptr);
        if (vxGetStatus(reinterpret_cast<vx_reference>(obj)) != VX_SUCCESS)
        {
            return nullptr;
        }

        vx_map_id map_id = 0;
        TIDL_CreateParams* params = nullptr;
        const vx_status status = vxMapUserDataObject(
            obj,
            0,
            sizeof(TIDL_CreateParams),
            &map_id,
            reinterpret_cast<void**>(&params),
            VX_WRITE_ONLY,
            VX_MEMORY_TYPE_HOST,
            0);
        if (status != VX_SUCCESS || params == nullptr)
        {
            vxReleaseUserDataObject(&obj);
            return nullptr;
        }

        TIDL_createParamsInit(params);
        params->isInbufsPaded = 1;
        params->quantRangeExpansionFactor = 1.0f;
        params->quantRangeUpdateFactor = 0.0f;
        params->traceLogLevel = 0;
        params->traceWriteLevel = 0;

        vxUnmapUserDataObject(obj, map_id);
        return obj;
    }

    vx_user_data_object createTidlInArgs()
    {
        vx_user_data_object obj =
            vxCreateUserDataObject(context_, "TIDL_InArgs", sizeof(TIDL_InArgs), nullptr);
        if (vxGetStatus(reinterpret_cast<vx_reference>(obj)) != VX_SUCCESS)
        {
            return nullptr;
        }

        vx_map_id map_id = 0;
        TIDL_InArgs* params = nullptr;
        const vx_status status = vxMapUserDataObject(
            obj,
            0,
            sizeof(TIDL_InArgs),
            &map_id,
            reinterpret_cast<void**>(&params),
            VX_WRITE_ONLY,
            VX_MEMORY_TYPE_HOST,
            0);
        if (status != VX_SUCCESS || params == nullptr)
        {
            vxReleaseUserDataObject(&obj);
            return nullptr;
        }

        params->iVisionInArgs.size = sizeof(TIDL_InArgs);
        params->iVisionInArgs.subFrameInfo = 0;
        params->numInBufs = static_cast<int32_t>(num_inputs_);
        for (uint32_t i = 0; i < num_inputs_; ++i)
        {
            params->scale[i] = io_desc_.inTensorScale[i];
        }

        vxUnmapUserDataObject(obj, map_id);
        return obj;
    }

    vx_user_data_object createTidlOutArgs()
    {
        vx_user_data_object obj =
            vxCreateUserDataObject(context_, "TIDL_outArgs", sizeof(TIDL_outArgs), nullptr);
        if (vxGetStatus(reinterpret_cast<vx_reference>(obj)) != VX_SUCCESS)
        {
            return nullptr;
        }

        vx_map_id map_id = 0;
        TIDL_outArgs* params = nullptr;
        const vx_status status = vxMapUserDataObject(
            obj,
            0,
            sizeof(TIDL_outArgs),
            &map_id,
            reinterpret_cast<void**>(&params),
            VX_WRITE_ONLY,
            VX_MEMORY_TYPE_HOST,
            0);
        if (status != VX_SUCCESS || params == nullptr)
        {
            vxReleaseUserDataObject(&obj);
            return nullptr;
        }

        params->iVisionOutArgs.size = sizeof(TIDL_outArgs);
        params->numOutBufs = static_cast<int32_t>(num_outputs_);

        vxUnmapUserDataObject(obj, map_id);
        return obj;
    }

    std::array<vx_size, 3> tensorShapeInput(uint32_t id) const
    {
        if (io_desc_.inLayout[id] == TIDL_LT_NCHW)
        {
            return {
                static_cast<vx_size>(io_desc_.inWidth[id] + io_desc_.inPadL[id] + io_desc_.inPadR[id]),
                static_cast<vx_size>(io_desc_.inHeight[id] + io_desc_.inPadT[id] + io_desc_.inPadB[id]),
                static_cast<vx_size>(io_desc_.inNumChannels[id])
            };
        }

        return {
            static_cast<vx_size>(io_desc_.inNumChannels[id]),
            static_cast<vx_size>(io_desc_.inWidth[id] + io_desc_.inPadL[id] + io_desc_.inPadR[id]),
            static_cast<vx_size>(io_desc_.inHeight[id] + io_desc_.inPadT[id] + io_desc_.inPadB[id])
        };
    }

    std::array<vx_size, 3> tensorShapeOutput(uint32_t id) const
    {
        if (io_desc_.outLayout[id] == TIDL_LT_NCHW)
        {
            return {
                static_cast<vx_size>(io_desc_.outWidth[id] + io_desc_.outPadL[id] + io_desc_.outPadR[id]),
                static_cast<vx_size>(io_desc_.outHeight[id] + io_desc_.outPadT[id] + io_desc_.outPadB[id]),
                static_cast<vx_size>(io_desc_.outNumChannels[id])
            };
        }

        return {
            static_cast<vx_size>(io_desc_.outNumChannels[id]),
            static_cast<vx_size>(io_desc_.outWidth[id] + io_desc_.outPadL[id] + io_desc_.outPadR[id]),
            static_cast<vx_size>(io_desc_.outHeight[id] + io_desc_.outPadT[id] + io_desc_.outPadB[id])
        };
    }

    std::array<vx_size, 4> tensorVxShapeOutput(uint32_t id) const
    {
        return {
            static_cast<vx_size>(io_desc_.outWidth[id] + io_desc_.outPadL[id] + io_desc_.outPadR[id]),
            static_cast<vx_size>(io_desc_.outHeight[id] + io_desc_.outPadT[id] + io_desc_.outPadB[id]),
            static_cast<vx_size>(
                (io_desc_.outNumChannels[id] + io_desc_.outPadCh[id]) *
                io_desc_.outDIM1[id] *
                io_desc_.outDIM2[id]),
            static_cast<vx_size>(io_desc_.outNumBatches[id])
        };
    }

    void ensureSupportedModel() const
    {
        if (num_inputs_ != 1)
        {
            throw std::runtime_error("Only single-input TIDL models are supported.");
        }
        if (io_desc_.inNumBatches[0] != 1)
        {
            throw std::runtime_error("Only batch=1 TIDL input is currently supported.");
        }
        if (io_desc_.inDIM1[0] != 1 || io_desc_.inDIM2[0] != 1)
        {
            throw std::runtime_error("Only TIDL input DIM1=DIM2=1 is currently supported.");
        }
        for (uint32_t i = 0; i < num_outputs_; ++i)
        {
            if (io_desc_.outNumBatches[i] != 1)
            {
                throw std::runtime_error("Only batch=1 TIDL outputs are currently supported.");
            }
            if (io_desc_.outDIM1[i] != 1 || io_desc_.outDIM2[i] != 1)
            {
                throw std::runtime_error("Only TIDL outputs with DIM1=DIM2=1 are currently supported.");
            }
        }
    }

    void createGraphSlot(TiGraphSlot& slot)
    {
        debugLog("createGraphSlot: vxCreateGraph begin");
        slot.graph = vxCreateGraph(context_);
        const vx_status graph_status = vxGetStatus(reinterpret_cast<vx_reference>(slot.graph));
        debugLogStatus("createGraphSlot: vxCreateGraph", graph_status);
        if (graph_status != VX_SUCCESS)
        {
            slot.graph = nullptr;
            throw std::runtime_error("vxCreateGraph failed.");
        }

        debugLog("createGraphSlot: create per-slot TIDL user data begin");
        slot.create_params = createTidlCreateParams();
        slot.in_args = createTidlInArgs();
        slot.out_args = createTidlOutArgs();
        if (slot.create_params == nullptr || slot.in_args == nullptr || slot.out_args == nullptr)
        {
            throw std::runtime_error("Failed to create per-slot TIDL user data objects.");
        }
        debugLog("createGraphSlot: create per-slot TIDL user data done");

        slot.input_tensors.assign(num_inputs_, nullptr);
        for (uint32_t i = 0; i < num_inputs_; ++i)
        {
            const auto sizes = tensorShapeInput(i);
            const vx_size data_type = getOpenVXType(io_desc_.inElementType[i]);
            debugLog("createGraphSlot: vxCreateTensor(input[" + std::to_string(i) +
                     "]) begin sizes=[" + std::to_string(static_cast<size_t>(sizes[0])) + "," +
                     std::to_string(static_cast<size_t>(sizes[1])) + "," +
                     std::to_string(static_cast<size_t>(sizes[2])) + "], type=" +
                     std::to_string(static_cast<size_t>(data_type)));
            slot.input_tensors[i] = vxCreateTensor(context_, 3, sizes.data(), data_type, 0);
            const vx_status tensor_status =
                vxGetStatus(reinterpret_cast<vx_reference>(slot.input_tensors[i]));
            debugLogStatus("createGraphSlot: vxCreateTensor(input[" + std::to_string(i) + "]",
                           tensor_status);
            if (tensor_status != VX_SUCCESS)
            {
                throw std::runtime_error("vxCreateTensor(input) failed.");
            }
        }

        slot.output_tensors.assign(num_outputs_, nullptr);
        for (uint32_t i = 0; i < num_outputs_; ++i)
        {
            const auto sizes = tensorVxShapeOutput(i);
            const vx_size data_type = getOpenVXType(io_desc_.outElementType[i]);
            debugLog("createGraphSlot: vxCreateTensor(output[" + std::to_string(i) +
                     "]) begin sizes=[" + std::to_string(static_cast<size_t>(sizes[0])) + "," +
                     std::to_string(static_cast<size_t>(sizes[1])) + "," +
                     std::to_string(static_cast<size_t>(sizes[2])) + "," +
                     std::to_string(static_cast<size_t>(sizes[3])) + "], type=" +
                     std::to_string(static_cast<size_t>(data_type)));
            slot.output_tensors[i] = vxCreateTensor(context_, 4, sizes.data(), data_type, 0);
            const vx_status tensor_status =
                vxGetStatus(reinterpret_cast<vx_reference>(slot.output_tensors[i]));
            debugLogStatus("createGraphSlot: vxCreateTensor(output[" + std::to_string(i) + "]",
                           tensor_status);
            if (tensor_status != VX_SUCCESS)
            {
                throw std::runtime_error("vxCreateTensor(output) failed.");
            }
        }

        vx_reference params[kMaxParams] = {nullptr};
        params[0] = reinterpret_cast<vx_reference>(config_);
        params[1] = reinterpret_cast<vx_reference>(network_);
        params[2] = reinterpret_cast<vx_reference>(slot.create_params);
        params[3] = reinterpret_cast<vx_reference>(slot.in_args);
        params[4] = reinterpret_cast<vx_reference>(slot.out_args);

        debugLog("createGraphSlot: tivxTIDLNode begin");
        slot.tidl_node = tivxTIDLNode(
            slot.graph,
            tidl_kernel_,
            params,
            slot.input_tensors.data(),
            slot.output_tensors.data());
        const vx_status node_status = vxGetStatus(reinterpret_cast<vx_reference>(slot.tidl_node));
        debugLogStatus("createGraphSlot: tivxTIDLNode", node_status);
        if (node_status != VX_SUCCESS)
        {
            slot.tidl_node = nullptr;
            throw std::runtime_error("tivxTIDLNode failed.");
        }

        debugLog("createGraphSlot: vxSetNodeTarget begin target=" + target_name_);
        const vx_status target_status =
            vxSetNodeTarget(slot.tidl_node, VX_TARGET_STRING, target_name_.c_str());
        debugLogStatus("createGraphSlot: vxSetNodeTarget", target_status);
        if (target_status != VX_SUCCESS)
        {
            throw std::runtime_error("vxSetNodeTarget failed, status=" + std::to_string(target_status));
        }

        debugLog("createGraphSlot: vxVerifyGraph begin");
        const vx_status status = vxVerifyGraph(slot.graph);
        debugLogStatus("createGraphSlot: vxVerifyGraph", status);
        if (status != VX_SUCCESS)
        {
            throw std::runtime_error("vxVerifyGraph failed, status=" + std::to_string(status));
        }
        debugLog("createGraphSlot: done");
    }

    void createGraphSlots()
    {
        graph_slots_.clear();
        graph_slots_.resize(graphSlotPoolSize());
        try
        {
            for (std::size_t i = 0; i < graph_slots_.size(); ++i)
            {
                debugLog("createGraphSlots: slot[" + std::to_string(i) + "] begin");
                createGraphSlot(graph_slots_[i]);
                debugLog("createGraphSlots: slot[" + std::to_string(i) + "] done");
            }
        }
        catch (...)
        {
            debugLog("createGraphSlots: exception, releaseGraphSlotsNoWait begin");
            releaseGraphSlotsNoWait();
            debugLog("createGraphSlots: exception, releaseGraphSlotsNoWait done");
            throw;
        }
    }

    std::shared_ptr<GraphSlotLease> acquireGraphSlot()
    {
        std::unique_lock<std::mutex> lk(graph_slot_mutex_);
        if (graph_slots_.empty())
        {
            throw std::runtime_error("TI graph slot pool is not initialized.");
        }

        graph_slot_cv_.wait(lk, [&] {
            for (const auto& slot : graph_slots_)
            {
                if (!slot.in_use)
                {
                    return true;
                }
            }
            return false;
        });

        for (std::size_t i = 0; i < graph_slots_.size(); ++i)
        {
            if (!graph_slots_[i].in_use)
            {
                graph_slots_[i].in_use = true;
                auto lease = std::make_shared<GraphSlotLease>();
                lease->model = this;
                lease->slotIndex = i;
                return lease;
            }
        }

        throw std::runtime_error("TI graph slot wait finished without a free slot.");
    }

    void releaseGraphSlot(std::size_t slotIndex) noexcept
    {
        {
            std::lock_guard<std::mutex> lk(graph_slot_mutex_);
            if (slotIndex < graph_slots_.size())
            {
                graph_slots_[slotIndex].in_use = false;
            }
        }
        graph_slot_cv_.notify_all();
    }

    void waitGraphSlotsIdle() noexcept
    {
        std::unique_lock<std::mutex> lk(graph_slot_mutex_);
        graph_slot_cv_.wait(lk, [&] {
            for (const auto& slot : graph_slots_)
            {
                if (slot.in_use)
                {
                    return false;
                }
            }
            return true;
        });
    }

    void releaseGraphSlotResources(TiGraphSlot& slot) noexcept
    {
        if (slot.tidl_node != nullptr)
        {
            vxReleaseNode(&slot.tidl_node);
        }
        if (slot.graph != nullptr)
        {
            vxReleaseGraph(&slot.graph);
        }

        for (auto& tensor : slot.input_tensors)
        {
            if (tensor != nullptr)
            {
                vxReleaseTensor(&tensor);
            }
        }
        for (auto& tensor : slot.output_tensors)
        {
            if (tensor != nullptr)
            {
                vxReleaseTensor(&tensor);
            }
        }
        slot.input_tensors.clear();
        slot.output_tensors.clear();
        std::memset(&slot.total_perf, 0, sizeof(slot.total_perf));
        std::memset(&slot.graph_perf, 0, sizeof(slot.graph_perf));

        if (slot.create_params != nullptr)
        {
            vxReleaseUserDataObject(&slot.create_params);
        }
        if (slot.in_args != nullptr)
        {
            vxReleaseUserDataObject(&slot.in_args);
        }
        if (slot.out_args != nullptr)
        {
            vxReleaseUserDataObject(&slot.out_args);
        }

        slot.in_use = false;
    }

    void releaseGraphSlotsNoWait() noexcept
    {
        for (auto& slot : graph_slots_)
        {
            releaseGraphSlotResources(slot);
        }
        graph_slots_.clear();
    }

    void releaseGraphSlots() noexcept
    {
        waitGraphSlotsIdle();
        releaseGraphSlotsNoWait();
    }

    std::vector<int> expectedInputShape() const
    {
        if (io_desc_.inLayout[0] == TIDL_LT_NHWC)
        {
            return {
                io_desc_.inNumBatches[0],
                io_desc_.inHeight[0],
                io_desc_.inWidth[0],
                io_desc_.inNumChannels[0]
            };
        }

        return {
            io_desc_.inNumBatches[0],
            io_desc_.inNumChannels[0],
            io_desc_.inHeight[0],
            io_desc_.inWidth[0]
        };
    }

    TensorOrder detectInputOrder(const cv::Mat& input) const
    {
        const int n = io_desc_.inNumBatches[0];
        const int c = io_desc_.inNumChannels[0];
        const int h = io_desc_.inHeight[0];
        const int w = io_desc_.inWidth[0];

        if (input.dims == 4)
        {
            if (input.size[0] == n && input.size[1] == c && input.size[2] == h && input.size[3] == w)
            {
                return TensorOrder::NCHW;
            }
            if (input.size[0] == n && input.size[1] == h && input.size[2] == w && input.size[3] == c)
            {
                return TensorOrder::NHWC;
            }
        }
        else if (input.dims == 3 && n == 1)
        {
            if (input.size[0] == c && input.size[1] == h && input.size[2] == w)
            {
                return TensorOrder::NCHW;
            }
            if (input.size[0] == h && input.size[1] == w && input.size[2] == c)
            {
                return TensorOrder::NHWC;
            }
        }

        throw std::runtime_error(
            "Input shape does not match TI model expectation. Got " +
            shapeToString(std::vector<int>(input.size.p, input.size.p + input.dims)) +
            ", expected around " + shapeToString(expectedInputShape()));
    }

    template <typename T>
    void writeInputTyped(T* dst,
                         const std::array<vx_size, 3>& strides,
                         TensorOrder src_order,
                         const float* src) const
    {
        const int c = io_desc_.inNumChannels[0];
        const int h = io_desc_.inHeight[0];
        const int w = io_desc_.inWidth[0];
        const int pad_l = io_desc_.inPadL[0];
        const int pad_t = io_desc_.inPadT[0];
        const float scale = io_desc_.inTensorScale[0];
        const int32_t zero_point = io_desc_.inZeroPoint[0];
        auto* dst_bytes = reinterpret_cast<uint8_t*>(dst);

        if (io_desc_.inLayout[0] == TIDL_LT_NCHW)
        {
            for (int ch = 0; ch < c; ++ch)
            {
                for (int y = 0; y < h; ++y)
                {
                    for (int x = 0; x < w; ++x)
                    {
                        const size_t src_idx = (src_order == TensorOrder::NCHW)
                            ? static_cast<size_t>(ch * h * w + y * w + x)
                            : static_cast<size_t>(y * w * c + x * c + ch);
                        auto* cell = reinterpret_cast<T*>(
                            dst_bytes +
                            static_cast<size_t>(ch) * static_cast<size_t>(strides[2]) +
                            static_cast<size_t>(pad_t + y) * static_cast<size_t>(strides[1]) +
                            static_cast<size_t>(pad_l + x) * static_cast<size_t>(strides[0]));
                        *cell = quantizeValue<T>(src[src_idx], scale, zero_point);
                    }
                }
            }
            return;
        }

        for (int y = 0; y < h; ++y)
        {
            for (int x = 0; x < w; ++x)
            {
                for (int ch = 0; ch < c; ++ch)
                {
                    const size_t src_idx = (src_order == TensorOrder::NHWC)
                        ? static_cast<size_t>(y * w * c + x * c + ch)
                        : static_cast<size_t>(ch * h * w + y * w + x);
                    auto* cell = reinterpret_cast<T*>(
                        dst_bytes +
                        static_cast<size_t>(pad_t + y) * static_cast<size_t>(strides[2]) +
                        static_cast<size_t>(pad_l + x) * static_cast<size_t>(strides[1]) +
                        static_cast<size_t>(ch) * static_cast<size_t>(strides[0]));
                    *cell = quantizeValue<T>(src[src_idx], scale, zero_point);
                }
            }
        }
    }

    bool writeInputFloatIdentityFast(void* dst,
                                     const std::array<vx_size, 3>& strides,
                                     TensorOrder src_order,
                                     const float* src) const
    {
        if (!isIdentityTensorScale(io_desc_.inTensorScale[0], io_desc_.inZeroPoint[0]))
        {
            return false;
        }
        if (strides[0] != static_cast<vx_size>(sizeof(float)))
        {
            return false;
        }

        const int c = io_desc_.inNumChannels[0];
        const int h = io_desc_.inHeight[0];
        const int w = io_desc_.inWidth[0];
        const int pad_l = io_desc_.inPadL[0];
        const int pad_t = io_desc_.inPadT[0];
        auto* dst_bytes = static_cast<uint8_t*>(dst);

        if (io_desc_.inLayout[0] == TIDL_LT_NCHW)
        {
            if (src_order != TensorOrder::NCHW)
            {
                return false;
            }

            const std::size_t row_bytes = static_cast<std::size_t>(w) * sizeof(float);
            if (!inputTensorHasPadding() &&
                strides[1] == static_cast<vx_size>(row_bytes) &&
                strides[2] == static_cast<vx_size>(h) * static_cast<vx_size>(row_bytes))
            {
                std::memcpy(dst_bytes,
                            src,
                            static_cast<std::size_t>(c) *
                                static_cast<std::size_t>(h) *
                                static_cast<std::size_t>(w) *
                                sizeof(float));
                return true;
            }

            for (int ch = 0; ch < c; ++ch)
            {
                for (int y = 0; y < h; ++y)
                {
                    auto* dst_row = dst_bytes +
                        static_cast<std::size_t>(ch) * static_cast<std::size_t>(strides[2]) +
                        static_cast<std::size_t>(pad_t + y) * static_cast<std::size_t>(strides[1]) +
                        static_cast<std::size_t>(pad_l) * static_cast<std::size_t>(strides[0]);
                    const float* src_row = src + static_cast<std::size_t>(ch * h * w + y * w);
                    std::memcpy(dst_row, src_row, row_bytes);
                }
            }
            return true;
        }

        if (src_order != TensorOrder::NHWC)
        {
            return false;
        }
        if (strides[1] != static_cast<vx_size>(c) * static_cast<vx_size>(sizeof(float)))
        {
            return false;
        }

        const std::size_t row_bytes = static_cast<std::size_t>(w) *
            static_cast<std::size_t>(c) * sizeof(float);
        if (!inputTensorHasPadding() &&
            strides[2] == static_cast<vx_size>(row_bytes))
        {
            std::memcpy(dst_bytes,
                        src,
                        static_cast<std::size_t>(h) * row_bytes);
            return true;
        }

        for (int y = 0; y < h; ++y)
        {
            auto* dst_row = dst_bytes +
                static_cast<std::size_t>(pad_t + y) * static_cast<std::size_t>(strides[2]) +
                static_cast<std::size_t>(pad_l) * static_cast<std::size_t>(strides[1]);
            const float* src_row = src + static_cast<std::size_t>(y * w * c);
            std::memcpy(dst_row, src_row, row_bytes);
        }
        return true;
    }

    bool inputTensorHasPadding() const
    {
        return io_desc_.inPadL[0] != 0 ||
            io_desc_.inPadR[0] != 0 ||
            io_desc_.inPadT[0] != 0 ||
            io_desc_.inPadB[0] != 0;
    }

    bool outputTensorHasPadding(uint32_t id) const
    {
        return io_desc_.outPadL[id] != 0 ||
            io_desc_.outPadR[id] != 0 ||
            io_desc_.outPadT[id] != 0 ||
            io_desc_.outPadB[id] != 0 ||
            io_desc_.outPadCh[id] != 0;
    }

    static size_t mappedTensorBytes(const std::array<vx_size, 3>& sizes,
                                    const std::array<vx_size, 3>& strides,
                                    size_t elem_size)
    {
        if (elem_size == 0U || sizes[0] == 0U || sizes[1] == 0U || sizes[2] == 0U)
        {
            return 0U;
        }

        return static_cast<size_t>(sizes[2] - 1U) * static_cast<size_t>(strides[2]) +
            static_cast<size_t>(sizes[1] - 1U) * static_cast<size_t>(strides[1]) +
            static_cast<size_t>(sizes[0] - 1U) * static_cast<size_t>(strides[0]) +
            elem_size;
    }

    void writeInputTensor(TiGraphSlot& slot, const cv::Mat& input)
    {
        const TensorOrder src_order = detectInputOrder(input);
        const auto sizes = tensorShapeInput(0);
        const vx_size data_type = getOpenVXType(io_desc_.inElementType[0]);
        const size_t elem_size = elementSize(data_type);
        if (elem_size == 0U)
        {
            throw std::runtime_error("Unsupported TIDL input tensor type.");
        }

        std::array<vx_size, 3> strides = {
            static_cast<vx_size>(elem_size),
            sizes[0] * static_cast<vx_size>(elem_size),
            sizes[0] * sizes[1] * static_cast<vx_size>(elem_size)
        };

        vx_map_id map_id = 0;
        void* buffer = nullptr;
        const vx_size start[3] = {0, 0, 0};
        const vx_status status = tivxMapTensorPatch(
            slot.input_tensors[0],
            3,
            start,
            sizes.data(),
            &map_id,
            strides.data(),
            &buffer,
            VX_WRITE_ONLY,
            VX_MEMORY_TYPE_HOST);
        if (status != VX_SUCCESS || buffer == nullptr)
        {
            throw std::runtime_error("tivxMapTensorPatch(input) failed.");
        }

        const size_t mapped_bytes = mappedTensorBytes(sizes, strides, elem_size);
        if (mapped_bytes == 0U)
        {
            tivxUnmapTensorPatch(slot.input_tensors[0], map_id);
            throw std::runtime_error("Invalid mapped input tensor size.");
        }
        const float* src = reinterpret_cast<const float*>(input.data);
        bool mapped_cleared = false;
        auto clearMappedInput = [&]() {
            if (!mapped_cleared)
            {
                std::memset(buffer, 0, mapped_bytes);
                mapped_cleared = true;
            }
        };

        if (data_type == VX_TYPE_FLOAT32)
        {
            if (inputTensorHasPadding())
            {
                clearMappedInput();
            }
            if (writeInputFloatIdentityFast(buffer, strides, src_order, src))
            {
                tivxUnmapTensorPatch(slot.input_tensors[0], map_id);
                return;
            }
        }

        clearMappedInput();

        switch (data_type)
        {
        case VX_TYPE_UINT8:
            writeInputTyped(reinterpret_cast<uint8_t*>(buffer), strides, src_order, src);
            break;
        case VX_TYPE_INT8:
            writeInputTyped(reinterpret_cast<int8_t*>(buffer), strides, src_order, src);
            break;
        case VX_TYPE_UINT16:
            writeInputTyped(reinterpret_cast<uint16_t*>(buffer), strides, src_order, src);
            break;
        case VX_TYPE_INT16:
            writeInputTyped(reinterpret_cast<int16_t*>(buffer), strides, src_order, src);
            break;
        case VX_TYPE_UINT32:
            writeInputTyped(reinterpret_cast<uint32_t*>(buffer), strides, src_order, src);
            break;
        case VX_TYPE_INT32:
            writeInputTyped(reinterpret_cast<int32_t*>(buffer), strides, src_order, src);
            break;
        case VX_TYPE_FLOAT32:
            writeInputTyped(reinterpret_cast<float*>(buffer), strides, src_order, src);
            break;
        default:
            tivxUnmapTensorPatch(slot.input_tensors[0], map_id);
            throw std::runtime_error("Unsupported input tensor OpenVX type.");
        }

        tivxUnmapTensorPatch(slot.input_tensors[0], map_id);
    }

    std::vector<int> outputShape(uint32_t id, bool squeeze_batch) const
    {
        std::vector<int> six_dims = {
            io_desc_.outNumBatches[id],
            io_desc_.outDIM1[id],
            io_desc_.outDIM2[id],
            io_desc_.outNumChannels[id],
            io_desc_.outHeight[id],
            io_desc_.outWidth[id]
        };

        const int valid_dims = std::max(1, io_desc_.numValidTensorDims[id]);
        int vector_offset = static_cast<int>(six_dims.size()) - valid_dims;
        if (six_dims[0] > 1)
        {
            vector_offset += 1;
        }

        std::vector<int> shape;
        if (six_dims[0] > 1)
        {
            shape.push_back(six_dims[0]);
        }
        for (size_t i = static_cast<size_t>(std::max(vector_offset, 0)); i < six_dims.size(); ++i)
        {
            shape.push_back(six_dims[i]);
        }

        if (shape.empty())
        {
            shape.push_back(1);
        }

        if (squeeze_batch && !shape.empty() && shape.front() == 1)
        {
            shape.erase(shape.begin());
        }

        if (shape.empty())
        {
            shape.push_back(1);
        }

        return shape;
    }

    std::vector<int> rawOutputShape(uint32_t id) const
    {
        std::vector<int> six_dims = {
            io_desc_.outNumBatches[id],
            io_desc_.outDIM1[id],
            io_desc_.outDIM2[id],
            io_desc_.outNumChannels[id],
            io_desc_.outHeight[id],
            io_desc_.outWidth[id]
        };

        const int valid_dims = std::max(1, io_desc_.numValidTensorDims[id]);
        const int vector_offset = static_cast<int>(six_dims.size()) - valid_dims;

        std::vector<int> shape;
        for (size_t i = static_cast<size_t>(std::max(vector_offset, 0)); i < six_dims.size(); ++i)
        {
            shape.push_back(six_dims[i]);
        }

        if (shape.empty())
        {
            shape.push_back(1);
        }

        return shape;
    }

    std::string outputName(uint32_t id) const
    {
        return tidlNameToString(io_desc_.outDataName[id], TIDL_STRING_SIZE);
    }

    struct MappedOutputTensorPatch
    {
        vx_tensor tensor{nullptr};
        vx_map_id map_id{0};
    };

    struct MappedOutputTensorOwner
    {
        ~MappedOutputTensorOwner()
        {
            unmapAll();
        }

        void unmapAll()
        {
            for (auto it = patches.rbegin(); it != patches.rend(); ++it)
            {
                if (it->tensor != nullptr)
                {
                    tivxUnmapTensorPatch(it->tensor, it->map_id);
                    it->tensor = nullptr;
                }
            }
            patches.clear();
        }

        std::vector<MappedOutputTensorPatch> patches;
        std::shared_ptr<GraphSlotLease> lease;
    };

    static size_t elementCount(const std::vector<int>& shape)
    {
        size_t count = 1U;
        for (int dim : shape)
        {
            if (dim <= 0)
            {
                return 0U;
            }
            count *= static_cast<size_t>(dim);
        }
        return count;
    }

    static std::array<vx_size, 3> contiguousFloatStrides(const std::array<vx_size, 3>& sizes)
    {
        constexpr vx_size elem_size = sizeof(float);
        return {
            elem_size,
            sizes[0] * elem_size,
            sizes[0] * sizes[1] * elem_size
        };
    }

    static std::array<vx_size, 3> contiguousStrides(const std::array<vx_size, 3>& sizes,
                                                    vx_size elem_size)
    {
        return {
            elem_size,
            sizes[0] * elem_size,
            sizes[0] * sizes[1] * elem_size
        };
    }

    static std::array<vx_size, 4> contiguousStrides4(const std::array<vx_size, 4>& sizes,
                                                     vx_size elem_size)
    {
        return {
            elem_size,
            sizes[0] * elem_size,
            sizes[0] * sizes[1] * elem_size,
            sizes[0] * sizes[1] * sizes[2] * elem_size
        };
    }

    bool canViewOutputTensor(uint32_t id, bool squeeze_batch) const
    {
        (void)id;
        (void)squeeze_batch;
        return false;
    }

    bool canViewAllOutputTensors(bool squeeze_batch) const
    {
        for (uint32_t id = 0; id < num_outputs_; ++id)
        {
            if (!canViewOutputTensor(id, squeeze_batch))
            {
                return false;
            }
        }
        return true;
    }

    std::vector<cv::Mat> viewOutputTensorsOrCopy(TiGraphSlot& slot,
                                                 bool squeeze_batch,
                                                 std::shared_ptr<void>& owner_out,
                                                 std::shared_ptr<GraphSlotLease> lease) const
    {
        owner_out.reset();
        if (!canViewAllOutputTensors(squeeze_batch))
        {
            return readOutputTensors(slot, squeeze_batch);
        }

        auto owner = std::make_shared<MappedOutputTensorOwner>();
        owner->lease = std::move(lease);
        owner->patches.reserve(num_outputs_);

        std::vector<cv::Mat> outputs;
        outputs.reserve(num_outputs_);

        for (uint32_t id = 0; id < num_outputs_; ++id)
        {
            const auto sizes = tensorShapeOutput(id);
            const auto expected_strides = contiguousFloatStrides(sizes);
            std::array<vx_size, 3> strides = expected_strides;

            vx_map_id map_id = 0;
            void* buffer = nullptr;
            const vx_size start[3] = {0, 0, 0};
            const vx_status status = tivxMapTensorPatch(
                slot.output_tensors[id],
                3,
                start,
                sizes.data(),
                &map_id,
                strides.data(),
                &buffer,
                VX_READ_ONLY,
                VX_MEMORY_TYPE_HOST);
            if (status != VX_SUCCESS || buffer == nullptr)
            {
                throw std::runtime_error("tivxMapTensorPatch(output view) failed.");
            }

            owner->patches.push_back(MappedOutputTensorPatch{slot.output_tensors[id], map_id});
            if (strides != expected_strides)
            {
                owner_out.reset();
                owner->unmapAll();
                return readOutputTensors(slot, squeeze_batch);
            }

            auto shape = outputShape(id, squeeze_batch);
            outputs.emplace_back(static_cast<int>(shape.size()), shape.data(), CV_32F, buffer);
        }

        owner_out = owner;
        return outputs;
    }

    template <typename T>
    void readOutputTyped(uint32_t id,
                         const T* src,
                         const std::array<vx_size, 3>& strides,
                         float* dst) const
    {
        const int c = io_desc_.outNumChannels[id];
        const int h = io_desc_.outHeight[id];
        const int w = io_desc_.outWidth[id];
        const int pad_l = io_desc_.outPadL[id];
        const int pad_t = io_desc_.outPadT[id];
        const float scale = io_desc_.outTensorScale[id];
        const int32_t zero_point = io_desc_.outZeroPoint[id];
        const auto* src_bytes = reinterpret_cast<const uint8_t*>(src);

        size_t out_idx = 0;
        if (io_desc_.outLayout[id] == TIDL_LT_NCHW)
        {
            for (int ch = 0; ch < c; ++ch)
            {
                for (int y = 0; y < h; ++y)
                {
                    for (int x = 0; x < w; ++x)
                    {
                        const auto* cell = reinterpret_cast<const T*>(
                            src_bytes +
                            static_cast<size_t>(ch) * static_cast<size_t>(strides[2]) +
                            static_cast<size_t>(pad_t + y) * static_cast<size_t>(strides[1]) +
                            static_cast<size_t>(pad_l + x) * static_cast<size_t>(strides[0]));
                        dst[out_idx++] = dequantizeValue(*cell, scale, zero_point);
                    }
                }
            }
            return;
        }

        for (int y = 0; y < h; ++y)
        {
            for (int x = 0; x < w; ++x)
            {
                for (int ch = 0; ch < c; ++ch)
                {
                    const auto* cell = reinterpret_cast<const T*>(
                        src_bytes +
                        static_cast<size_t>(pad_t + y) * static_cast<size_t>(strides[2]) +
                        static_cast<size_t>(pad_l + x) * static_cast<size_t>(strides[1]) +
                        static_cast<size_t>(ch) * static_cast<size_t>(strides[0]));
                    dst[out_idx++] = dequantizeValue(*cell, scale, zero_point);
                }
            }
        }
    }

    bool readOutputFloatIdentityFast(uint32_t id,
                                     const float* src,
                                     const std::array<vx_size, 3>& strides,
                                     float* dst) const
    {
        if (!isIdentityTensorScale(io_desc_.outTensorScale[id], io_desc_.outZeroPoint[id]))
        {
            return false;
        }
        if (strides[0] != static_cast<vx_size>(sizeof(float)))
        {
            return false;
        }

        const int c = io_desc_.outNumChannels[id];
        const int h = io_desc_.outHeight[id];
        const int w = io_desc_.outWidth[id];
        const int pad_l = io_desc_.outPadL[id];
        const int pad_t = io_desc_.outPadT[id];
        const auto* src_bytes = reinterpret_cast<const uint8_t*>(src);

        if (io_desc_.outLayout[id] == TIDL_LT_NCHW)
        {
            const std::size_t row_bytes = static_cast<std::size_t>(w) * sizeof(float);
            if (!outputTensorHasPadding(id) &&
                strides[1] == static_cast<vx_size>(row_bytes) &&
                strides[2] == static_cast<vx_size>(h) * static_cast<vx_size>(row_bytes))
            {
                std::memcpy(dst,
                            src,
                            static_cast<std::size_t>(c) *
                                static_cast<std::size_t>(h) *
                                static_cast<std::size_t>(w) *
                                sizeof(float));
                return true;
            }

            for (int ch = 0; ch < c; ++ch)
            {
                for (int y = 0; y < h; ++y)
                {
                    const auto* src_row = src_bytes +
                        static_cast<std::size_t>(ch) * static_cast<std::size_t>(strides[2]) +
                        static_cast<std::size_t>(pad_t + y) * static_cast<std::size_t>(strides[1]) +
                        static_cast<std::size_t>(pad_l) * static_cast<std::size_t>(strides[0]);
                    float* dst_row = dst + static_cast<std::size_t>(ch * h * w + y * w);
                    std::memcpy(dst_row, src_row, row_bytes);
                }
            }
            return true;
        }

        if (strides[1] != static_cast<vx_size>(c) * static_cast<vx_size>(sizeof(float)))
        {
            return false;
        }

        const std::size_t row_bytes = static_cast<std::size_t>(w) *
            static_cast<std::size_t>(c) * sizeof(float);
        if (!outputTensorHasPadding(id) &&
            strides[2] == static_cast<vx_size>(row_bytes))
        {
            std::memcpy(dst,
                        src,
                        static_cast<std::size_t>(h) * row_bytes);
            return true;
        }

        for (int y = 0; y < h; ++y)
        {
            const auto* src_row = src_bytes +
                static_cast<std::size_t>(pad_t + y) * static_cast<std::size_t>(strides[2]) +
                static_cast<std::size_t>(pad_l) * static_cast<std::size_t>(strides[1]);
            float* dst_row = dst + static_cast<std::size_t>(y * w * c);
            std::memcpy(dst_row, src_row, row_bytes);
        }
        return true;
    }

    void logOutputHeadDebug(uint32_t id, const cv::Mat& output, int max_values = 8) const
    {
        const int n = std::max(0, std::min(max_values, static_cast<int>(output.total())));
        std::string line = "TI raw output[" + std::to_string(id) + "] name=" + outputName(id) +
            " first " + std::to_string(n) + " elements:";
        const float* data = reinterpret_cast<const float*>(output.data);
        for (int i = 0; i < n; ++i)
        {
            line += " " + std::to_string(data[i]);
        }
        LOG_DEBUG("%s", line.c_str());
    }

    static std::string jsonEscape(const std::string& value)
    {
        std::ostringstream oss;
        for (char ch : value)
        {
            switch (ch)
            {
            case '\\': oss << "\\\\"; break;
            case '"': oss << "\\\""; break;
            case '\n': oss << "\\n"; break;
            case '\r': oss << "\\r"; break;
            case '\t': oss << "\\t"; break;
            default: oss << ch; break;
            }
        }
        return oss.str();
    }

    static std::string safeFilenamePart(const std::string& value)
    {
        std::string out;
        out.reserve(value.size());
        for (char ch : value)
        {
            const unsigned char uch = static_cast<unsigned char>(ch);
            if (std::isalnum(uch) != 0 || ch == '_' || ch == '-' || ch == '.')
            {
                out.push_back(ch);
            }
            else
            {
                out.push_back('_');
            }
        }
        return out.empty() ? "unnamed" : out;
    }

    static std::vector<int> matShape(const cv::Mat& mat)
    {
        return std::vector<int>(mat.size.p, mat.size.p + mat.dims);
    }

    static std::string vxSizeArrayToString(const std::array<vx_size, 3>& values)
    {
        std::ostringstream oss;
        oss << "[" << static_cast<size_t>(values[0])
            << "," << static_cast<size_t>(values[1])
            << "," << static_cast<size_t>(values[2]) << "]";
        return oss.str();
    }

    static std::string vxSizeArrayToString(const std::array<vx_size, 4>& values)
    {
        std::ostringstream oss;
        oss << "[" << static_cast<size_t>(values[0])
            << "," << static_cast<size_t>(values[1])
            << "," << static_cast<size_t>(values[2])
            << "," << static_cast<size_t>(values[3]) << "]";
        return oss.str();
    }

    uint64_t nextOutputDumpFrame() const
    {
        std::lock_guard<std::mutex> lk(output_dump_mutex_);
        return output_dump_frame_++;
    }

    void dumpOutputTensor(uint32_t id,
                          uint64_t frame_index,
                          const cv::Mat& output,
                          const std::array<vx_size, 4>& tensor_sizes,
                          const std::array<vx_size, 4>& map_strides) const
    {
        if (!dump_outputs_enabled_)
        {
            return;
        }

        try
        {
            namespace fs = std::filesystem;
            const fs::path dump_dir("ti_output_dump");
            fs::create_directories(dump_dir);

            const std::string name = outputName(id);
            const std::string stem = "frame_" + std::to_string(frame_index) +
                "_output_" + std::to_string(id) + "_" + safeFilenamePart(name);
            const fs::path bin_path = dump_dir / (stem + ".f32.bin");
            const fs::path json_path = dump_dir / (stem + ".json");

            const cv::Mat continuous = output.isContinuous() ? output : output.clone();
            const size_t value_count = continuous.total();
            const size_t byte_count = value_count * sizeof(float);

            {
                std::ofstream ofs(bin_path, std::ios::binary);
                if (!ofs)
                {
                    LOG_ERROR("Failed to open TI output dump file: %s", bin_path.string().c_str());
                    return;
                }
                ofs.write(reinterpret_cast<const char*>(continuous.ptr<float>()),
                          static_cast<std::streamsize>(byte_count));
                if (!ofs)
                {
                    LOG_ERROR("Failed to write TI output dump file: %s", bin_path.string().c_str());
                    return;
                }
            }

            {
                std::ofstream meta(json_path);
                if (!meta)
                {
                    LOG_ERROR("Failed to open TI output dump metadata file: %s",
                              json_path.string().c_str());
                    return;
                }

                meta << "{\n"
                     << "  \"file\": \"" << jsonEscape(bin_path.string()) << "\",\n"
                     << "  \"dtype\": \"float32\",\n"
                     << "  \"byte_order\": \"little_endian\",\n"
                     << "  \"order\": \"logical_contiguous_float_output\",\n"
                     << "  \"frame_index\": " << frame_index << ",\n"
                     << "  \"output_index\": " << id << ",\n"
                     << "  \"output_name\": \"" << jsonEscape(name) << "\",\n"
                     << "  \"shape\": " << shapeToString(matShape(continuous)) << ",\n"
                     << "  \"raw_shape\": " << shapeToString(rawOutputShape(id)) << ",\n"
                     << "  \"value_count\": " << value_count << ",\n"
                     << "  \"byte_count\": " << byte_count << ",\n"
                     << "  \"model_path\": \"" << jsonEscape(model_path_) << "\",\n"
                     << "  \"tidl_io_config\": \"" << jsonEscape(io_path_) << "\",\n"
                     << "  \"tidl_network\": \"" << jsonEscape(network_path_) << "\",\n"
                     << "  \"element_type\": \"" << typeName(io_desc_.outElementType[id]) << "\",\n"
                     << "  \"layout\": \"" << layoutName(io_desc_.outLayout[id]) << "\",\n"
                     << "  \"scale\": " << io_desc_.outTensorScale[id] << ",\n"
                     << "  \"zero_point\": " << io_desc_.outZeroPoint[id] << ",\n"
                     << "  \"tensor_sizes\": " << vxSizeArrayToString(tensor_sizes) << ",\n"
                     << "  \"map_strides_bytes\": " << vxSizeArrayToString(map_strides) << ",\n"
                     << "  \"tidl_dims\": {"
                     << "\"N\": " << io_desc_.outNumBatches[id]
                     << ", \"D1\": " << io_desc_.outDIM1[id]
                     << ", \"D2\": " << io_desc_.outDIM2[id]
                     << ", \"C\": " << io_desc_.outNumChannels[id]
                     << ", \"H\": " << io_desc_.outHeight[id]
                     << ", \"W\": " << io_desc_.outWidth[id]
                     << "},\n"
                     << "  \"pad\": {"
                     << "\"left\": " << io_desc_.outPadL[id]
                     << ", \"right\": " << io_desc_.outPadR[id]
                     << ", \"top\": " << io_desc_.outPadT[id]
                     << ", \"bottom\": " << io_desc_.outPadB[id]
                     << ", \"channels\": " << io_desc_.outPadCh[id]
                     << "},\n"
                     << "  \"channel_pitch\": " << io_desc_.outChannelPitch[id] << ",\n"
                     << "  \"buffer_size\": " << io_desc_.outBufSize[id] << ",\n"
                     << "  \"valid_dims\": " << io_desc_.numValidTensorDims[id] << "\n"
                     << "}\n";
            }

            LOG_INFO("Dumped TI output[%u] name=%s to %s",
                     id,
                     name.c_str(),
                     bin_path.string().c_str());
        }
        catch (const std::exception& e)
        {
            LOG_ERROR("Failed to dump TI output[%u] name=%s: %s",
                      id,
                      outputName(id).c_str(),
                      e.what());
        }
    }

    std::vector<cv::Mat> readOutputTensors(TiGraphSlot& slot, bool squeeze_batch) const
    {
        std::vector<cv::Mat> outputs;
        outputs.reserve(num_outputs_);
        const uint64_t dump_frame = nextOutputDumpFrame();

        for (uint32_t id = 0; id < num_outputs_; ++id)
        {
            const auto sizes = tensorVxShapeOutput(id);
            const vx_size data_type = getOpenVXType(io_desc_.outElementType[id]);
            const size_t elem_size = elementSize(data_type);
            if (elem_size == 0U)
            {
                throw std::runtime_error("Unsupported TIDL output tensor type.");
            }

            std::array<vx_size, 4> strides = contiguousStrides4(sizes, static_cast<vx_size>(elem_size));

            vx_map_id map_id = 0;
            void* buffer = nullptr;
            const vx_size start[4] = {0, 0, 0, 0};
            const vx_status status = tivxMapTensorPatch(
                slot.output_tensors[id],
                4,
                start,
                sizes.data(),
                &map_id,
                strides.data(),
                &buffer,
                VX_READ_ONLY,
                VX_MEMORY_TYPE_HOST);
            if (status != VX_SUCCESS || buffer == nullptr)
            {
                throw std::runtime_error("tivxMapTensorPatch(output) failed.");
            }

            auto shape = outputShape(id, squeeze_batch);
            cv::Mat out(static_cast<int>(shape.size()), shape.data(), CV_32F);
            float* dst = reinterpret_cast<float*>(out.data);
            const std::array<vx_size, 3> logical_strides = {strides[0], strides[1], strides[2]};

            switch (data_type)
            {
            case VX_TYPE_UINT8:
                readOutputTyped(id, reinterpret_cast<const uint8_t*>(buffer), logical_strides, dst);
                break;
            case VX_TYPE_INT8:
                readOutputTyped(id, reinterpret_cast<const int8_t*>(buffer), logical_strides, dst);
                break;
            case VX_TYPE_UINT16:
                readOutputTyped(id, reinterpret_cast<const uint16_t*>(buffer), logical_strides, dst);
                break;
            case VX_TYPE_INT16:
                readOutputTyped(id, reinterpret_cast<const int16_t*>(buffer), logical_strides, dst);
                break;
            case VX_TYPE_UINT32:
                readOutputTyped(id, reinterpret_cast<const uint32_t*>(buffer), logical_strides, dst);
                break;
            case VX_TYPE_INT32:
                readOutputTyped(id, reinterpret_cast<const int32_t*>(buffer), logical_strides, dst);
                break;
            case VX_TYPE_FLOAT32:
                if (!readOutputFloatIdentityFast(id, reinterpret_cast<const float*>(buffer), logical_strides, dst))
                {
                    readOutputTyped(id, reinterpret_cast<const float*>(buffer), logical_strides, dst);
                }
                break;
            default:
                tivxUnmapTensorPatch(slot.output_tensors[id], map_id);
                throw std::runtime_error("Unsupported output tensor OpenVX type.");
            }
            
            tivxUnmapTensorPatch(slot.output_tensors[id], map_id);
            logOutputHeadDebug(id, out);
            dumpOutputTensor(id, dump_frame, out, sizes, strides);
            outputs.emplace_back(std::move(out));
        }

        return outputs;
    }

    void printTiPerf(TiGraphSlot& slot,
                     Clock::time_point write_begin,
                     Clock::time_point write_end,
                     Clock::time_point graph_begin,
                     Clock::time_point graph_end,
                     Clock::time_point read_begin,
                     Clock::time_point read_end,
                     Clock::time_point total_begin,
                     Clock::time_point total_end)
    {
        if (!shouldPrintTiPerf())
        {
            return;
        }

        std::lock_guard<std::mutex> lock(perf_print_mutex_);
        ++perf_frame_count_;
        const uint64_t print_every = tiPerfPrintEvery();
        if ((perf_frame_count_ % print_every) != 0U)
        {
            return;
        }

        const long long write_us = durationUs(write_begin, write_end);
        const long long graph_us = durationUs(graph_begin, graph_end);
        const long long read_us = durationUs(read_begin, read_end);
        const long long total_us = durationUs(total_begin, total_end);

        std::fprintf(stderr,
                     "\n[MIN_DET_TIDL][PERF] frame=%llu target=%s write_input=%.3f ms, "
                     "vxProcessGraph=%.3f ms, read_output=%.3f ms, total=%.3f ms\n",
                     static_cast<unsigned long long>(perf_frame_count_),
                     target_name_.c_str(),
                     static_cast<double>(write_us) / 1000.0,
                     static_cast<double>(graph_us) / 1000.0,
                     static_cast<double>(read_us) / 1000.0,
                     static_cast<double>(total_us) / 1000.0);

        std::fprintf(stderr, "[MIN_DET_TIDL][PERF] appPerfStatsPrintAll begin\n");
        (void)appPerfStatsPrintAll();
        std::fprintf(stderr, "[MIN_DET_TIDL][PERF] tivx_utils_graph_perf_print begin\n");
        const vx_status graph_perf_status = tivx_utils_graph_perf_print(slot.graph);
        std::fprintf(stderr,
                     "[MIN_DET_TIDL][PERF] tivx_utils_graph_perf_print status=%d\n",
                     static_cast<int>(graph_perf_status));

        std::fprintf(stderr, "[MIN_DET_TIDL][PERF] appPerfPointPrint begin\n");
        appPerfPointPrint(&slot.graph_perf);
        appPerfPointPrint(&slot.total_perf);
        std::fprintf(stderr, "\n");
        appPerfPointPrintFPS(&slot.total_perf);
        std::fprintf(stderr, "\n");
        std::fflush(stderr);
    }

    void printModelIOInfo() const
    {
        LOG_INFO("=================== Model IO (OpenVX/TIDL) ===================");
        LOG_INFO("Resolved model path: %s", model_path_.c_str());
        LOG_INFO("TIDL IO config     : %s", io_path_.c_str());
        LOG_INFO("TIDL network       : %s", network_path_.c_str());

        for (uint32_t i = 0; i < num_inputs_; ++i)
        {
            const auto shape = expectedInputShape();
            LOG_INFO("Input[%u] name=%s, fmt=%s, layout=%s, shape=%s, scale=%f, zp=%d",
                     i,
                     io_desc_.inDataName[i],
                     typeName(io_desc_.inElementType[i]),
                     layoutName(io_desc_.inLayout[i]),
                     shapeToString(shape).c_str(),
                     io_desc_.inTensorScale[i],
                     io_desc_.inZeroPoint[i]);
        }

        for (uint32_t i = 0; i < num_outputs_; ++i)
        {
            const auto shape = rawOutputShape(i);
            const auto tensor_sizes = tensorShapeOutput(i);
            const auto vx_tensor_sizes = tensorVxShapeOutput(i);
            const vx_size data_type = getOpenVXType(io_desc_.outElementType[i]);
            const auto strides = contiguousStrides(tensor_sizes, elementSize(data_type));
            const auto vx_strides = contiguousStrides4(vx_tensor_sizes, elementSize(data_type));
            LOG_INFO("Output[%u] name=%s, fmt=%s, layout=%s, shape=%s, scale=%f, zp=%d",
                     i,
                     io_desc_.outDataName[i],
                     typeName(io_desc_.outElementType[i]),
                     layoutName(io_desc_.outLayout[i]),
                     shapeToString(shape).c_str(),
                     io_desc_.outTensorScale[i],
                     io_desc_.outZeroPoint[i]);
            LOG_INFO(
                "Output[%u] tidl_io_dims=(N=%d,D1=%d,D2=%d,C=%d,H=%d,W=%d), "
                "logical_tensor_sizes=[%zu,%zu,%zu], logical_strides_bytes=[%zu,%zu,%zu], "
                "vx_tensor_sizes=[%zu,%zu,%zu,%zu], vx_strides_bytes=[%zu,%zu,%zu,%zu], "
                "padLRTB=%d/%d/%d/%d, padCh=%d, chPitch=%d, bufSize=%d, validDims=%d",
                i,
                io_desc_.outNumBatches[i],
                io_desc_.outDIM1[i],
                io_desc_.outDIM2[i],
                io_desc_.outNumChannels[i],
                io_desc_.outHeight[i],
                io_desc_.outWidth[i],
                static_cast<size_t>(tensor_sizes[0]),
                static_cast<size_t>(tensor_sizes[1]),
                static_cast<size_t>(tensor_sizes[2]),
                static_cast<size_t>(strides[0]),
                static_cast<size_t>(strides[1]),
                static_cast<size_t>(strides[2]),
                static_cast<size_t>(vx_tensor_sizes[0]),
                static_cast<size_t>(vx_tensor_sizes[1]),
                static_cast<size_t>(vx_tensor_sizes[2]),
                static_cast<size_t>(vx_tensor_sizes[3]),
                static_cast<size_t>(vx_strides[0]),
                static_cast<size_t>(vx_strides[1]),
                static_cast<size_t>(vx_strides[2]),
                static_cast<size_t>(vx_strides[3]),
                io_desc_.outPadL[i],
                io_desc_.outPadR[i],
                io_desc_.outPadT[i],
                io_desc_.outPadB[i],
                io_desc_.outPadCh[i],
                io_desc_.outChannelPitch[i],
                io_desc_.outBufSize[i],
                io_desc_.numValidTensorDims[i]);
        }

        LOG_INFO("==============================================================");
    }

    void cleanup()
    {
        debugLog("cleanup: begin");
        if (context_ != nullptr)
        {
            debugLog("cleanup: vxDisableEvents begin");
            (void)vxDisableEvents(context_);
            debugLog("cleanup: vxDisableEvents done");
        }

        debugLog("cleanup: releaseGraphSlots begin");
        releaseGraphSlots();
        debugLog("cleanup: releaseGraphSlots done");

        if (tidl_kernel_ != nullptr)
        {
            debugLog("cleanup: vxRemoveKernel begin");
            vxRemoveKernel(tidl_kernel_);
            tidl_kernel_ = nullptr;
            debugLog("cleanup: vxRemoveKernel done");
        }

        if (config_ != nullptr)
        {
            debugLog("cleanup: vxReleaseUserDataObject(config) begin");
            vxReleaseUserDataObject(&config_);
            debugLog("cleanup: vxReleaseUserDataObject(config) done");
        }
        if (network_ != nullptr)
        {
            debugLog("cleanup: vxReleaseUserDataObject(network) begin");
            vxReleaseUserDataObject(&network_);
            debugLog("cleanup: vxReleaseUserDataObject(network) done");
        }

        if (context_ != nullptr && tidl_kernels_loaded_)
        {
            debugLog("cleanup: tivxTIDLUnLoadKernels begin");
            tivxTIDLUnLoadKernels(context_);
            tidl_kernels_loaded_ = false;
            debugLog("cleanup: tivxTIDLUnLoadKernels done");
        }
        if (context_ != nullptr)
        {
            debugLog("cleanup: vxReleaseContext begin");
            vxReleaseContext(&context_);
            debugLog("cleanup: vxReleaseContext done");
        }

        if (app_inited_)
        {
            debugLog("cleanup: appDeInit begin");
            appDeInit();
            app_inited_ = false;
            debugLog("cleanup: appDeInit done");
        }
        debugLog("cleanup: done");
    }

    std::string model_path_;
    std::string target_name_;
    std::string io_path_;
    std::string network_path_;

    bool app_inited_ = false;
    bool tidl_kernels_loaded_ = false;

    vx_context context_ = nullptr;
    vx_kernel tidl_kernel_ = nullptr;
    vx_user_data_object config_ = nullptr;
    vx_user_data_object network_ = nullptr;
    std::vector<TiGraphSlot> graph_slots_;
    std::mutex graph_slot_mutex_;
    std::condition_variable graph_slot_cv_;
    std::mutex graph_process_mutex_;
    std::mutex perf_print_mutex_;
    mutable std::mutex output_dump_mutex_;
    mutable uint64_t output_dump_frame_ = 0;
    uint64_t perf_frame_count_ = 0;
    app_perf_point_t stress_wall_perf_{};
    // bool dump_outputs_enabled_ = envFlagEnabled("VISPER_TI_DUMP_OUTPUTS", false);
    bool dump_outputs_enabled_ = envFlagEnabled("VISPER_TI_DUMP_OUTPUTS_NEW", false);       //TODO lyl

    sTIDL_IOBufDesc_t io_desc_{};
    uint32_t num_inputs_ = 0;
    uint32_t num_outputs_ = 0;
};
