#pragma once
#include <onnxruntime_cxx_api.h>
#include <opencv2/core.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstring>
#include <iostream>
#include <limits>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "common/logger.hpp"

class ONNXModel
{
public:
    explicit ONNXModel(const std::string& model_path, const std::string& device = "cpu")
        : env_(ORT_LOGGING_LEVEL_WARNING, "onnxmodel_cv"),
          session_options_{},
          session_(nullptr),
          using_gpu_(false),
          device_(device)
    {
        auto [backend, cuda_index] = parse_device(device);

        if (backend == "cuda")
        {
            std::cerr << "[ONNXModel] CUDA requested (" << device
                << ") but this build uses CPU EP only. Fallback to CPUExecutionProvider.\n";
            using_gpu_ = false;
        }

        // 仅 CPU（可在此设置线程数/优化等级）
        // session_options_.SetIntraOpNumThreads(1);
        // session_options_.SetInterOpNumThreads(1);
        // session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

        session_ = Ort::Session(env_, model_path.c_str(), session_options_);

        init_io_names();
        printModelIOInfo();
    }

    // 直接用 OpenCV 的 cv::Mat 作为输入；要求：
    // - N 维（1~n），type 为 CV_32F（float32），并且是连续内存（isContinuous()）
    // - 典型输入为 [N,C,H,W] 的 4D Mat（可用 dnn::blobFromImage 生成）
    // squeeze_batch=true：若某个输出第 0 维为 1，则构造返回 Mat 时去掉该维
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
        if (!input.isContinuous())
        {
            throw std::invalid_argument("input 必须为连续内存（cv::Mat::isContinuous() == true）。");
        }
        if (input_names_.empty())
        {
            throw std::runtime_error("模型未检测到输入。");
        }

        // 取 shape（cv::Mat 的 N 维尺寸）
        std::vector<int64_t> in_shape = mat_shape_to_vec_i64(input);

        // 构造 Ort 输入张量（CPU）
        Ort::MemoryInfo mem_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            mem_info,
            const_cast<float*>(reinterpret_cast<const float*>(input.data)),
            static_cast<size_t>(input.total()),
            in_shape.data(),
            static_cast<size_t>(in_shape.size()));

        const char* input_names_c[1] = {input_names_[0].c_str()};

        std::vector<const char*> output_names_c;
        output_names_c.reserve(output_names_.size());
        for (const auto& s : output_names_) output_names_c.push_back(s.c_str());

        auto out_vals = session_.Run(
            Ort::RunOptions{nullptr},
            input_names_c, &input_tensor, 1,
            output_names_c.data(), output_names_c.size());

        // 将 ORT 输出拷贝到新的 cv::Mat（确保生命周期安全）
        std::vector<cv::Mat> results;
        results.reserve(out_vals.size());

        for (auto& v : out_vals)
        {
            if (!v.IsTensor())
            {
                throw std::runtime_error("仅支持张量输出。");
            }
            auto info = v.GetTensorTypeAndShapeInfo();
            auto etype = info.GetElementType();
            if (etype != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
            {
                throw std::runtime_error("当前示例仅支持 float32 输出。如需其他类型请扩展。");
            }

            std::vector<int64_t> shape = info.GetShape();
            if (squeeze_batch && !shape.empty() && shape.front() == 1)
            {
                shape.erase(shape.begin());
            }
            const size_t elem_count = static_cast<size_t>(info.GetElementCount());
            const float* src = v.GetTensorData<float>();

            // 用 N 维 Mat 承接
            // 注意：cv::Mat 的尺寸数组是 int
            std::vector<int> sizes(shape.begin(), shape.end());
            if (sizes.empty())
            {
                // 标量情形：返回 1D 含 1 个元素
                sizes = {1};
            }
            cv::Mat out(static_cast<int>(sizes.size()), sizes.data(), CV_32F);
            std::memcpy(out.data, src, elem_count * sizeof(float));

            results.emplace_back(std::move(out));
        }

        return results;
    }

    void smokeTest(int nTimes)
    {
        if (nTimes <= 0)
        {
            LOG_INFO("[ONNXModel::SmokeTest] nTimes <= 0, nothing to do.");
            return;
        }

        if (input_names_.size() != 1)
        {
            throw std::runtime_error("ONNXModel::SmokeTest: only supports single-input models.");
        }

        Ort::TypeInfo ti = session_.GetInputTypeInfo(0);
        auto tinfo = ti.GetTensorTypeAndShapeInfo();
        if (tinfo.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
        {
            throw std::runtime_error("ONNXModel::SmokeTest: only supports float32 input models.");
        }

        std::vector<int64_t> input_shape = tinfo.GetShape();
        if (input_shape.empty())
        {
            input_shape.push_back(1);
        }

        std::vector<int> sizes;
        sizes.reserve(input_shape.size());
        for (size_t i = 0; i < input_shape.size(); ++i)
        {
            int64_t dim = input_shape[i];
            if (dim <= 0)
            {
                dim = 1; // 动态维度（-1）和未知维度用 1 兜底
            }
            if (dim > static_cast<int64_t>(std::numeric_limits<int>::max()))
            {
                throw std::runtime_error("ONNXModel::SmokeTest: input dimension exceeds int range.");
            }
            sizes.push_back(static_cast<int>(dim));
        }

        cv::Mat input(static_cast<int>(sizes.size()), sizes.data(), CV_32F);
        {
            std::mt19937 rng(1234);
            std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
            float* p = reinterpret_cast<float*>(input.data);
            const size_t total = static_cast<size_t>(input.total());
            for (size_t i = 0; i < total; ++i)
            {
                p[i] = dist(rng);
            }
        }

        auto printShape = [](const cv::Mat& m, const std::string& name) {
            std::string line = name + " dims=" + std::to_string(m.dims) + " [";
            for (int i = 0; i < m.dims; ++i)
            {
                line += std::to_string(m.size[i]);
                if (i + 1 != m.dims) line += ",";
            }
            line += "], type=" + std::to_string(m.type());
            LOG_INFO("%s", line.c_str());
        };

        auto printHead = [](const cv::Mat& m, int maxCount, const std::string& name) {
            const int n = std::max(0, std::min(maxCount, static_cast<int>(m.total())));
            std::string line = name + " first " + std::to_string(n) + " elements: ";

            if (m.type() == CV_32F)
            {
                const float* p = reinterpret_cast<const float*>(m.data);
                for (int i = 0; i < n; ++i)
                {
                    line += std::to_string(p[i]);
                    if (i + 1 != n) line += ", ";
                }
            }
            else if (m.type() == CV_32S)
            {
                const int* p = reinterpret_cast<const int*>(m.data);
                for (int i = 0; i < n; ++i)
                {
                    line += std::to_string(p[i]);
                    if (i + 1 != n) line += ", ";
                }
            }
            else if (m.type() == CV_16S)
            {
                const short* p = reinterpret_cast<const short*>(m.data);
                for (int i = 0; i < n; ++i)
                {
                    line += std::to_string(p[i]);
                    if (i + 1 != n) line += ", ";
                }
            }
            else if (m.type() == CV_8S)
            {
                const signed char* p = reinterpret_cast<const signed char*>(m.data);
                for (int i = 0; i < n; ++i)
                {
                    line += std::to_string(static_cast<int>(p[i]));
                    if (i + 1 != n) line += ", ";
                }
            }
            else if (m.type() == CV_8U)
            {
                const unsigned char* p = reinterpret_cast<const unsigned char*>(m.data);
                for (int i = 0; i < n; ++i)
                {
                    line += std::to_string(static_cast<int>(p[i]));
                    if (i + 1 != n) line += ", ";
                }
            }
            else
            {
                line += "(unsupported type)";
            }

            LOG_INFO("%s", line.c_str());
        };

        LOG_INFO("[ONNXModel::SmokeTest] Created random float32 input.");
        printShape(input, "Input");
        printHead(input, 10, "Input");

        long long totalUs = 0;
        long long minUs = std::numeric_limits<long long>::max();
        long long maxUs = 0;

        for (int i = 0; i < nTimes; ++i)
        {
            auto t0 = std::chrono::high_resolution_clock::now();
            auto outs = infer(input, true);
            auto t1 = std::chrono::high_resolution_clock::now();

            const long long us =
                std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

            totalUs += us;
            minUs = std::min(minUs, us);
            maxUs = std::max(maxUs, us);

            const double ms = static_cast<double>(us) / 1000.0;
            LOG_INFO("[ONNXModel::SmokeTest] Iter %d done, time = %.3f ms, output tensor count = %d",
                     i, ms, static_cast<int>(outs.size()));

            if (i == 0)
            {
                for (size_t k = 0; k < outs.size(); ++k)
                {
                    const std::string name = "Output[" + std::to_string(k) + "]";
                    printShape(outs[k], name);
                    printHead(outs[k], 10, name);
                }
            }
        }

        const double avgMs = static_cast<double>(totalUs) / static_cast<double>(nTimes) / 1000.0;
        const double minMs = static_cast<double>(minUs) / 1000.0;
        const double maxMs = static_cast<double>(maxUs) / 1000.0;

        LOG_INFO("[ONNXModel::SmokeTest] All %d iterations finished.", nTimes);
        LOG_INFO("[ONNXModel::SmokeTest] Avg = %.3f ms, Min = %.3f ms, Max = %.3f ms",
                 avgMs, minMs, maxMs);
    }

    const std::vector<std::string>& input_names() const noexcept { return input_names_; }
    const std::vector<std::string>& output_names() const noexcept { return output_names_; }
    bool using_gpu() const noexcept { return using_gpu_; }
    const std::string& device() const noexcept { return device_; }

private:
    static std::pair<std::string, std::optional<int>> parse_device(const std::string& device)
    {
        if (device == "cpu") return {"cpu", std::nullopt};
        if (device.rfind("cuda:", 0) == 0)
        {
            const auto idx_str = device.substr(5);
            if (!idx_str.empty() && std::all_of(idx_str.begin(), idx_str.end(), ::isdigit))
            {
                return {"cuda", std::stoi(idx_str)};
            }
        }
        throw std::invalid_argument("device 仅支持 'cpu' 或 'cuda:<index>'，例如 'cuda:0'");
    }

    static std::vector<int64_t> mat_shape_to_vec_i64(const cv::Mat& m)
    {
        std::vector<int64_t> shape;
        shape.reserve(static_cast<size_t>(m.dims));
        for (int i = 0; i < m.dims; ++i)
        {
            // OpenCV 的尺寸为 int，转为 int64_t
            shape.push_back(static_cast<int64_t>(m.size[i]));
        }
        return shape;
    }

    void init_io_names()
    {
        Ort::AllocatorWithDefaultOptions allocator;

        const size_t n_in = session_.GetInputCount();
        input_names_.reserve(n_in);
        for (size_t i = 0; i < n_in; ++i)
        {
            Ort::AllocatedStringPtr name = session_.GetInputNameAllocated(i, allocator);
            input_names_.emplace_back(name.get());
        }

        const size_t n_out = session_.GetOutputCount();
        output_names_.reserve(n_out);
        for (size_t i = 0; i < n_out; ++i)
        {
            Ort::AllocatedStringPtr name = session_.GetOutputNameAllocated(i, allocator);
            output_names_.emplace_back(name.get());
        }
    }

    void printModelIOInfo() const {
        Ort::AllocatorWithDefaultOptions allocator;

        LOG_INFO("=================== Model IO (ONNXRuntime) ===================");

        const size_t n_in = session_.GetInputCount();
        for (size_t i = 0; i < n_in; ++i) {
            Ort::AllocatedStringPtr name = session_.GetInputNameAllocated(i, allocator);

            Ort::TypeInfo ti = session_.GetInputTypeInfo(i);
            auto tinfo = ti.GetTensorTypeAndShapeInfo();
            const int etype = static_cast<int>(tinfo.GetElementType());
            std::vector<int64_t> shape = tinfo.GetShape();

            std::string line;
            line.reserve(256);
            line += "Input[";
            line += std::to_string(i);
            line += "] name=";
            line += name.get();
            line += ", fmt=";
            line += std::to_string(etype);
            line += ", dim=";
            line += std::to_string(shape.size());
            line += ", shape=[";

            for (size_t k = 0; k < shape.size(); ++k) {
                line += std::to_string(static_cast<long long>(shape[k]));
                if (k + 1 != shape.size()) line += ",";
            }
            line += "]";

            LOG_INFO("%s", line.c_str());
        }

        const size_t n_out = session_.GetOutputCount();
        for (size_t i = 0; i < n_out; ++i) {
            Ort::AllocatedStringPtr name = session_.GetOutputNameAllocated(i, allocator);

            Ort::TypeInfo ti = session_.GetOutputTypeInfo(i);
            auto tinfo = ti.GetTensorTypeAndShapeInfo();
            const int etype = static_cast<int>(tinfo.GetElementType());
            std::vector<int64_t> shape = tinfo.GetShape();

            std::string line;
            line.reserve(256);
            line += "Output[";
            line += std::to_string(i);
            line += "] name=";
            line += name.get();
            line += ", onnx fmt=";
            line += std::to_string(etype);
            line += ", dim=";
            line += std::to_string(shape.size());
            line += ", shape=[";

            for (size_t k = 0; k < shape.size(); ++k) {
                line += std::to_string(static_cast<long long>(shape[k]));
                if (k + 1 != shape.size()) line += ",";
            }
            line += "]";

            LOG_INFO("%s", line.c_str());
        }

        LOG_INFO("==============================================================");
    }

    Ort::Env env_;
    Ort::SessionOptions session_options_;
    Ort::Session session_;
    std::vector<std::string> input_names_;
    std::vector<std::string> output_names_;
    bool using_gpu_;
    std::string device_;
};
