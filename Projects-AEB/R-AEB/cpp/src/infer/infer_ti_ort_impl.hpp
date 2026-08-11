#pragma once

#include <onnxruntime/core/providers/cpu/cpu_provider_factory.h>
#include <onnxruntime/core/providers/tidl/tidl_provider_factory.h>
#include <onnxruntime/core/session/onnxruntime_cxx_api.h>
#include <opencv2/core.hpp>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "common/logger.hpp"

class TidlOnnxModel
{
public:
    explicit TidlOnnxModel(const std::string& model_path,
                           const std::string& artifacts_dir = std::string())
        : model_path_(model_path),
          artifacts_dir_(resolveArtifactsDir(artifacts_dir)),
          env_(ORT_LOGGING_LEVEL_WARNING, "visper_ti_x86_onnxrt"),
          session_options_{},
          session_(nullptr)
    {
        if (!std::filesystem::is_regular_file(model_path_))
        {
            throw std::runtime_error("TI x86 ONNX model not found: " + model_path_);
        }
        if (!std::filesystem::is_directory(artifacts_dir_))
        {
            throw std::runtime_error("TI x86 artifacts directory not found: " + artifacts_dir_);
        }

        session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
        session_options_.SetLogSeverityLevel(3);

        c_api_tidl_options tidl_options{};
#if defined(VISPER_TI_X86_TIDL_PROVIDER_OPTIONS_API)
        Ort::ThrowOnError(OrtSessionOptionsInitialize_Tidl(&tidl_options));
        setTidlOption(tidl_options, "artifacts_folder", artifacts_dir_);
        setTidlOptionFromEnv(tidl_options, "debug_level", "VISPER_TIDL_DEBUG_LEVEL");
        setTidlOptionFromEnv(tidl_options, "core_number", "VISPER_TIDL_CORE_NUMBER");
        setTidlOptionFromEnv(tidl_options, "priority", "VISPER_TIDL_PRIORITY");
        setTidlOptionFromEnv(tidl_options, "max_pre_empt_delay", "VISPER_TIDL_MAX_PRE_EMPT_DELAY");
#else
        Ort::ThrowOnError(OrtSessionsOptionsSetDefault_Tidl(&tidl_options));
        copyOptionString(tidl_options.artifacts_folder, artifacts_dir_);
        tidl_options.debug_level = readEnvInt("VISPER_TIDL_DEBUG_LEVEL", tidl_options.debug_level);
        tidl_options.core_number = readEnvInt("VISPER_TIDL_CORE_NUMBER", tidl_options.core_number);
        tidl_options.priority = readEnvInt("VISPER_TIDL_PRIORITY", tidl_options.priority);
        tidl_options.max_pre_empt_delay =
            readEnvFloat("VISPER_TIDL_MAX_PRE_EMPT_DELAY", tidl_options.max_pre_empt_delay);
#endif

        Ort::ThrowOnError(OrtSessionOptionsAppendExecutionProvider_Tidl(session_options_, &tidl_options));

        session_ = std::make_unique<Ort::Session>(env_, model_path_.c_str(), session_options_);
        initIoInfo();
        printModelIOInfo();
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
        if (input_names_.empty())
        {
            throw std::runtime_error("模型未检测到输入。");
        }

        const cv::Mat src = input.isContinuous() ? input : input.clone();
        std::vector<int64_t> in_shape = matShapeToVecI64(src);

        Ort::MemoryInfo mem_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            mem_info,
            const_cast<float*>(reinterpret_cast<const float*>(src.data)),
            static_cast<size_t>(src.total()),
            in_shape.data(),
            in_shape.size());

        const char* input_names_c[1] = {input_names_[0].c_str()};
        std::vector<const char*> output_names_c;
        output_names_c.reserve(output_names_.size());
        for (const auto& name : output_names_)
        {
            output_names_c.push_back(name.c_str());
        }

        auto output_values = session_->Run(
            Ort::RunOptions{nullptr},
            input_names_c,
            &input_tensor,
            1,
            output_names_c.data(),
            output_names_c.size());

        std::vector<cv::Mat> results;
        results.reserve(output_values.size());
        for (auto& value : output_values)
        {
            results.emplace_back(copyTensorToMat(value, squeeze_batch));
        }
        const uint64_t dump_frame = nextOutputDumpFrame();
        dumpOutputTensors("raw", output_names_, results, dump_frame);

        std::vector<cv::Mat> normalized = normalizeRaebOutputsByName(results);
        const std::vector<std::string> normalized_names = normalizedOutputNames(normalized);
        dumpOutputTensors("normalized", normalized_names, normalized, dump_frame);
        return normalized;
    }

    void smokeTest(int nTimes)
    {
        if (nTimes <= 0)
        {
            LOG_INFO("[TidlOnnxModel::SmokeTest] nTimes <= 0, nothing to do.");
            return;
        }
        if (input_shapes_.empty())
        {
            throw std::runtime_error("TidlOnnxModel::SmokeTest: no input tensor.");
        }

        std::vector<int> sizes;
        sizes.reserve(input_shapes_[0].size());
        for (int64_t dim : input_shapes_[0])
        {
            if (dim <= 0)
            {
                dim = 1;
            }
            if (dim > static_cast<int64_t>(std::numeric_limits<int>::max()))
            {
                throw std::runtime_error("TidlOnnxModel::SmokeTest: input dimension exceeds int range.");
            }
            sizes.push_back(static_cast<int>(dim));
        }
        if (sizes.empty())
        {
            sizes.push_back(1);
        }

        cv::Mat input(static_cast<int>(sizes.size()), sizes.data(), CV_32F);
        std::mt19937 rng(1234);
        std::uniform_real_distribution<float> dist(0.0f, 255.0f);
        float* data = reinterpret_cast<float*>(input.data);
        for (size_t i = 0; i < input.total(); ++i)
        {
            data[i] = dist(rng);
        }

        long long totalUs = 0;
        for (int i = 0; i < nTimes; ++i)
        {
            const auto t0 = std::chrono::high_resolution_clock::now();
            auto outs = infer(input, true);
            const auto t1 = std::chrono::high_resolution_clock::now();
            const long long us =
                std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
            totalUs += us;
            LOG_INFO("[TidlOnnxModel::SmokeTest] Iter %d done, time = %.3f ms, output tensor count = %d",
                     i,
                     static_cast<double>(us) / 1000.0,
                     static_cast<int>(outs.size()));
        }

        LOG_INFO("[TidlOnnxModel::SmokeTest] Avg = %.3f ms",
                 static_cast<double>(totalUs) / static_cast<double>(nTimes) / 1000.0);
    }

    const std::vector<std::string>& outputNames() const
    {
        return output_names_;
    }

private:
    static std::string envString(const char* name)
    {
        const char* value = std::getenv(name);
        if (value == nullptr || value[0] == '\0')
        {
            return {};
        }
        return value;
    }

    static std::string resolveArtifactsDir(const std::string& explicit_dir)
    {
        if (!explicit_dir.empty())
        {
            return explicit_dir;
        }

        std::string env_dir = envString("VISPER_TIDL_ARTIFACTS_DIR");
        if (!env_dir.empty())
        {
            return env_dir;
        }

        throw std::runtime_error(
            "TI x86 TIDL artifacts directory is not set. "
            "Pass it as infer_InitRAEB second argument or set VISPER_TIDL_ARTIFACTS_DIR.");
    }

    static int readEnvInt(const char* name, int fallback)
    {
        const std::string value = envString(name);
        if (value.empty())
        {
            return fallback;
        }
        return std::stoi(value);
    }

    static float readEnvFloat(const char* name, float fallback)
    {
        const std::string value = envString(name);
        if (value.empty())
        {
            return fallback;
        }
        return std::stof(value);
    }

    static bool readEnvFlag(const char* name, bool fallback)
    {
        const std::string value = envString(name);
        if (value.empty())
        {
            return fallback;
        }
        return value == "1" || value == "true" || value == "TRUE" || value == "on" || value == "ON" ||
               value == "yes" || value == "YES";
    }

    template <std::size_t N>
    static void copyOptionString(char (&dst)[N], const std::string& value)
    {
        if (value.size() >= N)
        {
            throw std::runtime_error("TI x86 artifacts path is too long: " + value);
        }
        std::memset(dst, 0, N);
        std::memcpy(dst, value.c_str(), value.size());
    }

#if defined(VISPER_TI_X86_TIDL_PROVIDER_OPTIONS_API)
    static constexpr std::size_t kTidlOptionMaxStringLength = 512;

    static void setTidlOption(c_api_tidl_options& options,
                              const char* key,
                              const std::string& value)
    {
        if (value.size() >= kTidlOptionMaxStringLength)
        {
            throw std::runtime_error(std::string("TI x86 TIDL option value is too long: ") + key);
        }
        Ort::ThrowOnError(OrtSessionOptionsSet_Tidl(&options, key, value.c_str()));
    }

    static void setTidlOptionFromEnv(c_api_tidl_options& options,
                                     const char* key,
                                     const char* env_name)
    {
        const std::string value = envString(env_name);
        if (!value.empty())
        {
            setTidlOption(options, key, value);
        }
    }
#endif

    static std::vector<int64_t> matShapeToVecI64(const cv::Mat& mat)
    {
        std::vector<int64_t> shape;
        shape.reserve(static_cast<size_t>(mat.dims));
        for (int i = 0; i < mat.dims; ++i)
        {
            shape.push_back(static_cast<int64_t>(mat.size[i]));
        }
        return shape;
    }

    static std::string shapeToString(const std::vector<int64_t>& shape)
    {
        std::ostringstream oss;
        oss << "[";
        for (size_t i = 0; i < shape.size(); ++i)
        {
            if (i != 0)
            {
                oss << ",";
            }
            oss << shape[i];
        }
        oss << "]";
        return oss.str();
    }

    static std::string shapeToString(const std::vector<int>& shape)
    {
        std::ostringstream oss;
        oss << "[";
        for (size_t i = 0; i < shape.size(); ++i)
        {
            if (i != 0)
            {
                oss << ",";
            }
            oss << shape[i];
        }
        oss << "]";
        return oss.str();
    }

    static std::vector<int> cvShapeFromOrtShape(std::vector<int64_t> shape, bool squeeze_batch)
    {
        if (squeeze_batch && !shape.empty() && shape.front() == 1)
        {
            shape.erase(shape.begin());
        }
        if (shape.empty())
        {
            shape.push_back(1);
        }

        std::vector<int> sizes;
        sizes.reserve(shape.size());
        for (int64_t dim : shape)
        {
            if (dim < 0)
            {
                throw std::runtime_error("Dynamic output dimension is not resolved.");
            }
            if (dim > static_cast<int64_t>(std::numeric_limits<int>::max()))
            {
                throw std::runtime_error("Output dimension exceeds int range.");
            }
            sizes.push_back(static_cast<int>(dim));
        }
        return sizes;
    }

    static int cvTypeForElement(ONNXTensorElementDataType element_type)
    {
        switch (element_type)
        {
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT:
            return CV_32F;
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8:
            return CV_8U;
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT8:
            return CV_8S;
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT16:
            return CV_16U;
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT16:
            return CV_16S;
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT32:
            return CV_32S;
        default:
            throw std::runtime_error("Unsupported TI x86 ONNX output tensor type: " +
                                     std::to_string(static_cast<int>(element_type)));
        }
    }

    static size_t elementSizeForElement(ONNXTensorElementDataType element_type)
    {
        switch (element_type)
        {
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT:
            return sizeof(float);
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT8:
            return sizeof(uint8_t);
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT8:
            return sizeof(int8_t);
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_UINT16:
            return sizeof(uint16_t);
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT16:
            return sizeof(int16_t);
        case ONNX_TENSOR_ELEMENT_DATA_TYPE_INT32:
            return sizeof(int32_t);
        default:
            throw std::runtime_error("Unsupported TI x86 ONNX tensor element size for type: " +
                                     std::to_string(static_cast<int>(element_type)));
        }
    }

    static cv::Mat copyTensorToMat(Ort::Value& value, bool squeeze_batch)
    {
        if (!value.IsTensor())
        {
            throw std::runtime_error("TI x86 ONNX output is not a tensor.");
        }

        auto info = value.GetTensorTypeAndShapeInfo();
        const auto element_type = info.GetElementType();
        std::vector<int64_t> shape = info.GetShape();
        const size_t elem_count = static_cast<size_t>(info.GetElementCount());
        std::vector<int> sizes = cvShapeFromOrtShape(shape, squeeze_batch);
        cv::Mat output(static_cast<int>(sizes.size()), sizes.data(), cvTypeForElement(element_type));
        std::memcpy(output.data,
                    value.GetTensorRawData(),
                    elem_count * elementSizeForElement(element_type));
        if (element_type == ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
        {
            return output;
        }

        cv::Mat as_float;
        output.convertTo(as_float, CV_32F);
        return as_float;
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

    uint64_t nextOutputDumpFrame()
    {
        return output_dump_frame_++;
    }

    std::vector<std::string> normalizedOutputNames(const std::vector<cv::Mat>& outputs) const
    {
        if (outputs.size() == 3U)
        {
            return {"det_cat", "proto", "angle"};
        }

        std::vector<std::string> names;
        names.reserve(outputs.size());
        for (size_t i = 0; i < outputs.size(); ++i)
        {
            names.push_back("output_" + std::to_string(i));
        }
        return names;
    }

    void dumpOutputTensors(const std::string& stage,
                           const std::vector<std::string>& names,
                           const std::vector<cv::Mat>& outputs,
                           uint64_t frame_index) const
    {
        if (!dump_outputs_enabled_)
        {
            return;
        }

        namespace fs = std::filesystem;
        const fs::path dump_dir = fs::path(dump_dir_);
        std::error_code ec;
        fs::create_directories(dump_dir, ec);
        if (ec)
        {
            LOG_ERROR("Failed to create TI x86 output dump dir: %s", dump_dir.string().c_str());
            return;
        }

        for (size_t i = 0; i < outputs.size(); ++i)
        {
            const std::string name = i < names.size() ? names[i] : ("output_" + std::to_string(i));
            dumpOutputTensor(stage, static_cast<uint32_t>(i), name, frame_index, outputs[i], dump_dir);
        }
    }

    void dumpOutputTensor(const std::string& stage,
                          uint32_t id,
                          const std::string& name,
                          uint64_t frame_index,
                          const cv::Mat& output,
                          const std::filesystem::path& dump_dir) const
    {
        try
        {
            namespace fs = std::filesystem;
            const std::string stem = "frame_" + std::to_string(frame_index) +
                "_" + stage +
                "_output_" + std::to_string(id) + "_" + safeFilenamePart(name);
            const fs::path bin_path = dump_dir / (stem + ".f32.bin");
            const fs::path json_path = dump_dir / (stem + ".json");

            cv::Mat as_float;
            const cv::Mat* src = &output;
            if (output.type() != CV_32F)
            {
                output.convertTo(as_float, CV_32F);
                src = &as_float;
            }
            const cv::Mat continuous = src->isContinuous() ? *src : src->clone();
            const size_t value_count = continuous.total();
            const size_t byte_count = value_count * sizeof(float);

            {
                std::ofstream ofs(bin_path, std::ios::binary);
                if (!ofs)
                {
                    LOG_ERROR("Failed to open TI x86 output dump file: %s", bin_path.string().c_str());
                    return;
                }
                ofs.write(reinterpret_cast<const char*>(continuous.ptr<float>()),
                          static_cast<std::streamsize>(byte_count));
                if (!ofs)
                {
                    LOG_ERROR("Failed to write TI x86 output dump file: %s", bin_path.string().c_str());
                    return;
                }
            }

            {
                std::ofstream meta(json_path);
                if (!meta)
                {
                    LOG_ERROR("Failed to open TI x86 output dump metadata file: %s",
                              json_path.string().c_str());
                    return;
                }

                meta << "{\n"
                     << "  \"file\": \"" << jsonEscape(bin_path.string()) << "\",\n"
                     << "  \"dtype\": \"float32\",\n"
                     << "  \"byte_order\": \"little_endian\",\n"
                     << "  \"order\": \"logical_contiguous_float_output\",\n"
                     << "  \"backend\": \"ti_x86_onnxruntime_tidl_ep\",\n"
                     << "  \"stage\": \"" << jsonEscape(stage) << "\",\n"
                     << "  \"frame_index\": " << frame_index << ",\n"
                     << "  \"output_index\": " << id << ",\n"
                     << "  \"output_name\": \"" << jsonEscape(name) << "\",\n"
                     << "  \"shape\": " << shapeToString(matShape(continuous)) << ",\n"
                     << "  \"value_count\": " << value_count << ",\n"
                     << "  \"byte_count\": " << byte_count << ",\n"
                     << "  \"onnx_model\": \"" << jsonEscape(model_path_) << "\",\n"
                     << "  \"artifacts_folder\": \"" << jsonEscape(artifacts_dir_) << "\"\n"
                     << "}\n";
            }

            LOG_INFO("Dumped TI x86 %s output[%u] name=%s to %s",
                     stage.c_str(),
                     id,
                     name.c_str(),
                     bin_path.string().c_str());
        }
        catch (const std::exception& e)
        {
            LOG_ERROR("Failed to dump TI x86 %s output[%u] name=%s: %s",
                      stage.c_str(),
                      id,
                      name.c_str(),
                      e.what());
        }
    }

    int outputIndex(const std::string& name) const
    {
        const auto it = std::find(output_names_.begin(), output_names_.end(), name);
        if (it == output_names_.end())
        {
            return -1;
        }
        return static_cast<int>(std::distance(output_names_.begin(), it));
    }

    static cv::Mat asFeatureRows(const cv::Mat& tensor, int expected_rows, const std::string& name)
    {
        if (tensor.type() != CV_32F)
        {
            throw std::runtime_error(name + " must be CV_32F.");
        }

        cv::Mat shaped;
        if (tensor.dims == 1)
        {
            shaped = tensor.reshape(1, 1);
        }
        else if (tensor.dims == 2)
        {
            shaped = tensor;
        }
        else
        {
            throw std::runtime_error(name + " must be 1D or 2D after batch squeeze.");
        }

        if (shaped.rows == expected_rows)
        {
            return shaped.isContinuous() ? shaped.clone() : shaped.clone();
        }
        if (shaped.cols == expected_rows)
        {
            cv::Mat transposed;
            cv::transpose(shaped, transposed);
            return transposed;
        }

        std::ostringstream oss;
        oss << name << " shape mismatch, expected one dimension to be " << expected_rows
            << ", got rows=" << shaped.rows << ", cols=" << shaped.cols;
        throw std::runtime_error(oss.str());
    }

    static cv::Mat asFeatureRowsWithLength(const cv::Mat& tensor,
                                           int expected_length,
                                           const std::string& name)
    {
        if (tensor.type() != CV_32F)
        {
            throw std::runtime_error(name + " must be CV_32F.");
        }
        if (tensor.dims != 2)
        {
            throw std::runtime_error(name + " must be 2D after batch squeeze.");
        }

        if (tensor.cols == expected_length)
        {
            return tensor.isContinuous() ? tensor.clone() : tensor.clone();
        }
        if (tensor.rows == expected_length)
        {
            cv::Mat transposed;
            cv::transpose(tensor, transposed);
            return transposed;
        }

        std::ostringstream oss;
        oss << name << " length mismatch, expected one dimension to be " << expected_length
            << ", got rows=" << tensor.rows << ", cols=" << tensor.cols;
        throw std::runtime_error(oss.str());
    }

    std::vector<cv::Mat> normalizeRaebOutputsByName(const std::vector<cv::Mat>& outputs) const
    {
        const int det_cat_idx = outputIndex("det_cat");
        const int proto_idx = outputIndex("proto");
        const int angle_idx = outputIndex("angle");
        if (det_cat_idx >= 0 && proto_idx >= 0 && angle_idx >= 0)
        {
            return {outputs[det_cat_idx], outputs[proto_idx], outputs[angle_idx]};
        }

        const int det_box_idx = outputIndex("det_box");
        const int det_cls_idx = outputIndex("det_cls");
        const int det_mask_idx = outputIndex("det_mask");
        if (det_box_idx >= 0 && det_cls_idx >= 0 && det_mask_idx >= 0 &&
            proto_idx >= 0 && angle_idx >= 0)
        {
            const cv::Mat det_box = asFeatureRows(outputs[det_box_idx], 4, "det_box");
            int mask_rows = -1;
            const cv::Mat& proto = outputs[proto_idx];
            if (proto.dims == 3)
            {
                mask_rows = proto.size[0];
            }
            if (mask_rows <= 0)
            {
                throw std::runtime_error("proto must be 3D and expose mask channel count.");
            }

            const cv::Mat det_mask = asFeatureRows(outputs[det_mask_idx], mask_rows, "det_mask");
            const int det_len = det_box.cols;
            if (det_mask.cols != det_len)
            {
                throw std::runtime_error("det_box and det_mask prediction lengths do not match.");
            }
            const cv::Mat det_cls = asFeatureRowsWithLength(outputs[det_cls_idx], det_len, "det_cls");

            cv::Mat det_cat;
            std::vector<cv::Mat> parts = {det_box, det_cls, det_mask};
            cv::vconcat(parts, det_cat);
            return {det_cat, proto, outputs[angle_idx]};
        }

        return outputs;
    }

    void initIoInfo()
    {
        Ort::AllocatorWithDefaultOptions allocator;

        const size_t input_count = session_->GetInputCount();
        input_names_.reserve(input_count);
        input_shapes_.reserve(input_count);
        for (size_t i = 0; i < input_count; ++i)
        {
            auto name = session_->GetInputNameAllocated(i, allocator);
            input_names_.emplace_back(name.get());
            auto type_info = session_->GetInputTypeInfo(i);
            auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
            input_shapes_.push_back(tensor_info.GetShape());
        }

        const size_t output_count = session_->GetOutputCount();
        output_names_.reserve(output_count);
        output_shapes_.reserve(output_count);
        for (size_t i = 0; i < output_count; ++i)
        {
            auto name = session_->GetOutputNameAllocated(i, allocator);
            output_names_.emplace_back(name.get());
            auto type_info = session_->GetOutputTypeInfo(i);
            auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
            output_shapes_.push_back(tensor_info.GetShape());
        }
    }

    void printModelIOInfo() const
    {
        LOG_INFO("=================== Model IO (TI x86 ONNXRuntime/TIDL EP) ===================");
        LOG_INFO("ONNX model       : %s", model_path_.c_str());
        LOG_INFO("Artifacts folder : %s", artifacts_dir_.c_str());
        for (size_t i = 0; i < input_names_.size(); ++i)
        {
            LOG_INFO("Input[%zu]  name=%s shape=%s",
                     i,
                     input_names_[i].c_str(),
                     shapeToString(input_shapes_[i]).c_str());
        }
        for (size_t i = 0; i < output_names_.size(); ++i)
        {
            LOG_INFO("Output[%zu] name=%s shape=%s",
                     i,
                     output_names_[i].c_str(),
                     shapeToString(output_shapes_[i]).c_str());
        }
        LOG_INFO("=========================================================================");
    }

    std::string model_path_;
    std::string artifacts_dir_;
    Ort::Env env_;
    Ort::SessionOptions session_options_;
    std::unique_ptr<Ort::Session> session_;
    std::vector<std::string> input_names_;
    std::vector<std::string> output_names_;
    std::vector<std::vector<int64_t>> input_shapes_;
    std::vector<std::vector<int64_t>> output_shapes_;
    uint64_t output_dump_frame_ = 0;
    bool dump_outputs_enabled_ = readEnvFlag("VISPER_TI_DUMP_OUTPUTS", false);
    std::string dump_dir_ = "ti_output_dump";
};
