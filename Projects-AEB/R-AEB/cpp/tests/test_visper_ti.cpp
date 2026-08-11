#if !defined(VISPER_ARCH_TI) && !defined(VISPER_ARCH_TI_X86)
#error "test_visper_ti.cpp must be built with ARCH=ti or ARCH=ti_x86"
#endif

// tests/test_mathops_unittest.cpp
// 最小可运行示例：参数通过命令行传入（支持 -m -c --xxx 形式）
// 用法：
//   ./demo -t <task> -c <config_path> -m <model_path> -i <image_path> [-o <out_dir>] [-r] [-l]
// 示例：
//   ./demo -t RAEB -c /path/config.jsonc -m /path/yolo11x-seg.onnx -i /path/img_or_dir -o vis -r -l

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cstdint>
#include <filesystem>
#include <set>
#include <string>
#include <vector>
#include <cstring>
#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "VisPer.h"
#include "common/logger.hpp"  // LOG_INFO/LOG_WARNING/LOG_ERROR/LOG_CRITICAL
#include "infer/infer_api.h"
#include "preproc/preproc_api.h"

namespace fs = std::filesystem;

struct Args {
    std::string task;
    std::string config_path;
    std::string model_path;
    std::string ti_target = "DSP_C7-2";
    fs::path input_path;
    fs::path out_dir = "vis";
    bool recursive = false;
    bool loop = false; // 是否循环一直跑
    bool direct_tidl = false; // 直接压 TIDL，跳过 VisPer 外层和 StageB
    int workers = 1;
    std::size_t stress_frames = 300;
};

static void print_usage(const char* prog) {
    std::cerr
        << "Usage:\n  " << prog
        << " -t <task> -c <config_path> -m <model_path> -i <image_path> [-o <out_dir>] [-r] [-l] [-d <target>]\n"
        << "  -t, --task        任务名（如 RAEB）\n"
        << "  -c, --config      配置文件路径（.json/.jsonc）\n"
        << "  -m, --model       模型文件路径（如 .onnx）\n"
        << "  -i, --input       图片文件或文件夹路径\n"
        << "  -o, --outdir      可视化输出目录（默认：vis）\n"
        << "  -d, --ti-target   TI TIDL target（默认：DSP_C7-2）\n"
        << "      --direct-tidl  直接循环调用 TIDL infer API，用于压测 DSP 饱和度\n"
        << "      --workers N    direct TIDL pipeline 深度（默认：1，建议 2 或 3）\n"
        << "      --stress-frames N  direct TIDL 每轮压测帧数（默认：300）\n"
        << "  -r, --recursive   递归遍历输入文件夹\n"
        << "  -l, --loop        循环一直跑，直到手动中断\n"
        << "  -h, --help        显示此帮助\n";
}

static bool has_image_ext(const fs::path& p) {
    static const std::set<std::string> exts = {
        ".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"
    };
    std::string e = p.extension().string();
    std::transform(e.begin(), e.end(), e.begin(), ::tolower);
    return exts.count(e) > 0;
}

static bool parse_args(int argc, char** argv, Args& args) {
    if (argc <= 1) return false;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];

        auto need_val = [&](const char* opt, std::string& out) -> bool {
            if (i + 1 >= argc) {
                std::cerr << "Option requires a value: " << opt << "\n";
                return false;
            }
            out = std::string(argv[++i]);
            return true;
        };

        if (a == "-h" || a == "--help") {
            print_usage(argv[0]);
            return false;
        } else if (a == "-t" || a == "--task") {
            if (!need_val(a.c_str(), args.task)) return false;
        } else if (a == "-c" || a == "--config") {
            if (!need_val(a.c_str(), args.config_path)) return false;
        } else if (a == "-m" || a == "--model") {
            if (!need_val(a.c_str(), args.model_path)) return false;
        } else if (a == "-i" || a == "--input") {
            std::string v;
            if (!need_val(a.c_str(), v)) return false;
            args.input_path = v;
        } else if (a == "-o" || a == "--outdir") {
            std::string v;
            if (!need_val(a.c_str(), v)) return false;
            args.out_dir = v;
        } else if (a == "-d" || a == "--ti-target" || a == "--ti_target") {
            if (!need_val(a.c_str(), args.ti_target)) return false;
        } else if (a == "--direct-tidl" || a == "--direct_tidl") {
            args.direct_tidl = true;
        } else if (a == "--workers") {
            std::string v;
            if (!need_val(a.c_str(), v)) return false;
            try {
                args.workers = std::max(1, std::stoi(v));
            } catch (...) {
                std::cerr << "Invalid --workers value: " << v << "\n";
                return false;
            }
        } else if (a == "--stress-frames" || a == "--stress_frames") {
            std::string v;
            if (!need_val(a.c_str(), v)) return false;
            try {
                const unsigned long long parsed = std::stoull(v);
                args.stress_frames = std::max<std::size_t>(1U, static_cast<std::size_t>(parsed));
            } catch (...) {
                std::cerr << "Invalid --stress-frames value: " << v << "\n";
                return false;
            }
        } else if (a == "-r" || a == "--recursive") {
            args.recursive = true;
        } else if (a == "-l" || a == "--loop") {
            args.loop = true;
        } else {
            std::cerr << "Unknown option: " << a << "\n";
            return false;
        }
    }

    if (args.task.empty() || args.config_path.empty() || args.model_path.empty() || args.input_path.empty()) {
        std::cerr << "Missing required options.\n";
        return false;
    }
    return true;
}

struct PreparedInput {
    fs::path path;
    std::vector<std::uint8_t> nv12;
    cv::Mat tensor;
};

// RGB -> NV12；成功返回 true，失败返回 false
static bool rgb_to_nv12(const cv::Mat& rgb, std::vector<std::uint8_t>& nv12) {
    nv12.clear();

    if (rgb.empty()) return false;
    if (rgb.type() != CV_8UC3) return false;

    const int width = rgb.cols;
    const int height = rgb.rows;

    if ((width % 2) != 0 || (height % 2) != 0) return false;

    cv::Mat yuv_i420;
    cv::cvtColor(rgb, yuv_i420, cv::COLOR_RGB2YUV_I420);

    if (!yuv_i420.isContinuous()) {
        yuv_i420 = yuv_i420.clone();
        if (!yuv_i420.isContinuous()) return false;
    }

    const std::size_t y_size = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    const std::size_t uv_size = y_size / 4;

    const std::uint8_t* src = yuv_i420.ptr<std::uint8_t>(0);
    const std::uint8_t* src_y = src;
    const std::uint8_t* src_u = src_y + y_size;
    const std::uint8_t* src_v = src_u + uv_size;

    nv12.resize(y_size + 2 * uv_size);

    std::uint8_t* dst_y = nv12.data();
    std::uint8_t* dst_uv = dst_y + y_size;

    std::memcpy(dst_y, src_y, y_size);

    for (std::size_t i = 0; i < uv_size; ++i) {
        dst_uv[2 * i + 0] = src_u[i];
        dst_uv[2 * i + 1] = src_v[i];
    }

    return true;
}

int main(int argc, char** argv) {
    Args args;
    if (!parse_args(argc, argv, args)) {
        print_usage(argv[0]);
        return 1;
    }

    LOG_INFO("Init task=%s, config=%s, model=%s, ti_target=%s",
             args.task.c_str(), args.config_path.c_str(), args.model_path.c_str(), args.ti_target.c_str());

    if (args.direct_tidl) {
        const std::string slots = std::to_string(std::max(1, args.workers));
        setenv("VISPER_TI_GRAPH_SLOTS", slots.c_str(), 1);

        std::ifstream ifs(args.config_path);
        if (!ifs) {
            LOG_ERROR("无法打开配置文件: %s", args.config_path.c_str());
            return 1;
        }
        nlohmann::json cfg = nlohmann::json::parse(ifs, nullptr, true, true);
        preproc_Init(
            cfg["cyl"]["image_width"].get<int>(),
            cfg["cyl"]["image_height"].get<int>(),
            cfg["cyl"]["image_type"].get<std::string>(),
            cfg["model"]["target_type"].get<std::string>());
        infer_InitRAEB(args.model_path, args.ti_target);
    } else {
        // 初始化（一次）
        VisPer_InitTask(args.task, args.config_path, args.model_path, args.ti_target);
    }

    // 收集待处理图片
    if (!fs::exists(args.input_path)) {
        LOG_ERROR("Path not found: %s", args.input_path.string().c_str());
        if (args.direct_tidl) {
            infer_DeinitRAEB();
        } else {
            VisPer_CleanUp();
        }
        return 1;
    }

    std::vector<fs::path> images;
    if (fs::is_regular_file(args.input_path) && has_image_ext(args.input_path)) {
        images.push_back(args.input_path);
    } else if (fs::is_directory(args.input_path)) {
        if (args.recursive) {
            for (auto it = fs::recursive_directory_iterator(args.input_path);
                 it != fs::recursive_directory_iterator(); ++it) {
                if (it->is_regular_file() && has_image_ext(it->path())) {
                    images.push_back(it->path());
                }
            }
        } else {
            for (const auto& entry : fs::directory_iterator(args.input_path)) {
                if (entry.is_regular_file() && has_image_ext(entry.path())) {
                    images.push_back(entry.path());
                }
            }
        }
        std::sort(images.begin(), images.end());
    } else {
        LOG_ERROR("input_path 非法：既不是支持的图片文件，也不是目录: %s",
                  args.input_path.string().c_str());
        if (args.direct_tidl) {
            infer_DeinitRAEB();
        } else {
            VisPer_CleanUp();
        }
        return 1;
    }

    if (images.empty()) {
        LOG_ERROR("在路径中未找到任何支持的图片: %s", args.input_path.string().c_str());
        if (args.direct_tidl) {
            infer_DeinitRAEB();
        } else {
            VisPer_CleanUp();
        }
        return 1;
    }

    std::vector<PreparedInput> prepared_inputs;
    prepared_inputs.reserve(images.size());
    double image_load_ms = 0.0;
    double preprocess_ms = 0.0;
    for (const auto& p : images) {
        const auto load_begin = std::chrono::steady_clock::now();
        cv::Mat img_bgr = cv::imread(p.string(), cv::IMREAD_COLOR);
        const auto load_end = std::chrono::steady_clock::now();
        image_load_ms += static_cast<double>(
            std::chrono::duration_cast<std::chrono::microseconds>(load_end - load_begin).count()) / 1000.0;
        if (img_bgr.empty()) {
            LOG_WARNING("读图失败: %s", p.string().c_str());
            continue;
        }

        const auto prep_begin = std::chrono::steady_clock::now();
        cv::Mat img_rgb;
        cv::cvtColor(img_bgr, img_rgb, cv::COLOR_BGR2RGB);

        PreparedInput prepared;
        prepared.path = p;
        if (!rgb_to_nv12(img_rgb, prepared.nv12)) {
            LOG_WARNING("转 NV12 失败: %s", p.string().c_str());
            continue;
        }

        try {
            prepared.tensor = preproc_ToData(img_rgb, 0, std::vector<int>{0, 3, 1, 2}, CV_32F);
        } catch (const std::exception& e) {
            LOG_WARNING("预处理 tensor 失败: %s, error=%s", p.string().c_str(), e.what());
            continue;
        }
        const auto prep_end = std::chrono::steady_clock::now();
        preprocess_ms += static_cast<double>(
            std::chrono::duration_cast<std::chrono::microseconds>(prep_end - prep_begin).count()) / 1000.0;

        prepared_inputs.push_back(std::move(prepared));
    }

    if (prepared_inputs.empty()) {
        LOG_ERROR("没有成功预处理任何输入。");
        if (args.direct_tidl) {
            infer_DeinitRAEB();
        } else {
            VisPer_CleanUp();
        }
        return 2;
    }

    LOG_INFO("Found %zu images, prepared=%zu. out_dir=%s recursive=%d loop=%d direct_tidl=%d workers=%d stress_frames=%zu",
             images.size(),
             prepared_inputs.size(),
             args.out_dir.string().c_str(),
             args.recursive ? 1 : 0,
             args.loop ? 1 : 0,
             args.direct_tidl ? 1 : 0,
             args.workers,
             args.stress_frames);
    LOG_INFO("Input prepare timing: image_load=%.3f ms total (%.3f ms/img), preprocess=%.3f ms total (%.3f ms/img)",
             image_load_ms,
             image_load_ms / static_cast<double>(prepared_inputs.size()),
             preprocess_ms,
             preprocess_ms / static_cast<double>(prepared_inputs.size()));

    auto run_once = [&]() -> std::size_t {
        if (args.direct_tidl) {
            std::vector<cv::Mat> tensors;
            tensors.reserve(prepared_inputs.size());
            for (const auto& input : prepared_inputs) {
                tensors.push_back(input.tensor);
            }

            const std::size_t depth = static_cast<std::size_t>(std::max(1, args.workers));
            auto result = infer_RunRAEBTiPipelineStress(
                tensors,
                args.stress_frames,
                depth,
                /*read_outputs=*/false);
            result.preprocess_ms = preprocess_ms;
            const double completed = static_cast<double>(std::max<std::size_t>(1U, result.completed));
            LOG_INFO("direct TIDL pipeline done: submitted=%zu completed=%zu elapsed=%.3fms fps=%.2f depth=%zu",
                     result.submitted,
                     result.completed,
                     result.elapsed_ms,
                     result.fps,
                     depth);
            LOG_INFO("direct TIDL timing total: preprocess_once=%.3fms write_input=%.3fms schedule=%.3fms wait_graph=%.3fms read_output=%.3fms",
                     result.preprocess_ms,
                     result.write_input_ms,
                     result.schedule_ms,
                     result.wait_graph_ms,
                     result.read_output_ms);
            LOG_INFO("direct TIDL timing avg: write_input=%.3fms/frame schedule=%.3fms/frame wait_graph=%.3fms/frame read_output=%.3fms/frame wall=%.3fms/frame",
                     result.write_input_ms / completed,
                     result.schedule_ms / completed,
                     result.wait_graph_ms / completed,
                     result.read_output_ms / completed,
                     result.elapsed_ms / completed);
            return result.completed;
        }

        std::size_t ok_count = 0;

        for (std::size_t i = 0; i < prepared_inputs.size(); ++i) {
            const PreparedInput& input = prepared_inputs[i];
            const std::int64_t ts = static_cast<std::int64_t>(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()
                ).count()
            );

            auto t0 = std::chrono::steady_clock::now();
            const std::uint8_t* data = input.nv12.data();
            const std::size_t length = input.nv12.size();
            VisPer_RunInfer(data, length, ts);
            auto t1 = std::chrono::steady_clock::now();
            const long long infer_ms =
                (long long)std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

            auto result = VisPer_GetResult(args.task);
            (void)result;
            ++ok_count;
            LOG_INFO("处理完成: %s infer_ms=%lldms", input.path.string().c_str(), infer_ms);
        }

        return ok_count;
    };

    if (args.loop) {
        std::int64_t round = 0;
        while (true) {
            LOG_INFO("===== 新一轮处理 start (round=%lld) =====", (long long)round);

            std::size_t ok_count = run_once();

            if (ok_count == 0) {
                LOG_WARNING("本轮没有成功生成任何可视化结果。");
            } else {
                LOG_INFO("本轮完成：ok_count=%zu / %zu", ok_count, images.size());
            }

            ++round;
        }

        // 当前逻辑下不会走到这里；如果以后改成可退出循环，记得在退出前调用：
        // args.direct_tidl ? infer_DeinitRAEB() : VisPer_CleanUp();
        // return 0;
    } else {
        std::size_t ok_count = run_once();
        if (ok_count == 0) {
            LOG_ERROR("没有成功生成任何可视化结果。");
            if (args.direct_tidl) {
                infer_DeinitRAEB();
            } else {
                VisPer_CleanUp();
            }
            return 2;
        }
        LOG_INFO("完成：ok_count=%zu / %zu", ok_count, images.size());
        if (args.direct_tidl) {
            infer_DeinitRAEB();
        } else {
            VisPer_CleanUp();
        }
        return 0;
    }
}
