#if !defined(VISPER_ARCH_SGS)
#error "test_visper_sgs.cpp must be built with ARCH=sgs"
#endif

// tests/test_mathops_unittest.cpp
// 最小可运行示例：参数通过命令行传入（支持 -m -c --xxx 形式）
// 用法：
//   ./demo -t <task> -c <config_path> -m <model_path> -i <image_path> [-o <out_dir>] [-r] [-l]
// 示例：
//   ./demo -t RAEB -c /path/config.jsonc -m /path/yolo11x-seg.onnx -i /path/img_or_dir -o vis -r -l

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <filesystem>
#include <set>
#include <string>
#include <vector>
#include <cstring>
#include <chrono>
#include <opencv2/opencv.hpp>

#include "VisPer.h"
#include "common/logger.hpp"  // LOG_INFO/LOG_WARNING/LOG_ERROR/LOG_CRITICAL

#if defined(VISPER_ARCH_SGS)
#include "mi_sys.h"
#include "mi_sys_datatype.h"
#endif

namespace fs = std::filesystem;

struct Args {
    std::string task;
    std::string config_path;
    std::string model_path;
    fs::path input_path;
    fs::path out_dir = "vis";
    bool recursive = false;
    bool loop = false; // 是否循环一直跑
};

static void print_usage(const char* prog) {
    std::cerr
        << "Usage:\n  " << prog
        << " -t <task> -c <config_path> -m <model_path> -i <image_path> [-o <out_dir>] [-r] [-l]\n"
        << "  -t, --task        任务名（如 RAEB）\n"
        << "  -c, --config      配置文件路径（.json/.jsonc）\n"
        << "  -m, --model       模型文件路径（如 .onnx）\n"
        << "  -i, --input       图片文件或文件夹路径\n"
        << "  -o, --outdir      可视化输出目录（默认：vis）\n"
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

#if defined(VISPER_ARCH_SGS)
class SgsMmaFrameBuffer {
public:
    SgsMmaFrameBuffer() = default;

    ~SgsMmaFrameBuffer() {
        release();
    }

    SgsMmaFrameBuffer(const SgsMmaFrameBuffer&) = delete;
    SgsMmaFrameBuffer& operator=(const SgsMmaFrameBuffer&) = delete;

    bool assign_nv12(const std::vector<std::uint8_t>& nv12, int width, int height) {
        if (nv12.empty() || width <= 0 || height <= 0) {
            return false;
        }
        if (width > 0xffff || height > 0xffff) {
            return false;
        }

        const MI_U32 required = static_cast<MI_U32>(nv12.size());
        if (!ensure_capacity(required)) {
            return false;
        }

        std::memcpy(vir_, nv12.data(), nv12.size());

        std::memset(&frame_, 0, sizeof(frame_));
        frame_.ePixelFormat = E_MI_SYS_PIXEL_FRAME_YUV_SEMIPLANAR_420;
        frame_.ePhylayoutType = NORMAL_FRAME_DATA;
        frame_.u16Width = static_cast<MI_U16>(width);
        frame_.u16Height = static_cast<MI_U16>(height);
        frame_.pVirAddr[0] = vir_;
        frame_.phyAddr[0] = phy_;
        frame_.u32Stride[0] = static_cast<MI_U32>(width);
        frame_.u32Stride[1] = static_cast<MI_U32>(width);
        frame_.u32BufSize = required;

        const std::size_t y_size = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
        if (y_size < nv12.size()) {
            frame_.pVirAddr[1] = static_cast<std::uint8_t*>(vir_) + y_size;
            frame_.phyAddr[1] = phy_ + static_cast<MI_PHY>(y_size);
        }
        return true;
    }

    const MI_SYS_FrameData_t& frame() const {
        return frame_;
    }

private:
    bool ensure_capacity(MI_U32 required) {
        if (vir_ != nullptr && phy_ != 0 && size_ >= required) {
            return true;
        }

        release();

        MI_PHY phy = 0;
        MI_S32 ret = MI_SYS_MMA_Alloc(0, NULL, required, &phy);
        if (ret != MI_SUCCESS) {
            LOG_ERROR("MI_SYS_MMA_Alloc failed ret=%d size=%u", static_cast<int>(ret), static_cast<unsigned>(required));
            return false;
        }

        void* vir = nullptr;
        ret = MI_SYS_Mmap(phy, required, &vir, TRUE);
        if (ret != MI_SUCCESS) {
            LOG_ERROR("MI_SYS_Mmap failed ret=%d phy=0x%llx size=%u", static_cast<int>(ret), static_cast<unsigned long long>(phy), static_cast<unsigned>(required));
            MI_SYS_MMA_Free(0, phy);
            return false;
        }

        phy_ = phy;
        vir_ = vir;
        size_ = required;
        return true;
    }

    void release() {
        if (vir_ != nullptr && size_ != 0) {
            MI_SYS_Munmap(vir_, size_);
        }
        if (phy_ != 0) {
            MI_SYS_MMA_Free(0, phy_);
        }
        vir_ = nullptr;
        phy_ = 0;
        size_ = 0;
        std::memset(&frame_, 0, sizeof(frame_));
    }

    MI_SYS_FrameData_t frame_{};
    void* vir_{nullptr};
    MI_PHY phy_{0};
    MI_U32 size_{0};
};
#endif

int main(int argc, char** argv) {
    Args args;
    if (!parse_args(argc, argv, args)) {
        print_usage(argv[0]);
        return 1;
    }

    LOG_INFO("Init task=%s, config=%s, model=%s",
             args.task.c_str(), args.config_path.c_str(), args.model_path.c_str());

    // 初始化（一次）
    VisPer_InitTask(args.task, args.config_path, args.model_path);

    // 收集待处理图片
    if (!fs::exists(args.input_path)) {
        LOG_ERROR("Path not found: %s", args.input_path.string().c_str());
        VisPer_CleanUp();
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
        VisPer_CleanUp();
        return 1;
    }

    if (images.empty()) {
        LOG_ERROR("在路径中未找到任何支持的图片: %s", args.input_path.string().c_str());
        VisPer_CleanUp();
        return 1;
    }

    LOG_INFO("Found %zu images. out_dir=%s recursive=%d loop=%d",
             images.size(), args.out_dir.string().c_str(),
             args.recursive ? 1 : 0, args.loop ? 1 : 0);

    auto run_once = [&]() -> std::size_t {
        std::size_t ok_count = 0;
#if defined(VISPER_ARCH_SGS)
        SgsMmaFrameBuffer sgs_frame;
#endif

        for (std::size_t i = 0; i < images.size(); ++i) {
            const fs::path& p = images[i];

            cv::Mat img_bgr = cv::imread(p.string(), cv::IMREAD_COLOR);
            if (img_bgr.empty()) {
                LOG_WARNING("读图失败: %s", p.string().c_str());
                continue;
            }

            cv::Mat img_rgb;
            cv::cvtColor(img_bgr, img_rgb, cv::COLOR_BGR2RGB);

            std::vector<std::uint8_t> nv12;
            if (!rgb_to_nv12(img_rgb, nv12)) {
                LOG_WARNING("转 NV12 失败: %s", p.string().c_str());
                continue;
            }

#if defined(VISPER_ARCH_SGS)
            if (!sgs_frame.assign_nv12(nv12, img_rgb.cols, img_rgb.rows)) {
                LOG_WARNING("构造 SGS frame 失败: %s", p.string().c_str());
                continue;
            }
#endif
            const std::int64_t ts = static_cast<std::int64_t>(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()
                ).count()
            );

            auto t0 = std::chrono::steady_clock::now();
#if defined(VISPER_ARCH_SGS)
            VisPer_RunInfer(sgs_frame.frame(), ts);
#else
            const std::uint8_t* data = nv12.data();
            const std::size_t length = nv12.size();
            VisPer_RunInfer(data, length, ts);
#endif
            auto t1 = std::chrono::steady_clock::now();
            const long long infer_ms =
                (long long)std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

            auto result = VisPer_GetResult(args.task);
            (void)result;
            ++ok_count;
            LOG_INFO("处理完成: %s infer_ms=%lldms", p.string().c_str(), infer_ms);
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
        // VisPer_CleanUp();
        // return 0;
    } else {
        std::size_t ok_count = run_once();
        if (ok_count == 0) {
            LOG_ERROR("没有成功生成任何可视化结果。");
            VisPer_CleanUp();
            return 2;
        }
        LOG_INFO("完成：ok_count=%zu / %zu", ok_count, images.size());
        VisPer_CleanUp();
        return 0;
    }
}
