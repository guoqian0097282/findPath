#include "min_det.hpp"

#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>
#include <string>

#include <opencv2/imgcodecs.hpp>

namespace {

std::string defaultConfigPath()
{
    return "/opt_disk3/rd234421/Projects-SGS/R-AEB/assets/RAEB/config.jsonc";
}

std::string defaultModelPath()
{
#if defined(VISPER_ARCH_TI)
    return "/opt_disk3/rd234421/Projects-SGS/edgeai-tidl-tools-2/runtimes/examples/model-artifacts/ti_lyl_user_16bit_layers_rerun_20260703/artifacts";
#else
    return "/opt_disk3/rd234421/Projects-SGS/R-AEB/assets/RAEB/TI_lyl.onnx";
#endif
}

std::string defaultArtifactsPath()
{
    return "/opt_disk3/rd234421/Projects-SGS/edgeai-tidl-tools-2/runtimes/examples/model-artifacts/ti_lyl_user_16bit_layers_rerun_20260703/artifacts";
}

std::string defaultInputPath()
{
    return "/opt_disk3/rd234421/Projects-SGS/R-AEB/cpp/tests/vis-bike2";
}

void printUsage(const char* argv0)
{
    std::cerr
        << "Usage:\n"
        << "  " << argv0 << " [options]\n\n"
        << "Options:\n"
        << "  -c, --config <path>       RAEB config json/jsonc. Default: " << defaultConfigPath() << "\n"
        << "  -m, --model <path>        RAEB model path. ONNX for x86/ti_x86, TIDL .bin dir/file for ti. Default: " << defaultModelPath() << "\n"
        << "  -a, --artifacts <dir>     TI x86 TIDL artifacts dir. Default: " << defaultArtifactsPath() << "\n"
        << "  -d, --ti-target <target>  TI target for ARCH=ti. Default: DSP_C7-2\n"
        << "  -i, --input <path>        Image file or directory. Default: " << defaultInputPath() << "\n"
        << "  -o, --outdir <dir>        Output directory. Default: min_det_vis\n"
        << "  --size <w>x<h>            Override model input size from config.\n"
        << "  --conf <value>            Override confidence threshold from config.\n"
        << "  --nms <value>             Override NMS IoU threshold from config.\n"
        << "  --mask-thresh <value>     Override mask threshold from config. Default: 0.5\n"
        << "  --input-scale <value>     Input scale after uint8 to float. Default: 1.0\n"
        << "  --max-det <num>           Max detections per image. Default: 100\n"
        << "  --no-nms                  Draw all boxes above threshold without NMS.\n"
        << "  --no-mask                 Do not overlay segmentation masks.\n"
        << "  --dump-outputs            Print output tensor shapes once.\n"
        << "  -r, --recursive           Recursively scan input directory.\n"
        << "  -h, --help                Show this message.\n";
}

bool parseSize(const std::string& text, int& w, int& h)
{
    const std::size_t x = text.find('x');
    if (x == std::string::npos) {
        return false;
    }
    try {
        w = std::stoi(text.substr(0, x));
        h = std::stoi(text.substr(x + 1));
    } catch (const std::exception&) {
        return false;
    }
    return w > 0 && h > 0;
}

MinDetArgs parseArgs(int argc, char** argv)
{
    MinDetArgs args;
    args.config_path = defaultConfigPath();
    args.model_path = defaultModelPath();
    args.artifacts_dir = defaultArtifactsPath();
    args.input_path = defaultInputPath();

    for (int i = 1; i < argc; ++i) {
        const std::string opt = argv[i];
        auto needValue = [&](const char* name) -> std::string {
            if (i + 1 >= argc) {
                throw std::runtime_error(std::string("missing value for ") + name);
            }
            return argv[++i];
        };

        if (opt == "-h" || opt == "--help") {
            printUsage(argv[0]);
            std::exit(0);
        } else if (opt == "-c" || opt == "--config") {
            args.config_path = needValue(opt.c_str());
        } else if (opt == "-m" || opt == "--model") {
            args.model_path = needValue(opt.c_str());
        } else if (opt == "-a" || opt == "--artifacts") {
            args.artifacts_dir = needValue(opt.c_str());
        } else if (opt == "-d" || opt == "--ti-target" || opt == "--ti_target") {
            args.ti_target = needValue(opt.c_str());
        } else if (opt == "-i" || opt == "--input") {
            args.input_path = needValue(opt.c_str());
        } else if (opt == "-o" || opt == "--outdir") {
            args.output_dir = needValue(opt.c_str());
        } else if (opt == "--size") {
            const std::string value = needValue(opt.c_str());
            if (!parseSize(value, args.input_width, args.input_height)) {
                throw std::runtime_error("invalid --size value, expected WxH");
            }
        } else if (opt == "--conf") {
            args.conf_thresh = std::stof(needValue(opt.c_str()));
        } else if (opt == "--nms") {
            args.nms_thresh = std::stof(needValue(opt.c_str()));
        } else if (opt == "--mask-thresh") {
            args.mask_thresh = std::stof(needValue(opt.c_str()));
        } else if (opt == "--input-scale") {
            args.input_scale = std::stof(needValue(opt.c_str()));
        } else if (opt == "--max-det") {
            args.max_det = std::stoi(needValue(opt.c_str()));
        } else if (opt == "--no-nms") {
            args.use_nms = false;
        } else if (opt == "--no-mask") {
            args.draw_masks = false;
        } else if (opt == "--dump-outputs") {
            args.dump_outputs = true;
        } else if (opt == "-r" || opt == "--recursive") {
            args.recursive = true;
        } else {
            throw std::runtime_error("unknown option: " + opt);
        }
    }

    loadRaebConfig(args);
    return args;
}

} // namespace

int main(int argc, char** argv)
{
    try {
        MinDetArgs args = parseArgs(argc, argv);
        const std::vector<std::filesystem::path> images =
            collectInputImages(args.input_path, args.recursive);
        if (images.empty()) {
            throw std::runtime_error("no input images found: " + args.input_path);
        }

        std::filesystem::create_directories(args.output_dir);

        MinDetPipeline pipeline(args);
        std::size_t ok = 0;
        for (const std::filesystem::path& image_path : images) {
            cv::Mat image = cv::imread(image_path.string(), cv::IMREAD_COLOR);
            if (image.empty()) {
                std::cerr << "[WARN] skip unreadable image: " << image_path << "\n";
                continue;
            }

            std::vector<Detection> dets = pipeline.infer(image);
            cv::Mat vis = pipeline.draw(image, dets);

            const std::filesystem::path out_path =
                std::filesystem::path(args.output_dir) / image_path.filename();
            if (!cv::imwrite(out_path.string(), vis)) {
                std::cerr << "[WARN] failed to save: " << out_path << "\n";
                continue;
            }

            std::cout << "[OK] " << image_path.filename().string()
                      << " det=" << dets.size()
                      << " -> " << out_path << "\n";
            ++ok;
        }

        std::cout << "Done: " << ok << "/" << images.size()
                  << " images, output_dir=" << args.output_dir << "\n";
        return ok == 0 ? 2 : 0;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] " << e.what() << "\n";
        printUsage(argv[0]);
        return 1;
    }
}
