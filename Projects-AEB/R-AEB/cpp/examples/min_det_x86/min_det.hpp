#pragma once

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#if defined(VISPER_ARCH_TI_X86)
class TidlOnnxModel;
#elif defined(VISPER_ARCH_TI)
class TIDLModel;
#elif defined(VISPER_ARCH_X86)
class ONNXModel;
#endif

struct MinDetArgs {
    std::string config_path;
    std::string model_path;
    std::string artifacts_dir;
    std::string ti_target = "DSP_C7-2";
    std::string input_path;
    std::string output_dir = "min_det_vis";
    int input_width = 0;
    int input_height = 0;
    int num_cls = 0;
    int num_mask = 0;
    float conf_thresh = -1.0f;
    float nms_thresh = -1.0f;
    float input_scale = 1.0f;
    float mask_thresh = 0.5f;
    int max_det = 100;
    bool recursive = false;
    bool use_nms = true;
    bool draw_masks = true;
    bool dump_outputs = false;
    std::vector<std::string> class_names;
};

struct Detection {
    cv::Rect2f box;
    float score = 0.0f;
    int cls = 0;
    cv::Mat mask; // Original image size, CV_8U, values 0/255.
};

class MinDetPipeline {
public:
    explicit MinDetPipeline(const MinDetArgs& args);
    ~MinDetPipeline();

    std::vector<Detection> infer(const cv::Mat& bgr) const;
    cv::Mat draw(const cv::Mat& bgr, const std::vector<Detection>& dets) const;

private:
    cv::Mat makeInputBlob(const cv::Mat& bgr) const;
    std::vector<Detection> decodeRaebDetections(const std::vector<cv::Mat>& outputs,
                                                const cv::Size& original_size) const;
    void dumpOutputsOnce(const std::vector<cv::Mat>& outputs) const;

    MinDetArgs args_;
    mutable bool outputs_dumped_ = false;

#if defined(VISPER_ARCH_TI_X86)
    std::unique_ptr<TidlOnnxModel> model_;
#elif defined(VISPER_ARCH_TI)
    std::unique_ptr<TIDLModel> model_;
#elif defined(VISPER_ARCH_X86)
    std::unique_ptr<ONNXModel> model_;
#endif
};

void loadRaebConfig(MinDetArgs& args);

std::vector<std::filesystem::path> collectInputImages(const std::filesystem::path& input,
                                                      bool recursive);
