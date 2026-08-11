#include "min_det.hpp"

#if defined(VISPER_ARCH_TI_X86)
#include "infer/infer_ti_ort_impl.hpp"
#elif defined(VISPER_ARCH_TI)
#include "infer/infer_ti_impl.hpp"
#elif defined(VISPER_ARCH_X86)
#include "infer/infer_ort_impl.hpp"
#else
#error "min_det_x86 only supports VISPER_ARCH_X86, VISPER_ARCH_TI_X86, or VISPER_ARCH_TI"
#endif

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <unordered_set>

#include <opencv2/imgproc.hpp>
#include <nlohmann/json.hpp>

namespace {

bool hasImageExtension(const std::filesystem::path& p)
{
    static const std::unordered_set<std::string> exts = {
        ".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"
    };
    std::string ext = p.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return exts.find(ext) != exts.end();
}

std::string getenvString(const char* name)
{
    const char* value = std::getenv(name);
    if (value == nullptr || value[0] == '\0') {
        return {};
    }
    return value;
}

std::vector<int> matShape(const cv::Mat& m)
{
    std::vector<int> shape;
    shape.reserve(static_cast<std::size_t>(m.dims));
    for (int i = 0; i < m.dims; ++i) {
        shape.push_back(m.size[i]);
    }
    return shape;
}

std::string shapeString(const cv::Mat& m)
{
    std::ostringstream oss;
    oss << "[";
    for (int i = 0; i < m.dims; ++i) {
        if (i != 0) {
            oss << ",";
        }
        oss << m.size[i];
    }
    oss << "]";
    return oss.str();
}

int product(const std::vector<int>& values)
{
    return std::accumulate(values.begin(), values.end(), 1, std::multiplies<int>());
}

cv::Mat squeezeTo2D(const cv::Mat& tensor, const std::string& name)
{
    if (tensor.empty()) {
        throw std::runtime_error(name + " is empty");
    }
    if (tensor.type() != CV_32F) {
        throw std::runtime_error(name + " must be CV_32F");
    }

    cv::Mat src = tensor.isContinuous() ? tensor : tensor.clone();
    std::vector<int> squeezed;
    squeezed.reserve(static_cast<std::size_t>(src.dims));
    for (int i = 0; i < src.dims; ++i) {
        if (src.size[i] != 1) {
            squeezed.push_back(src.size[i]);
        }
    }
    if (squeezed.empty()) {
        squeezed.push_back(static_cast<int>(src.total()));
    }
    if (squeezed.size() == 1U) {
        return src.reshape(1, 1).clone();
    }
    if (squeezed.size() == 2U && product(squeezed) == static_cast<int>(src.total())) {
        return src.reshape(1, 2, squeezed.data()).clone();
    }

    throw std::runtime_error(name + " cannot be squeezed to 2D, shape=" + shapeString(tensor));
}

cv::Mat asFeatureRows(const cv::Mat& tensor, int expected_rows, const std::string& name)
{
    cv::Mat m = squeezeTo2D(tensor, name);
    if (m.rows == expected_rows) {
        return m;
    }
    if (m.cols == expected_rows) {
        cv::Mat t;
        cv::transpose(m, t);
        return t;
    }

    std::ostringstream oss;
    oss << name << " shape mismatch, expected one dimension to be " << expected_rows
        << ", got " << shapeString(tensor);
    throw std::runtime_error(oss.str());
}

cv::Mat asDetCatRows(const cv::Mat& tensor, int expected_rows, int min_rows, const std::string& name)
{
    cv::Mat m = squeezeTo2D(tensor, name);
    if (expected_rows > 0) {
        if (m.rows == expected_rows) {
            return m;
        }
        if (m.cols == expected_rows) {
            cv::Mat t;
            cv::transpose(m, t);
            return t;
        }
    }
    if (m.rows >= min_rows && m.rows <= 512 && m.cols > m.rows) {
        return m;
    }
    if (m.cols >= min_rows && m.cols <= 512 && m.rows > m.cols) {
        cv::Mat t;
        cv::transpose(m, t);
        return t;
    }

    std::ostringstream oss;
    oss << name << " is not a RAEB det_cat tensor, shape=" << shapeString(tensor)
        << ", min_rows=" << min_rows << ", expected_rows=" << expected_rows;
    throw std::runtime_error(oss.str());
}

int findName(const std::vector<std::string>& names, const std::string& wanted)
{
    const auto it = std::find(names.begin(), names.end(), wanted);
    if (it == names.end()) {
        return -1;
    }
    return static_cast<int>(std::distance(names.begin(), it));
}

std::vector<std::string> configuredOutputNames(
#if defined(VISPER_ARCH_TI_X86)
    const TidlOnnxModel& model
#elif defined(VISPER_ARCH_TI)
    const TIDLModel& model
#elif defined(VISPER_ARCH_X86)
    const ONNXModel& model
#endif
)
{
#if defined(VISPER_ARCH_TI_X86)
    return model.outputNames();
#elif defined(VISPER_ARCH_TI)
    return model.outputNames();
#elif defined(VISPER_ARCH_X86)
    return model.output_names();
#endif
}

cv::Mat rgbHwcToNchwF32(const cv::Mat& rgb, float scale)
{
    if (rgb.empty() || rgb.type() != CV_8UC3 || rgb.dims != 2) {
        throw std::runtime_error("rgb image must be non-empty HxWx3 CV_8UC3");
    }

    const int sizes[4] = {1, 3, rgb.rows, rgb.cols};
    cv::Mat blob(4, sizes, CV_32F);
    float* dst = blob.ptr<float>(0);
    const int hw = rgb.rows * rgb.cols;

    for (int y = 0; y < rgb.rows; ++y) {
        const cv::Vec3b* row = rgb.ptr<cv::Vec3b>(y);
        for (int x = 0; x < rgb.cols; ++x) {
            const int idx = y * rgb.cols + x;
            const cv::Vec3b& pix = row[x];
            dst[idx] = static_cast<float>(pix[0]) * scale;
            dst[hw + idx] = static_cast<float>(pix[1]) * scale;
            dst[2 * hw + idx] = static_cast<float>(pix[2]) * scale;
        }
    }
    return blob;
}

cv::Scalar colorForClass(int cls)
{
    static const cv::Scalar palette[] = {
        {0, 255, 0},     {255, 128, 0},   {0, 128, 255},   {255, 0, 255},
        {0, 255, 255},   {180, 80, 255},  {255, 255, 0},   {80, 180, 255},
        {80, 255, 180},  {255, 80, 120},  {160, 220, 80},  {220, 160, 80},
    };
    const int n = static_cast<int>(sizeof(palette) / sizeof(palette[0]));
    const int idx = ((cls % n) + n) % n;
    return palette[idx];
}

bool isProtoTensor(const cv::Mat& tensor, int num_mask)
{
    if (tensor.empty() || tensor.type() != CV_32F || tensor.dims != 3) {
        return false;
    }
    if (num_mask > 0) {
        return tensor.size[0] == num_mask || tensor.size[2] == num_mask;
    }
    return tensor.size[0] <= 512 && tensor.size[1] > 0 && tensor.size[2] > 0;
}

cv::Mat continuousTensor(const cv::Mat& tensor)
{
    return tensor.isContinuous() ? tensor : tensor.clone();
}

cv::Mat decodeInstanceMask(const cv::Mat& mask_rows,
                           const cv::Mat& proto,
                           int pred_idx,
                           const cv::Rect2f& original_box,
                           const MinDetArgs& args,
                           const cv::Size& original_size)
{
    if (!args.draw_masks || args.num_mask <= 0 || mask_rows.empty() || proto.empty()) {
        return {};
    }
    if (mask_rows.type() != CV_32F || mask_rows.dims != 2 ||
        pred_idx < 0 || pred_idx >= mask_rows.cols ||
        proto.type() != CV_32F || proto.dims != 3 ||
        original_size.width <= 0 || original_size.height <= 0) {
        return {};
    }

    bool proto_chw = true;
    if (proto.size[0] == mask_rows.rows) {
        proto_chw = true;
    } else if (proto.size[2] == mask_rows.rows) {
        proto_chw = false;
    } else {
        return {};
    }

    const int nm = mask_rows.rows;
    const int hm = proto_chw ? proto.size[1] : proto.size[0];
    const int wm = proto_chw ? proto.size[2] : proto.size[1];
    if (nm <= 0 || hm <= 0 || wm <= 0 ||
        args.input_width <= 0 || args.input_height <= 0) {
        return {};
    }

    const cv::Mat proto_cont = continuousTensor(proto);
    const float* proto_data = proto_cont.ptr<float>(0);
    const int hwm = hm * wm;

    std::vector<float> coeff(static_cast<std::size_t>(nm));
    for (int c = 0; c < nm; ++c) {
        coeff[static_cast<std::size_t>(c)] = mask_rows.at<float>(c, pred_idx);
    }

    cv::Mat low_mask(hm, wm, CV_32F);
    for (int y = 0; y < hm; ++y) {
        float* dst = low_mask.ptr<float>(y);
        for (int x = 0; x < wm; ++x) {
            float v = 0.0f;
            if (proto_chw) {
                const int hw_idx = y * wm + x;
                for (int c = 0; c < nm; ++c) {
                    v += coeff[static_cast<std::size_t>(c)] *
                         proto_data[static_cast<std::size_t>(c) * static_cast<std::size_t>(hwm) +
                                    static_cast<std::size_t>(hw_idx)];
                }
            } else {
                const float* pix = proto_data +
                    (static_cast<std::size_t>(y) * static_cast<std::size_t>(wm) +
                     static_cast<std::size_t>(x)) * static_cast<std::size_t>(nm);
                for (int c = 0; c < nm; ++c) {
                    v += coeff[static_cast<std::size_t>(c)] * pix[c];
                }
            }
            v = std::clamp(v, -50.0f, 50.0f);
            dst[x] = 1.0f / (1.0f + std::exp(-v));
        }
    }

    cv::Mat model_mask;
    cv::resize(low_mask,
               model_mask,
               cv::Size(args.input_width, args.input_height),
               0.0,
               0.0,
               cv::INTER_LINEAR);

    const float inv_sx = static_cast<float>(args.input_width) /
                         static_cast<float>(original_size.width);
    const float inv_sy = static_cast<float>(args.input_height) /
                         static_cast<float>(original_size.height);
    int x1 = static_cast<int>(std::floor(original_box.x * inv_sx));
    int y1 = static_cast<int>(std::floor(original_box.y * inv_sy));
    int x2 = static_cast<int>(std::ceil((original_box.x + original_box.width) * inv_sx));
    int y2 = static_cast<int>(std::ceil((original_box.y + original_box.height) * inv_sy));
    x1 = std::clamp(x1, 0, args.input_width);
    y1 = std::clamp(y1, 0, args.input_height);
    x2 = std::clamp(x2, 0, args.input_width);
    y2 = std::clamp(y2, 0, args.input_height);
    if (x2 <= x1 || y2 <= y1) {
        return {};
    }

    cv::Mat cropped = cv::Mat::zeros(model_mask.size(), CV_32F);
    const cv::Rect roi(x1, y1, x2 - x1, y2 - y1);
    model_mask(roi).copyTo(cropped(roi));

    cv::Mat original_mask_prob;
    if (original_size.width == args.input_width && original_size.height == args.input_height) {
        original_mask_prob = cropped;
    } else {
        cv::resize(cropped, original_mask_prob, original_size, 0.0, 0.0, cv::INTER_LINEAR);
    }

    cv::Mat binary;
    cv::compare(original_mask_prob, args.mask_thresh, binary, cv::CMP_GT);
    return binary;
}

float rectIou(const cv::Rect2f& a, const cv::Rect2f& b)
{
    const float x1 = std::max(a.x, b.x);
    const float y1 = std::max(a.y, b.y);
    const float x2 = std::min(a.x + a.width, b.x + b.width);
    const float y2 = std::min(a.y + a.height, b.y + b.height);
    const float w = std::max(0.0f, x2 - x1);
    const float h = std::max(0.0f, y2 - y1);
    const float inter = w * h;
    const float area_a = std::max(0.0f, a.width) * std::max(0.0f, a.height);
    const float area_b = std::max(0.0f, b.width) * std::max(0.0f, b.height);
    return inter / std::max(area_a + area_b - inter, 1e-12f);
}

std::vector<int> simpleNms(const std::vector<cv::Rect2f>& boxes,
                           const std::vector<float>& scores,
                           float iou_thresh)
{
    std::vector<int> order(scores.size());
    std::iota(order.begin(), order.end(), 0);
    std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
        return scores[static_cast<std::size_t>(a)] > scores[static_cast<std::size_t>(b)];
    });

    std::vector<int> keep;
    keep.reserve(order.size());
    for (int idx : order) {
        bool ok = true;
        for (int kept : keep) {
            if (rectIou(boxes[static_cast<std::size_t>(idx)],
                        boxes[static_cast<std::size_t>(kept)]) >= iou_thresh) {
                ok = false;
                break;
            }
        }
        if (ok) {
            keep.push_back(idx);
        }
    }
    return keep;
}

template <typename T>
T jsonValueOr(const nlohmann::json& j, const char* key, T fallback)
{
    if (!j.is_object() || !j.contains(key) || j.at(key).is_null()) {
        return fallback;
    }
    return j.at(key).get<T>();
}

} // namespace

void loadRaebConfig(MinDetArgs& args)
{
    std::ifstream ifs(args.config_path);
    if (!ifs) {
        throw std::runtime_error("failed to open RAEB config: " + args.config_path);
    }

    nlohmann::json cfg = nlohmann::json::parse(ifs, nullptr, true, true);
    if (!cfg.contains("model") || !cfg["model"].is_object()) {
        throw std::runtime_error("RAEB config missing object: model");
    }
    const nlohmann::json& model = cfg["model"];

    const int cyl_w = cfg.contains("cyl") && cfg["cyl"].is_object()
                          ? jsonValueOr<int>(cfg["cyl"], "image_width", 960)
                          : 960;
    const int cyl_h = cfg.contains("cyl") && cfg["cyl"].is_object()
                          ? jsonValueOr<int>(cfg["cyl"], "image_height", 480)
                          : 480;

    if (args.input_width <= 0) {
        args.input_width = jsonValueOr<int>(model, "target_width", cyl_w);
    }
    if (args.input_height <= 0) {
        args.input_height = jsonValueOr<int>(model, "target_height", cyl_h);
    }
    args.num_cls = jsonValueOr<int>(model, "num_cls", args.num_cls);
    args.num_mask = jsonValueOr<int>(model, "num_mask", args.num_mask);
    if (args.conf_thresh < 0.0f) {
        args.conf_thresh = jsonValueOr<float>(model, "conf_thresh", 0.5f);
    }
    if (args.nms_thresh < 0.0f) {
        args.nms_thresh = jsonValueOr<float>(model, "iou_thresh", 0.3f);
    }
    args.mask_thresh = jsonValueOr<float>(model, "mask_thresh", args.mask_thresh);

    if (args.input_width <= 0 || args.input_height <= 0) {
        throw std::runtime_error("invalid RAEB input size from config");
    }
    if (args.num_cls <= 0) {
        throw std::runtime_error("invalid RAEB num_cls from config");
    }
    if (args.num_mask < 0) {
        throw std::runtime_error("invalid RAEB num_mask from config");
    }

    args.class_names.assign(static_cast<std::size_t>(args.num_cls), std::string());
    for (int i = 0; i < args.num_cls; ++i) {
        args.class_names[static_cast<std::size_t>(i)] = std::to_string(i);
    }
    if (model.contains("id2name") && model["id2name"].is_object()) {
        for (const auto& item : model["id2name"].items()) {
            try {
                const int id = std::stoi(item.key());
                if (id >= 0 && id < args.num_cls && item.value().is_string()) {
                    args.class_names[static_cast<std::size_t>(id)] = item.value().get<std::string>();
                }
            } catch (const std::exception&) {
            }
        }
    }
}

MinDetPipeline::MinDetPipeline(const MinDetArgs& args)
    : args_(args)
{
    if (args_.model_path.empty()) {
        throw std::runtime_error("model path is empty");
    }
    if (args_.input_width <= 0 || args_.input_height <= 0) {
        throw std::runtime_error("input width/height must be positive");
    }

#if defined(VISPER_ARCH_TI_X86)
    std::string artifacts = args_.artifacts_dir;
    if (artifacts.empty()) {
        artifacts = getenvString("VISPER_TIDL_ARTIFACTS_DIR");
    }
    if (artifacts.empty()) {
        throw std::runtime_error(
            "TI x86 artifacts directory is empty. Pass --artifacts or set VISPER_TIDL_ARTIFACTS_DIR.");
    }
    model_ = std::make_unique<TidlOnnxModel>(args_.model_path, artifacts);
#elif defined(VISPER_ARCH_TI)
    model_ = std::make_unique<TIDLModel>(args_.model_path, args_.ti_target);
#elif defined(VISPER_ARCH_X86)
    model_ = std::make_unique<ONNXModel>(args_.model_path, "cpu");
#endif
}

MinDetPipeline::~MinDetPipeline() = default;

cv::Mat MinDetPipeline::makeInputBlob(const cv::Mat& bgr) const
{
    if (bgr.empty() || bgr.type() != CV_8UC3) {
        throw std::runtime_error("input image must be non-empty CV_8UC3");
    }

    cv::Mat resized;
    cv::resize(bgr, resized, cv::Size(args_.input_width, args_.input_height), 0.0, 0.0, cv::INTER_LINEAR);

    cv::Mat rgb;
    cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);

    cv::Mat blob = rgbHwcToNchwF32(rgb, args_.input_scale);
    return blob.isContinuous() ? blob : blob.clone();
}

std::vector<Detection> MinDetPipeline::infer(const cv::Mat& bgr) const
{
    cv::Mat blob = makeInputBlob(bgr);
    std::vector<cv::Mat> outputs = model_->infer(blob, true);
    dumpOutputsOnce(outputs);
    return decodeRaebDetections(outputs, bgr.size());
}

void MinDetPipeline::dumpOutputsOnce(const std::vector<cv::Mat>& outputs) const
{
    if (!args_.dump_outputs || outputs_dumped_) {
        return;
    }
    outputs_dumped_ = true;

    std::vector<std::string> names;
    if (model_) {
        names = configuredOutputNames(*model_);
    }
    std::cerr << "[INFO] model returned " << outputs.size() << " output tensor(s)\n";
    for (std::size_t i = 0; i < outputs.size(); ++i) {
        std::cerr << "  output[" << i << "]";
        if (names.size() == outputs.size()) {
            std::cerr << " name=" << names[i];
        }
        std::cerr << " shape=" << shapeString(outputs[i])
                  << " type=" << outputs[i].type()
                  << " total=" << outputs[i].total() << "\n";
    }
}

std::vector<Detection> MinDetPipeline::decodeRaebDetections(const std::vector<cv::Mat>& outputs,
                                                            const cv::Size& original_size) const
{
    if (outputs.empty()) {
        throw std::runtime_error("model returned no outputs");
    }

    cv::Mat box_rows;
    cv::Mat cls_rows;
    cv::Mat mask_rows;
    cv::Mat proto;

    std::vector<std::string> names;
    if (model_) {
        names = configuredOutputNames(*model_);
    }

    if (names.size() == outputs.size()) {
        const int box_idx = findName(names, "det_box");
        const int cls_idx = findName(names, "det_cls");
        const int mask_idx = findName(names, "det_mask");
        const int proto_idx = findName(names, "proto");
        if (box_idx >= 0 && cls_idx >= 0) {
            box_rows = asFeatureRows(outputs[static_cast<std::size_t>(box_idx)], 4, "det_box");
            cls_rows = asFeatureRows(outputs[static_cast<std::size_t>(cls_idx)], args_.num_cls, "det_cls");
            if (mask_idx >= 0 && args_.num_mask > 0) {
                mask_rows = asFeatureRows(outputs[static_cast<std::size_t>(mask_idx)],
                                          args_.num_mask,
                                          "det_mask");
            }
        }
        if (proto_idx >= 0 && isProtoTensor(outputs[static_cast<std::size_t>(proto_idx)], args_.num_mask)) {
            proto = outputs[static_cast<std::size_t>(proto_idx)];
        }
    }

    if (box_rows.empty() || cls_rows.empty()) {
        const int expected_cat_rows = 4 + args_.num_cls + args_.num_mask;
        const int min_cat_rows = 4 + args_.num_cls;
        std::string last_error;
        for (std::size_t i = 0; i < outputs.size(); ++i) {
            try {
                cv::Mat det_cat = asDetCatRows(outputs[i], expected_cat_rows, min_cat_rows, "det_cat");
                box_rows = det_cat.rowRange(0, 4).clone();
                cls_rows = det_cat.rowRange(4, 4 + args_.num_cls).clone();
                if (args_.num_mask > 0 && det_cat.rows >= 4 + args_.num_cls + args_.num_mask) {
                    mask_rows = det_cat.rowRange(4 + args_.num_cls,
                                                 4 + args_.num_cls + args_.num_mask).clone();
                }
                break;
            } catch (const std::exception& e) {
                last_error = e.what();
            }
        }
        if (box_rows.empty() || cls_rows.empty()) {
            throw std::runtime_error("cannot find RAEB det_box/det_cls or det_cat outputs. Last check: " +
                                     last_error);
        }
    }
    if (proto.empty()) {
        for (const cv::Mat& output : outputs) {
            if (isProtoTensor(output, args_.num_mask)) {
                proto = output;
                break;
            }
        }
    }

    if (box_rows.cols != cls_rows.cols) {
        throw std::runtime_error("det_box and det_cls prediction lengths do not match");
    }
    if (!mask_rows.empty() && mask_rows.cols != box_rows.cols) {
        mask_rows.release();
    }

    const int num_preds = box_rows.cols;
    const float sx = static_cast<float>(original_size.width) / static_cast<float>(args_.input_width);
    const float sy = static_cast<float>(original_size.height) / static_cast<float>(args_.input_height);

    std::vector<cv::Rect2f> boxes_f;
    std::vector<float> scores;
    std::vector<int> classes;
    std::vector<int> pred_indices;
    boxes_f.reserve(static_cast<std::size_t>(num_preds));
    scores.reserve(static_cast<std::size_t>(num_preds));
    classes.reserve(static_cast<std::size_t>(num_preds));
    pred_indices.reserve(static_cast<std::size_t>(num_preds));

    const float* row_x = box_rows.ptr<float>(0);
    const float* row_y = box_rows.ptr<float>(1);
    const float* row_w = box_rows.ptr<float>(2);
    const float* row_h = box_rows.ptr<float>(3);

    for (int i = 0; i < num_preds; ++i) {
        const float cx = row_x[i];
        const float cy = row_y[i];
        const float w = row_w[i];
        const float h = row_h[i];
        if (!std::isfinite(cx) || !std::isfinite(cy) || !std::isfinite(w) || !std::isfinite(h) ||
            w <= 0.0f || h <= 0.0f) {
            continue;
        }

        int cls = 0;
        float score = -std::numeric_limits<float>::infinity();
        for (int c = 0; c < args_.num_cls; ++c) {
            const float v = cls_rows.at<float>(c, i);
            if (std::isfinite(v) && v > score) {
                score = v;
                cls = c;
            }
        }
        if (score < args_.conf_thresh) {
            continue;
        }

        float x1 = (cx - 0.5f * w) * sx;
        float y1 = (cy - 0.5f * h) * sy;
        float x2 = (cx + 0.5f * w) * sx;
        float y2 = (cy + 0.5f * h) * sy;

        x1 = std::clamp(x1, 0.0f, static_cast<float>(std::max(0, original_size.width - 1)));
        y1 = std::clamp(y1, 0.0f, static_cast<float>(std::max(0, original_size.height - 1)));
        x2 = std::clamp(x2, 0.0f, static_cast<float>(std::max(0, original_size.width - 1)));
        y2 = std::clamp(y2, 0.0f, static_cast<float>(std::max(0, original_size.height - 1)));
        if (x2 <= x1 || y2 <= y1) {
            continue;
        }

        cv::Rect2f box_f(x1, y1, x2 - x1, y2 - y1);
        boxes_f.push_back(box_f);
        scores.push_back(score);
        classes.push_back(cls);
        pred_indices.push_back(i);
    }

    std::vector<int> keep;
    if (args_.use_nms) {
        keep = simpleNms(boxes_f, scores, args_.nms_thresh);
    } else {
        keep.resize(scores.size());
        std::iota(keep.begin(), keep.end(), 0);
        std::stable_sort(keep.begin(), keep.end(), [&](int a, int b) {
            return scores[static_cast<std::size_t>(a)] > scores[static_cast<std::size_t>(b)];
        });
    }

    if (args_.max_det > 0 && static_cast<int>(keep.size()) > args_.max_det) {
        keep.resize(static_cast<std::size_t>(args_.max_det));
    }

    std::vector<Detection> dets;
    dets.reserve(keep.size());
    for (int idx : keep) {
        Detection d;
        d.box = boxes_f[static_cast<std::size_t>(idx)];
        d.score = scores[static_cast<std::size_t>(idx)];
        d.cls = classes[static_cast<std::size_t>(idx)];
        if (!mask_rows.empty() && !proto.empty()) {
            d.mask = decodeInstanceMask(mask_rows,
                                        proto,
                                        pred_indices[static_cast<std::size_t>(idx)],
                                        d.box,
                                        args_,
                                        original_size);
        }
        dets.push_back(d);
    }
    return dets;
}

cv::Mat MinDetPipeline::draw(const cv::Mat& bgr, const std::vector<Detection>& dets) const
{
    cv::Mat out = bgr.clone();
    if (args_.draw_masks) {
        cv::Mat overlay = out.clone();
        for (const Detection& d : dets) {
            if (d.mask.empty() || d.mask.type() != CV_8U ||
                d.mask.rows != out.rows || d.mask.cols != out.cols) {
                continue;
            }
            overlay.setTo(colorForClass(d.cls), d.mask);
        }
        cv::addWeighted(overlay, 0.35, out, 0.65, 0.0, out);
    }

    for (const Detection& d : dets) {
        const cv::Scalar color = colorForClass(d.cls);
        cv::rectangle(out, d.box, color, 2, cv::LINE_AA);

        char label[64];
        const std::string cls_name =
            (d.cls >= 0 && d.cls < static_cast<int>(args_.class_names.size()))
                ? args_.class_names[static_cast<std::size_t>(d.cls)]
                : std::to_string(d.cls);
        std::snprintf(label, sizeof(label), "%s %.2f", cls_name.c_str(), d.score);
        int base = 0;
        cv::Size ts = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &base);
        int x = std::max(0, static_cast<int>(std::floor(d.box.x)));
        int y = std::max(0, static_cast<int>(std::floor(d.box.y)) - ts.height - 4);
        cv::rectangle(out,
                      cv::Rect(x, y, ts.width + 6, ts.height + base + 4) &
                          cv::Rect(0, 0, out.cols, out.rows),
                      color,
                      cv::FILLED);
        cv::putText(out,
                    label,
                    cv::Point(x + 3, y + ts.height + 1),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(0, 0, 0),
                    1,
                    cv::LINE_AA);
    }
    return out;
}

std::vector<std::filesystem::path> collectInputImages(const std::filesystem::path& input,
                                                      bool recursive)
{
    namespace fs = std::filesystem;
    std::vector<fs::path> images;
    std::error_code ec;
    if (!fs::exists(input, ec)) {
        throw std::runtime_error("input path does not exist: " + input.string());
    }

    if (fs::is_regular_file(input, ec)) {
        if (!hasImageExtension(input)) {
            throw std::runtime_error("input file is not a supported image: " + input.string());
        }
        return {input};
    }

    if (!fs::is_directory(input, ec)) {
        throw std::runtime_error("input path is neither file nor directory: " + input.string());
    }

    if (recursive) {
        for (const auto& entry : fs::recursive_directory_iterator(input)) {
            if (entry.is_regular_file() && hasImageExtension(entry.path())) {
                images.push_back(entry.path());
            }
        }
    } else {
        for (const auto& entry : fs::directory_iterator(input)) {
            if (entry.is_regular_file() && hasImageExtension(entry.path())) {
                images.push_back(entry.path());
            }
        }
    }

    std::sort(images.begin(), images.end());
    return images;
}
