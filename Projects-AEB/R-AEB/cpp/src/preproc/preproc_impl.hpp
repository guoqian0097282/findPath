// ImageConvert.hpp
#pragma once

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cctype>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

class ImageConvert {
public:
    // 输入类型：NV12 / BGR / RGB
    // 目标类型：RGB / BGR / NV12
    ImageConvert(int width, int height, const std::string& input_type, const std::string& target_type)
        : width_(width),
          height_(height),
          input_type_(ParseType(input_type)),
          target_type_(ParseType(target_type)) {}


    // ============== 从裸数据构造只读 Mat 视图 ==============
    // 注意：OpenCV 的 cv::Mat 构造函数需要非 const 指针，这里会 const_cast；
    //      语义上仍当作“只读视图”，请勿通过返回的 Mat 写入。
    //      返回的 Mat 不拥有 data 的生命周期，data 必须在 Mat 使用期间保持有效。
    cv::Mat to_mat(const uint8_t* data, std::size_t length) const {
        if (!data || length == 0) throw std::invalid_argument("empty data");

        const int w = width_;
        const int h = height_;

        if (input_type_ == ColorType::NV12) {
            ensure_even_dims(w, h);
            const std::size_t expect = static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 3 / 2;
            if (length != expect) throw std::runtime_error("invalid NV12 buffer length");
            return cv::Mat(h + h / 2, w, CV_8UC1, const_cast<uint8_t*>(data));
        }

        // RGB/BGR 都是 3 通道 uint8
        const std::size_t expect = static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 3;
        if (length != expect) throw std::runtime_error("invalid RGB/BGR buffer length");
        return cv::Mat(h, w, CV_8UC3, const_cast<uint8_t*>(data));
    }

    std::size_t input_bytes() const {
        const int w = width_;
        const int h = height_;
        if (input_type_ == ColorType::NV12) {
            ensure_even_dims(w, h);
            return static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 3 / 2;
        }
        return static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 3;
    }

    // 可选 target_type：空字符串表示使用构造时的 target_type
    cv::Mat convert(const cv::Mat& img, const std::string& target_type = "") const {
        if (img.empty()) throw std::invalid_argument("empty mat");
        validate_input_mat_shape(img);

        const ColorType t = target_type.empty() ? target_type_ : ParseType(target_type);
        return convert_to_target_with_type(img, t);
    }


    // 合并版：按需执行 expand_dims / transpose / astype（按顺序依次执行）
    // - expand_axis:  -1 表示不 expand；否则插入 size=1 的维度
    // - transpose_axes: 为空表示不 transpose；否则是 0..ndims-1 的排列
    // - out_depth: -1 表示不改 dtype；否则为 CV_8U/CV_32F 等（只传 depth，不传 channels）
    static cv::Mat to_data(
        const cv::Mat& img,
        int expand_axis = -1,
        const std::vector<int>& transpose_axes = {},
        int cv_dtype = -1
    ) {
        if (img.empty()) throw std::invalid_argument("empty input");

        // ======== 快速路径 1：x86，HWC uint8 → NCHW float32 ========
        const bool is_hwc_u8 =
            (img.dims == 2) && (img.channels() == 3) && (img.type() == CV_8UC3);

        if (is_hwc_u8 &&
            expand_axis == 0 &&
            transpose_axes.size() == 4 &&
            transpose_axes[0] == 0 &&
            transpose_axes[1] == 3 &&
            transpose_axes[2] == 1 &&
            transpose_axes[3] == 2 &&
            cv_dtype == CV_32F) {
            // 典型调用: preproc_ToData(img_rgb, 0, {0,3,1,2}, CV_32F);
            return hwc_u8_to_nchw_f32(img);
        }

        // ======== 快速路径 2：ARM，单通道 HxW → (1,H,W) uint8 ========
        const bool is_hw_u8 =
            (img.dims == 2) && (img.channels() == 1) && (img.type() == CV_8UC1);

        if (is_hw_u8 &&
            expand_axis == 0 &&
            transpose_axes.size() == 3 &&
            transpose_axes[0] == 0 &&
            transpose_axes[1] == 1 &&
            transpose_axes[2] == 2 &&
            (cv_dtype == -1 || cv_dtype == CV_8U)) {
            // 典型调用: preproc_ToData(img, 0, {0,1,2}, CV_8U);
            return hw_u8_to_nhw_u8(img);
        }

        // ======== 其他情况走原来的通用逻辑 ========

        cv::Mat nd = to_nd_single_channel(img);

        if (expand_axis != -1) {
            nd = expand_dims_copy(nd, expand_axis);
        }

        if (!transpose_axes.empty()) {
            // 小优化：如果 transpose_axes 本身就是 [0,1,2,...]，就不做 transpose
            bool is_identity = true;
            if (static_cast<int>(transpose_axes.size()) == nd.dims) {
                for (int i = 0; i < nd.dims; ++i) {
                    if (transpose_axes[i] != i) {
                        is_identity = false;
                        break;
                    }
                }
            } else {
                is_identity = false;
            }

            if (!is_identity) {
                nd = transpose_copy(nd, transpose_axes);
            }
        }

        if (cv_dtype != -1 && cv_dtype != nd.depth()) {
            cv::Mat converted;
            converted.create(nd.dims, nd.size.p, CV_MAKETYPE(cv_dtype, 1));
            nd.convertTo(converted, CV_MAKETYPE(cv_dtype, 1));
            nd = converted;
        }

        return nd;
    }


private:
    enum class ColorType { NV12, BGR, RGB };

    int width_{0};
    int height_{0};
    ColorType input_type_{ColorType::BGR};
    ColorType target_type_{ColorType::RGB};

    static std::string upper(std::string s) {
        std::transform(s.begin(), s.end(), s.begin(),
                       [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
        return s;
    }

    static ColorType ParseType(const std::string& type) {
        const std::string t = upper(type);
        if (t == "NV12") return ColorType::NV12;
        if (t == "BGR") return ColorType::BGR;
        if (t == "RGB") return ColorType::RGB;
        throw std::invalid_argument("unsupported type: " + type);
    }

    void validate_input_mat_shape(const cv::Mat& img) const {
        const int h = height_;
        const int w = width_;

        if (input_type_ == ColorType::NV12) {
            if (img.dims != 2 || img.type() != CV_8UC1 || img.rows != (h * 3 / 2) || img.cols != w) {
                throw std::runtime_error("invalid NV12 shape");
            }
        }
        else {
            if (img.dims != 2 || img.type() != CV_8UC3 || img.rows != h || img.cols != w) {
                throw std::runtime_error(input_type_ == ColorType::BGR ? "invalid BGR shape" : "invalid RGB shape");
            }
        }
    }

    static void ensure_even_dims(int w, int h) {
        if ((w & 1) || (h & 1)) throw std::runtime_error("NV12 requires even width and height");
    }

    static cv::Mat i420_to_nv12(const cv::Mat& i420, int w, int h) {
        if (i420.dims != 2 || i420.type() != CV_8UC1 || i420.rows != (h * 3 / 2) || i420.cols != w) {
            throw std::runtime_error("invalid I420 shape");
        }

        cv::Mat src = i420.isContinuous() ? i420 : i420.clone();

        const int y_size = w * h;
        const int uv_size = (w / 2) * (h / 2);

        const unsigned char* flat = src.ptr<unsigned char>(0);

        cv::Mat nv12(h + h / 2, w, CV_8UC1);
        unsigned char* dst = nv12.ptr<unsigned char>(0);

        // Y
        std::memcpy(dst, flat, static_cast<size_t>(y_size));

        // UV interleave
        const unsigned char* u = flat + y_size;
        const unsigned char* v = flat + y_size + uv_size;
        for (int i = 0; i < uv_size; ++i) {
            dst[y_size + 2 * i] = u[i];
            dst[y_size + 2 * i + 1] = v[i];
        }
        return nv12;
    }

    cv::Mat rgb_to_nv12(const cv::Mat& rgb) const {
        ensure_even_dims(width_, height_);
        cv::Mat i420;
        cv::cvtColor(rgb, i420, cv::COLOR_RGB2YUV_I420);
        return i420_to_nv12(i420, width_, height_);
    }

    cv::Mat bgr_to_nv12(const cv::Mat& bgr) const {
        ensure_even_dims(width_, height_);
        cv::Mat i420;
        cv::cvtColor(bgr, i420, cv::COLOR_BGR2YUV_I420);
        return i420_to_nv12(i420, width_, height_);
    }

    cv::Mat convert_to_target_with_type(const cv::Mat& src, ColorType target) const {
        // 同类型：返回拷贝，避免外部缓冲区悬挂
        if (input_type_ == target) return src.clone();

        // 目标 NV12
        if (target == ColorType::NV12) {
            if (input_type_ == ColorType::RGB) return rgb_to_nv12(src);
            if (input_type_ == ColorType::BGR) return bgr_to_nv12(src);
            throw std::runtime_error("unexpected path to NV12");
        }

        // 输入 NV12 -> RGB/BGR
        if (input_type_ == ColorType::NV12) {
            cv::Mat dst;
            const int code = (target == ColorType::RGB) ? cv::COLOR_YUV2RGB_NV12 : cv::COLOR_YUV2BGR_NV12;
            cv::cvtColor(src, dst, code);
            return dst;
        }

        // RGB <-> BGR
        if (input_type_ == ColorType::RGB && target == ColorType::BGR) {
            cv::Mat dst;
            cv::cvtColor(src, dst, cv::COLOR_RGB2BGR);
            return dst;
        }
        if (input_type_ == ColorType::BGR && target == ColorType::RGB) {
            cv::Mat dst;
            cv::cvtColor(src, dst, cv::COLOR_BGR2RGB);
            return dst;
        }

        throw std::runtime_error("unsupported colorspace conversion");
    }

    static cv::Mat to_nd_single_channel(const cv::Mat& img) {
        if (img.dims != 2) {
            if (img.channels() != 1) throw std::runtime_error("only single-channel ND Mat is supported");
            return img.isContinuous() ? img.clone() : img.clone();
        }

        const int H = img.rows;
        const int W = img.cols;
        const int ch = img.channels();

        if (ch == 1) {
            cv::Mat out = img.isContinuous() ? img.clone() : img.clone();
            return out;
        }

        if (ch == 3) {
            // CV_8UC3 / CV_32FC3 ... -> ND (H,W,3) with 1 channel
            cv::Mat tmp = img.isContinuous() ? img : img.clone();
            const int sizes[3] = {H, W, 3};
            cv::Mat out(3, sizes, CV_MAKETYPE(tmp.depth(), 1));
            std::memcpy(out.data, tmp.data, tmp.total() * tmp.elemSize());
            return out;
        }

        throw std::runtime_error("unsupported channels for to_data");
    }

    static cv::Mat expand_dims_copy(const cv::Mat& src, int axis) {
        const int nd = src.dims;
        if (axis < 0) axis += (nd + 1);
        if (axis < 0 || axis > nd) throw std::runtime_error("invalid expand_axis");

        std::vector<int> sizes;
        sizes.reserve(static_cast<size_t>(nd + 1));
        for (int i = 0; i < nd + 1; ++i) {
            if (i == axis) sizes.push_back(1);
            else sizes.push_back(src.size[i < axis ? i : (i - 1)]);
        }

        cv::Mat out(nd + 1, sizes.data(), src.type());
        if (!src.isContinuous() || !out.isContinuous()) {
            // src/out 都应当连续；否则直接逐字节拷贝也仍可行，但这里简单兜底 clone
            cv::Mat tmp = src.clone();
            std::memcpy(out.data, tmp.data, tmp.total() * tmp.elemSize());
            return out;
        }

        std::memcpy(out.data, src.data, src.total() * src.elemSize());
        return out;
    }

    static void validate_permutation(const std::vector<int>& axes, int ndims) {
        if (static_cast<int>(axes.size()) != ndims) throw std::runtime_error("transpose_axes size mismatch");
        std::vector<int> seen(ndims, 0);
        for (int a : axes) {
            if (a < 0 || a >= ndims) throw std::runtime_error("invalid axis in transpose_axes");
            if (seen[a]++) throw std::runtime_error("duplicate axis in transpose_axes");
        }
    }

    // 通用 N 维 transpose（单通道 ND Mat）
    static cv::Mat transpose_copy(const cv::Mat& src, const std::vector<int>& axes) {
        if (src.channels() != 1) throw std::runtime_error("transpose_copy requires single-channel Mat");
        const int nd = src.dims;
        validate_permutation(axes, nd);

        std::vector<int> out_sizes(nd);
        for (int i = 0; i < nd; ++i) out_sizes[i] = src.size[axes[i]];

        cv::Mat dst(nd, out_sizes.data(), src.type());

        const size_t esz = src.elemSize1(); // 单通道
        std::vector<size_t> src_stride(nd), dst_stride(nd);
        for (int i = 0; i < nd; ++i) {
            src_stride[i] = src.step[i] / esz;
            dst_stride[i] = dst.step[i] / esz;
        }

        const size_t total = static_cast<size_t>(dst.total());
        const unsigned char* sp = src.ptr<unsigned char>(0);
        unsigned char* dp = dst.ptr<unsigned char>(0);

        // 为了少分配，复用 index buffer
        std::vector<int> out_idx(nd, 0);
        std::vector<int> in_idx(nd, 0);

        for (size_t linear = 0; linear < total; ++linear) {
            // linear -> out_idx (row-major)
            size_t t = linear;
            for (int k = nd - 1; k >= 0; --k) {
                const int dim = out_sizes[k];
                out_idx[k] = static_cast<int>(t % static_cast<size_t>(dim));
                t /= static_cast<size_t>(dim);
            }

            // out_idx -> in_idx  (out[k] = in[axes[k]])
            for (int k = 0; k < nd; ++k) {
                in_idx[axes[k]] = out_idx[k];
            }

            size_t so = 0, doff = 0;
            for (int k = 0; k < nd; ++k) {
                so += static_cast<size_t>(in_idx[k]) * src_stride[k];
                doff += static_cast<size_t>(out_idx[k]) * dst_stride[k];
            }

            std::memcpy(dp + doff * esz, sp + so * esz, esz);
        }

        return dst;
    }

    // 快速路径：2D CV_8UC3 输入，输出 NCHW float32: shape = (1, 3, H, W)
    static cv::Mat hwc_u8_to_nchw_f32(const cv::Mat& img) {
        if (img.dims != 2 || img.channels() != 3 || img.type() != CV_8UC3) {
            throw std::runtime_error("hwc_u8_to_nchw_f32 expects 2D CV_8UC3 image");
        }

        const int H = img.rows;
        const int W = img.cols;

        int sizes[4] = {1, 3, H, W};
        cv::Mat out(4, sizes, CV_32FC1);

        const uint8_t* src = img.ptr<uint8_t>(0);
        float* dst = out.ptr<float>(0);

        const int HW = H * W;

        for (int h = 0; h < H; ++h) {
            const uint8_t* row = src + h * W * 3;
            for (int w = 0; w < W; ++w) {
                const uint8_t* pix = row + w * 3;
                const int idx = h * W + w;

                // 注意：这里的通道顺序就是 img 的原始顺序（BGR 或 RGB）
                dst[0 * HW + idx] = static_cast<float>(pix[0]);
                dst[1 * HW + idx] = static_cast<float>(pix[1]);
                dst[2 * HW + idx] = static_cast<float>(pix[2]);
            }
        }

        return out;
    }

    // 快速路径：2D CV_8UC1 灰度图，输出 shape=(1,H,W) 的 CV_8UC1
    static cv::Mat hw_u8_to_nhw_u8(const cv::Mat& img) {
        if (img.dims != 2 || img.channels() != 1 || img.type() != CV_8UC1) {
            throw std::runtime_error("hw_u8_to_nhw_u8 expects 2D CV_8UC1 image");
        }
        const int H = img.rows;
        const int W = img.cols;

        int sizes[3] = {1, H, W};
        cv::Mat out(3, sizes, CV_8UC1);

        if (img.isContinuous() && out.isContinuous()) {
            std::memcpy(out.data, img.data, static_cast<size_t>(H) * W * sizeof(uint8_t));
        } else {
            // 保守一点，逐行拷贝
            for (int h = 0; h < H; ++h) {
                const uint8_t* src_row = img.ptr<uint8_t>(h);
                uint8_t* dst_row = out.ptr<uint8_t>(0) + h * W;
                std::memcpy(dst_row, src_row, static_cast<size_t>(W) * sizeof(uint8_t));
            }
        }

        return out;
    }

};
