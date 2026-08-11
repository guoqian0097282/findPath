#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <stdexcept>
#include <vector>
#include <algorithm>


namespace detail {
    inline void assert_single_channel(const cv::Mat& x) {
        if (x.empty()) return;
        if (x.channels() != 1) throw std::runtime_error("cv::Mat must be single-channel");
    }

    inline void assert_depth_float(const cv::Mat& x) {
        if (x.empty()) return;
        const int d = x.depth();
        if (d != CV_32F && d != CV_64F) throw std::runtime_error("cv::Mat must be CV_32F or CV_64F");
    }

    inline int last_dim(const cv::Mat& x) {
        if (x.dims == 0) return 0;
        return x.size[x.dims - 1];
    }

    inline cv::Mat empty_like(const cv::Mat& x) {
        if (x.empty()) return cv::Mat();
        return cv::Mat(x.dims, x.size.p, x.type());
    }

    inline cv::Mat empty_like_lastdim(const cv::Mat& x, int new_last_dim, int type = -1) {
        if (x.empty()) return cv::Mat();
        std::vector<int> sz((size_t)x.dims);
        for (int i = 0; i < x.dims; ++i) sz[(size_t)i] = x.size[i];
        sz.back() = new_last_dim;
        return cv::Mat((int)sz.size(), sz.data(), (type < 0 ? x.type() : type));
    }

    // 对应 Python clip_boxes(x, (h-eps, w-eps)) 的简单等价：把 xyxy 裁剪进 [0, w_max]/[0, h_max]
    template <typename T>
    inline cv::Mat clip_boxes_xyxy_impl(const cv::Mat& x, T h_max, T w_max) {
        cv::Mat y = x.clone();
        const int ld = last_dim(y);
        if (ld != 4) throw std::runtime_error("clip_boxes_xyxy expects last dim = 4 (xyxy)");
        if (!y.isContinuous()) y = y.clone();

        T* p = y.ptr<T>();
        const size_t nbox = y.total() / 4;

        for (size_t i = 0; i < nbox; ++i) {
            T& x1 = p[i * 4 + 0];
            T& y1 = p[i * 4 + 1];
            T& x2 = p[i * 4 + 2];
            T& y2 = p[i * 4 + 3];

            x1 = std::min(std::max(x1, (T)0), w_max);
            x2 = std::min(std::max(x2, (T)0), w_max);
            y1 = std::min(std::max(y1, (T)0), h_max);
            y2 = std::min(std::max(y2, (T)0), h_max);
        }
        return y;
    }

    inline cv::Mat clip_boxes_xyxy(const cv::Mat& x, float h_max, float w_max) {
        detail::assert_single_channel(x);
        detail::assert_depth_float(x);
        if (detail::last_dim(x) != 4) throw std::runtime_error("clip_boxes_xyxy expects last dim = 4");
        if (x.depth() == CV_32F) return clip_boxes_xyxy_impl<float>(x, h_max, w_max);
        return clip_boxes_xyxy_impl<double>(x, (double)h_max, (double)w_max);
    }
} // namespace detail


// ============================
// xyxy <-> xywh
// ============================

// Python: xyxy2xywh
inline cv::Mat xyxy2xywh(const cv::Mat& x) {
    detail::assert_single_channel(x);
    detail::assert_depth_float(x);
    if (detail::last_dim(x) != 4) throw std::runtime_error("xyxy2xywh expects last dim = 4");

    cv::Mat y = detail::empty_like(x);
    cv::Mat xx = x;
    if (!xx.isContinuous()) xx = xx.clone();
    if (!y.isContinuous()) y = y.clone();

    const size_t nbox = x.total() / 4;

    if (x.depth() == CV_32F) {
        const float* p = xx.ptr<float>();
        float* q = y.ptr<float>();
        for (size_t i = 0; i < nbox; ++i) {
            const float x1 = p[i * 4 + 0];
            const float y1 = p[i * 4 + 1];
            const float x2 = p[i * 4 + 2];
            const float y2 = p[i * 4 + 3];
            q[i * 4 + 0] = (x1 + x2) * 0.5f;
            q[i * 4 + 1] = (y1 + y2) * 0.5f;
            q[i * 4 + 2] = (x2 - x1);
            q[i * 4 + 3] = (y2 - y1);
        }
    }
    else {
        const double* p = xx.ptr<double>();
        double* q = y.ptr<double>();
        for (size_t i = 0; i < nbox; ++i) {
            const double x1 = p[i * 4 + 0];
            const double y1 = p[i * 4 + 1];
            const double x2 = p[i * 4 + 2];
            const double y2 = p[i * 4 + 3];
            q[i * 4 + 0] = (x1 + x2) * 0.5;
            q[i * 4 + 1] = (y1 + y2) * 0.5;
            q[i * 4 + 2] = (x2 - x1);
            q[i * 4 + 3] = (y2 - y1);
        }
    }
    return y;
}

// Python: xywh2xyxy
inline cv::Mat xywh2xyxy(const cv::Mat& x) {
    detail::assert_single_channel(x);
    detail::assert_depth_float(x);
    if (detail::last_dim(x) != 4) throw std::runtime_error("xywh2xyxy expects last dim = 4");

    cv::Mat y = detail::empty_like(x);
    cv::Mat xx = x;
    if (!xx.isContinuous()) xx = xx.clone();
    if (!y.isContinuous()) y = y.clone();

    const size_t nbox = x.total() / 4;

    if (x.depth() == CV_32F) {
        const float* p = xx.ptr<float>();
        float* q = y.ptr<float>();
        for (size_t i = 0; i < nbox; ++i) {
            const float xc = p[i * 4 + 0];
            const float yc = p[i * 4 + 1];
            const float w = p[i * 4 + 2];
            const float h = p[i * 4 + 3];
            const float hw = w * 0.5f;
            const float hh = h * 0.5f;
            q[i * 4 + 0] = xc - hw;
            q[i * 4 + 1] = yc - hh;
            q[i * 4 + 2] = xc + hw;
            q[i * 4 + 3] = yc + hh;
        }
    }
    else {
        const double* p = xx.ptr<double>();
        double* q = y.ptr<double>();
        for (size_t i = 0; i < nbox; ++i) {
            const double xc = p[i * 4 + 0];
            const double yc = p[i * 4 + 1];
            const double w = p[i * 4 + 2];
            const double h = p[i * 4 + 3];
            const double hw = w * 0.5;
            const double hh = h * 0.5;
            q[i * 4 + 0] = xc - hw;
            q[i * 4 + 1] = yc - hh;
            q[i * 4 + 2] = xc + hw;
            q[i * 4 + 3] = yc + hh;
        }
    }
    return y;
}


// ============================
// normalized <-> pixel
// ============================

// Python: xywhn2xyxy
inline cv::Mat xywhn2xyxy(const cv::Mat& x, int w = 640, int h = 640, int padw = 0, int padh = 0) {
    detail::assert_single_channel(x);
    detail::assert_depth_float(x);
    if (detail::last_dim(x) != 4) throw std::runtime_error("xywhn2xyxy expects last dim = 4");

    cv::Mat y = detail::empty_like(x);
    cv::Mat xx = x;
    if (!xx.isContinuous()) xx = xx.clone();
    if (!y.isContinuous()) y = y.clone();

    const size_t nbox = x.total() / 4;

    if (x.depth() == CV_32F) {
        const float* p = xx.ptr<float>();
        float* q = y.ptr<float>();
        const float fw = (float)w, fh = (float)h;
        const float fpw = (float)padw, fph = (float)padh;
        for (size_t i = 0; i < nbox; ++i) {
            const float xc = p[i * 4 + 0];
            const float yc = p[i * 4 + 1];
            const float bw = p[i * 4 + 2];
            const float bh = p[i * 4 + 3];
            const float hw = bw * 0.5f;
            const float hh = bh * 0.5f;
            q[i * 4 + 0] = fw * (xc - hw) + fpw;
            q[i * 4 + 1] = fh * (yc - hh) + fph;
            q[i * 4 + 2] = fw * (xc + hw) + fpw;
            q[i * 4 + 3] = fh * (yc + hh) + fph;
        }
    }
    else {
        const double* p = xx.ptr<double>();
        double* q = y.ptr<double>();
        const double fw = (double)w, fh = (double)h;
        const double fpw = (double)padw, fph = (double)padh;
        for (size_t i = 0; i < nbox; ++i) {
            const double xc = p[i * 4 + 0];
            const double yc = p[i * 4 + 1];
            const double bw = p[i * 4 + 2];
            const double bh = p[i * 4 + 3];
            const double hw = bw * 0.5;
            const double hh = bh * 0.5;
            q[i * 4 + 0] = fw * (xc - hw) + fpw;
            q[i * 4 + 1] = fh * (yc - hh) + fph;
            q[i * 4 + 2] = fw * (xc + hw) + fpw;
            q[i * 4 + 3] = fh * (yc + hh) + fpw; // NOTE: keep x formula? (typo guard)
            q[i * 4 + 3] = fh * (yc + hh) + fph; // correct line
        }
    }
    return y;
}

// Python: xyxy2xywhn
inline cv::Mat xyxy2xywhn(const cv::Mat& x, int w = 640, int h = 640, bool clip = false, float eps = 0.0f) {
    detail::assert_single_channel(x);
    detail::assert_depth_float(x);
    if (detail::last_dim(x) != 4) throw std::runtime_error("xyxy2xywhn expects last dim = 4");

    cv::Mat xx = x;
    if (clip) {
        const float hmax = (float)h - eps;
        const float wmax = (float)w - eps;
        xx = detail::clip_boxes_xyxy(xx, hmax, wmax);
    }

    cv::Mat y = detail::empty_like(xx);
    if (!xx.isContinuous()) xx = xx.clone();
    if (!y.isContinuous()) y = y.clone();

    const size_t nbox = xx.total() / 4;

    if (xx.depth() == CV_32F) {
        const float* p = xx.ptr<float>();
        float* q = y.ptr<float>();
        const float fw = (float)w, fh = (float)h;
        for (size_t i = 0; i < nbox; ++i) {
            const float x1 = p[i * 4 + 0];
            const float y1 = p[i * 4 + 1];
            const float x2 = p[i * 4 + 2];
            const float y2 = p[i * 4 + 3];
            q[i * 4 + 0] = ((x1 + x2) * 0.5f) / fw;
            q[i * 4 + 1] = ((y1 + y2) * 0.5f) / fh;
            q[i * 4 + 2] = (x2 - x1) / fw;
            q[i * 4 + 3] = (y2 - y1) / fh;
        }
    }
    else {
        const double* p = xx.ptr<double>();
        double* q = y.ptr<double>();
        const double fw = (double)w, fh = (double)h;
        for (size_t i = 0; i < nbox; ++i) {
            const double x1 = p[i * 4 + 0];
            const double y1 = p[i * 4 + 1];
            const double x2 = p[i * 4 + 2];
            const double y2 = p[i * 4 + 3];
            q[i * 4 + 0] = ((x1 + x2) * 0.5) / fw;
            q[i * 4 + 1] = ((y1 + y2) * 0.5) / fh;
            q[i * 4 + 2] = (x2 - x1) / fw;
            q[i * 4 + 3] = (y2 - y1) / fh;
        }
    }
    return y;
}


// ============================
// ltwh / xywh / xyxy conversions
// ============================

// Python: xywh2ltwh  (center xywh -> top-left ltwh)
inline cv::Mat xywh2ltwh(const cv::Mat& x) {
    detail::assert_single_channel(x);
    detail::assert_depth_float(x);
    if (detail::last_dim(x) != 4) throw std::runtime_error("xywh2ltwh expects last dim = 4");

    cv::Mat y = x.clone();
    if (!y.isContinuous()) y = y.clone();

    const size_t nbox = y.total() / 4;

    if (y.depth() == CV_32F) {
        float* p = y.ptr<float>();
        for (size_t i = 0; i < nbox; ++i) {
            p[i * 4 + 0] = p[i * 4 + 0] - p[i * 4 + 2] * 0.5f;
            p[i * 4 + 1] = p[i * 4 + 1] - p[i * 4 + 3] * 0.5f;
        }
    }
    else {
        double* p = y.ptr<double>();
        for (size_t i = 0; i < nbox; ++i) {
            p[i * 4 + 0] = p[i * 4 + 0] - p[i * 4 + 2] * 0.5;
            p[i * 4 + 1] = p[i * 4 + 1] - p[i * 4 + 3] * 0.5;
        }
    }
    return y;
}

// Python: xyxy2ltwh  (xyxy -> ltwh where w/h computed)
inline cv::Mat xyxy2ltwh(const cv::Mat& x) {
    detail::assert_single_channel(x);
    detail::assert_depth_float(x);
    if (detail::last_dim(x) != 4) throw std::runtime_error("xyxy2ltwh expects last dim = 4");

    cv::Mat y = x.clone();
    if (!y.isContinuous()) y = y.clone();

    const size_t nbox = y.total() / 4;

    if (y.depth() == CV_32F) {
        float* p = y.ptr<float>();
        for (size_t i = 0; i < nbox; ++i) {
            p[i * 4 + 2] = p[i * 4 + 2] - p[i * 4 + 0];
            p[i * 4 + 3] = p[i * 4 + 3] - p[i * 4 + 1];
        }
    }
    else {
        double* p = y.ptr<double>();
        for (size_t i = 0; i < nbox; ++i) {
            p[i * 4 + 2] = p[i * 4 + 2] - p[i * 4 + 0];
            p[i * 4 + 3] = p[i * 4 + 3] - p[i * 4 + 1];
        }
    }
    return y;
}

// Python: ltwh2xywh  (top-left ltwh -> center xywh)
inline cv::Mat ltwh2xywh(const cv::Mat& x) {
    detail::assert_single_channel(x);
    detail::assert_depth_float(x);
    if (detail::last_dim(x) != 4) throw std::runtime_error("ltwh2xywh expects last dim = 4");

    cv::Mat y = x.clone();
    if (!y.isContinuous()) y = y.clone();

    const size_t nbox = y.total() / 4;

    if (y.depth() == CV_32F) {
        float* p = y.ptr<float>();
        for (size_t i = 0; i < nbox; ++i) {
            p[i * 4 + 0] = p[i * 4 + 0] + p[i * 4 + 2] * 0.5f;
            p[i * 4 + 1] = p[i * 4 + 1] + p[i * 4 + 3] * 0.5f;
        }
    }
    else {
        double* p = y.ptr<double>();
        for (size_t i = 0; i < nbox; ++i) {
            p[i * 4 + 0] = p[i * 4 + 0] + p[i * 4 + 2] * 0.5;
            p[i * 4 + 1] = p[i * 4 + 1] + p[i * 4 + 3] * 0.5;
        }
    }
    return y;
}

// Python: ltwh2xyxy  (top-left ltwh -> xyxy)
inline cv::Mat ltwh2xyxy(const cv::Mat& x) {
    detail::assert_single_channel(x);
    detail::assert_depth_float(x);
    if (detail::last_dim(x) != 4) throw std::runtime_error("ltwh2xyxy expects last dim = 4");

    cv::Mat y = x.clone();
    if (!y.isContinuous()) y = y.clone();

    const size_t nbox = y.total() / 4;

    if (y.depth() == CV_32F) {
        float* p = y.ptr<float>();
        for (size_t i = 0; i < nbox; ++i) {
            p[i * 4 + 2] = p[i * 4 + 2] + p[i * 4 + 0];
            p[i * 4 + 3] = p[i * 4 + 3] + p[i * 4 + 1];
        }
    }
    else {
        double* p = y.ptr<double>();
        for (size_t i = 0; i < nbox; ++i) {
            p[i * 4 + 2] = p[i * 4 + 2] + p[i * 4 + 0];
            p[i * 4 + 3] = p[i * 4 + 3] + p[i * 4 + 1];
        }
    }
    return y;
}


// ============================
// OBB conversions
// ============================

// Python: xyxyxyxy2xywhr
// 输入 last dim = 8: [x1,y1,x2,y2,x3,y3,x4,y4]，输出 last dim = 5: [cx,cy,w,h,rotation(rad)]
inline cv::Mat xyxyxyxy2xywhr(const cv::Mat& x) {
    detail::assert_single_channel(x);
    detail::assert_depth_float(x);
    if (detail::last_dim(x) != 8) throw std::runtime_error("xyxyxyxy2xywhr expects last dim = 8");

    cv::Mat xx = x;
    if (!xx.isContinuous()) xx = xx.clone();

    cv::Mat y = detail::empty_like_lastdim(xx, 5, xx.type());
    if (!y.isContinuous()) y = y.clone();

    const size_t nbox = xx.total() / 8;
    constexpr double kPi = 3.14159265358979323846;

    if (xx.depth() == CV_32F) {
        const float* p = xx.ptr<float>();
        float* q = y.ptr<float>();
        for (size_t i = 0; i < nbox; ++i) {
            cv::Point2f pts[4] = {
                {p[i * 8 + 0], p[i * 8 + 1]},
                {p[i * 8 + 2], p[i * 8 + 3]},
                {p[i * 8 + 4], p[i * 8 + 5]},
                {p[i * 8 + 6], p[i * 8 + 7]},
            };
            cv::RotatedRect rr = cv::minAreaRect(std::vector<cv::Point2f>(pts, pts + 4));
            const float cx = rr.center.x;
            const float cy = rr.center.y;
            const float w = rr.size.width;
            const float h = rr.size.height;
            const float ang_rad = (float)(rr.angle / 180.0 * kPi);

            q[i * 5 + 0] = cx;
            q[i * 5 + 1] = cy;
            q[i * 5 + 2] = w;
            q[i * 5 + 3] = h;
            q[i * 5 + 4] = ang_rad;
        }
    }
    else {
        const double* p = xx.ptr<double>();
        double* q = y.ptr<double>();
        for (size_t i = 0; i < nbox; ++i) {
            cv::Point2f pts[4] = {
                {(float)p[i * 8 + 0], (float)p[i * 8 + 1]},
                {(float)p[i * 8 + 2], (float)p[i * 8 + 3]},
                {(float)p[i * 8 + 4], (float)p[i * 8 + 5]},
                {(float)p[i * 8 + 6], (float)p[i * 8 + 7]},
            };
            cv::RotatedRect rr = cv::minAreaRect(std::vector<cv::Point2f>(pts, pts + 4));
            const double cx = rr.center.x;
            const double cy = rr.center.y;
            const double w = rr.size.width;
            const double h = rr.size.height;
            const double ang_rad = rr.angle / 180.0 * kPi;

            q[i * 5 + 0] = cx;
            q[i * 5 + 1] = cy;
            q[i * 5 + 2] = w;
            q[i * 5 + 3] = h;
            q[i * 5 + 4] = ang_rad;
        }
    }
    return y;
}

// Python: xywhr2xyxyxyxy
// 输入 last dim = 5: [cx,cy,w,h,rotation(rad)]，输出 last dims = (4,2)
inline cv::Mat xywhr2xyxyxyxy(const cv::Mat& x) {
    detail::assert_single_channel(x);
    detail::assert_depth_float(x);
    if (detail::last_dim(x) != 5) throw std::runtime_error("xywhr2xyxyxyxy expects last dim = 5");

    cv::Mat xx = x;
    if (!xx.isContinuous()) xx = xx.clone();

    // output dims = x.dims + 1, sizes: replace last=5 -> 4, append 2
    std::vector<int> sz((size_t)xx.dims + 1);
    for (int i = 0; i < xx.dims - 1; ++i) sz[(size_t)i] = xx.size[i];
    sz[(size_t)(xx.dims - 1)] = 4;
    sz.back() = 2;

    cv::Mat y((int)sz.size(), sz.data(), xx.type());
    if (!y.isContinuous()) y = y.clone();

    const size_t nbox = xx.total() / 5;

    if (xx.depth() == CV_32F) {
        const float* p = xx.ptr<float>();
        float* q = y.ptr<float>(); // flattened write

        for (size_t i = 0; i < nbox; ++i) {
            const float cx = p[i * 5 + 0];
            const float cy = p[i * 5 + 1];
            const float w = p[i * 5 + 2];
            const float h = p[i * 5 + 3];
            const float a = p[i * 5 + 4];

            const float c = std::cos(a);
            const float s = std::sin(a);

            const float w2 = w * 0.5f;
            const float h2 = h * 0.5f;

            const float v1x = w2 * c;
            const float v1y = w2 * s;
            const float v2x = -h2 * s;
            const float v2y = h2 * c;

            // pt1..pt4 对齐 Python
            const float x1 = cx + v1x + v2x;
            const float y1 = cy + v1y + v2y;
            const float x2 = cx + v1x - v2x;
            const float y2 = cy + v1y - v2y;
            const float x3 = cx - v1x - v2x;
            const float y3 = cy - v1y - v2y;
            const float x4 = cx - v1x + v2x;
            const float y4 = cy - v1y + v2y;

            // flattened layout: (..,4,2) => 8 values per box
            const size_t base = i * 8;
            q[base + 0] = x1;
            q[base + 1] = y1;
            q[base + 2] = x2;
            q[base + 3] = y2;
            q[base + 4] = x3;
            q[base + 5] = y3;
            q[base + 6] = x4;
            q[base + 7] = y4;
        }
    }
    else {
        const double* p = xx.ptr<double>();
        double* q = y.ptr<double>();

        for (size_t i = 0; i < nbox; ++i) {
            const double cx = p[i * 5 + 0];
            const double cy = p[i * 5 + 1];
            const double w = p[i * 5 + 2];
            const double h = p[i * 5 + 3];
            const double a = p[i * 5 + 4];

            const double c = std::cos(a);
            const double s = std::sin(a);

            const double w2 = w * 0.5;
            const double h2 = h * 0.5;

            const double v1x = w2 * c;
            const double v1y = w2 * s;
            const double v2x = -h2 * s;
            const double v2y = h2 * c;

            const double x1 = cx + v1x + v2x;
            const double y1 = cy + v1y + v2y;
            const double x2 = cx + v1x - v2x;
            const double y2 = cy + v1y - v2y;
            const double x3 = cx - v1x - v2x;
            const double y3 = cy - v1y - v2y;
            const double x4 = cx - v1x + v2x;
            const double y4 = cy - v1y + v2y;

            const size_t base = i * 8;
            q[base + 0] = x1;
            q[base + 1] = y1;
            q[base + 2] = x2;
            q[base + 3] = y2;
            q[base + 4] = x3;
            q[base + 5] = y3;
            q[base + 6] = x4;
            q[base + 7] = y4;
        }
    }
    return y;
}
