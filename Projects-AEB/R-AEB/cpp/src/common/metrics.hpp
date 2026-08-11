#pragma once
// Ultralytics 🚀 AGPL-3.0 License - https://ultralytics.com/license
// Header-only C++ port (Torch-free): numpy/scipy/torch -> Eigen

#include <Eigen/Core>
#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>


namespace detail {

using Scalar = float;

inline Scalar clamp0(Scalar v) { return v > Scalar(0) ? v : Scalar(0); }

inline Scalar sqr(Scalar v) { return v * v; }

inline Scalar pi() { return static_cast<Scalar>(3.14159265358979323846); }

// Ensure box is x1y1x2y2, no ordering assumption in input (optional normalize)
inline void normalize_xyxy(Scalar& x1, Scalar& y1, Scalar& x2, Scalar& y2) {
    if (x2 < x1) std::swap(x1, x2);
    if (y2 < y1) std::swap(y1, y2);
}

}  // namespace detail

// ---------------------------------------------
// bbox_ioa: inter / area(box2)  (or IoU if iou=true)
// box1: (N,4) xyxy, box2: (M,4) xyxy  -> (N,M)
// ---------------------------------------------
inline Eigen::MatrixXf bbox_ioa(const Eigen::Ref<const Eigen::MatrixXf>& box1,
                               const Eigen::Ref<const Eigen::MatrixXf>& box2,
                               bool iou = false,
                               float eps = 1e-7f) {
    if (box1.cols() != 4 || box2.cols() != 4) {
        throw std::invalid_argument("bbox_ioa: box1/box2 must have shape (N,4)/(M,4)");
    }
    const int N = static_cast<int>(box1.rows());
    const int M = static_cast<int>(box2.rows());

    Eigen::MatrixXf out(N, M);
    for (int i = 0; i < N; ++i) {
        float b1x1 = box1(i, 0), b1y1 = box1(i, 1), b1x2 = box1(i, 2), b1y2 = box1(i, 3);
        detail::normalize_xyxy(b1x1, b1y1, b1x2, b1y2);
        const float b1_area = (b1x2 - b1x1) * (b1y2 - b1y1);

        for (int j = 0; j < M; ++j) {
            float b2x1 = box2(j, 0), b2y1 = box2(j, 1), b2x2 = box2(j, 2), b2y2 = box2(j, 3);
            detail::normalize_xyxy(b2x1, b2y1, b2x2, b2y2);

            const float iw = detail::clamp0(std::min(b1x2, b2x2) - std::max(b1x1, b2x1));
            const float ih = detail::clamp0(std::min(b1y2, b2y2) - std::max(b1y1, b2y1));
            const float inter = iw * ih;

            float denom = (b2x2 - b2x1) * (b2y2 - b2y1);
            if (iou) denom = denom + b1_area - inter;
            out(i, j) = inter / (denom + eps);
        }
    }
    return out;
}

// ---------------------------------------------
// box_iou: pairwise IoU for xyxy boxes
// box1: (N,4), box2: (M,4) -> (N,M)
// ---------------------------------------------
inline Eigen::MatrixXf box_iou(const Eigen::Ref<const Eigen::MatrixXf>& box1,
                              const Eigen::Ref<const Eigen::MatrixXf>& box2,
                              float eps = 1e-7f) {
    return bbox_ioa(box1, box2, /*iou=*/true, eps);
}

// ---------------------------------------------
// bbox_iou: IoU/GIoU/DIoU/CIoU (Torch-free)
// Supports broadcasting only in a practical sense:
// - box1 rows == box2 rows  -> returns (N)
// - box1 rows == 1, box2 rows == M -> returns (M)
// - box2 rows == 1, box1 rows == N -> returns (N)
// Inputs can be xywh (center) or xyxy based on xywh flag.
// ---------------------------------------------
inline Eigen::VectorXf bbox_iou(const Eigen::Ref<const Eigen::MatrixXf>& box1,
                               const Eigen::Ref<const Eigen::MatrixXf>& box2,
                               bool xywh = true,
                               bool GIoU = false,
                               bool DIoU = false,
                               bool CIoU = false,
                               float eps = 1e-7f) {
    if (box1.cols() != 4 || box2.cols() != 4) {
        throw std::invalid_argument("bbox_iou: box1/box2 must have shape (N,4)/(M,4)");
    }
    const int N1 = static_cast<int>(box1.rows());
    const int N2 = static_cast<int>(box2.rows());

    int N = 0;
    bool b1_single = false, b2_single = false;
    if (N1 == N2) {
        N = N1;
    } else if (N1 == 1) {
        N = N2;
        b1_single = true;
    } else if (N2 == 1) {
        N = N1;
        b2_single = true;
    } else {
        throw std::invalid_argument("bbox_iou: rows must match or one side must have 1 row (broadcast)");
    }

    Eigen::VectorXf out(N);

    for (int i = 0; i < N; ++i) {
        const int i1 = b1_single ? 0 : i;
        const int i2 = b2_single ? 0 : i;

        float b1_x1, b1_y1, b1_x2, b1_y2;
        float b2_x1, b2_y1, b2_x2, b2_y2;
        float w1, h1, w2, h2;

        if (xywh) {
            const float x1 = box1(i1, 0), y1 = box1(i1, 1), ww1 = box1(i1, 2), hh1 = box1(i1, 3);
            const float x2 = box2(i2, 0), y2 = box2(i2, 1), ww2 = box2(i2, 2), hh2 = box2(i2, 3);

            const float w1_ = ww1 * 0.5f, h1_ = hh1 * 0.5f;
            const float w2_ = ww2 * 0.5f, h2_ = hh2 * 0.5f;

            b1_x1 = x1 - w1_; b1_x2 = x1 + w1_;
            b1_y1 = y1 - h1_; b1_y2 = y1 + h1_;
            b2_x1 = x2 - w2_; b2_x2 = x2 + w2_;
            b2_y1 = y2 - h2_; b2_y2 = y2 + h2_;

            w1 = ww1; h1 = hh1;
            w2 = ww2; h2 = hh2;
        } else {
            b1_x1 = box1(i1, 0); b1_y1 = box1(i1, 1); b1_x2 = box1(i1, 2); b1_y2 = box1(i1, 3);
            b2_x1 = box2(i2, 0); b2_y1 = box2(i2, 1); b2_x2 = box2(i2, 2); b2_y2 = box2(i2, 3);
            detail::normalize_xyxy(b1_x1, b1_y1, b1_x2, b1_y2);
            detail::normalize_xyxy(b2_x1, b2_y1, b2_x2, b2_y2);

            w1 = (b1_x2 - b1_x1);
            h1 = (b1_y2 - b1_y1) + eps;
            w2 = (b2_x2 - b2_x1);
            h2 = (b2_y2 - b2_y1) + eps;
        }

        const float inter_w = detail::clamp0(std::min(b1_x2, b2_x2) - std::max(b1_x1, b2_x1));
        const float inter_h = detail::clamp0(std::min(b1_y2, b2_y2) - std::max(b1_y1, b2_y1));
        const float inter = inter_w * inter_h;

        const float union_ = w1 * h1 + w2 * h2 - inter + eps;
        float iou = inter / union_;

        if (CIoU || DIoU || GIoU) {
            const float cw = std::max(b1_x2, b2_x2) - std::min(b1_x1, b2_x1);
            const float ch = std::max(b1_y2, b2_y2) - std::min(b1_y1, b2_y1);

            if (CIoU || DIoU) {
                const float c2 = detail::sqr(cw) + detail::sqr(ch) + eps;
                const float rho2 =
                    (detail::sqr((b2_x1 + b2_x2) - (b1_x1 + b1_x2)) +
                     detail::sqr((b2_y1 + b2_y2) - (b1_y1 + b1_y2))) * 0.25f;

                if (CIoU) {
                    const float atan2_ = std::atan((w2 / h2)) - std::atan((w1 / h1));
                    const float v = (4.0f / (detail::pi() * detail::pi())) * detail::sqr(atan2_);
                    const float alpha = v / (v - iou + (1.0f + eps));
                    out(i) = iou - (rho2 / c2 + v * alpha);
                    continue;
                }
                out(i) = iou - rho2 / c2;
                continue;
            }

            // GIoU
            const float c_area = cw * ch + eps;
            out(i) = iou - (c_area - union_) / c_area;
            continue;
        }

        out(i) = iou;
    }

    return out;
}

// ---------------------------------------------
// mask_iou: mask1(N,n), mask2(M,n) -> (N,M)
// (assumes masks are {0,1} or non-negative floats; follows original logic)
// ---------------------------------------------
inline Eigen::MatrixXf mask_iou(const Eigen::Ref<const Eigen::MatrixXf>& mask1,
                               const Eigen::Ref<const Eigen::MatrixXf>& mask2,
                               float eps = 1e-7f) {
    if (mask1.cols() != mask2.cols()) {
        throw std::invalid_argument("mask_iou: mask1/mask2 must have same number of columns (flattened pixels)");
    }
    // intersection = mask1 @ mask2^T
    Eigen::MatrixXf inter = (mask1 * mask2.transpose()).cwiseMax(0.0f);

    Eigen::VectorXf area1 = mask1.rowwise().sum();
    Eigen::VectorXf area2 = mask2.rowwise().sum();

    Eigen::MatrixXf uni = area1.replicate(1, mask2.rows()) + area2.transpose().replicate(mask1.rows(), 1) - inter;
    return inter.array() / (uni.array() + eps);
}

// ---------------------------------------------
// kpt_iou (OKS): kpt1(N,17,3), kpt2(M,17,3) -> (N,M)
// 用 std::vector<Eigen::Matrix<float,17,3,RowMajor>> 表示关键点
// 每个关键点: (x,y,v) 其中 v!=0 表示可见/标注
// ---------------------------------------------
using Keypoints17x3 = Eigen::Matrix<float, 17, 3, Eigen::RowMajor>;

inline Eigen::MatrixXf kpt_iou(const std::vector<Keypoints17x3>& kpt1,
                               const std::vector<Keypoints17x3>& kpt2,
                               const Eigen::Ref<const Eigen::VectorXf>& area,   // (N,)
                               const std::array<float, 17>& sigma,
                               float eps = 1e-7f) {
    const int N = static_cast<int>(kpt1.size());
    const int M = static_cast<int>(kpt2.size());
    if (area.size() != N) {
        throw std::invalid_argument("kpt_iou: area must have size N (same as kpt1)");
    }

    Eigen::MatrixXf out(N, M);
    out.setZero();

    // precompute (2*sigma)^2
    std::array<float, 17> denom_sigma{};
    for (int k = 0; k < 17; ++k) {
        const float v = 2.0f * sigma[k];
        denom_sigma[k] = v * v;
    }

    for (int i = 0; i < N; ++i) {
        // mask count
        int vis_cnt = 0;
        for (int k = 0; k < 17; ++k) {
            if (kpt1[i](k, 2) != 0.0f) ++vis_cnt;
        }

        for (int j = 0; j < M; ++j) {
            float sum = 0.0f;
            for (int k = 0; k < 17; ++k) {
                if (kpt1[i](k, 2) == 0.0f) continue;

                const float dx = kpt1[i](k, 0) - kpt2[j](k, 0);
                const float dy = kpt1[i](k, 1) - kpt2[j](k, 1);
                const float d2 = dx * dx + dy * dy;

                // e = d / ((2*sigma)^2 * (area + eps) * 2)
                const float e = d2 / (denom_sigma[k] * (area(i) + eps) * 2.0f);
                sum += std::exp(-e);
            }
            out(i, j) = sum / (static_cast<float>(vis_cnt) + eps);
        }
    }

    return out;
}

// ---------------------------------------------
// Oriented bbox helpers: obb format xywhr = [cx, cy, w, h, r]
// _get_covariance_matrix returns (a, b, c) each (N,)
// ---------------------------------------------
inline std::tuple<Eigen::VectorXf, Eigen::VectorXf, Eigen::VectorXf>
get_covariance_matrix(const Eigen::Ref<const Eigen::MatrixXf>& boxes_xywhr) {
    if (boxes_xywhr.cols() != 5) {
        throw std::invalid_argument("get_covariance_matrix: boxes must have shape (N,5) in xywhr");
    }
    const int N = static_cast<int>(boxes_xywhr.rows());

    Eigen::VectorXf a(N), b(N), c(N);
    for (int i = 0; i < N; ++i) {
        const float w = boxes_xywhr(i, 2);
        const float h = boxes_xywhr(i, 3);
        const float r = boxes_xywhr(i, 4);

        // gbbs = [w^2/12, h^2/12, r]
        const float aa = (w * w) / 12.0f;
        const float bb = (h * h) / 12.0f;
        const float cc = r;

        const float cs = std::cos(cc);
        const float sn = std::sin(cc);
        const float cs2 = cs * cs;
        const float sn2 = sn * sn;

        a(i) = aa * cs2 + bb * sn2;
        b(i) = aa * sn2 + bb * cs2;
        c(i) = (aa - bb) * cs * sn;
    }
    return {a, b, c};
}

// ---------------------------------------------
// probiou: obb1(N,5), obb2(N,5) -> (N)
// ---------------------------------------------
inline Eigen::VectorXf probiou(const Eigen::Ref<const Eigen::MatrixXf>& obb1,
                              const Eigen::Ref<const Eigen::MatrixXf>& obb2,
                              bool CIoU = false,
                              float eps = 1e-7f) {
    if (obb1.cols() != 5 || obb2.cols() != 5) {
        throw std::invalid_argument("probiou: obb1/obb2 must be (N,5)");
    }
    if (obb1.rows() != obb2.rows()) {
        throw std::invalid_argument("probiou: obb1 and obb2 must have same N (elementwise)");
    }

    const int N = static_cast<int>(obb1.rows());
    Eigen::VectorXf out(N);

    auto [a1, b1, c1] = get_covariance_matrix(obb1);
    auto [a2, b2, c2] = get_covariance_matrix(obb2);

    for (int i = 0; i < N; ++i) {
        const float x1 = obb1(i, 0), y1 = obb1(i, 1);
        const float x2 = obb2(i, 0), y2 = obb2(i, 1);

        const float A = a1(i) + a2(i);
        const float B = b1(i) + b2(i);
        const float C = c1(i) + c2(i);

        const float denom = (A * B - C * C + eps);

        const float t1 = ((A * detail::sqr(y1 - y2) + B * detail::sqr(x1 - x2)) / denom) * 0.25f;
        const float t2 = ((C * (x2 - x1) * (y1 - y2)) / denom) * 0.5f;

        const float det1 = std::max(0.0f, a1(i) * b1(i) - c1(i) * c1(i));
        const float det2 = std::max(0.0f, a2(i) * b2(i) - c2(i) * c2(i));
        const float t3 = std::log(
                             ( (A * B - C * C) /
                               (4.0f * std::sqrt(det1 * det2) + eps) + eps)
                         ) * 0.5f;

        float bd = std::min(100.0f, std::max(eps, t1 + t2 + t3));
        const float hd = std::sqrt(1.0f - std::exp(-bd) + eps);
        float iou = 1.0f - hd;

        if (CIoU) {
            const float w1 = obb1(i, 2), h1 = obb1(i, 3);
            const float w2 = obb2(i, 2), h2 = obb2(i, 3);
            const float v = (4.0f / (detail::pi() * detail::pi())) * detail::sqr(std::atan(w2 / h2) - std::atan(w1 / h1));
            const float alpha = v / (v - iou + (1.0f + eps));
            iou = iou - v * alpha;
        }

        out(i) = iou;
    }

    return out;
}

// ---------------------------------------------
// batch_probiou: obb1(N,5), obb2(M,5) -> (N,M)
// ---------------------------------------------
inline Eigen::MatrixXf batch_probiou(const Eigen::Ref<const Eigen::MatrixXf>& obb1,
                                    const Eigen::Ref<const Eigen::MatrixXf>& obb2,
                                    float eps = 1e-7f) {
    if (obb1.cols() != 5 || obb2.cols() != 5) {
        throw std::invalid_argument("batch_probiou: obb1/obb2 must be (N,5)/(M,5)");
    }
    const int N = static_cast<int>(obb1.rows());
    const int M = static_cast<int>(obb2.rows());

    auto [a1, b1, c1] = get_covariance_matrix(obb1);
    auto [a2, b2, c2] = get_covariance_matrix(obb2);

    Eigen::MatrixXf out(N, M);

    for (int i = 0; i < N; ++i) {
        const float x1 = obb1(i, 0), y1 = obb1(i, 1);

        for (int j = 0; j < M; ++j) {
            const float x2 = obb2(j, 0), y2 = obb2(j, 1);

            const float A = a1(i) + a2(j);
            const float B = b1(i) + b2(j);
            const float C = c1(i) + c2(j);

            const float denom = (A * B - C * C + eps);

            const float t1 = ((A * detail::sqr(y1 - y2) + B * detail::sqr(x1 - x2)) / denom) * 0.25f;
            const float t2 = ((C * (x2 - x1) * (y1 - y2)) / denom) * 0.5f;

            const float det1 = std::max(0.0f, a1(i) * b1(i) - c1(i) * c1(i));
            const float det2 = std::max(0.0f, a2(j) * b2(j) - c2(j) * c2(j));
            const float t3 = std::log(
                                 ( (A * B - C * C) /
                                   (4.0f * std::sqrt(det1 * det2) + eps) + eps)
                             ) * 0.5f;

            float bd = std::min(100.0f, std::max(eps, t1 + t2 + t3));
            const float hd = std::sqrt(1.0f - std::exp(-bd) + eps);
            out(i, j) = 1.0f - hd;
        }
    }

    return out;
}

// ---------------------------------------------
// smooth_bce
// ---------------------------------------------
inline std::pair<float, float> smooth_bce(float eps = 0.1f) {
    return {1.0f - 0.5f * eps, 0.5f * eps};
}

