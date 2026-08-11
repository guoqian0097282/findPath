#pragma once

#include <opencv2/core.hpp>

#include <algorithm>
#include <cstddef>
#include <numeric>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>
#include <array>
#include <memory>

// ====== 你会在 BYTETracker 里用到：
// matching::linear_assignment(cost, thresh)
// matching::iou_distance(tracks, dets)
// matching::fuse_score(cost, dets)
//
// 这里不使用 namespace，用 struct matching 来“模拟模块”。
// ======

struct LinearAssignmentResult {
    std::vector<std::pair<int, int>> matches;  // (row, col)
    std::vector<int> unmatched_a;              // rows
    std::vector<int> unmatched_b;              // cols
};

inline float _iou_xyxy(const std::array<float, 4>& a,
                       const std::array<float, 4>& b) {
    const float ax1 = a[0], ay1 = a[1], ax2 = a[2], ay2 = a[3];
    const float bx1 = b[0], by1 = b[1], bx2 = b[2], by2 = b[3];

    const float ix1 = std::max(ax1, bx1);
    const float iy1 = std::max(ay1, by1);
    const float ix2 = std::min(ax2, bx2);
    const float iy2 = std::min(ay2, by2);

    const float iw = std::max(0.0f, ix2 - ix1);
    const float ih = std::max(0.0f, iy2 - iy1);
    const float inter = iw * ih;

    const float aw = std::max(0.0f, ax2 - ax1);
    const float ah = std::max(0.0f, ay2 - ay1);
    const float bw = std::max(0.0f, bx2 - bx1);
    const float bh = std::max(0.0f, by2 - by1);

    const float area_a = aw * ah;
    const float area_b = bw * bh;
    const float uni = area_a + area_b - inter;
    if (uni <= 0.0f) return 0.0f;
    return inter / uni;
}

// Hungarian(min) with "dummy cols" trick (cost_limit=thresh)
inline std::vector<int> _hungarian_minimize(const std::vector<std::vector<float>>& a) {
    const int n = static_cast<int>(a.size());
    const int m = n > 0 ? static_cast<int>(a[0].size()) : 0;
    if (n == 0) return {};
    if (m == 0) return std::vector<int>(n, -1);

    const float INF = 1e30f;
    std::vector<float> u(n + 1, 0.0f), v(m + 1, 0.0f);
    std::vector<int> p(m + 1, 0), way(m + 1, 0);

    for (int i = 1; i <= n; ++i) {
        p[0] = i;
        int j0 = 0;
        std::vector<float> minv(m + 1, INF);
        std::vector<char> used(m + 1, false);

        do {
            used[j0] = true;
            const int i0 = p[j0];
            int j1 = 0;
            float delta = INF;

            for (int j = 1; j <= m; ++j) {
                if (used[j]) continue;
                const float cur = a[i0 - 1][j - 1] - u[i0] - v[j];
                if (cur < minv[j]) {
                    minv[j] = cur;
                    way[j] = j0;
                }
                if (minv[j] < delta) {
                    delta = minv[j];
                    j1 = j;
                }
            }
            for (int j = 0; j <= m; ++j) {
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
        } while (p[j0] != 0);

        do {
            const int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0 != 0);
    }

    std::vector<int> ans(n, -1);
    for (int j = 1; j <= m; ++j) {
        if (p[j] > 0) ans[p[j] - 1] = j - 1;
    }
    return ans;
}

template <class T>
struct _is_shared_ptr : std::false_type {};
template <class T>
struct _is_shared_ptr<std::shared_ptr<T>> : std::true_type {};

template <class T>
inline std::array<float, 4> _get_xyxy(const T& t) {
    if constexpr (std::is_pointer_v<T>) {
        return t->xyxy();
    } else if constexpr (_is_shared_ptr<T>::value) {
        return t->xyxy();
    } else {
        return t.xyxy();
    }
}

template <class T>
inline float _get_score(const T& t) {
    if constexpr (std::is_pointer_v<T>) {
        return t->score();
    } else if constexpr (_is_shared_ptr<T>::value) {
        return t->score();
    } else {
        return t.score();
    }
}

struct matching {
    static LinearAssignmentResult linear_assignment(const cv::Mat& cost_matrix, float thresh) {
        LinearAssignmentResult out;

        if (cost_matrix.empty()) {
            out.matches.clear();
            out.unmatched_a.clear();
            out.unmatched_b.clear();
            return out;
        }
        if (cost_matrix.dims != 2 || cost_matrix.type() != CV_32F) {
            throw std::runtime_error("cost_matrix must be 2D CV_32F");
        }

        const int n = cost_matrix.rows;
        const int m = cost_matrix.cols;

        if (n == 0 || m == 0) {
            out.matches.clear();
            out.unmatched_a.resize(n);
            std::iota(out.unmatched_a.begin(), out.unmatched_a.end(), 0);
            out.unmatched_b.resize(m);
            std::iota(out.unmatched_b.begin(), out.unmatched_b.end(), 0);
            return out;
        }

        // add dummy cols to allow "unmatched" with cost=thresh
        const int madd = m + n;
        std::vector<std::vector<float>> a(n, std::vector<float>(madd, thresh));
        for (int i = 0; i < n; ++i) {
            const float* row = cost_matrix.ptr<float>(i);
            for (int j = 0; j < m; ++j) a[i][j] = row[j];
        }

        const auto assign = _hungarian_minimize(a);
        std::vector<int> col_used(m, 0);

        for (int i = 0; i < n; ++i) {
            const int j = assign[i];
            if (j >= 0 && j < m && a[i][j] <= thresh) {
                out.matches.emplace_back(i, j);
                col_used[j] = 1;
            } else {
                out.unmatched_a.push_back(i);
            }
        }
        for (int j = 0; j < m; ++j) if (!col_used[j]) out.unmatched_b.push_back(j);

        return out;
    }

    template <class TrackT>
    static cv::Mat iou_distance(const std::vector<TrackT>& atracks,
                                const std::vector<TrackT>& btracks) {
        cv::Mat cost(static_cast<int>(atracks.size()), static_cast<int>(btracks.size()), CV_32F, cv::Scalar(0));
        for (int i = 0; i < cost.rows; ++i) {
            const auto a = _get_xyxy(atracks[i]);
            for (int j = 0; j < cost.cols; ++j) {
                const auto b = _get_xyxy(btracks[j]);
                const float iou = _iou_xyxy(a, b);
                cost.at<float>(i, j) = 1.0f - iou;
            }
        }
        return cost;
    }

    template <class DetT>
    static cv::Mat fuse_score(const cv::Mat& cost_matrix,
                              const std::vector<DetT>& detections) {
        if (cost_matrix.empty()) return cost_matrix;
        if (cost_matrix.type() != CV_32F || cost_matrix.dims != 2) {
            throw std::runtime_error("cost_matrix must be 2D CV_32F");
        }
        cv::Mat fused = cost_matrix.clone();
        for (int i = 0; i < fused.rows; ++i) {
            for (int j = 0; j < fused.cols; ++j) {
                const float iou_sim = 1.0f - fused.at<float>(i, j);
                const float ds = _get_score(detections[j]);
                const float fuse_sim = iou_sim * ds;
                fused.at<float>(i, j) = 1.0f - fuse_sim;
            }
        }
        return fused;
    }
};
