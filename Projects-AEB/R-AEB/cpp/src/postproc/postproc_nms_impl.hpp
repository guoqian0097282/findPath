#pragma once

#include <opencv2/core.hpp>
#include <vector>
#include <algorithm>
#include <numeric>
#include <limits>
#include <cmath>

class NumpyNMS {
private:
    static inline bool isVec1D(const cv::Mat& m) {
        return m.dims == 2 && m.channels() == 1 && (m.rows == 1 || m.cols == 1);
    }

    static inline int vecLen1D(const cv::Mat& m) {
        return static_cast<int>(m.total());
    }

    static inline float get1DFloat(const cv::Mat& m, int i) {
        CV_Assert(isVec1D(m));
        CV_Assert(i >= 0 && i < vecLen1D(m));
        const int r = (m.rows == 1) ? 0 : i;
        const int c = (m.rows == 1) ? i : 0;

        switch (m.depth()) {
            case CV_32F: return m.at<float>(r, c);
            case CV_64F: return static_cast<float>(m.at<double>(r, c));
            case CV_32S: return static_cast<float>(m.at<int>(r, c));
            default: CV_Error(cv::Error::StsUnsupportedFormat, "scores/idxs must be CV_32F/CV_64F/CV_32S");
        }
    }

    static inline void set1DFloat(cv::Mat& m, int i, float v) {
        CV_Assert(isVec1D(m));
        CV_Assert(i >= 0 && i < vecLen1D(m));
        const int r = (m.rows == 1) ? 0 : i;
        const int c = (m.rows == 1) ? i : 0;

        switch (m.depth()) {
            case CV_32F: m.at<float>(r, c) = v; break;
            case CV_64F: m.at<double>(r, c) = static_cast<double>(v); break;
            case CV_32S: m.at<int>(r, c) = static_cast<int>(std::lround(v)); break;
            default: CV_Error(cv::Error::StsUnsupportedFormat, "scores must be CV_32F/CV_64F/CV_32S");
        }
    }

    static inline int get1DInt(const cv::Mat& m, int i) {
        CV_Assert(isVec1D(m));
        CV_Assert(i >= 0 && i < vecLen1D(m));
        const int r = (m.rows == 1) ? 0 : i;
        const int c = (m.rows == 1) ? i : 0;

        switch (m.depth()) {
            case CV_32S: return m.at<int>(r, c);
            case CV_32F: return static_cast<int>(std::lround(m.at<float>(r, c)));
            case CV_64F: return static_cast<int>(std::lround(m.at<double>(r, c)));
            default: CV_Error(cv::Error::StsUnsupportedFormat, "idxs must be CV_32S/CV_32F/CV_64F");
        }
    }

    static inline float getBoxVal(const cv::Mat& boxes, int i, int j) {
        // boxes: Nx4, single-channel
        CV_Assert(boxes.dims == 2 && boxes.channels() == 1);
        CV_Assert(j >= 0 && j < boxes.cols);
        CV_Assert(i >= 0 && i < boxes.rows);

        switch (boxes.depth()) {
            case CV_32F: return boxes.at<float>(i, j);
            case CV_64F: return static_cast<float>(boxes.at<double>(i, j));
            default: CV_Error(cv::Error::StsUnsupportedFormat, "boxes must be CV_32F or CV_64F");
        }
    }

    static inline void setBoxVal(cv::Mat& boxes, int i, int j, float v) {
        CV_Assert(boxes.dims == 2 && boxes.channels() == 1);
        CV_Assert(j >= 0 && j < boxes.cols);
        CV_Assert(i >= 0 && i < boxes.rows);

        switch (boxes.depth()) {
            case CV_32F: boxes.at<float>(i, j) = v; break;
            case CV_64F: boxes.at<double>(i, j) = static_cast<double>(v); break;
            default: CV_Error(cv::Error::StsUnsupportedFormat, "boxes must be CV_32F or CV_64F");
        }
    }

    static inline cv::Mat indicesToMat(const std::vector<int>& idx) {
        cv::Mat out(static_cast<int>(idx.size()), 1, CV_32S);
        for (int k = 0; k < static_cast<int>(idx.size()); ++k) out.at<int>(k, 0) = idx[k];
        return out;
    }

public:
    // boxes1: (M,4), boxes2: (N,4) in xyxy
    // return: (M,N) IoU, CV_32F
    static inline cv::Mat box_iou(const cv::Mat& boxes1, const cv::Mat& boxes2) {
        CV_Assert(boxes1.empty() || (boxes1.dims == 2 && boxes1.channels() == 1 && boxes1.cols == 4));
        CV_Assert(boxes2.empty() || (boxes2.dims == 2 && boxes2.channels() == 1 && boxes2.cols == 4));
        CV_Assert(boxes1.empty() || (boxes1.depth() == CV_32F || boxes1.depth() == CV_64F));
        CV_Assert(boxes2.empty() || (boxes2.depth() == CV_32F || boxes2.depth() == CV_64F));

        const int M = boxes1.empty() ? 0 : boxes1.rows;
        const int N = boxes2.empty() ? 0 : boxes2.rows;

        if (M == 0 || N == 0) {
            return cv::Mat::zeros(M, N, CV_32F);
        }

        cv::Mat out(M, N, CV_32F);

        for (int i = 0; i < M; ++i) {
            const float x11 = getBoxVal(boxes1, i, 0);
            const float y11 = getBoxVal(boxes1, i, 1);
            const float x12 = getBoxVal(boxes1, i, 2);
            const float y12 = getBoxVal(boxes1, i, 3);

            const float a1w = std::max(0.0f, x12 - x11);
            const float a1h = std::max(0.0f, y12 - y11);
            const float area1 = a1w * a1h;

            for (int j = 0; j < N; ++j) {
                const float x21 = getBoxVal(boxes2, j, 0);
                const float y21 = getBoxVal(boxes2, j, 1);
                const float x22 = getBoxVal(boxes2, j, 2);
                const float y22 = getBoxVal(boxes2, j, 3);

                const float xx1 = std::max(x11, x21);
                const float yy1 = std::max(y11, y21);
                const float xx2 = std::min(x12, x22);
                const float yy2 = std::min(y12, y22);

                const float w = std::max(0.0f, xx2 - xx1);
                const float h = std::max(0.0f, yy2 - yy1);
                const float inter = w * h;

                const float a2w = std::max(0.0f, x22 - x21);
                const float a2h = std::max(0.0f, y22 - y21);
                const float area2 = a2w * a2h;

                const float uni = area1 + area2 - inter;
                const float denom = std::max(uni, 1e-12f);
                out.at<float>(i, j) = inter / denom;
            }
        }
        return out;
    }

    // fast_nms:
    // boxes: (N,4), scores: (N,1) or (1,N)
    // return: kept indices into original order, as CV_32S column vector
    // NOTE: when use_triu==false, this function updates `scores` in-place (matches your python behavior).
    static inline cv::Mat fast_nms(
        const cv::Mat& boxes,
        cv::Mat& scores,
        float iou_threshold,
        bool use_triu = true,
        bool exit_early = true
    ) {
        CV_Assert(boxes.empty() || (boxes.dims == 2 && boxes.channels() == 1 && boxes.cols == 4));
        CV_Assert(boxes.empty() || (boxes.depth() == CV_32F || boxes.depth() == CV_64F));
        CV_Assert(scores.empty() || isVec1D(scores));

        const int n = boxes.empty() ? 0 : boxes.rows;
        if (n == 0) {
            if (exit_early) return cv::Mat(0, 1, CV_32S);
            return cv::Mat(0, 1, CV_32S);
        }
        CV_Assert(vecLen1D(scores) == n);

        std::vector<int> sorted_idx(n);
        std::iota(sorted_idx.begin(), sorted_idx.end(), 0);

        std::stable_sort(sorted_idx.begin(), sorted_idx.end(),
            [&](int a, int b) {
                const float sa = get1DFloat(scores, a);
                const float sb = get1DFloat(scores, b);
                return sa > sb;
            }
        );

        // boxes_s (sorted)
        std::vector<float> x1(static_cast<size_t>(n), 0.0f);
        std::vector<float> y1(static_cast<size_t>(n), 0.0f);
        std::vector<float> x2(static_cast<size_t>(n), 0.0f);
        std::vector<float> y2(static_cast<size_t>(n), 0.0f);
        std::vector<float> area(static_cast<size_t>(n), 0.0f);
        for (int k = 0; k < n; ++k) {
            const int src = sorted_idx[k];
            const float bx1 = getBoxVal(boxes, src, 0);
            const float by1 = getBoxVal(boxes, src, 1);
            const float bx2 = getBoxVal(boxes, src, 2);
            const float by2 = getBoxVal(boxes, src, 3);
            x1[static_cast<size_t>(k)] = bx1;
            y1[static_cast<size_t>(k)] = by1;
            x2[static_cast<size_t>(k)] = bx2;
            y2[static_cast<size_t>(k)] = by2;
            area[static_cast<size_t>(k)] =
                std::max(0.0f, bx2 - bx1) * std::max(0.0f, by2 - by1);
        }

        auto iou_sorted = [&](int i, int j) -> float {
            const float x11 = x1[static_cast<size_t>(i)];
            const float y11 = y1[static_cast<size_t>(i)];
            const float x12 = x2[static_cast<size_t>(i)];
            const float y12 = y2[static_cast<size_t>(i)];
            const float x21 = x1[static_cast<size_t>(j)];
            const float y21 = y1[static_cast<size_t>(j)];
            const float x22 = x2[static_cast<size_t>(j)];
            const float y22 = y2[static_cast<size_t>(j)];

            const float xx1 = std::max(x11, x21);
            const float yy1 = std::max(y11, y21);
            const float xx2 = std::min(x12, x22);
            const float yy2 = std::min(y12, y22);
            const float w = std::max(0.0f, xx2 - xx1);
            const float h = std::max(0.0f, yy2 - yy1);
            const float inter = w * h;
            const float uni = area[static_cast<size_t>(i)] + area[static_cast<size_t>(j)] - inter;
            return inter / std::max(uni, 1e-12f);
        };

        if (use_triu) {
            std::vector<int> pick;
            pick.reserve(n);

            for (int j = 0; j < n; ++j) {
                bool keepj = true;
                for (int i = 0; i < j; ++i) { // only upper triangle k=1 effect (i < j)
                    if (iou_sorted(i, j) >= iou_threshold) {
                        keepj = false;
                        break;
                    }
                }
                if (keepj) pick.push_back(j);
            }

            std::vector<int> out_idx;
            out_idx.reserve(pick.size());
            for (int p : pick) out_idx.push_back(sorted_idx[p]);
            return indicesToMat(out_idx);
        } else {
            // mimic python: scores_ = scores[sorted_idx].copy(); suppressed -> 0; scores[sorted_idx] = scores_;
            std::vector<float> scores_sorted(n);
            for (int k = 0; k < n; ++k) scores_sorted[k] = get1DFloat(scores, sorted_idx[k]);

            for (int j = 0; j < n; ++j) {
                bool keepj = true;
                for (int i = 0; i < j; ++i) {
                    if (iou_sorted(i, j) >= iou_threshold) {
                        keepj = false;
                        break;
                    }
                }
                if (!keepj) scores_sorted[j] = 0.0f;
            }

            for (int k = 0; k < n; ++k) {
                set1DFloat(scores, sorted_idx[k], scores_sorted[k]);
            }

            std::vector<int> pick(n);
            std::iota(pick.begin(), pick.end(), 0);
            std::stable_sort(pick.begin(), pick.end(),
                [&](int a, int b) { return scores_sorted[a] > scores_sorted[b]; }
            );

            std::vector<int> out_idx;
            out_idx.reserve(n);
            for (int p : pick) out_idx.push_back(sorted_idx[p]);
            return indicesToMat(out_idx);
        }
    }

    // classic greedy NMS
    static inline cv::Mat nms(const cv::Mat& boxes, const cv::Mat& scores, float iou_threshold) {
        CV_Assert(boxes.empty() || (boxes.dims == 2 && boxes.channels() == 1 && boxes.cols == 4));
        CV_Assert(boxes.empty() || (boxes.depth() == CV_32F || boxes.depth() == CV_64F));
        CV_Assert(scores.empty() || isVec1D(scores));

        const int n = boxes.empty() ? 0 : boxes.rows;
        if (n == 0) return cv::Mat(0, 1, CV_32S);
        CV_Assert(vecLen1D(scores) == n);

        std::vector<float> x1(n), y1(n), x2(n), y2(n), area(n);
        for (int i = 0; i < n; ++i) {
            x1[i] = getBoxVal(boxes, i, 0);
            y1[i] = getBoxVal(boxes, i, 1);
            x2[i] = getBoxVal(boxes, i, 2);
            y2[i] = getBoxVal(boxes, i, 3);
            const float w = std::max(0.0f, x2[i] - x1[i]);
            const float h = std::max(0.0f, y2[i] - y1[i]);
            area[i] = w * h;
        }

        std::vector<int> order(n);
        std::iota(order.begin(), order.end(), 0);
        std::stable_sort(order.begin(), order.end(),
            [&](int a, int b) { return get1DFloat(scores, a) > get1DFloat(scores, b); }
        );

        std::vector<int> keep;
        keep.reserve(n);

        while (!order.empty()) {
            const int i = order.front();
            keep.push_back(i);
            if (order.size() == 1) break;

            std::vector<int> rest(order.begin() + 1, order.end());
            order.clear();
            order.reserve(rest.size());

            float inter_sum = 0.0f;
            std::vector<float> iou(rest.size(), 0.0f);

            for (size_t k = 0; k < rest.size(); ++k) {
                const int j = rest[k];
                const float xx1 = std::max(x1[i], x1[j]);
                const float yy1 = std::max(y1[i], y1[j]);
                const float xx2 = std::min(x2[i], x2[j]);
                const float yy2 = std::min(y2[i], y2[j]);
                const float w = std::max(0.0f, xx2 - xx1);
                const float h = std::max(0.0f, yy2 - yy1);
                const float inter = w * h;
                inter_sum += inter;
                if (inter > 0.0f) {
                    const float denom = (area[i] + area[j] - inter);
                    iou[k] = inter / std::max(denom, 1e-12f);
                } else {
                    iou[k] = 0.0f;
                }
            }

            if (inter_sum == 0.0f) {
                order = std::move(rest);
                continue;
            }

            for (size_t k = 0; k < rest.size(); ++k) {
                if (iou[k] <= iou_threshold) order.push_back(rest[k]);
            }
        }

        return indicesToMat(keep);
    }

    // batched_nms: offset boxes by class idxs, then run NMS
    static inline cv::Mat batched_nms(
        const cv::Mat& boxes,
        cv::Mat& scores,
        const cv::Mat& idxs,
        float iou_threshold,
        bool use_fast_nms = false
    ) {
        CV_Assert(boxes.empty() || (boxes.dims == 2 && boxes.channels() == 1 && boxes.cols == 4));
        CV_Assert(boxes.empty() || (boxes.depth() == CV_32F || boxes.depth() == CV_64F));
        CV_Assert(scores.empty() || isVec1D(scores));
        CV_Assert(idxs.empty() || isVec1D(idxs));

        const int n = boxes.empty() ? 0 : boxes.rows;
        if (n == 0) return cv::Mat(0, 1, CV_32S);

        CV_Assert(vecLen1D(scores) == n);
        CV_Assert(vecLen1D(idxs) == n);

        // max_coordinate = boxes.max()
        float max_coord = -std::numeric_limits<float>::infinity();
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < 4; ++j) {
                max_coord = std::max(max_coord, getBoxVal(boxes, i, j));
            }
        }

        const float stride = max_coord + 1.0f;

        // boxes_for_nms = boxes + offsets[:,None]
        cv::Mat boxes_for_nms(boxes.rows, boxes.cols, CV_32F);
        for (int i = 0; i < n; ++i) {
            const float off = static_cast<float>(get1DInt(idxs, i)) * stride;
            boxes_for_nms.at<float>(i, 0) = getBoxVal(boxes, i, 0) + off;
            boxes_for_nms.at<float>(i, 1) = getBoxVal(boxes, i, 1) + off;
            boxes_for_nms.at<float>(i, 2) = getBoxVal(boxes, i, 2) + off;
            boxes_for_nms.at<float>(i, 3) = getBoxVal(boxes, i, 3) + off;
        }

        if (use_fast_nms) {
            return fast_nms(boxes_for_nms, scores, iou_threshold, /*use_triu=*/true, /*exit_early=*/true);
        } else {
            return nms(boxes_for_nms, scores, iou_threshold);
        }
    }
};
