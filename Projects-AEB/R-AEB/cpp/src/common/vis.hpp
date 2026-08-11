#pragma once

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <unordered_map>
#include <vector>


// ------------------------------------------------------------
// 活泼年轻风固定颜色表（BGR）
// ------------------------------------------------------------
static inline const std::array<cv::Scalar, 36>& COLOR_MAP_BGR() {
    static const std::array<cv::Scalar, 36> cmap = {
        cv::Scalar(107, 107, 255),  // id 0  | RGB #FF6B6B | Coral Red (珊瑚红)
        cv::Scalar( 43, 146, 255),  // id 1  | RGB #FF922B | Vivid Orange (活力橙)
        cv::Scalar( 59, 212, 255),  // id 2  | RGB #FFD43B | Sunny Yellow (阳光黄)
        cv::Scalar( 75, 227, 169),  // id 3  | RGB #A9E34B | Lime Green (青柠绿)
        cv::Scalar( 87, 192,  64),  // id 4  | RGB #40C057 | Fresh Green (清新绿)
        cv::Scalar(151, 201,  32),  // id 5  | RGB #20C997 | Mint/Teal (薄荷青)
        cv::Scalar(191, 170,  21),  // id 6  | RGB #15AABF | Cyan Teal (青蓝)
        cv::Scalar(230, 139,  34),  // id 7  | RGB #228BE6 | Bright Blue (亮蓝)
        cv::Scalar(245, 110,  76),  // id 8  | RGB #4C6EF5 | Indigo Blue (靛蓝)
        cv::Scalar(242,  80, 121),  // id 9  | RGB #7950F2 | Purple (紫罗兰)
        cv::Scalar(219,  75, 190),  // id 10 | RGB #BE4BDB | Neon Purple (霓虹紫)
        cv::Scalar(149, 101, 240),  // id 11 | RGB #F06595 | Hot Pink (亮粉)

        cv::Scalar(135, 135, 255),  // id 12 | RGB #FF8787 | Soft Coral (浅珊瑚)
        cv::Scalar( 77, 169, 255),  // id 13 | RGB #FFA94D | Soft Orange (浅橙)
        cv::Scalar(102, 224, 255),  // id 14 | RGB #FFE066 | Soft Yellow (浅黄)
        cv::Scalar(162, 245, 216),  // id 15 | RGB #D8F5A2 | Pastel Lime (柔青柠)
        cv::Scalar(154, 233, 140),  // id 16 | RGB #8CE99A | Pastel Green (柔绿)
        cv::Scalar(190, 230,  99),  // id 17 | RGB #63E6BE | Pastel Mint (柔薄荷)
        cv::Scalar(232, 217, 102),  // id 18 | RGB #66D9E8 | Pastel Cyan (柔青蓝)
        cv::Scalar(252, 192, 116),  // id 19 | RGB #74C0FC | Pastel Sky (柔天蓝)
        cv::Scalar(255, 167, 145),  // id 20 | RGB #91A7FF | Pastel Periwinkle (柔紫蓝)
        cv::Scalar(252, 151, 177),  // id 21 | RGB #B197FC | Pastel Purple (柔紫)
        cv::Scalar(247, 153, 229),  // id 22 | RGB #E599F7 | Pastel Magenta (柔洋红)
        cv::Scalar(215, 194, 252),  // id 23 | RGB #FCC2D7 | Pastel Pink (柔粉)

        cv::Scalar( 49,  49, 224),  // id 24 | RGB #E03131 | Deep Red (深红)
        cv::Scalar( 92,  37, 194),  // id 25 | RGB #C2255C | Raspberry (树莓红)
        cv::Scalar(  0, 140, 240),  // id 26 | RGB #F08C00 | Amber Orange (琥珀橙)
        cv::Scalar(  5, 176, 250),  // id 27 | RGB #FAB005 | Golden Yellow (金黄)
        cv::Scalar( 68, 158,  47),  // id 28 | RGB #2F9E44 | Emerald Green (祖母绿)
        cv::Scalar(120, 166,  12),  // id 29 | RGB #0CA678 | Teal Green (蓝绿)
        cv::Scalar(173, 152,  16),  // id 30 | RGB #1098AD | Deep Teal (深青)
        cv::Scalar(171, 100,  24),  // id 31 | RGB #1864AB | Deep Blue (深蓝)
        cv::Scalar(199,  79,  54),  // id 32 | RGB #364FC7 | Royal Blue (皇家蓝)
        cv::Scalar(196,  61,  95),  // id 33 | RGB #5F3DC4 | Deep Purple (深紫)
        cv::Scalar(156,  46, 134),  // id 34 | RGB #862E9C | Grape Purple (葡萄紫)
        cv::Scalar( 77,  30, 166),  // id 35 | RGB #A61E4D | Wine Magenta (酒红紫)
    };
    return cmap;
}

// 只保留这一份：Python 的 int(cid) % len(COLOR_MAP_BGR)（含负数行为）
static inline cv::Scalar color_for_cls(int cid) {
    const auto& cmap = COLOR_MAP_BGR();
    const int n = static_cast<int>(cmap.size());
    int r = cid % n;
    if (r < 0) r += n;
    return cmap[static_cast<size_t>(r)];
}

// ------------------------------------------------------------
// draw_2d_boxes
// img: (H,W,3) CV_8UC3
// objs: (N,6) CV_32F  或 (6,) CV_32F（dims==1, size[0]==6）
// ------------------------------------------------------------
static inline cv::Mat draw_2d_boxes(
    const cv::Mat& img,
    const cv::Mat& objs,
    const std::unordered_map<int, std::string>* id2name = nullptr,
    const std::vector<std::string>* txts = nullptr,
    bool draw_label = true,
    int thickness = 1
) {
    if (img.empty()) throw std::runtime_error("img is empty");
    if (!(img.dims == 2 && img.type() == CV_8UC3)) throw std::runtime_error("img must be (H,W,3) CV_8UC3");
    if (objs.empty() || objs.total() == 0) return img.clone();
    if (objs.type() != CV_32F) throw std::runtime_error("objs must be CV_32F");

    int N = 0;
    bool single = false;

    if (objs.dims == 1) {
        if (objs.size[0] != 6) throw std::runtime_error("objs must be (N,6) or (6,)");
        N = 1;
        single = true;
    } else if (objs.dims == 2) {
        if (objs.cols != 6) throw std::runtime_error("objs must have shape (N,6)");
        N = objs.rows;
    } else {
        throw std::runtime_error("objs must be (N,6) or (6,)");
    }

    if (txts && (int)txts->size() != N) throw std::runtime_error("txts length must match objs N");

    cv::Mat out = img.clone();
    const int H = out.rows, W = out.cols;

    // argsort by conf (col 4)
    std::vector<int> order(N);
    for (int i = 0; i < N; ++i) order[i] = i;

    auto conf_at = [&](int i) -> float {
        if (single) {
            int idx[1] = {4};
            return objs.at<float>(idx);
        } else {
            return objs.at<float>(i, 4);
        }
    };
    std::sort(order.begin(), order.end(), [&](int a, int b) { return conf_at(a) < conf_at(b); });

    for (int ii = 0; ii < N; ++ii) {
        int i = order[ii];

        float x1, y1, x2, y2, conf, cls;
        if (single) {
            int i0[1]={0}, i1[1]={1}, i2[1]={2}, i3[1]={3}, i4[1]={4}, i5[1]={5};
            x1 = objs.at<float>(i0);
            y1 = objs.at<float>(i1);
            x2 = objs.at<float>(i2);
            y2 = objs.at<float>(i3);
            conf = objs.at<float>(i4);
            cls  = objs.at<float>(i5);
        } else {
            x1 = objs.at<float>(i, 0);
            y1 = objs.at<float>(i, 1);
            x2 = objs.at<float>(i, 2);
            y2 = objs.at<float>(i, 3);
            conf = objs.at<float>(i, 4);
            cls  = objs.at<float>(i, 5);
        }

        int cls_id = (int)std::lround(cls);
        cv::Scalar color = color_for_cls(cls_id);

        int x1i = std::clamp((int)std::lround(x1), 0, W - 1);
        int y1i = std::clamp((int)std::lround(y1), 0, H - 1);
        int x2i = std::clamp((int)std::lround(x2), 0, W - 1);
        int y2i = std::clamp((int)std::lround(y2), 0, H - 1);

        cv::rectangle(out, {x1i, y1i}, {x2i, y2i}, color, thickness, cv::LINE_AA);

        if (draw_label) {
            std::string txt;
            if (txts) {
                txt = (*txts)[i];
            } else {
                std::string name = std::to_string(cls_id);
                if (id2name) {
                    auto it = id2name->find(cls_id);
                    if (it != id2name->end()) name = it->second;
                }
                char buf[64];
                std::snprintf(buf, sizeof(buf), "%.2f", conf);
                txt = name + " " + buf;
            }

            if (!txt.empty()) {
                int base = 0;
                auto ts = cv::getTextSize(txt, cv::FONT_HERSHEY_SIMPLEX, 0.45, 1, &base);
                const int pad_x = 3, pad_y = 2;

                int x0 = std::max(0, x1i);
                int y0 = std::max(0, y1i - ts.height - 2 * pad_y);
                int x1b = std::min(W - 1, x0 + ts.width + 2 * pad_x);
                int y1b = std::min(H - 1, y0 + ts.height + 2 * pad_y);

                cv::rectangle(out, {x0, y0}, {x1b, y1b}, color, cv::FILLED);
                cv::Point org(x0 + pad_x, y0 + ts.height + pad_y - base / 2);
                cv::putText(out, txt, org, cv::FONT_HERSHEY_SIMPLEX, 0.45, {0, 0, 0}, 1, cv::LINE_8);
            }
        }
    }

    return out;
}

// ------------------------------------------------------------
// draw_2d_instances
// masks: (N,H,W) CV_32F/CV_8U 或 (H,W) CV_32F/CV_8U
// ------------------------------------------------------------
static inline cv::Mat draw_2d_instances(
    const cv::Mat& img,
    const cv::Mat& objs,
    const cv::Mat& masks,
    const std::unordered_map<int, std::string>* id2name = nullptr,
    const std::vector<std::string>* txts = nullptr,
    bool draw_bbox = true,
    bool draw_label = true,
    double mask_alpha = 0.45,
    int thickness = 1
) {
    if (img.empty()) throw std::runtime_error("img is empty");
    if (!(img.dims == 2 && img.type() == CV_8UC3)) throw std::runtime_error("img must be (H,W,3) CV_8UC3");
    if (objs.empty() || objs.total() == 0) return img.clone();
    if (objs.type() != CV_32F) throw std::runtime_error("objs must be CV_32F");

    const int H = img.rows, W = img.cols;

    // masks is None/empty
    if (masks.empty() || masks.total() == 0) {
        if (draw_bbox || draw_label) return draw_2d_boxes(img, objs, id2name, txts, draw_label, thickness);
        return img.clone();
    }
    if (masks.type() != CV_32F && masks.type() != CV_8U) {
        throw std::runtime_error("masks must be CV_32F or CV_8U");
    }

    // objs N（只按 python 支持的两种）
    int N_obj = 0;
    bool objs_single = false;
    if (objs.dims == 1) {
        if (objs.size[0] != 6) throw std::runtime_error("objs must be (N,6) or (6,)");
        N_obj = 1;
        objs_single = true;
    } else if (objs.dims == 2) {
        if (objs.cols != 6) throw std::runtime_error("objs must have shape (N,6)");
        N_obj = objs.rows;
    } else {
        throw std::runtime_error("objs must be (N,6) or (6,)");
    }

    // masks N
    int N_mask = 0;
    bool mask_single = false;
    if (masks.dims == 2) {
        if (!(masks.rows == H && masks.cols == W)) throw std::runtime_error("mask size must match image size");
        N_mask = 1;
        mask_single = true;
    } else if (masks.dims == 3) {
        if (!(masks.size[1] == H && masks.size[2] == W)) throw std::runtime_error("mask size must match image size");
        N_mask = masks.size[0];
    } else {
        throw std::runtime_error("masks must be (N,H,W) or (H,W)");
    }

    int N = std::min(N_obj, N_mask);

    // order = argsort(objs[:N,4])
    std::vector<int> order(N);
    for (int i = 0; i < N; ++i) order[i] = i;

    auto conf_at = [&](int i) -> float {
        if (objs_single) {
            int idx[1] = {4};
            return objs.at<float>(idx);
        } else {
            return objs.at<float>(i, 4);
        }
    };
    std::sort(order.begin(), order.end(), [&](int a, int b) { return conf_at(a) < conf_at(b); });

    cv::Mat out = img.clone();
    cv::Mat mb(H, W, CV_8U);
    cv::Mat overlay(out.size(), out.type(), cv::Scalar(0, 0, 0));

    for (int ii = 0; ii < N; ++ii) {
        int i = order[ii];

        // cls_id
        float cls = 0.f;
        if (objs_single) {
            int idx[1] = {5};
            cls = objs.at<float>(idx);
        } else {
            cls = objs.at<float>(i, 5);
        }
        int cls_id = (int)std::lround(cls);
        cv::Scalar color = color_for_cls(cls_id);

        // m = masks[i]
        cv::Mat m2d;
        if (mask_single) {
            m2d = masks;
        } else {
            if (masks.type() == CV_32F) {
                const float* base = masks.ptr<float>(i);
                m2d = cv::Mat(H, W, CV_32F, const_cast<float*>(base), masks.step[1]);
            } else {
                const uchar* base = masks.ptr<uchar>(i);
                m2d = cv::Mat(H, W, CV_8U, const_cast<uchar*>(base), masks.step[1]);
            }
        }

        // 与原逻辑保持一致：先构造 overlay，再做 addWeighted(out,1.0,overlay,alpha,0)。
        if (m2d.type() == CV_8U) {
            cv::compare(m2d, cv::Scalar(0), mb, cv::CMP_NE);
        } else {
            cv::compare(m2d, 0.0f, mb, cv::CMP_NE);
        }
        if (cv::countNonZero(mb) > 0) {
            overlay.setTo(cv::Scalar(0, 0, 0));
            overlay.setTo(color, mb);
            cv::addWeighted(out, 1.0, overlay, mask_alpha, 0.0, out);
        }
    }

    if (draw_bbox || draw_label) {
        // python: objs = objs[:N]
        cv::Mat objs_cut;
        if (objs_single) {
            objs_cut = objs;
        } else {
            objs_cut = objs.rowRange(0, N);
        }

        // python: txts = list(txts)[:N]
        std::vector<std::string> txts_cut;
        const std::vector<std::string>* txts_use = nullptr;
        if (txts) {
            if ((int)txts->size() != N_obj) throw std::runtime_error("txts length must match objs N");
            txts_cut.assign(txts->begin(), txts->begin() + N);
            txts_use = &txts_cut;
        }

        out = draw_2d_boxes(out, objs_cut, id2name, txts_use, draw_label, thickness);
    }

    return out;
}

// ------------------------------------------------------------
// boxes3d_to_corners3d
// boxes: (N,7) CV_32F 或 (7,) CV_32F
// corners: (N,8,3) CV_32F
// ------------------------------------------------------------
static inline cv::Mat boxes3d_to_corners3d(const cv::Mat& boxes) {
    if (boxes.empty() || boxes.total() == 0) {
        int sz[3] = {0, 8, 3};
        return cv::Mat(3, sz, CV_32F);
    }
    if (boxes.type() != CV_32F) throw std::runtime_error("boxes must be CV_32F");

    int N = 0;
    bool single = false;

    if (boxes.dims == 1) {
        if (boxes.size[0] != 7) throw std::runtime_error("boxes must be (N,7) or (7,)");
        N = 1;
        single = true;
    } else if (boxes.dims == 2) {
        if (boxes.cols != 7) throw std::runtime_error("boxes must have shape (N,7)");
        N = boxes.rows;
    } else {
        throw std::runtime_error("boxes must be (N,7) or (7,)");
    }

    int out_sz[3] = {N, 8, 3};
    cv::Mat corners(3, out_sz, CV_32F);

    for (int i = 0; i < N; ++i) {
        float cx, cy, cz, l, w, h, theta;

        if (single) {
            int i0[1]={0}, i1[1]={1}, i2[1]={2}, i3[1]={3}, i4[1]={4}, i5[1]={5}, i6[1]={6};
            cx = boxes.at<float>(i0);
            cy = boxes.at<float>(i1);
            cz = boxes.at<float>(i2);
            l  = boxes.at<float>(i3);
            w  = boxes.at<float>(i4);
            h  = boxes.at<float>(i5);
            theta = boxes.at<float>(i6);
        } else {
            cx = boxes.at<float>(i, 0);
            cy = boxes.at<float>(i, 1);
            cz = boxes.at<float>(i, 2);
            l  = boxes.at<float>(i, 3);
            w  = boxes.at<float>(i, 4);
            h  = boxes.at<float>(i, 5);
            theta = boxes.at<float>(i, 6);
        }

        float x_c[8] = {-l/2, +l/2, +l/2, -l/2, -l/2, +l/2, +l/2, -l/2};
        float y_c[8] = {-w/2, -w/2, +w/2, +w/2, -w/2, -w/2, +w/2, +w/2};
        float z_c[8] = {-h/2, -h/2, -h/2, -h/2, +h/2, +h/2, +h/2, +h/2};

        float c = std::cos(theta);
        float s = std::sin(theta);

        for (int k = 0; k < 8; ++k) {
            float x_w = c * x_c[k] - s * y_c[k] + cx;
            float y_w = s * x_c[k] + c * y_c[k] + cy;
            float z_w = z_c[k] + cz;

            int ix[3] = {i, k, 0};
            int iy[3] = {i, k, 1};
            int iz[3] = {i, k, 2};
            corners.at<float>(ix) = x_w;
            corners.at<float>(iy) = y_w;
            corners.at<float>(iz) = z_w;
        }
    }

    return corners;
}

// ------------------------------------------------------------
// corners3d_to_boxes3d
// corners: (N,8,3) CV_32F 或 (8,3) CV_32F
// boxes: (N,7) CV_32F
// ------------------------------------------------------------
static inline cv::Mat corners3d_to_boxes3d(const cv::Mat& corners) {
    if (corners.empty() || corners.total() == 0) {
        return cv::Mat(0, 7, CV_32F);
    }
    if (corners.type() != CV_32F) throw std::runtime_error("corners must be CV_32F");

    int N = 0;
    bool single = false;

    if (corners.dims == 2) {
        if (!(corners.rows == 8 && corners.cols == 3)) throw std::runtime_error("corners must be (N,8,3) or (8,3)");
        N = 1;
        single = true;
    } else if (corners.dims == 3) {
        if (!(corners.size[1] == 8 && corners.size[2] == 3)) throw std::runtime_error("corners must have shape (N,8,3)");
        N = corners.size[0];
    } else {
        throw std::runtime_error("corners must be (N,8,3) or (8,3)");
    }

    cv::Mat boxes(N, 7, CV_32F);

    auto get_corner = [&](int i, int k, int d) -> float {
        if (single) return corners.at<float>(k, d);
        int idx[3] = {i, k, d};
        return corners.at<float>(idx);
    };

    for (int i = 0; i < N; ++i) {
        float sx=0, sy=0, sz=0;
        for (int k = 0; k < 8; ++k) {
            sx += get_corner(i, k, 0);
            sy += get_corner(i, k, 1);
            sz += get_corner(i, k, 2);
        }
        float cx = sx / 8.0f;
        float cy = sy / 8.0f;
        float cz = sz / 8.0f;

        float x0 = get_corner(i, 0, 0), y0 = get_corner(i, 0, 1);
        float x1 = get_corner(i, 1, 0), y1 = get_corner(i, 1, 1);
        float x2 = get_corner(i, 2, 0), y2 = get_corner(i, 2, 1);
        float x3 = get_corner(i, 3, 0), y3 = get_corner(i, 3, 1);

        float v01x = x1 - x0, v01y = y1 - y0;
        float v12x = x2 - x1, v12y = y2 - y1;

        float l = std::sqrt(v01x*v01x + v01y*v01y + 1e-8f);
        float w = std::sqrt(v12x*v12x + v12y*v12y + 1e-8f);

        float zmin = get_corner(i, 0, 2);
        float zmax = zmin;
        for (int k = 1; k < 8; ++k) {
            float z = get_corner(i, k, 2);
            zmin = std::min(zmin, z);
            zmax = std::max(zmax, z);
        }
        float h = std::max(0.0f, zmax - zmin);

        float v32x = x2 - x3, v32y = y2 - y3;

        auto norm2 = [](float x, float y) {
            float n = std::sqrt(x*x + y*y + 1e-8f);
            return std::pair<float,float>(x/n, y/n);
        };
        auto n01 = norm2(v01x, v01y);
        auto n32 = norm2(v32x, v32y);

        float vdirx = n01.first + n32.first;
        float vdiry = n01.second + n32.second;
        float theta = std::atan2(vdiry, vdirx);

        boxes.at<float>(i, 0) = cx;
        boxes.at<float>(i, 1) = cy;
        boxes.at<float>(i, 2) = cz;
        boxes.at<float>(i, 3) = l;
        boxes.at<float>(i, 4) = w;
        boxes.at<float>(i, 5) = h;
        boxes.at<float>(i, 6) = theta;
    }

    return boxes;
}

// ------------------------------------------------------------
// draw_3d_boxes
// corners_uv: (N,8,2) CV_32F 或 (8,2) CV_32F
// objs: 可选 (N,6) CV_32F（用于颜色/label/排序）
// ------------------------------------------------------------
static inline cv::Mat draw_3d_boxes(
    const cv::Mat& img,
    const cv::Mat& corners_uv,
    const cv::Mat* objs_ptr = nullptr,
    const std::unordered_map<int, std::string>* id2name = nullptr,
    const std::vector<std::string>* txts = nullptr,
    bool draw_label = true,
    int thickness = 1
) {
    if (img.empty()) throw std::runtime_error("img is empty");
    if (!(img.dims == 2 && img.type() == CV_8UC3)) throw std::runtime_error("img must be (H,W,3) CV_8UC3");
    if (corners_uv.empty() || corners_uv.total() == 0) return img.clone();
    if (corners_uv.type() != CV_32F) throw std::runtime_error("corners_uv must be CV_32F");

    int N = 0;
    bool single = false;

    if (corners_uv.dims == 2) {
        if (!(corners_uv.rows == 8 && corners_uv.cols == 2)) throw std::runtime_error("corners_uv must be (N,8,2) or (8,2)");
        N = 1;
        single = true;
    } else if (corners_uv.dims == 3) {
        if (!(corners_uv.size[1] == 8 && corners_uv.size[2] == 2)) throw std::runtime_error("corners_uv must have shape (N,8,2)");
        N = corners_uv.size[0];
    } else {
        throw std::runtime_error("corners_uv must be (N,8,2) or (8,2)");
    }

    cv::Mat objs;
    bool has_objs = false;
    if (objs_ptr && !objs_ptr->empty() && objs_ptr->total() != 0) {
        objs = *objs_ptr;
        if (!(objs.dims == 2 && objs.type() == CV_32F && objs.rows == N && objs.cols == 6)) {
            throw std::runtime_error("objs must be (N,6) CV_32F and match corners_uv N");
        }
        has_objs = true;
    }

    if (txts && (int)txts->size() != N) throw std::runtime_error("txts length must match N");

    cv::Mat out = img.clone();
    const int H = out.rows, W = out.cols;

    const std::vector<std::pair<int,int>> edges = {
        {0,1},{1,2},{2,3},{3,0}, // bottom
        {4,5},{5,6},{6,7},{7,4}, // top
        {0,4},{1,5},{2,6},{3,7}  // vertical
    };
    const std::vector<std::pair<int,int>> front_edges = {
        {1,6},{2,5}
    };

    std::vector<int> order(N);
    for (int i = 0; i < N; ++i) order[i] = i;
    if (has_objs) {
        std::sort(order.begin(), order.end(), [&](int a, int b) {
            return objs.at<float>(a, 4) < objs.at<float>(b, 4);
        });
    }

    auto get_uv = [&](int i, int k, int d) -> float {
        if (single) return corners_uv.at<float>(k, d);
        int idx[3] = {i, k, d};
        return corners_uv.at<float>(idx);
    };

    for (int ii = 0; ii < N; ++ii) {
        int i = order[ii];

        cv::Point p[8];
        for (int k = 0; k < 8; ++k) {
            float u = get_uv(i, k, 0);
            float v = get_uv(i, k, 1);
            int uu = std::clamp((int)std::lround(u), 0, W - 1);
            int vv = std::clamp((int)std::lround(v), 0, H - 1);
            p[k] = {uu, vv};
        }

        int cls_id = -1;
        float conf = 1.0f;
        cv::Scalar color;

        if (has_objs) {
            conf = objs.at<float>(i, 4);
            cls_id = (int)std::lround(objs.at<float>(i, 5));
            color = color_for_cls(cls_id);
        } else {
            color = cv::Scalar(0, 255, 0);
        }

        for (auto [a,b] : edges) cv::line(out, p[a], p[b], color, thickness, cv::LINE_AA);
        for (auto [a,b] : front_edges) cv::line(out, p[a], p[b], color, thickness, cv::LINE_AA);

        if (draw_label) {
            std::string txt;
            if (txts) {
                txt = (*txts)[i];
            } else if (has_objs) {
                std::string name = std::to_string(cls_id);
                if (id2name) {
                    auto it = id2name->find(cls_id);
                    if (it != id2name->end()) name = it->second;
                }
                char buf[64];
                std::snprintf(buf, sizeof(buf), "%.2f", conf);
                txt = name + " " + buf;
            } else {
                txt.clear();
            }

            if (!txt.empty()) {
                int x1i = (int)std::lround((p[4].x + p[5].x + p[6].x + p[7].x) / 4.0);
                int y1i = (int)std::lround((p[4].y + p[5].y + p[6].y + p[7].y) / 4.0);
                x1i = std::clamp(x1i, 0, W - 1);
                y1i = std::clamp(y1i, 0, H - 1);

                int base = 0;
                auto ts = cv::getTextSize(txt, cv::FONT_HERSHEY_SIMPLEX, 0.45, 1, &base);
                const int pad_x = 3, pad_y = 2;

                int x0 = std::clamp(x1i, 0, W - 1);
                int y0 = std::clamp(y1i - ts.height - 2 * pad_y, 0, H - 1);
                int x1b = std::clamp(x0 + ts.width + 2 * pad_x, 0, W - 1);
                int y1b = std::clamp(y0 + ts.height + 2 * pad_y, 0, H - 1);

                cv::rectangle(out, {x0, y0}, {x1b, y1b}, color, cv::FILLED);
                cv::Point org(x0 + pad_x, y0 + ts.height + pad_y - base / 2);
                cv::putText(out, txt, org, cv::FONT_HERSHEY_SIMPLEX, 0.45, {0,0,0}, 1, cv::LINE_8);
            }
        }
    }

    return out;
}
