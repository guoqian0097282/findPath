#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstring>
#include <exception>
#include <iostream>
#include <limits>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "../common/logger.hpp"


// =========================== Cyl3DBoxEstimator ===========================
// 说明：
// - 本文件实现与 Python 版 Cyl3DBoxEstimator 对齐的圆柱相机 3D 盒推算与绘制。
// - 关键修改点（相对你给的版本）：
//   1) 行人（cid==0）按“圆柱体底面圆”建模：mask 反推 z_base 时用“射线-圆求交”，而不是矩形边。
//      这样与“行人中心由两射线与圆相切求得”保持一致，并且 z_base 不再依赖 theta_abs。
//   2) whitelist 解析与注释对齐：支持 name->meta（通过 meta["id"] 取类 id），也支持 key 直接是数字字符串（兼容旧数据）。
class Cyl3DBoxEstimator {
public:
    // cyl_cam:
    //  - focal_u, center_u, center_v : number
    //  - R_act(3x3), t_act(3), R_cyl_to_cam(3x3) : [9] 或 [[3],[3],[3]]
    // whitelist: name -> { "id": int, "size": [l,w,h] }   或   "0"->{"size":[...]}（兼容）
    explicit Cyl3DBoxEstimator(const nlohmann::json::object_t& cyl_cam,
                               const nlohmann::json::object_t& whitelist) {
        f_ = get_number(cyl_cam, "focal_u");
        cu_ = get_number(cyl_cam, "center_u");
        cv_ = get_number(cyl_cam, "center_v");

        R_w2c_ = parse_mat33(get_obj(cyl_cam, "R_act"));
        R_c2w_ = R_w2c_.t();
        t_ = parse_vec3(get_obj(cyl_cam, "t_act"));

        // 世界系相机中心 C_w = -R^T * t
        C_w_ = -(R_c2w_ * t_);

        cv::Matx33d R_cyl_to_cam = parse_mat33(get_obj(cyl_cam, "R_cyl_to_cam"));
        tan_pitch_ = -R_cyl_to_cam(0, 2) / (R_cyl_to_cam(0, 0) + 1e-12);

        // 读取白名单：name -> {"id":int, "size":[l,w,h]}
        // 兼容：key 是数字字符串时，直接用 key 作为 cls_id
        for (const auto& kv : whitelist) {
            const std::string& k = kv.first;
            const auto& meta = kv.second;
            if (!meta.is_object()) continue;

            auto it_sz = meta.find("size");
            if (it_sz == meta.end()) continue;

            int cid = -1;

            // 1) 兼容旧格式：key 是 "0","1","2" 这种数字字符串
            //    （如果不是纯数字，转到 2)）
            if (!k.empty()) {
                int parsed = -1;
                const char* begin = k.data();
                const char* end = k.data() + k.size();
                auto [ptr, ec] = std::from_chars(begin, end, parsed);
                if (ec == std::errc{} && ptr == end) {
                    cid = parsed;
                }
            }

            // 2) 标准格式：name -> {"id":..., "size":[...]}
            if (cid < 0) {
                auto it_id = meta.find("id");
                if (it_id == meta.end()) continue;
                cid = json_to_int(*it_id);
            }

            auto sz = json_to_vec3(*it_sz);
            id2size_[cid] = cv::Vec3d(sz[0], sz[1], sz[2]);
        }
    }

    // 像素(u,v) → 世界系射线方向
    cv::Vec3d world_ray_dir(double u, double v) const {
        const double phi = (u - cu_) / f_;
        const double Y_rel = (cv_ - v) / f_;
        const double Y_abs = Y_rel - tan_pitch_;
        const double cphi = std::cos(phi);
        const double sphi = std::sin(phi);
        cv::Vec3d dir_cyl(cphi, -sphi, Y_abs);
        cv::Vec3d d_w = R_c2w_ * dir_cyl;
        return d_w;
    }

    // 与 Z = z_world 平面求交
    bool intersect_uv_with_z(double u, double v, double z_world,
                             cv::Vec3d& out_xyz) const {
        if (!std::isfinite(u) || !std::isfinite(v) || !std::isfinite(z_world)) {
            return false;
        }
        cv::Vec3d d_w = world_ray_dir(u, v);
        const double dz = d_w[2];
        if (std::abs(dz) < 1e-12) {
            return false;
        }
        const double lam = (z_world - C_w_[2]) / dz;
        cv::Vec3d Xw = C_w_ + lam * d_w;
        out_xyz = cv::Vec3d(Xw[0], Xw[1], z_world);
        return true;
    }

    // objs: Nx7 [x1,y1,x2,y2,conf,cls,theta_rel] 或至少前4列
    // 返回 Nx3(CV_64F) 世界坐标；失败为 NaN。
    cv::Mat points_from_boxes(const cv::Mat& objs,
                              double z_world,
                              cv::Mat* img_cyl = nullptr,
                              cv::Scalar draw_color = {0, 255, 0},
                              double text_scale = 0.45,
                              int text_thickness = 1) const {
        CV_Assert(objs.type() == CV_32F);
        CV_Assert(objs.cols >= 4);

        const int N = objs.rows;
        cv::Mat out(N, 3, CV_64F,
                    cv::Scalar(std::numeric_limits<double>::quiet_NaN()));

        int H = 0;
        int W = 0;
        if (img_cyl && !img_cyl->empty()) {
            H = img_cyl->rows;
            W = img_cyl->cols;
        }

        for (int i = 0; i < N; ++i) {
            const double x1 = static_cast<double>(objs.at<float>(i, 0));
            const double y1 = static_cast<double>(objs.at<float>(i, 1));
            const double x2 = static_cast<double>(objs.at<float>(i, 2));
            const double y2 = static_cast<double>(objs.at<float>(i, 3));
            const double u_mid = 0.5 * (x1 + x2);
            const double v_bot = y2;

            cv::Vec3d pt;
            const bool ok = intersect_uv_with_z(u_mid, v_bot, z_world, pt);

            if (img_cyl && W > 0 && H > 0) {
                const int ui = static_cast<int>(std::llround(u_mid));
                const int vi = static_cast<int>(std::llround(v_bot));
                if (0 <= ui && ui < W && 0 <= vi && vi < H) {
                    cv::circle(*img_cyl, {ui, vi}, 4, draw_color, -1, cv::LINE_AA);

                    std::string txt;
                    cv::Scalar txt_col(255, 255, 255);
                    cv::Scalar box_col(0, 0, 0);
                    if (ok) {
                        char buf[128];
                        std::snprintf(buf, sizeof(buf),
                                      "(%.3f,%.3f,%.3f)", pt[0], pt[1], pt[2]);
                        txt = buf;
                    } else {
                        txt = "INVALID";
                        txt_col = cv::Scalar(0, 255, 255);
                    }

                    int base = 0;
                    cv::Size ts = cv::getTextSize(
                        txt, cv::FONT_HERSHEY_SIMPLEX,
                        text_scale, text_thickness, &base
                    );
                    int x0 = std::max(0, ui + 6);
                    int y0 = std::max(0, vi - ts.height - 6);
                    int x1r = std::min(W - 1, x0 + ts.width + 6);
                    int y1r = std::min(H - 1, y0 + ts.height + base + 6);

                    cv::rectangle(*img_cyl, {x0, y0}, {x1r, y1r}, box_col, cv::FILLED);
                    cv::putText(
                        *img_cyl, txt, {x0 + 3, y1r - base - 3},
                        cv::FONT_HERSHEY_SIMPLEX,
                        text_scale, txt_col,
                        text_thickness, cv::LINE_AA
                    );
                }
            }

            if (ok) {
                out.at<double>(i, 0) = pt[0];
                out.at<double>(i, 1) = pt[1];
                out.at<double>(i, 2) = pt[2];
            }
        }
        return out;
    }

    // objs: (N,7) [x1,y1,x2,y2,conf,cls,theta_rel]
    // z_world: 标量，作为底面高度的先验 / fallback
    // masks: 可选，3D (N,H,W) CV_8U；允许 empty()
    //
    // 返回 cuboids: (N,9) CV_32F
    //   [cx, cy, cz, l, w, h, conf, cls, theta_abs]
    //
    // 对于几何/先验都无法解出的目标：
    //   cx,cy,cz = 0
    //   l,w,h   = -1
    // conf, cls, theta_abs 仍来自 objs（theta_abs 解不出时为 0）
    cv::Mat cuboids_from_boxes(const cv::Mat& objs,
                               double z_world,
                               const cv::Mat& masks) const {
        CV_Assert(objs.type() == CV_32F);
        CV_Assert(objs.cols == 7);

        const int N = objs.rows;

        // z 向量（当前实现标量 z_world 的情况，对应 Python 标量）
        std::vector<double> z_vec(N, z_world);

        // masks: (N,H,W) CV_8U；若 empty() 表示不使用 mask
        const bool has_masks = !masks.empty();
        int maskH = 0, maskW = 0;
        if (has_masks) {
            CV_Assert(masks.type() == CV_8U);
            CV_Assert(masks.dims == 3);
            CV_Assert(masks.size[0] == N);
            maskH = masks.size[1];
            maskW = masks.size[2];
            CV_Assert(maskH > 0 && maskW > 0);
        }

        // 提取 cls / conf / theta_rel
        std::vector<int> cls_ids(N);
        std::vector<double> confs(N);
        std::vector<double> theta_rel(N);
        for (int i = 0; i < N; ++i) {
            confs[i] = static_cast<double>(objs.at<float>(i, 4));
            cls_ids[i] = static_cast<int>(std::llround(objs.at<float>(i, 5)));
            theta_rel[i] = static_cast<double>(objs.at<float>(i, 6));
        }

        // 输出：预先全部填成 “失败格式”
        // cx,cy,cz = 0；l,w,h = -1；conf,cls,theta_abs = 0
        cv::Mat cuboids(N, 9, CV_32F, cv::Scalar(0));
        for (int i = 0; i < N; ++i) {
            cuboids.at<float>(i, 3) = -1.0f; // l
            cuboids.at<float>(i, 4) = -1.0f; // w
            cuboids.at<float>(i, 5) = -1.0f; // h
        }

        const double cam_cx = C_w_[0];
        const double cam_cy = C_w_[1];
        const double cam_cz = C_w_[2];

        // 先把 conf / cls 填进去，即使几何失败也有检测信息
        for (int i = 0; i < N; ++i) {
            cuboids.at<float>(i, 6) = static_cast<float>(confs[i]);
            cuboids.at<float>(i, 7) = static_cast<float>(cls_ids[i]);
        }

        for (int i = 0; i < N; ++i) {
            const int cid = cls_ids[i];
            auto it_sz = id2size_.find(cid);
            if (it_sz == id2size_.end()) {
                // 无先验尺寸，保持失败格式
                continue;
            }

            const double l = it_sz->second[0];
            const double w = it_sz->second[1];
            const double h = it_sz->second[2];
            const double z_plane_prior = z_vec[i];

            const double x1 = static_cast<double>(objs.at<float>(i, 0));
            const double y1 = static_cast<double>(objs.at<float>(i, 1));
            const double x2 = static_cast<double>(objs.at<float>(i, 2));
            const double y2 = static_cast<double>(objs.at<float>(i, 3));

            const double u_left = x1;
            const double u_right = x2;
            const double v_bot = y2;
            const double u_mid = 0.5 * (x1 + x2);

            // 左/右 下角对应射线
            cv::Vec3d dL = world_ray_dir(u_left, v_bot);
            cv::Vec3d dR = world_ray_dir(u_right, v_bot);
            const double dxL = dL[0], dyL = dL[1];
            const double dxR = dR[0], dyR = dR[1];

            if (!std::isfinite(dxL) || !std::isfinite(dyL) ||
                !std::isfinite(dxR) || !std::isfinite(dyR)) {
                continue;
            }
            if (std::hypot(dxL, dyL) < 1e-8 || std::hypot(dxR, dyR) < 1e-8) {
                continue;
            }

            const double alpha = std::atan2(dyL, dxL);
            const double beta = std::atan2(dyR, dxR);

            // 中点射线 -> 绝对朝向 theta_abs
            // 行人圆柱体的几何中心与 z_base 求交本应不依赖 theta，
            // 但为了与 Python 输出字段一致，这里仍然计算 theta_abs 并写入 cuboids[i,8]。
            double theta_abs = 0.0;
            cv::Vec3d d_mid = world_ray_dir(u_mid, v_bot);
            if (std::isfinite(d_mid[0]) && std::isfinite(d_mid[1]) &&
                std::hypot(d_mid[0], d_mid[1]) >= 1e-8) {
                const double az = std::atan2(d_mid[1], d_mid[0]);
                theta_abs = theta_rel[i] + az;
                theta_abs = wrap_to_pi(theta_abs);
            } else {
                theta_abs = 0.0;
            }

            // 先把 theta_abs 写入输出，即使后面几何失败也有值
            cuboids.at<float>(i, 8) = static_cast<float>(theta_abs);

            if (solver_ground0_dynamic_width_) 
            {
                auto intersect_ray_with_plane_z = [&](const cv::Vec3d& ray_dir,
                                                      double z_plane,
                                                      cv::Vec3d& out_xyz,
                                                      double eps = 1e-9) -> bool {
                    const double dx = ray_dir[0];
                    const double dy = ray_dir[1];
                    const double dz = ray_dir[2];
                    if (!std::isfinite(dx) || !std::isfinite(dy) || !std::isfinite(dz)) {
                        return false;
                    }
                    if (std::abs(dz) < eps) {
                        return false;
                    }

                    const double lam = (z_plane - cam_cz) / dz;
                    if (!std::isfinite(lam) || lam <= eps) {
                        return false;
                    }

                    const double px = cam_cx + lam * dx;
                    const double py = cam_cy + lam * dy;
                    if (!std::isfinite(px) || !std::isfinite(py)) {
                        return false;
                    }

                    out_xyz = cv::Vec3d(px, py, z_plane);
                    return true;
                };

                cv::Vec3d pC, pL, pR;
                if (!intersect_ray_with_plane_z(d_mid, 0.0, pC) ||
                    !intersect_ray_with_plane_z(dL, 0.0, pL) ||
                    !intersect_ray_with_plane_z(dR, 0.0, pR)) {
                    continue;
                }

                const double dyn_w = std::hypot(pR[0] - pL[0], pR[1] - pL[1]);
                if (!std::isfinite(dyn_w) || dyn_w <= 1e-4) {
                    continue;
                }

                const double cx = pC[0];
                const double cy = pC[1];
                const double z_base = 0.0;

                const double h_use = h;
                double l_use = l;
                const double w_use = dyn_w;
                if (cid == 0) {
                    l_use = w_use;
                }

                cuboids.at<float>(i, 0) = static_cast<float>(cx);
                cuboids.at<float>(i, 1) = static_cast<float>(cy);
                cuboids.at<float>(i, 2) = static_cast<float>(z_base + 0.5 * h_use);
                cuboids.at<float>(i, 3) = static_cast<float>(l_use);
                cuboids.at<float>(i, 4) = static_cast<float>(w_use);
                cuboids.at<float>(i, 5) = static_cast<float>(h_use);
                continue;
            }

            // 在相机地面投影坐标系下，通过两条射线夹角 + 先验尺寸求底面中心 (cx_rel, cy_rel)
            double cx_rel = 0.0;
            double cy_rel = 0.0;

            // 行人底面圆半径（仅 cid==0 有效），后面 mask 求 z_base 时会复用
            double r_person = std::numeric_limits<double>::quiet_NaN();

            if (cid == 0) {
                // Legacy branch: keep radius formula aligned with current Python implementation.
                r_person = person_radius_legacy_solver(l, w);
                auto cc = find_circle_center_tangent_to_rays(r_person, alpha, beta);
                if (!cc.has_value()) {
                    continue;
                }
                cx_rel = cc->first;
                cy_rel = cc->second;
            } else {
                // 其他类别：矩形底面求中心（依赖 theta_abs）
                std::vector<std::pair<double, double>> centers;
                try {
                    centers = find_rectangle_centers_between_rays(
                        l, w, theta_abs, alpha, beta, true
                    );
                } catch (const std::runtime_error&) {
                    continue;
                }
                if (centers.empty()) {
                    continue;
                }
                cx_rel = centers[0].first;
                cy_rel = centers[0].second;
            }

            const double cx = cam_cx + cx_rel;
            const double cy = cam_cy + cy_rel;

            // 使用 mask 估计底面 z_base（如果提供）
            // 修改点：
            //  - 行人（圆柱体）：射线与底面圆求交 -> z_base
            //  - 其他类别（矩形底面）：射线与底面矩形边求交 -> z_base
            double z_base = std::numeric_limits<double>::quiet_NaN();
            bool has_z_base = false;

            if (has_masks) {
                // 取第 i 张 mask：masks[i, :, :] -> 2D view（零拷贝）
                int idx3[3] = {i, 0, 0};
                const uchar* base = masks.ptr<uchar>(idx3);

                // 只读：const_cast 与原实现一致
                cv::Mat mask_i(maskH, maskW, CV_8U, const_cast<uchar*>(base), masks.step[1]);

                // 直接从底部向上扫，找到最底非零行并计算该行 x 均值。
                // 相比 findNonZero，可避免每个目标分配/遍历整点集。
                bool found_bottom = false;
                int v_max = -1;
                double u_mask = 0.0;
                for (int y = maskH - 1; y >= 0; --y) {
                    const uchar* row = mask_i.ptr<uchar>(y);
                    int cnt = 0;
                    double sum_x = 0.0;
                    for (int x = 0; x < maskW; ++x) {
                        if (row[x] != 0) {
                            sum_x += static_cast<double>(x);
                            cnt += 1;
                        }
                    }
                    if (cnt > 0) {
                        found_bottom = true;
                        v_max = y;
                        u_mask = sum_x / static_cast<double>(cnt);
                        break;
                    }
                }

                if (found_bottom) {
                    const double v_mask = static_cast<double>(v_max);

                    // 底点射线（世界系）
                    cv::Vec3d d_mask = world_ray_dir(u_mask, v_mask);

                    if (cid == 0) {
                        // ============ 行人：底面圆求交（与“圆相切求中心”一致）============
                        // 注意：这里不需要 bottom_corners_xy，也不应依赖 theta_abs
                        if (std::isfinite(r_person) && r_person > 0.0) {
                            auto inter = intersect_ray_with_bottom_circle_z(
                                cam_cx, cam_cy, cam_cz,
                                d_mask,
                                cv::Vec2d(cx, cy),
                                r_person
                            );
                            if (inter.first) {
                                has_z_base = true;
                                z_base = inter.second;
                            }
                        }
                    } else {
                        // ============ 其他：底面矩形边求交（保持原逻辑）============
                        // 底面四角在 XY 平面上的坐标
                        const double half_l = 0.5 * l;
                        const double half_w = 0.5 * w;

                        std::array<cv::Vec2d, 4> local_xy = {
                            cv::Vec2d(half_l, half_w),
                            cv::Vec2d(half_l, -half_w),
                            cv::Vec2d(-half_l, -half_w),
                            cv::Vec2d(-half_l, half_w)
                        };

                        const double cth = std::cos(theta_abs);
                        const double sth = std::sin(theta_abs);
                        cv::Matx22d Rz_xy(
                            cth, -sth,
                            sth,  cth
                        );

                        std::array<cv::Vec2d, 4> bottom_corners_xy;
                        for (int k = 0; k < 4; ++k) {
                            cv::Vec2d rxy = Rz_xy * local_xy[k];
                            bottom_corners_xy[k] = cv::Vec2d(
                                rxy[0] + cx,
                                rxy[1] + cy
                            );
                        }

                        auto inter = intersect_ray_with_bottom_edges_z(
                            cam_cx, cam_cy, cam_cz,
                            d_mask,
                            bottom_corners_xy
                        );
                        if (inter.first) {
                            has_z_base = true;
                            z_base = inter.second;
                        }
                    }
                }
            }

            // 若无法从 mask 得到底面 z_base，则 fallback 到 z_plane_prior
            if (!has_z_base) {
                z_base = z_plane_prior;
            }

            // 几何上的 z_base 必须为有限数
            if (!std::isfinite(z_base)) {
                continue;
            }

            const double cz = z_base + 0.5 * h;

            // 写回成功解出的 cuboid
            cuboids.at<float>(i, 0) = static_cast<float>(cx);
            cuboids.at<float>(i, 1) = static_cast<float>(cy);
            cuboids.at<float>(i, 2) = static_cast<float>(cz);
            cuboids.at<float>(i, 3) = static_cast<float>(l);
            cuboids.at<float>(i, 4) = static_cast<float>(w);
            cuboids.at<float>(i, 5) = static_cast<float>(h);
            // 6,7,8 前面已经填好
        }

        return cuboids;
    }

    // 单个 3D 盒在圆柱图上绘制
    // cuboid: [cx,cy,cz,l,w,h,conf,cls,theta_abs]
    void draw_3dbox_on_cyl_inplace(cv::Mat& img,
                                   const std::array<double, 9>& cuboid,
                                   int thickness = 1,
                                   const std::string* label = nullptr,
                                   bool show_info = false,
                                   int samples_per_edge = 8,
                                   const std::unordered_map<int, cv::Scalar>* class_colors = nullptr) const {
        CV_Assert(samples_per_edge >= 1);
        const int H = img.rows;
        const int W = img.cols;

        auto color_for_cls = [&](int cid)-> cv::Scalar {
            if (class_colors) {
                auto it = class_colors->find(cid);
                if (it != class_colors->end()) return it->second;
            }
            switch (cid) {
            case 0: return {0, 255, 0};       // person
            case 1: return {255, 0, 0};       // bicycle
            case 2: return {0, 165, 255};     // car
            case 3: return {255, 0, 255};     // motorcycle
            case 5: return {0, 255, 255};     // bus
            case 7: return {255, 255, 0};     // truck
            default: {
                auto mod256 = [](int x) { return (x % 256 + 256) % 256; };
                int r = mod256(37 * cid + 127);
                int g = mod256(17 * cid + 191);
                int b = mod256(29 * cid + 159);
                return cv::Scalar(b, g, r);
            }
            }
        };

        const double cx = cuboid[0];
        const double cy = cuboid[1];
        const double cz = cuboid[2];
        const double l  = cuboid[3];
        const double w  = cuboid[4];
        const double h  = cuboid[5];
        const double conf = cuboid[6];
        const int cls = static_cast<int>(std::llround(cuboid[7]));
        const double theta = cuboid[8];

        const cv::Scalar color = color_for_cls(cls);

        const double hl = 0.5 * l;
        const double hw = 0.5 * w;
        const double hh = 0.5 * h;

        std::array<cv::Vec3d, 8> corners_local = {
            cv::Vec3d(hl,  hw,  hh),
            cv::Vec3d(hl, -hw,  hh),
            cv::Vec3d(-hl,-hw,  hh),
            cv::Vec3d(-hl, hw,  hh),
            cv::Vec3d(hl,  hw, -hh),
            cv::Vec3d(hl, -hw, -hh),
            cv::Vec3d(-hl,-hw, -hh),
            cv::Vec3d(-hl, hw, -hh)
        };

        const double c = std::cos(theta);
        const double s = std::sin(theta);
        cv::Matx33d Rz(
            c,  -s, 0.0,
            s,   c, 0.0,
            0.0, 0.0, 1.0
        );
        const cv::Vec3d center_w(cx, cy, cz);

        std::array<cv::Vec3d, 8> corners_w;
        for (int i = 0; i < 8; ++i) {
            corners_w[i] = (Rz * corners_local[i]) + center_w;
        }

        struct Proj {
            double u_raw;
            double v;
            double phi_raw;
            bool ok;
        };

        // 世界点 -> 圆柱像素
        auto project_world_to_uv = [&](const cv::Vec3d& Pw)-> Proj {
            cv::Vec3d Pc = R_w2c_ * Pw + t_;
            const double x = Pc[0];
            const double y = Pc[1];
            const double z = Pc[2];
            const double kxy = std::hypot(x, y);
            if (kxy <= 1e-12) {
                return {0.0, 0.0, 0.0, false};
            }
            const double phi = std::atan2(-y, x); // [-pi, pi]
            const double Y_abs = z / kxy;
            const double Y_rel = Y_abs + tan_pitch_;
            const double u_raw = cu_ + f_ * phi;
            const double v = cv_ - f_ * Y_rel;
            if (!std::isfinite(u_raw) || !std::isfinite(v)) {
                return {0.0, 0.0, 0.0, false};
            }
            return {u_raw, v, phi, true};
        };

        auto unwrap = [](std::optional<double>& prev_phi_unw, double phi_now)-> double {
            if (!prev_phi_unw) {
                prev_phi_unw = phi_now;
                return *prev_phi_unw;
            }
            double d = phi_now - *prev_phi_unw;
            double out = phi_now;
            if (d > kPi)  out = phi_now - 2.0 * kPi;
            if (d < -kPi) out = phi_now + 2.0 * kPi;
            prev_phi_unw = out;
            return out;
        };

        auto draw_segment_world = [&](const cv::Vec3d& P0, const cv::Vec3d& P1) {
            std::vector<cv::Point> seg;
            seg.reserve(samples_per_edge + 1);
            std::optional<double> prev_phi_unw;
            bool prev_valid = false;

            for (int k = 0; k <= samples_per_edge; ++k) {
                const double t = static_cast<double>(k) / samples_per_edge;
                cv::Vec3d Pw = (1.0 - t) * P0 + t * P1;

                Proj pr = project_world_to_uv(Pw);
                if (!pr.ok) {
                    if (seg.size() >= 2) {
                        cv::polylines(img, seg, false, color, thickness, cv::LINE_AA);
                    }
                    seg.clear();
                    prev_phi_unw.reset();
                    prev_valid = false;
                    continue;
                }

                double phi_unw = unwrap(prev_phi_unw, pr.phi_raw);
                double u = cu_ + f_ * phi_unw;
                double v = pr.v;

                if (!(0.0 <= u && u < W && 0.0 <= v && v < H)) {
                    if (seg.size() >= 2) {
                        cv::polylines(img, seg, false, color, thickness, cv::LINE_AA);
                    }
                    seg.clear();
                    prev_valid = false;
                    continue;
                }

                cv::Point pt(
                    static_cast<int>(std::llround(u)),
                    static_cast<int>(std::llround(v))
                );

                if (!prev_valid && seg.size() >= 2) {
                    cv::polylines(img, seg, false, color, thickness, cv::LINE_AA);
                    seg.clear();
                }
                seg.push_back(pt);
                prev_valid = true;
            }

            if (seg.size() >= 2) {
                cv::polylines(img, seg, false, color, thickness, cv::LINE_AA);
            }
        };

        // 1) 12 条边
        const std::array<std::pair<int, int>, 12> edges = {
            {
                {0, 1}, {1, 2}, {2, 3}, {3, 0},
                {4, 5}, {5, 6}, {6, 7}, {7, 4},
                {0, 4}, {1, 5}, {2, 6}, {3, 7}
            }
        };
        for (auto e : edges) {
            draw_segment_world(corners_w[e.first], corners_w[e.second]);
        }

        // 2) 朝向线：底面中心 → 最“朝前”的底边中心（沿 box 自身 x 轴）
        const int bi[4] = {4, 5, 6, 7};
        cv::Vec3d bottom_center(0, 0, 0);
        for (int k = 0; k < 4; ++k) {
            bottom_center += corners_w[bi[k]];
        }
        bottom_center *= 0.25;

        cv::Vec3d forward_w = Rz * cv::Vec3d(1.0, 0.0, 0.0);
        const std::array<std::pair<int, int>, 4> bedges = {
            {
                {4, 5}, {5, 6}, {6, 7}, {7, 4}
            }
        };

        double best_score = -1e18;
        cv::Vec3d best_center(0, 0, 0);
        for (auto e : bedges) {
            cv::Vec3d cedge = 0.5 * (corners_w[e.first] + corners_w[e.second]);
            double sc = (cedge - bottom_center).dot(forward_w);
            if (sc > best_score) {
                best_score = sc;
                best_center = cedge;
            }
        }
        draw_segment_world(bottom_center, best_center);

        // 3) 标签：在底面中心绘制 xyz/cls/conf/自定义 label
        if (show_info || label) {
            Proj prb = project_world_to_uv(bottom_center);
            if (prb.ok) {
                std::optional<double> prev_phi_unw;
                double phi_unw = unwrap(prev_phi_unw, prb.phi_raw);
                double u = cu_ + f_ * phi_unw;
                double v = prb.v;

                if (0.0 <= u && u < W && 0.0 <= v && v < H) {
                    cv::Point pb(
                        static_cast<int>(std::llround(u)),
                        static_cast<int>(std::llround(v))
                    );
                    cv::circle(img, pb, 3, color, -1, cv::LINE_AA);

                    std::vector<std::string> lines;
                    if (show_info) {
                        char b1[96], b2[64];
                        std::snprintf(b1, sizeof(b1),
                                      "x=%.2f, y=%.2f, z=%.2f", cx, cy, cz);
                        std::snprintf(b2, sizeof(b2),
                                      "cls=%d, conf=%.2f", cls, conf);
                        lines.emplace_back(b1);
                        lines.emplace_back(b2);
                    }
                    if (label) {
                        std::string::size_type start = 0;
                        while (start <= label->size()) {
                            const std::string::size_type pos = label->find('\n', start);
                            if (pos == std::string::npos) {
                                lines.emplace_back(label->substr(start));
                                break;
                            }
                            lines.emplace_back(label->substr(start, pos - start));
                            start = pos + 1;
                        }
                    }

                    if (!lines.empty()) {
                        int base = 0;
                        int tw = 0;
                        int th_sum = 0;
                        std::vector<cv::Size> szs;
                        szs.reserve(lines.size());
                        for (const auto& sline : lines) {
                            int b = 0;
                            cv::Size z_ = cv::getTextSize(
                                sline, cv::FONT_HERSHEY_SIMPLEX, 0.45, 1, &b
                            );
                            base = b;
                            tw = std::max(tw, z_.width);
                            th_sum += z_.height;
                            szs.push_back(z_);
                        }

                        const int pad = 6;
                        int x0 = std::max(0, pb.x + 6);
                        int y0 = std::max(0, pb.y - th_sum - pad);
                        int x1 = std::min(W - 1, x0 + tw + 2 * pad);
                        int y1 = std::min(H - 1, y0 + th_sum + 2 * pad);

                        cv::rectangle(img, {x0, y0}, {x1, y1},
                                      cv::Scalar(0, 0, 0), cv::FILLED);

                        int ycur = y0 + pad + szs[0].height;
                        for (size_t ii = 0; ii < lines.size(); ++ii) {
                            cv::putText(
                                img,
                                lines[ii],
                                {x0 + pad, ycur},
                                cv::FONT_HERSHEY_SIMPLEX,
                                0.45,
                                cv::Scalar(255, 255, 255),
                                1,
                                cv::LINE_AA
                            );
                            ycur += szs[ii].height;
                        }
                    }
                }
            }
        }

        return;
    }

    cv::Mat draw_3dbox_on_cyl(const cv::Mat& cyl_img,
                              const std::array<double, 9>& cuboid,
                              int thickness = 1,
                              const std::string* label = nullptr,
                              bool show_info = false,
                              int samples_per_edge = 8,
                              const std::unordered_map<int, cv::Scalar>* class_colors = nullptr) const {
        cv::Mat img = cyl_img.clone();
        draw_3dbox_on_cyl_inplace(
            img, cuboid, thickness, label, show_info, samples_per_edge, class_colors
        );
        return img;
    }

    // 多个 3D 盒
    // cuboids: Nx9([cx,cy,cz,l,w,h,conf,cls,theta_abs]), type = CV_32F
    cv::Mat draw_3dboxes_on_cyl(const cv::Mat& cyl_img,
                                const cv::Mat& cuboids,
                                int thickness = 1,
                                const std::vector<std::string>* labels = nullptr,
                                int samples_per_edge = 8,
                                const std::unordered_map<int, cv::Scalar>* class_colors = nullptr,
                                bool show_info = false) const {
        if (cuboids.empty()) {
            return cyl_img.clone();
        }

        CV_Assert(cuboids.type() == CV_32F);
        CV_Assert(cuboids.cols == 9);

        cv::Mat img = cyl_img.clone();
        const int N = cuboids.rows;

        for (int i = 0; i < N; ++i) {
            const float* row = cuboids.ptr<float>(i);
            std::array<double, 9> box{};
            for (int j = 0; j < 9; ++j) {
                box[j] = static_cast<double>(row[j]);
            }
            const std::string* lab = nullptr;
            if (labels && i < static_cast<int>(labels->size())) {
                lab = &labels->at(i);
            }
            draw_3dbox_on_cyl_inplace(
                img, box, thickness, lab, show_info, samples_per_edge, class_colors
            );
        }
        return img;
    }

private:
    // --------------------- 常量：避免依赖 M_PI ---------------------
    static constexpr double kPi = 3.1415926535897932384626433832795;
    static constexpr double kTwoPi = 2.0 * kPi;

    // --------------------- JSON / 基础解析 ---------------------
    static const nlohmann::json& get_obj(const nlohmann::json::object_t& j,
                                         const char* key) {
        auto it = j.find(key);
        if (it == j.end()) {
            throw std::invalid_argument(std::string("missing key: ") + key);
        }
        return it->second;
    }

    static double get_number(const nlohmann::json::object_t& j,
                             const char* key) {
        const auto& v = get_obj(j, key);
        if (!v.is_number()) {
            throw std::invalid_argument(std::string("key not a number: ") + key);
        }
        return v.get<double>();
    }

    static int json_to_int(const nlohmann::json& v) {
        if (v.is_boolean()) return v.get<bool>() ? 1 : 0;
        if (v.is_number_integer()) return v.get<int>();
        if (v.is_number()) return static_cast<int>(std::llround(v.get<double>()));
        throw std::invalid_argument("id must be number/bool");
    }

    static std::array<double, 3> json_to_vec3(const nlohmann::json& v) {
        if (!v.is_array() || v.size() != 3) {
            throw std::invalid_argument("vec3 must be array of 3 numbers");
        }
        return {v[0].get<double>(), v[1].get<double>(), v[2].get<double>()};
    }

    static cv::Matx33d parse_mat33(const nlohmann::json& v) {
        if (v.is_array() && v.size() == 3 && v[0].is_array()) {
            return cv::Matx33d(
                v[0][0].get<double>(), v[0][1].get<double>(), v[0][2].get<double>(),
                v[1][0].get<double>(), v[1][1].get<double>(), v[1][2].get<double>(),
                v[2][0].get<double>(), v[2][1].get<double>(), v[2][2].get<double>()
            );
        }
        if (v.is_array() && v.size() == 9) {
            return cv::Matx33d(
                v[0].get<double>(), v[1].get<double>(), v[2].get<double>(),
                v[3].get<double>(), v[4].get<double>(), v[5].get<double>(),
                v[6].get<double>(), v[7].get<double>(), v[8].get<double>()
            );
        }
        throw std::invalid_argument("R matrix must be 3x3");
    }

    static cv::Vec3d parse_vec3(const nlohmann::json& v) {
        if (!v.is_array() || v.size() != 3) {
            throw std::invalid_argument("t must be length-3 array");
        }
        return cv::Vec3d(
            v[0].get<double>(),
            v[1].get<double>(),
            v[2].get<double>()
        );
    }

    // --------------------- 角度/扇形/矩形几何工具 ---------------------
    static double normalize_angle(double angle) {
        angle = std::fmod(angle, kTwoPi);
        if (angle < 0.0) angle += kTwoPi;
        return angle;
    }

    // wrap 到 [-pi, pi)
    static double wrap_to_pi(double angle) {
        angle = std::fmod(angle + kPi, kTwoPi);
        if (angle < 0.0) angle += kTwoPi;
        return angle - kPi;
    }

    static std::pair<double, double> canonicalize_rays(double alpha,
                                                       double beta,
                                                       double eps = 1e-9) {
        (void)eps;
        double a = normalize_angle(alpha);
        double b = normalize_angle(beta);

        double diff = std::fmod(b - a + kTwoPi, kTwoPi);

        if (diff < 1e-9 || std::fabs(diff - kPi) < 1e-9) {
            throw std::runtime_error(
                "Rays are (almost) parallel or opposite; acute wedge not well-defined."
            );
        }
        if (diff > kPi) {
            std::swap(a, b);
        }
        return {a, b};
    }

    static double cross2d(double ax, double ay, double bx, double by) {
        return ax * by - ay * bx;
    }

    // ===== 对齐 Python：行人特例（外接圆与两射线相切）=====
    static std::optional<std::pair<double, double>>
    find_circle_center_tangent_to_rays(double radius,
                                       double alpha,
                                       double beta,
                                       double eps = 1e-9) {
        if (!(radius > 0.0) || !std::isfinite(radius)) {
            return std::nullopt;
        }

        double a = 0.0, b = 0.0;
        try {
            auto ab = canonicalize_rays(alpha, beta, eps);
            a = ab.first;
            b = ab.second;
        } catch (const std::runtime_error&) {
            return std::nullopt;
        }

        const double delta = std::fmod(b - a + kTwoPi, kTwoPi); // (0, pi)
        if (!(delta > eps) || !(delta < kPi - eps)) {
            return std::nullopt;
        }

        const double s = std::sin(0.5 * delta);
        if (std::fabs(s) < eps) {
            return std::nullopt;
        }

        // 圆心位于角平分线上，距离 d = r / sin(delta/2)
        const double d = radius / s;
        const double theta_b = a + 0.5 * delta;

        const double cx = d * std::cos(theta_b);
        const double cy = d * std::sin(theta_b);
        return std::make_optional(std::make_pair(cx, cy));
    }

    // 在两条射线之间，已知矩形 (length,width,theta)，解所有可能的矩形中心 (cx,cy)
    static std::vector<std::pair<double, double>>
    find_rectangle_centers_between_rays(double length,
                                        double width,
                                        double theta,
                                        double alpha,
                                        double beta,
                                        bool check_inside_wedge = true,
                                        double eps = 1e-9) {
        auto ab = canonicalize_rays(alpha, beta, eps);
        double alpha_c = ab.first;
        double beta_c = ab.second;

        const double u1x = std::cos(alpha_c);
        const double u1y = std::sin(alpha_c);
        const double u2x = std::cos(beta_c);
        const double u2y = std::sin(beta_c);

        const double half_l = 0.5 * length;
        const double half_w = 0.5 * width;

        std::array<std::pair<double, double>, 4> local_vertices = {
            {
                {half_l, half_w},
                {-half_l, half_w},
                {-half_l, -half_w},
                {half_l, -half_w}
            }
        };

        const double c = std::cos(theta);
        const double s = std::sin(theta);

        auto rotate = [&](double vx, double vy)-> std::pair<double, double> {
            return {c * vx - s * vy, s * vx + c * vy};
        };

        std::vector<std::pair<double, double>> centers_raw;

        for (int i1 = 0; i1 < 4; ++i1) {
            for (int i2 = 0; i2 < 4; ++i2) {
                if (i1 == i2) continue;

                auto v1 = local_vertices[i1];
                auto v2 = local_vertices[i2];

                double v1x = v1.first, v1y = v1.second;
                double v2x = v2.first, v2y = v2.second;

                double dvx = v1x - v2x;
                double dvy = v1y - v2y;

                auto k = rotate(dvx, dvy);
                double kx = k.first, ky = k.second;

                // 解 t1, t2：t1 * u1 - t2 * u2 = R(v1 - v2)
                double det = u1x * (-u2y) - u1y * (-u2x);
                if (std::fabs(det) < eps) continue;

                double inv11 = -u2y / det;
                double inv12 = u2x / det;
                double inv21 = -u1y / det;
                double inv22 = u1x / det;

                double t1 = inv11 * kx + inv12 * ky;
                double t2 = inv21 * kx + inv22 * ky;

                if (t1 < -eps || t2 < -eps) continue;

                auto rv1 = rotate(v1x, v1y);
                double rv1x = rv1.first, rv1y = rv1.second;

                double cx = t1 * u1x - rv1x;
                double cy = t1 * u1y - rv1y;

                if (!check_inside_wedge) {
                    centers_raw.emplace_back(cx, cy);
                    continue;
                }

                std::array<std::pair<double, double>, 4> world_vertices;
                for (int vi = 0; vi < 4; ++vi) {
                    auto vv = local_vertices[vi];
                    auto rr = rotate(vv.first, vv.second);
                    world_vertices[vi] = {cx + rr.first, cy + rr.second};
                }

                auto p1 = world_vertices[i1];
                auto p2 = world_vertices[i2];

                // 顶点在射线上：u × p ≈ 0
                if (std::fabs(cross2d(u1x, u1y, p1.first, p1.second)) > 1e-6) continue;
                if (std::fabs(cross2d(u2x, u2y, p2.first, p2.second)) > 1e-6) continue;

                // 落在射线正向：点与射线方向点积 >= 0
                if (p1.first * u1x + p1.second * u1y < -eps) continue;
                if (p2.first * u2x + p2.second * u2y < -eps) continue;

                // 其他顶点必须在楔形内部（含边界）
                bool valid = true;
                for (int vi = 0; vi < 4 && valid; ++vi) {
                    if (vi == i1 || vi == i2) continue;
                    auto pv = world_vertices[vi];
                    double c1 = cross2d(u1x, u1y, pv.first, pv.second); // >=0 在 ray1 左侧
                    double c2 = cross2d(u2x, u2y, pv.first, pv.second); // <=0 在 ray2 右侧
                    if (c1 < -1e-6 || c2 > 1e-6) valid = false;
                }

                if (!valid) continue;
                centers_raw.emplace_back(cx, cy);
            }
        }

        // 去重
        std::vector<std::pair<double, double>> centers;
        for (auto& cxy : centers_raw) {
            bool dup = false;
            for (auto& u : centers) {
                if (std::hypot(cxy.first - u.first, cxy.second - u.second) < 1e-6) {
                    dup = true;
                    break;
                }
            }
            if (!dup) centers.push_back(cxy);
        }
        return centers;
    }

    // --------------------- mask 反推 z_base：矩形 / 圆 ---------------------

    // 射线与底面矩形四条边求交，用于从 mask 底点反推 z_base
    // 返回 (has_intersection, z_base)。若 has_intersection=false，则无解。
    static std::pair<bool, double>
    intersect_ray_with_bottom_edges_z(double cam_cx,
                                      double cam_cy,
                                      double cam_cz,
                                      const cv::Vec3d& ray_dir,
                                      const std::array<cv::Vec2d, 4>& bottom_corners_xy,
                                      double eps = 1e-9) {
        const double dx = ray_dir[0];
        const double dy = ray_dir[1];
        const double dz = ray_dir[2];

        const double Cx = cam_cx;
        const double Cy = cam_cy;
        const double Cz = cam_cz;

        // 四条底边：0-1,1-2,2-3,3-0
        const std::array<std::pair<int, int>, 4> edges = {
            {
                {0, 1}, {1, 2}, {2, 3}, {3, 0}
            }
        };

        double best_lambda = std::numeric_limits<double>::infinity();
        double best_z_base = std::numeric_limits<double>::quiet_NaN();
        bool found = false;

        for (auto e : edges) {
            const int i0 = e.first;
            const int i1 = e.second;
            const double x0 = bottom_corners_xy[i0][0];
            const double y0 = bottom_corners_xy[i0][1];
            const double x1 = bottom_corners_xy[i1][0];
            const double y1 = bottom_corners_xy[i1][1];

            const double ex = x1 - x0;
            const double ey = y1 - y0;

            // 解：
            // C + λ d = P0 + μ e
            // -> λ d_xy - μ e_xy = (P0 - C)_xy
            //
            // [ dx  -ex ] [λ] = [ x0 - Cx ]
            // [ dy  -ey ] [μ]   [ y0 - Cy ]

            const double det = dx * (-ey) - dy * (-ex); // = -(dx*ey - dy*ex)
            if (std::fabs(det) < eps) {
                // 射线与边几乎平行
                continue;
            }

            const double inv11 = -ey / det;
            const double inv12 = ex / det;
            const double inv21 = -dy / det;
            const double inv22 = dx / det;

            const double bx = x0 - Cx;
            const double by = y0 - Cy;

            const double lam = inv11 * bx + inv12 * by;
            const double mu  = inv21 * bx + inv22 * by;

            if (lam <= eps) {
                // 在相机后方或太近
                continue;
            }
            if (mu < -eps || mu > 1.0 + eps) {
                // 不在线段内部
                continue;
            }

            const double z_base = Cz + lam * dz;

            if (!found || lam < best_lambda) {
                found = true;
                best_lambda = lam;
                best_z_base = z_base;
            }
        }

        return {found, best_z_base};
    }

    // 射线与底面圆（XY 平面）求交，用于行人圆柱体：从 mask 底点反推 z_base
    // 返回 (has_intersection, z_base)。若 has_intersection=false，则无解。
    static std::pair<bool, double>
    intersect_ray_with_bottom_circle_z(double cam_cx,
                                       double cam_cy,
                                       double cam_cz,
                                       const cv::Vec3d& ray_dir,
                                       const cv::Vec2d& center_xy,
                                       double radius,
                                       double eps = 1e-9) {
        if (!(radius > 0.0) || !std::isfinite(radius)) {
            return {false, std::numeric_limits<double>::quiet_NaN()};
        }

        const double dx = ray_dir[0];
        const double dy = ray_dir[1];
        const double dz = ray_dir[2];

        // 只在 XY 平面做求交：P_xy(λ) = C_xy + λ d_xy
        const double A = dx * dx + dy * dy;
        if (A < eps) {
            // 射线在 XY 几乎没有分量（近似竖直），无法稳定求交
            return {false, std::numeric_limits<double>::quiet_NaN()};
        }

        const double ox = center_xy[0];
        const double oy = center_xy[1];

        // q = C_xy - O_xy
        const double qx = cam_cx - ox;
        const double qy = cam_cy - oy;

        // 二次方程：A λ^2 + B λ + C = 0
        const double B = 2.0 * (dx * qx + dy * qy);
        const double C = (qx * qx + qy * qy) - radius * radius;

        const double disc = B * B - 4.0 * A * C;
        if (disc < 0.0) {
            return {false, std::numeric_limits<double>::quiet_NaN()};
        }

        const double sdisc = std::sqrt(disc);
        const double lam1 = (-B - sdisc) / (2.0 * A);
        const double lam2 = (-B + sdisc) / (2.0 * A);

        // 取“最小的正根”：沿射线前方第一次击中圆周
        double best = std::numeric_limits<double>::infinity();
        bool found = false;

        if (lam1 > eps) { best = lam1; found = true; }
        if (lam2 > eps && lam2 < best) { best = lam2; found = true; }

        if (!found) {
            return {false, std::numeric_limits<double>::quiet_NaN()};
        }

        const double z_base = cam_cz + best * dz;
        return {true, z_base};
    }

private:
    // 相机/姿态（全部 double）
    double f_{};
    double cu_{};
    double cv_{};
    double tan_pitch_{};
    cv::Matx33d R_w2c_{};
    cv::Matx33d R_c2w_{};
    cv::Vec3d t_{};
    cv::Vec3d C_w_{};

    // 类别尺寸 (id -> (l,w,h))
    static double person_radius_legacy_solver(double length, double width) {
        return 0.5 * std::hypot(length, width);
    }

    bool solver_ground0_dynamic_width_{true};
    std::unordered_map<int, cv::Vec3d> id2size_;
};
