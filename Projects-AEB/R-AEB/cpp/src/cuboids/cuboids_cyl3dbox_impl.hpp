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

    // objs: (N,7) [x1,y1,x2,y2,conf,cls,theta_rel]
    // points3d: (N,16) [p0.x,p0.y,...,p7.x,p7.y], CV_32F/CV_64F,
    //           or (N,8) with 2 channels (CV_32FC2/CV_64FC2).
    // p0..p3 are bottom corners (front-left, front-right, rear-right,
    // rear-left); p4..p7 are their corresponding top corners.
    // Returns the same (N,9) cuboid format as cuboids_from_boxes.
    cv::Mat cuboids_from_boxesAnd3D(const cv::Mat& objs,
                                    const cv::Mat& points3d,
                                    double z_world,
                                    const cv::Mat& masks) const {
        CV_Assert(objs.empty() || (objs.type() == CV_32F && objs.cols == 7));
        CV_Assert(points3d.empty() ||
                  (points3d.channels() == 1 &&
                   (points3d.cols == 18) &&
                   (points3d.type() == CV_32F || points3d.type() == CV_64F)));

        const int n2d = objs.rows;
        const int n3d = points3d.rows;
        if (n2d == 0 && n3d == 0) {
            return cv::Mat(0, 9, CV_32F);
        }
        if (n3d == 0) {
            return cuboids_from_boxes(objs, z_world, masks);
        }
        if (n2d == 0) {
            cv::Mat synthetic_objs(n3d, 7, CV_32F, cv::Scalar(0));
            cv::Mat point16(n3d, 16, CV_32F);
            for (int i = 0; i < n3d; ++i) {
                double min_u = std::numeric_limits<double>::infinity();
                double max_u = -std::numeric_limits<double>::infinity();
                double min_v = std::numeric_limits<double>::infinity();
                double max_v = -std::numeric_limits<double>::infinity();
                for (int k = 0; k < 16; k += 2) {
                    const double u = points3d.type() == CV_32F
                        ? points3d.at<float>(i, k)
                        : points3d.at<double>(i, k);
                    const double v = points3d.type() == CV_32F
                        ? points3d.at<float>(i, k + 1)
                        : points3d.at<double>(i, k + 1);
                    point16.at<float>(i, k) = static_cast<float>(u);
                    point16.at<float>(i, k + 1) = static_cast<float>(v);
                    min_u = std::min(min_u, u);
                    max_u = std::max(max_u, u);
                    min_v = std::min(min_v, v);
                    max_v = std::max(max_v, v);
                }
                synthetic_objs.at<float>(i, 0) = static_cast<float>(min_u);
                synthetic_objs.at<float>(i, 1) = static_cast<float>(min_v);
                synthetic_objs.at<float>(i, 2) = static_cast<float>(max_u);
                synthetic_objs.at<float>(i, 3) = static_cast<float>(max_v);
                synthetic_objs.at<float>(i, 4) = points3d.type() == CV_32F
                    ? points3d.at<float>(i, 16)
                    : static_cast<float>(points3d.at<double>(i, 16));
                synthetic_objs.at<float>(i, 5) = points3d.type() == CV_32F
                    ? points3d.at<float>(i, 17)
                    : static_cast<float>(points3d.at<double>(i, 17));
            }
            return cuboids_from_boxesAnd3DAligned(
                synthetic_objs, point16, z_world, cv::Mat());
        }

        cv::Mat point16(n3d, 16, CV_32F);
        std::vector<cv::Rect2d> boxes3d(n3d);
        std::vector<int> classes3d(n3d);
        std::vector<double> confs3d(n3d);
        for (int i = 0; i < n3d; ++i) {
            double min_u = std::numeric_limits<double>::infinity();
            double max_u = -std::numeric_limits<double>::infinity();
            double min_v = std::numeric_limits<double>::infinity();
            double max_v = -std::numeric_limits<double>::infinity();
            for (int k = 0; k < 16; k += 2) {
                const double u = points3d.type() == CV_32F
                    ? points3d.at<float>(i, k)
                    : points3d.at<double>(i, k);
                const double v = points3d.type() == CV_32F
                    ? points3d.at<float>(i, k + 1)
                    : points3d.at<double>(i, k + 1);
                point16.at<float>(i, k) = static_cast<float>(u);
                point16.at<float>(i, k + 1) = static_cast<float>(v);
                min_u = std::min(min_u, u);
                max_u = std::max(max_u, u);
                min_v = std::min(min_v, v);
                max_v = std::max(max_v, v);
            }
            boxes3d[i] = cv::Rect2d(min_u, min_v, max_u - min_u, max_v - min_v);
            confs3d[i] = points3d.type() == CV_32F
                ? points3d.at<float>(i, 16)
                : points3d.at<double>(i, 16);
            classes3d[i] = static_cast<int>(std::llround(
                points3d.type() == CV_32F
                    ? points3d.at<float>(i, 17)
                    : points3d.at<double>(i, 17)));
        }

        auto iou = [](const cv::Rect2d& a, const cv::Rect2d& b) {
            const cv::Rect2d inter = a & b;
            const double inter_area = inter.area();
            const double union_area = a.area() + b.area() - inter_area;
            return union_area > 1e-9 ? inter_area / union_area : 0.0;
        };

        std::vector<int> match2d(n2d, -1);
        std::vector<bool> used3d(n3d, false);
        for (int i = 0; i < n2d; ++i) {
            const cv::Rect2d box2d(
                objs.at<float>(i, 0), objs.at<float>(i, 1),
                objs.at<float>(i, 2) - objs.at<float>(i, 0),
                objs.at<float>(i, 3) - objs.at<float>(i, 1));
            const int cls2d = static_cast<int>(
                std::llround(objs.at<float>(i, 5)));
            double best_iou = 0.70;
            int best_j = -1;
            for (int j = 0; j < n3d; ++j) {
                if (used3d[j] || classes3d[j] != cls2d) continue;
                const double overlap = iou(box2d, boxes3d[j]);
                if (overlap >= best_iou) {
                    best_iou = overlap;
                    best_j = j;
                }
            }
            if (best_j >= 0) {
                match2d[i] = best_j;
                used3d[best_j] = true;
            }
        }

        auto single_mask = [&](int row) {
            if (masks.empty()) return cv::Mat();
            CV_Assert(masks.dims == 3 && masks.type() == CV_8U);
            int sizes[3] = {1, masks.size[1], masks.size[2]};
            cv::Mat result(3, sizes, CV_8U);
            cv::Mat src(masks.size[1], masks.size[2], CV_8U,
                        const_cast<uchar*>(masks.ptr<uchar>(row)),
                        masks.step[1]);
            cv::Mat dst(masks.size[1], masks.size[2], CV_8U,
                        result.ptr<uchar>(0), result.step[1]);
            src.copyTo(dst);
            return result;
        };

        std::vector<cv::Mat> result_rows;
        for (int i = 0; i < n2d; ++i) {
            cv::Mat obj_row = objs.row(i).clone();
            cv::Mat result;
            if (match2d[i] >= 0) {
                result = cuboids_from_boxesAnd3DAligned(
                    obj_row, point16.row(match2d[i]).clone(), z_world,
                    single_mask(i));
            } else {
                result = cuboids_from_boxes(
                    obj_row, z_world, single_mask(i));
            }
            result_rows.push_back(result);
        }
        for (int j = 0; j < n3d; ++j) {
            if (used3d[j]) continue;
            cv::Mat synthetic_obj(1, 7, CV_32F, cv::Scalar(0));
            synthetic_obj.at<float>(0, 0) = static_cast<float>(boxes3d[j].x);
            synthetic_obj.at<float>(0, 1) = static_cast<float>(boxes3d[j].y);
            synthetic_obj.at<float>(0, 2) = static_cast<float>(
                boxes3d[j].x + boxes3d[j].width);
            synthetic_obj.at<float>(0, 3) = static_cast<float>(
                boxes3d[j].y + boxes3d[j].height);
            synthetic_obj.at<float>(0, 4) = static_cast<float>(confs3d[j]);
            synthetic_obj.at<float>(0, 5) = static_cast<float>(classes3d[j]);
            result_rows.push_back(cuboids_from_boxesAnd3DAligned(
                synthetic_obj, point16.row(j).clone(), z_world, cv::Mat()));
        }

        cv::Mat output(static_cast<int>(result_rows.size()), 9, CV_32F);
        for (int i = 0; i < static_cast<int>(result_rows.size()); ++i) {
            result_rows[i].row(0).copyTo(output.row(i));
        }
        return output;
    }

    // points3d: (M,18) [p0.x,p0.y,...,p7.x,p7.y,conf,cls].
    // p0..p3 are bottom corners and p4..p7 are their corresponding top
    // corners. This method does not require a 2D detection input.
    cv::Mat cuboids_from_3D(const cv::Mat& points3d,
                            double z_world) const {
        CV_Assert(points3d.empty() ||
                  (points3d.channels() == 1 &&
                   points3d.cols == 18 &&
                   (points3d.type() == CV_32F ||
                    points3d.type() == CV_64F)));
        const int N = points3d.rows;
        cv::Mat cuboids(N, 9, CV_32F, cv::Scalar(0));
        for (int i = 0; i < N; ++i) {
            cuboids.at<float>(i, 3) = -1.0f;
            cuboids.at<float>(i, 4) = -1.0f;
            cuboids.at<float>(i, 5) = -1.0f;
            cuboids.at<float>(i, 6) = points3d.type() == CV_32F
                ? points3d.at<float>(i, 16)
                : static_cast<float>(points3d.at<double>(i, 16));
            cuboids.at<float>(i, 7) = points3d.type() == CV_32F
                ? points3d.at<float>(i, 17)
                : static_cast<float>(points3d.at<double>(i, 17));

            const int cid = static_cast<int>(std::llround(
                cuboids.at<float>(i, 7)));
            const auto size_it = id2size_.find(cid);
            const double prior_h = size_it == id2size_.end()
                ? std::numeric_limits<double>::quiet_NaN()
                : size_it->second[2];

            auto read_uv = [&](int point_index) {
                const int col = 2 * point_index;
                if (points3d.type() == CV_32F) {
                    return cv::Vec2d(points3d.at<float>(i, col),
                                     points3d.at<float>(i, col + 1));
                }
                return cv::Vec2d(points3d.at<double>(i, col),
                                 points3d.at<double>(i, col + 1));
            };

            // 第二路观测：由有效 3D 投影点生成伪 2D 框，复用原始 solver。
            cv::Mat pseudo_obj(1, 7, CV_32F, cv::Scalar(0));
            pseudo_obj.at<float>(0, 4) = cuboids.at<float>(i, 6);
            pseudo_obj.at<float>(0, 5) = cuboids.at<float>(i, 7);
            int valid_point_count = 0;
            double min_u = std::numeric_limits<double>::infinity();
            double max_u = -std::numeric_limits<double>::infinity();
            double min_v = std::numeric_limits<double>::infinity();
            double max_v = -std::numeric_limits<double>::infinity();
            for (int k = 0; k < 8; ++k) {
                const cv::Vec2d uv = read_uv(k);
                if (!std::isfinite(uv[0]) || !std::isfinite(uv[1])) continue;
                ++valid_point_count;
                min_u = std::min(min_u, uv[0]);
                max_u = std::max(max_u, uv[0]);
                min_v = std::min(min_v, uv[1]);
                max_v = std::max(max_v, uv[1]);
            }
            cv::Mat pseudo_cuboid;
            if (valid_point_count >= 4 && max_u > min_u && max_v > min_v) {
                pseudo_obj.at<float>(0, 0) = static_cast<float>(min_u);
                pseudo_obj.at<float>(0, 1) = static_cast<float>(min_v);
                pseudo_obj.at<float>(0, 2) = static_cast<float>(max_u);
                pseudo_obj.at<float>(0, 3) = static_cast<float>(max_v);
                pseudo_cuboid = cuboids_from_boxes(
                    pseudo_obj, z_world, cv::Mat());
            }

            std::array<cv::Vec3d, 4> bottom;
            std::vector<double> heights;
            bool valid = std::isfinite(z_world);

            for (int k = 0; k < 4 && valid; ++k) {
                const cv::Vec2d uv = read_uv(k);
                valid = intersect_uv_with_z(uv[0], uv[1], z_world, bottom[k]);
            }
            if (!valid) {
                if (!pseudo_cuboid.empty()) {
                    pseudo_cuboid.row(0).copyTo(cuboids.row(i));
                }
                continue;
            }

            for (int k = 0; k < 4 && valid; ++k) {
                const cv::Vec2d top_uv = read_uv(k + 4);
                const cv::Vec3d top_ray = world_ray_dir(
                    top_uv[0], top_uv[1]);
                const double norm2 = top_ray[0] * top_ray[0] +
                                     top_ray[1] * top_ray[1];
                if (!std::isfinite(norm2) || norm2 < 1e-12) {
                    valid = false;
                    break;
                }

                const double tx = bottom[k][0] - C_w_[0];
                const double ty = bottom[k][1] - C_w_[1];
                const double lambda = (tx * top_ray[0] +
                                       ty * top_ray[1]) / norm2;
                if (!std::isfinite(lambda) || lambda <= 1e-8) {
                    valid = false;
                    break;
                }

                const double top_z = C_w_[2] + lambda * top_ray[2];
                const double xy_error = std::hypot(
                    C_w_[0] + lambda * top_ray[0] - bottom[k][0],
                    C_w_[1] + lambda * top_ray[1] - bottom[k][1]);
                const double height = top_z - z_world;
                if (!std::isfinite(height) || height <= 1e-4 ||
                    !std::isfinite(xy_error) || xy_error > 0.15) {
                    valid = false;
                    break;
                }
                heights.push_back(height);
            }

            if (!valid || heights.size() != 4) {
                if (!std::isfinite(prior_h) || prior_h <= 1e-4) {
                    if (!pseudo_cuboid.empty()) {
                        pseudo_cuboid.row(0).copyTo(cuboids.row(i));
                    }
                    continue;
                }
                heights.assign(4, prior_h);
            }

            std::sort(heights.begin(), heights.end());
            double height = 0.5 * (heights[1] + heights[2]);
            if (!std::isfinite(height) || height <= 1e-4) {
                if (!pseudo_cuboid.empty()) {
                    pseudo_cuboid.row(0).copyTo(cuboids.row(i));
                }
                continue;
            }

            // 所有障碍物均采用动态宽度：由前后两条底边的实测距离
            // 平均得到，不再仅使用 whitelist 中的固定 width 先验。
            const double dynamic_width = 0.5 * (
                std::hypot(bottom[1][0] - bottom[0][0],
                           bottom[1][1] - bottom[0][1]) +
                std::hypot(bottom[2][0] - bottom[3][0],
                           bottom[2][1] - bottom[3][1]));
            const double dynamic_length = 0.5 * (
                std::hypot(bottom[2][0] - bottom[1][0],
                           bottom[2][1] - bottom[1][1]) +
                std::hypot(bottom[3][0] - bottom[0][0],
                           bottom[3][1] - bottom[0][1]));
            if (!std::isfinite(dynamic_width) ||
                !std::isfinite(dynamic_length) ||
                dynamic_width <= 1e-4 || dynamic_length <= 1e-4) {
                if (!pseudo_cuboid.empty()) {
                    pseudo_cuboid.row(0).copyTo(cuboids.row(i));
                }
                continue;
            }

            cv::Vec3d bottom_center(0.0, 0.0, z_world);
            for (const auto& p : bottom) {
                bottom_center[0] += 0.25 * p[0];
                bottom_center[1] += 0.25 * p[1];
            }

            double theta = 0.0;
            if (cid == 0) {
                // 人员 3D 点的前后方向不稳定，使用相机到人员中心的
                // 径向方向，行为与原始 2D 人员 solver 更接近。
                theta = std::atan2(
                    bottom_center[1] - C_w_[1],
                    bottom_center[0] - C_w_[0]);
            } else {
                const cv::Vec2d front_mid(
                    0.5 * (bottom[0][0] + bottom[1][0]),
                    0.5 * (bottom[0][1] + bottom[1][1]));
                const cv::Vec2d rear_mid(
                    0.5 * (bottom[2][0] + bottom[3][0]),
                    0.5 * (bottom[2][1] + bottom[3][1]));
                const cv::Vec2d direction = front_mid - rear_mid;
                if (!std::isfinite(direction[0]) ||
                    !std::isfinite(direction[1]) ||
                    std::hypot(direction[0], direction[1]) <= 1e-4) {
                    continue;
                }
                theta = std::atan2(direction[1], direction[0]);
            }

            // 人员按圆柱底面处理，长度使用宽度，避免不稳定的前后方向
            // 将人员尺寸拉成长条。
            const double output_l = cid == 0 ? dynamic_width : dynamic_length;
            const double output_w = dynamic_width;
            cuboids.at<float>(i, 0) = static_cast<float>(bottom_center[0]);
            cuboids.at<float>(i, 1) = static_cast<float>(bottom_center[1]);
            cuboids.at<float>(i, 2) = static_cast<float>(
                z_world + 0.5 * height);
            cuboids.at<float>(i, 3) = static_cast<float>(output_l);
            cuboids.at<float>(i, 4) = static_cast<float>(output_w);
            cuboids.at<float>(i, 5) = static_cast<float>(height);
            cuboids.at<float>(i, 8) = static_cast<float>(wrap_to_pi(theta));

            const bool direct_valid =
                std::isfinite(cuboids.at<float>(i, 0)) &&
                std::isfinite(cuboids.at<float>(i, 1)) &&
                std::isfinite(cuboids.at<float>(i, 2)) &&
                cuboids.at<float>(i, 3) > 0.0f &&
                cuboids.at<float>(i, 4) > 0.0f &&
                cuboids.at<float>(i, 5) > 0.0f;
            const bool pseudo_valid =
                !pseudo_cuboid.empty() &&
                pseudo_cuboid.at<float>(0, 3) > 0.0f &&
                pseudo_cuboid.at<float>(0, 4) > 0.0f &&
                pseudo_cuboid.at<float>(0, 5) > 0.0f;
            if (pseudo_valid && direct_valid) {
                const double center_error = std::hypot(
                    cuboids.at<float>(i, 0) - pseudo_cuboid.at<float>(0, 0),
                    cuboids.at<float>(i, 1) - pseudo_cuboid.at<float>(0, 1));
                const double depth_error = std::abs(
                    cuboids.at<float>(i, 2) - pseudo_cuboid.at<float>(0, 2));
                const double length_error = std::abs(
                    cuboids.at<float>(i, 3) - pseudo_cuboid.at<float>(0, 3)) /
                    std::max(1e-3, static_cast<double>(
                        cuboids.at<float>(i, 3)));
                const double width_error = std::abs(
                    cuboids.at<float>(i, 4) - pseudo_cuboid.at<float>(0, 4)) /
                    std::max(1e-3, static_cast<double>(
                        cuboids.at<float>(i, 4)));
                const double theta_error = std::abs(wrap_to_pi(
                    cuboids.at<float>(i, 8) - pseudo_cuboid.at<float>(0, 8)));
                const bool geometry_consistent =
                    center_error <= 1.5 && depth_error <= 1.5 &&
                    length_error <= 1.0 && width_error <= 1.0 &&
                    (cid == 0 || theta_error <= 0.5 * CV_PI);

                if (geometry_consistent) {
                    const double direct_quality = std::max(
                        0.2, static_cast<double>(valid_point_count) / 8.0);
                    const double pseudo_quality = std::max(
                        0.2, 1.0 - 0.5 * std::min(1.0, width_error));
                    const double direct_weight =
                        direct_quality / (direct_quality + pseudo_quality);
                    const double pseudo_weight = 1.0 - direct_weight;
                    for (int k = 0; k < 6; ++k) {
                        cuboids.at<float>(i, k) = static_cast<float>(
                            direct_weight * cuboids.at<float>(i, k) +
                            pseudo_weight * pseudo_cuboid.at<float>(0, k));
                    }
                    if (cid != 0) {
                        const double sin_theta =
                            direct_weight * std::sin(cuboids.at<float>(i, 8)) +
                            pseudo_weight * std::sin(pseudo_cuboid.at<float>(0, 8));
                        const double cos_theta =
                            direct_weight * std::cos(cuboids.at<float>(i, 8)) +
                            pseudo_weight * std::cos(pseudo_cuboid.at<float>(0, 8));
                        cuboids.at<float>(i, 8) = static_cast<float>(
                            std::atan2(sin_theta, cos_theta));
                    }
                }
            }
        }
        return cuboids;
    }

    cv::Mat cuboids_from_boxesAnd3DAligned(const cv::Mat& objs,
                                           const cv::Mat& points3d,
                                           double z_world,
                                           const cv::Mat& masks) const {
        CV_Assert(objs.type() == CV_32F);
        CV_Assert(objs.cols == 7);
        cv::Mat fallback = cuboids_from_boxes(objs, z_world, masks);
        if (points3d.empty()) {
            return fallback;
        }
        CV_Assert(points3d.rows == objs.rows);

        const bool flat_points =
            (points3d.channels() == 1 &&
             (points3d.cols == 16) &&
             (points3d.type() == CV_32F || points3d.type() == CV_64F));
        const bool packed_points =
            (points3d.channels() == 2 &&
             (points3d.cols == 8) &&
             (points3d.type() == CV_32FC2 || points3d.type() == CV_64FC2));
        CV_Assert(flat_points || packed_points);

        auto read_point = [&](int row, int point_index) -> cv::Vec2d {
            if (flat_points) {
                if (points3d.type() == CV_32F) {
                    return cv::Vec2d(
                        points3d.at<float>(row, 2 * point_index),
                        points3d.at<float>(row, 2 * point_index + 1));
                }
                return cv::Vec2d(
                    points3d.at<double>(row, 2 * point_index),
                    points3d.at<double>(row, 2 * point_index + 1));
            }
            if (points3d.type() == CV_32FC2) {
                const cv::Vec2f p = points3d.at<cv::Vec2f>(row, point_index);
                return cv::Vec2d(p[0], p[1]);
            }
            const cv::Vec2d p = points3d.at<cv::Vec2d>(row, point_index);
            return p;
        };

        auto finite_vec2 = [](const cv::Vec2d& p) {
            return std::isfinite(p[0]) && std::isfinite(p[1]);
        };

        auto recover_edge = [&](const cv::Vec2d& bottom_uv,
                                const cv::Vec2d& top_uv,
                                double expected_height,
                                cv::Vec3d& bottom,
                                cv::Vec3d& top) -> bool {
            if (!finite_vec2(bottom_uv) || !finite_vec2(top_uv)) return false;
            const cv::Vec3d db = world_ray_dir(bottom_uv[0], bottom_uv[1]);
            const cv::Vec3d dt = world_ray_dir(top_uv[0], top_uv[1]);
            if (!std::isfinite(expected_height) || expected_height <= 1e-4) {
                return false;
            }

            // Solve lb*db - lt*dt = (0, 0, -height) in least squares.
            const double a[3][2] = {
                {db[0], -dt[0]},
                {db[1], -dt[1]},
                {db[2], -dt[2]},
            };
            const double b[3] = {0.0, 0.0, -expected_height};
            double ata00 = 0.0, ata01 = 0.0, ata11 = 0.0;
            double atb0 = 0.0, atb1 = 0.0;
            for (int r = 0; r < 3; ++r) {
                ata00 += a[r][0] * a[r][0];
                ata01 += a[r][0] * a[r][1];
                ata11 += a[r][1] * a[r][1];
                atb0 += a[r][0] * b[r];
                atb1 += a[r][1] * b[r];
            }
            const double det = ata00 * ata11 - ata01 * ata01;
            if (!std::isfinite(det) || std::abs(det) < 1e-12) return false;
            const double lb = (atb0 * ata11 - ata01 * atb1) / det;
            const double lt = (ata00 * atb1 - ata01 * atb0) / det;
            if (!std::isfinite(lb) || !std::isfinite(lt) ||
                lb <= 1e-8 || lt <= 1e-8) return false;

            const cv::Vec3d pb = C_w_ + lb * db;
            const cv::Vec3d pt = C_w_ + lt * dt;
            const double residual = std::sqrt(
                std::pow(pb[0] - pt[0], 2) +
                std::pow(pb[1] - pt[1], 2) +
                std::pow((pt[2] - pb[2]) - expected_height, 2));
            if (!std::isfinite(residual) || residual > 0.05) return false;
            bottom = pb;
            top = pt;
            return true;
        };

        // 将 3D 底部投影点转换为伪 2D 框，再复用原始 2D solver。
        // 对圆柱图像按一个周期展开，避免目标跨越图像首尾边界时宽度异常。
        cv::Mat points_box_objs = objs.clone();
        const double u_period = 2.0 * CV_PI * f_;
        for (int i = 0; i < objs.rows; ++i) {
            const float invalid_coord =
                std::numeric_limits<float>::quiet_NaN();
            points_box_objs.at<float>(i, 0) = invalid_coord;
            points_box_objs.at<float>(i, 1) = invalid_coord;
            points_box_objs.at<float>(i, 2) = invalid_coord;
            points_box_objs.at<float>(i, 3) = invalid_coord;
            std::array<cv::Vec2d, 4> bottom_uv;
            for (int k = 0; k < 4; ++k) {
                bottom_uv[k] = read_point(i, k);
            }

            bool valid_points = true;
            for (const auto& p : bottom_uv) {
                valid_points = valid_points && finite_vec2(p);
            }
            if (!valid_points || !std::isfinite(u_period) || u_period <= 0.0) {
                continue;
            }

            const double u_ref = bottom_uv[0][0];
            double u_left = u_ref;
            double u_right = u_ref;
            double v_bottom = bottom_uv[0][1];
            for (const auto& p : bottom_uv) {
                double u = p[0];
                while (u - u_ref > 0.5 * u_period) u -= u_period;
                while (u - u_ref < -0.5 * u_period) u += u_period;
                u_left = std::min(u_left, u);
                u_right = std::max(u_right, u);
                v_bottom = std::max(v_bottom, p[1]);
            }

            points_box_objs.at<float>(i, 0) = static_cast<float>(u_left);
            points_box_objs.at<float>(i, 1) = static_cast<float>(v_bottom);
            points_box_objs.at<float>(i, 2) = static_cast<float>(u_right);
            points_box_objs.at<float>(i, 3) = static_cast<float>(v_bottom);

            // 使用底面前边中点(p0,p1)到后边中点(p2,p3)的方向，
            // 生成伪 2D solver 所需的相对朝向。
            cv::Vec3d p0_world, p1_world, p2_world, p3_world;
            if (intersect_uv_with_z(bottom_uv[0][0], bottom_uv[0][1],
                                    z_world, p0_world) &&
                intersect_uv_with_z(bottom_uv[1][0], bottom_uv[1][1],
                                    z_world, p1_world) &&
                intersect_uv_with_z(bottom_uv[2][0], bottom_uv[2][1],
                                    z_world, p2_world) &&
                intersect_uv_with_z(bottom_uv[3][0], bottom_uv[3][1],
                                    z_world, p3_world)) {
                const cv::Vec2d front_mid(
                    0.5 * (p0_world[0] + p1_world[0]),
                    0.5 * (p0_world[1] + p1_world[1]));
                const cv::Vec2d rear_mid(
                    0.5 * (p2_world[0] + p3_world[0]),
                    0.5 * (p2_world[1] + p3_world[1]));
                const cv::Vec2d direction = front_mid - rear_mid;
                if (std::isfinite(direction[0]) &&
                    std::isfinite(direction[1]) &&
                    std::hypot(direction[0], direction[1]) > 1e-4) {
                    const double theta_3d =
                        std::atan2(direction[1], direction[0]);
                    const double u_mid = 0.5 * (u_left + u_right);
                    const cv::Vec3d mid_ray = world_ray_dir(u_mid, v_bottom);
                    if (std::hypot(mid_ray[0], mid_ray[1]) > 1e-8) {
                        const double ray_azimuth =
                            std::atan2(mid_ray[1], mid_ray[0]);
                        points_box_objs.at<float>(i, 6) = static_cast<float>(
                            wrap_to_pi(theta_3d - ray_azimuth));
                    }
                }
            }
        }
        const cv::Mat cuboids_from_points_box =
            cuboids_from_boxes(points_box_objs, z_world, cv::Mat());

        for (int i = 0; i < objs.rows; ++i) {
            const int cid = static_cast<int>(std::llround(objs.at<float>(i, 5)));
            double expected_height = static_cast<double>(fallback.at<float>(i, 5));
            if (!std::isfinite(expected_height) || expected_height <= 1e-4) {
                const auto size_it = id2size_.find(cid);
                if (size_it == id2size_.end()) continue;
                expected_height = size_it->second[2];
            }
            std::array<cv::Vec3d, 4> bottoms;
            std::array<cv::Vec3d, 4> tops;
            bool valid = true;
            for (int k = 0; k < 4; ++k) {
                if (!recover_edge(read_point(i, k), read_point(i, k + 4),
                                  expected_height,
                                  bottoms[k], tops[k])) {
                    valid = false;
                    break;
                }
            }
            if (!valid) continue;

            cv::Vec3d bottom_center(0.0, 0.0, 0.0);
            cv::Vec3d top_center(0.0, 0.0, 0.0);
            for (int k = 0; k < 4; ++k) {
                bottom_center += bottoms[k];
                top_center += tops[k];
            }
            bottom_center *= 0.25;
            top_center *= 0.25;

            // Keep the original cuboids_from_boxes height as the stable
            // class prior; the eight points refine position, length, width,
            // and orientation without changing the output contract.
            const double h = expected_height;
            const double width_front = std::hypot(
                bottoms[1][0] - bottoms[0][0],
                bottoms[1][1] - bottoms[0][1]);
            const double width_rear = std::hypot(
                bottoms[2][0] - bottoms[3][0],
                bottoms[2][1] - bottoms[3][1]);
            const double length_right = std::hypot(
                bottoms[2][0] - bottoms[1][0],
                bottoms[2][1] - bottoms[1][1]);
            const double length_left = std::hypot(
                bottoms[3][0] - bottoms[0][0],
                bottoms[3][1] - bottoms[0][1]);
            const cv::Vec2d front_mid(
                0.5 * (bottoms[0][0] + bottoms[1][0]),
                0.5 * (bottoms[0][1] + bottoms[1][1]));
            const cv::Vec2d rear_mid(
                0.5 * (bottoms[2][0] + bottoms[3][0]),
                0.5 * (bottoms[2][1] + bottoms[3][1]));
            const cv::Vec2d direction = front_mid - rear_mid;
            const double theta = std::atan2(direction[1], direction[0]);
            const double l = 0.5 * (length_left + length_right);
            const double w = 0.5 * (width_front + width_rear);
            if (!std::isfinite(h) || !std::isfinite(l) || !std::isfinite(w) ||
                h <= 1e-4 || l <= 1e-4 || w <= 1e-4 ||
                !std::isfinite(theta)) {
                continue;
            }

            cv::Mat& out = fallback;
            // 先融合“原始框 solver”和“3D 点生成伪框 solver”，
            // 再与直接八点结果融合，形成三路相互验证。
            const bool points_box_valid =
                std::isfinite(cuboids_from_points_box.at<float>(i, 0)) &&
                std::isfinite(cuboids_from_points_box.at<float>(i, 1)) &&
                std::isfinite(cuboids_from_points_box.at<float>(i, 2)) &&
                cuboids_from_points_box.at<float>(i, 3) > 0.0f &&
                cuboids_from_points_box.at<float>(i, 4) > 0.0f &&
                cuboids_from_points_box.at<float>(i, 5) > 0.0f;
            const bool original_valid_before_box =
                std::isfinite(out.at<float>(i, 0)) &&
                std::isfinite(out.at<float>(i, 1)) &&
                std::isfinite(out.at<float>(i, 2)) &&
                out.at<float>(i, 3) > 0.0f &&
                out.at<float>(i, 4) > 0.0f &&
                out.at<float>(i, 5) > 0.0f;
            if (points_box_valid && original_valid_before_box) {
                const double box_xy_error = std::hypot(
                    out.at<float>(i, 0) - cuboids_from_points_box.at<float>(i, 0),
                    out.at<float>(i, 1) - cuboids_from_points_box.at<float>(i, 1));
                const double box_z_error = std::abs(
                    out.at<float>(i, 2) - cuboids_from_points_box.at<float>(i, 2));
                const double box_l_error = std::abs(
                    out.at<float>(i, 3) - cuboids_from_points_box.at<float>(i, 3)) /
                    std::max(1e-3, static_cast<double>(out.at<float>(i, 3)));
                const double box_w_error = std::abs(
                    out.at<float>(i, 4) - cuboids_from_points_box.at<float>(i, 4)) /
                    std::max(1e-3, static_cast<double>(out.at<float>(i, 4)));
                if (box_xy_error <= 1.0 && box_z_error <= 1.0 &&
                    box_l_error <= 0.75 && box_w_error <= 0.75) {
                    constexpr double original_box_weight = 0.5;
                    constexpr double points_box_weight = 1.0 - original_box_weight;
                    for (int k = 0; k < 6; ++k) {
                        out.at<float>(i, k) = static_cast<float>(
                            original_box_weight * out.at<float>(i, k) +
                            points_box_weight *
                                cuboids_from_points_box.at<float>(i, k));
                    }
                }
            }

            double point_cx = 0.5 * (bottom_center[0] + top_center[0]);
            double point_cy = 0.5 * (bottom_center[1] + top_center[1]);
            double point_cz = 0.5 * (bottom_center[2] + top_center[2]);
            double point_l = l;
            double point_w = w;
            double point_h = h;
            double point_height_spread = 1.0;

            // 用原始结果提供底面深度尺度，再由顶部 3D 点反推出实际高度。
            // 这样 fixed-size 和 dynamic-width 两种原始 solver 都能修正先验 h。
            const bool original_geometry_valid =
                out.at<float>(i, 3) > 0.0f &&
                out.at<float>(i, 4) > 0.0f &&
                out.at<float>(i, 5) > 0.0f &&
                std::isfinite(out.at<float>(i, 2));
            const double base_z = original_geometry_valid
                ? static_cast<double>(out.at<float>(i, 2)) -
                      0.5 * static_cast<double>(out.at<float>(i, 5))
                : z_world;
            double original_quality = 0.0;
            double point_quality = 0.0;
            if (std::isfinite(base_z)) {
                std::array<cv::Vec3d, 4> measured_bottoms;
                std::vector<double> measured_heights;
                bool refined_geometry_valid = true;
                for (int k = 0; k < 4; ++k) {
                    if (!intersect_uv_with_z(
                            read_point(i, k)[0], read_point(i, k)[1],
                            base_z, measured_bottoms[k])) {
                        refined_geometry_valid = false;
                        break;
                    }

                    const cv::Vec2d top_uv = read_point(i, k + 4);
                    const cv::Vec3d top_ray = world_ray_dir(top_uv[0], top_uv[1]);
                    const double dx = top_ray[0];
                    const double dy = top_ray[1];
                    const double norm2 = dx * dx + dy * dy;
                    if (!std::isfinite(norm2) || norm2 < 1e-12) {
                        refined_geometry_valid = false;
                        break;
                    }

                    const double tx = measured_bottoms[k][0] - C_w_[0];
                    const double ty = measured_bottoms[k][1] - C_w_[1];
                    const double lambda = (tx * dx + ty * dy) / norm2;
                    if (!std::isfinite(lambda) || lambda <= 1e-8) {
                        refined_geometry_valid = false;
                        break;
                    }

                    const double top_z = C_w_[2] + lambda * top_ray[2];
                    const double measured_h = top_z - base_z;
                    const double xy_error = std::hypot(
                        C_w_[0] + lambda * dx - measured_bottoms[k][0],
                        C_w_[1] + lambda * dy - measured_bottoms[k][1]);
                    if (!std::isfinite(measured_h) || measured_h <= 1e-4 ||
                        !std::isfinite(xy_error) || xy_error > 0.15) {
                        refined_geometry_valid = false;
                        break;
                    }
                    measured_heights.push_back(measured_h);
                }

                if (refined_geometry_valid && measured_heights.size() == 4) {
                    std::sort(measured_heights.begin(), measured_heights.end());
                    const double refined_h =
                        0.5 * (measured_heights[1] + measured_heights[2]);
                    point_height_spread =
                        (measured_heights[3] - measured_heights[0]) /
                        std::max(1e-3, refined_h);
                    const double refined_l = 0.5 * (
                        std::hypot(measured_bottoms[2][0] - measured_bottoms[1][0],
                                   measured_bottoms[2][1] - measured_bottoms[1][1]) +
                        std::hypot(measured_bottoms[3][0] - measured_bottoms[0][0],
                                   measured_bottoms[3][1] - measured_bottoms[0][1]));
                    const double refined_w = 0.5 * (
                        std::hypot(measured_bottoms[1][0] - measured_bottoms[0][0],
                                   measured_bottoms[1][1] - measured_bottoms[0][1]) +
                        std::hypot(measured_bottoms[2][0] - measured_bottoms[3][0],
                                   measured_bottoms[2][1] - measured_bottoms[3][1]));
                    if (std::isfinite(refined_h) && std::isfinite(refined_l) &&
                        std::isfinite(refined_w) && refined_h > 1e-4 &&
                        refined_l > 1e-4 && refined_w > 1e-4) {
                        cv::Vec3d measured_bottom_center(0.0, 0.0, base_z);
                        for (const auto& p : measured_bottoms) {
                            measured_bottom_center[0] += p[0] * 0.25;
                            measured_bottom_center[1] += p[1] * 0.25;
                        }
                        point_cx = measured_bottom_center[0];
                        point_cy = measured_bottom_center[1];
                        point_cz = base_z + 0.5 * refined_h;
                        point_l = refined_l;
                        point_w = refined_w;
                        point_h = refined_h;
                    }
                }

                auto clamp01 = [](double value) {
                    if (!std::isfinite(value)) return 0.0;
                    return std::max(0.0, std::min(1.0, value));
                };

                // 2D 观测不确定性：低置信度、小目标、极端长宽比都会降低可靠性。
                const double detection_conf = clamp01(
                    static_cast<double>(objs.at<float>(i, 4)));
                const double box_width = std::abs(
                    static_cast<double>(objs.at<float>(i, 2)) -
                    static_cast<double>(objs.at<float>(i, 0)));
                const double box_height = std::abs(
                    static_cast<double>(objs.at<float>(i, 3)) -
                    static_cast<double>(objs.at<float>(i, 1)));
                const double box_scale = std::sqrt(std::max(1.0, box_width * box_height));
                const double size_quality = box_scale / (box_scale + 32.0);
                const double aspect_ratio = std::max(
                    box_width / std::max(1e-3, box_height),
                    box_height / std::max(1e-3, box_width));
                const double aspect_penalty = std::max(
                    0.0, std::log(std::max(1.0, aspect_ratio)) - std::log(2.0));
                const double aspect_quality =
                    1.0 / (1.0 + aspect_penalty);
                original_quality = clamp01(
                    detection_conf * size_quality * aspect_quality);

                // 3D 观测不确定性：几何不一致越大，3D 源可靠性越低。
                const double point_geometry_quality = clamp01(
                    1.0 / (1.0 + 4.0 * std::max(0.0, point_height_spread)));
                point_quality = clamp01(
                    0.85 * point_geometry_quality + 0.15);
            }

            if (cid == 0) {
                // 人员的点朝向不稳定：用原始解算的朝向和尺寸，
                // 只在 3D 点与原始结果相互一致时融合空间位置。
                const bool original_valid =
                    std::isfinite(out.at<float>(i, 0)) &&
                    std::isfinite(out.at<float>(i, 1)) &&
                    std::isfinite(out.at<float>(i, 2)) &&
                    out.at<float>(i, 3) > 0.0f &&
                    out.at<float>(i, 4) > 0.0f &&
                    out.at<float>(i, 5) > 0.0f;
                if (original_valid) {
                    const double original_cx = out.at<float>(i, 0);
                    const double original_cy = out.at<float>(i, 1);
                    const double original_cz = out.at<float>(i, 2);
                    const double xy_error = std::hypot(
                        point_cx - original_cx, point_cy - original_cy);
                    const double z_error = std::abs(point_cz - original_cz);
                    const double theta_error = std::abs(
                        wrap_to_pi(theta - out.at<float>(i, 8)));

                    // 两个独立观测差异过大时，认为点输入异常，不污染原始结果。
                    if (theta_error > 0.25 * CV_PI &&
                        std::isfinite(xy_error) && xy_error <= 1.0 &&
                        std::isfinite(z_error) && z_error <= 1.0) {
                        // 方向冲突时，以 3D 前后中点方向为准。
                        out.at<float>(i, 0) = static_cast<float>(point_cx);
                        out.at<float>(i, 1) = static_cast<float>(point_cy);
                        out.at<float>(i, 2) = static_cast<float>(point_cz);
                        out.at<float>(i, 3) = static_cast<float>(point_l);
                        out.at<float>(i, 4) = static_cast<float>(point_w);
                        out.at<float>(i, 5) = static_cast<float>(point_h);
                        out.at<float>(i, 8) = static_cast<float>(
                            wrap_to_pi(theta));
                    } else if (std::isfinite(xy_error) && std::isfinite(z_error) &&
                        xy_error <= 1.0 && z_error <= 1.0) {
                        const double quality_sum = original_quality + point_quality;
                        const double original_weight = quality_sum > 1e-8
                            ? original_quality / quality_sum : 0.5;
                        const double point_weight = 1.0 - original_weight;
                        out.at<float>(i, 0) = static_cast<float>(
                            original_weight * original_cx + point_weight * point_cx);
                        out.at<float>(i, 1) = static_cast<float>(
                            original_weight * original_cy + point_weight * point_cy);
                        out.at<float>(i, 2) = static_cast<float>(
                            original_weight * original_cz + point_weight * point_cz);
                        const double l_error = std::abs(point_l - out.at<float>(i, 3)) /
                                               std::max(1e-3, static_cast<double>(out.at<float>(i, 3)));
                        const double w_error = std::abs(point_w - out.at<float>(i, 4)) /
                                               std::max(1e-3, static_cast<double>(out.at<float>(i, 4)));
                        const double h_error = std::abs(point_h - out.at<float>(i, 5)) /
                                               std::max(1e-3, static_cast<double>(out.at<float>(i, 5)));
                        if (l_error <= 0.75 && w_error <= 0.75 &&
                            h_error <= 0.75) {
                            out.at<float>(i, 3) = static_cast<float>(
                                original_weight * out.at<float>(i, 3) +
                                point_weight * point_l);
                            out.at<float>(i, 4) = static_cast<float>(
                                original_weight * out.at<float>(i, 4) +
                                point_weight * point_w);
                            out.at<float>(i, 5) = static_cast<float>(
                                original_weight * out.at<float>(i, 5) +
                                point_weight * point_h);
                        }
                    }
                } else {
                    // 原始结果无效时，使用通过高度先验验证过的 3D 点结果。
                    out.at<float>(i, 0) = static_cast<float>(point_cx);
                    out.at<float>(i, 1) = static_cast<float>(point_cy);
                    out.at<float>(i, 2) = static_cast<float>(point_cz);
                    out.at<float>(i, 3) = static_cast<float>(
                        point_l);
                    out.at<float>(i, 4) = static_cast<float>(
                        point_w);
                    out.at<float>(i, 5) = static_cast<float>(point_h);
                }
                continue;
            }

            const bool original_valid =
                std::isfinite(out.at<float>(i, 0)) &&
                std::isfinite(out.at<float>(i, 1)) &&
                std::isfinite(out.at<float>(i, 2)) &&
                out.at<float>(i, 3) > 0.0f &&
                out.at<float>(i, 4) > 0.0f &&
                out.at<float>(i, 5) > 0.0f;
            if (!original_valid) {
                out.at<float>(i, 0) = static_cast<float>(point_cx);
                out.at<float>(i, 1) = static_cast<float>(point_cy);
                out.at<float>(i, 2) = static_cast<float>(point_cz);
                out.at<float>(i, 3) = static_cast<float>(point_l);
                out.at<float>(i, 4) = static_cast<float>(point_w);
                out.at<float>(i, 5) = static_cast<float>(point_h);
                out.at<float>(i, 8) = static_cast<float>(wrap_to_pi(theta));
                continue;
            }

            const double original_cx = out.at<float>(i, 0);
            const double original_cy = out.at<float>(i, 1);
            const double original_cz = out.at<float>(i, 2);
            const double original_l = out.at<float>(i, 3);
            const double original_w = out.at<float>(i, 4);
            const double original_h = out.at<float>(i, 5);
            const double original_theta = out.at<float>(i, 8);
            const double xy_error = std::hypot(
                point_cx - original_cx, point_cy - original_cy);
            const double z_error = std::abs(point_cz - original_cz);
            const double l_error = std::abs(point_l - original_l) /
                                   std::max(1e-3, original_l);
            const double w_error = std::abs(point_w - original_w) /
                                   std::max(1e-3, original_w);
            const double h_error = std::abs(point_h - original_h) /
                                   std::max(1e-3, original_h);
            const double theta_error = std::abs(
                wrap_to_pi(theta - original_theta));
            const bool geometry_consistent =
                std::isfinite(xy_error) && xy_error <= 1.0 &&
                std::isfinite(z_error) && z_error <= 1.0 &&
                std::isfinite(l_error) && l_error <= 0.75 &&
                std::isfinite(w_error) && w_error <= 0.75 &&
                std::isfinite(h_error) && h_error <= 0.75;

            // 只有两路结果相互一致时才融合，避免异常 3D 点污染原始结果。
            if (geometry_consistent && theta_error > 0.25 * CV_PI) {
                // 2D 方向发生明显翻转或误检时，用 3D 前后中点方向
                // 直接矫正整个 2D 解算结果。
                out.at<float>(i, 0) = static_cast<float>(point_cx);
                out.at<float>(i, 1) = static_cast<float>(point_cy);
                out.at<float>(i, 2) = static_cast<float>(point_cz);
                out.at<float>(i, 3) = static_cast<float>(point_l);
                out.at<float>(i, 4) = static_cast<float>(point_w);
                out.at<float>(i, 5) = static_cast<float>(point_h);
                out.at<float>(i, 8) = static_cast<float>(wrap_to_pi(theta));
                continue;
            }

            const bool mutually_consistent =
                geometry_consistent &&
                std::isfinite(theta_error) &&
                theta_error <= (0.5 * CV_PI);
            if (mutually_consistent) {
                const double quality_sum = original_quality + point_quality;
                const double original_weight = quality_sum > 1e-8
                    ? original_quality / quality_sum : 0.5;
                const double point_weight = 1.0 - original_weight;
                out.at<float>(i, 0) = static_cast<float>(
                    original_weight * original_cx + point_weight * point_cx);
                out.at<float>(i, 1) = static_cast<float>(
                    original_weight * original_cy + point_weight * point_cy);
                out.at<float>(i, 2) = static_cast<float>(
                    original_weight * original_cz + point_weight * point_cz);
                out.at<float>(i, 3) = static_cast<float>(
                    original_weight * original_l + point_weight * point_l);
                out.at<float>(i, 4) = static_cast<float>(
                    original_weight * original_w + point_weight * point_w);
                out.at<float>(i, 5) = static_cast<float>(
                    original_weight * original_h + point_weight * point_h);

                // 角度使用圆周线性融合，避免在 -pi/pi 边界处跳变。
                const double sin_theta =
                    original_weight * std::sin(original_theta) +
                    point_weight * std::sin(theta);
                const double cos_theta =
                    original_weight * std::cos(original_theta) +
                    point_weight * std::cos(theta);
                out.at<float>(i, 8) = static_cast<float>(
                    std::atan2(sin_theta, cos_theta));
            }
        }

        return fallback;
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
