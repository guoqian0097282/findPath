#pragma once
// common/cimg.hpp
// Header-only 实现：CalibratedImage + ImageProcessor
// 依赖：OpenCV, nlohmann::json

#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>
#include <array>
#include <tuple>
#include <stdexcept>
#include <algorithm>
#include <iostream>

struct CalibratedImage {
    // ---------------- 图像数据 (BGR) ----------------
    cv::Mat image; // HxWx3, CV_8UC3

    // ---------------- 标定参数（与 JSON calib 对应） ----------------
    std::string type = "pinhole"; // "pinhole" | "fisheye"

    // --- extrinsics 外参（世界->相机，主动定义）--------------
    double world_x = 0.0;
    double world_y = 0.0;
    double world_z = 0.0;
    double pitch = 0.0;  // rad
    double yaw   = 0.0;  // rad
    double roll  = 0.0;  // rad
    std::string angle_type = "rad"; // 固定为 "rad"（load 时标准化）

    // --- intrinsics 内参（像素坐标系） --------------
    std::string image_type = "BGR"; // "RGB" | "BGR"
    int    image_width  = 0;
    int    image_height = 0;
    double focal_u = 0.0;
    double focal_v = 0.0;
    double center_u = 0.0;
    double center_v = 0.0;

    // 视场角（度）
    double fov = 0.0;

    // 3x3 旋转矩阵（世界->相机，主动；Python版以 list[list] 存）
    cv::Matx33d R_wc = cv::Matx33d::eye();
    // 3x1 平移向量（米）
    cv::Vec3d   t_wc = cv::Vec3d(0, 0, 0);

    // --- distortion 畸变参数 ---------------------
    std::vector<double> pinhole_distort; // [k1,k2,p1,p2]（未在此文件使用）
    std::vector<double> fisheye_distort; // [k1,k2,k3,k4]

    // ---------------- I/O ----------------
    static CalibratedImage load(const cv::Mat& img_src,
                                const nlohmann::json& calib_params) {
        using json = nlohmann::json;
        CalibratedImage obj;

        const auto get_str  = [&](const json& j, const char* k, const std::string& d="") -> std::string {
            return j.contains(k) ? j.at(k).get<std::string>() : d;
        };
        const auto get_num  = [&](const json& j, const char* k, double d=0.0) -> double {
            return j.contains(k) ? j.at(k).get<double>() : d;
        };
        const auto get_int  = [&](const json& j, const char* k, int d=0) -> int {
            return j.contains(k) ? j.at(k).get<int>() : d;
        };
        const auto get_vecd = [&](const json& j, const char* k) -> std::vector<double> {
            if (!j.contains(k)) return {};
            std::vector<double> v;
            for (auto& x : j.at(k)) v.push_back(x.get<double>());
            return v;
        };

        obj.type       = get_str(calib_params, "type", "pinhole");
        obj.image_type = get_str(calib_params, "image_type", "BGR");
        obj.image      = img_src;

        // 角度：支持 degree / rad，统一转 rad
        std::string angle_type_raw = get_str(calib_params, "angle_type", "rad");
        std::string angle_lc = angle_type_raw;
        std::transform(angle_lc.begin(), angle_lc.end(), angle_lc.begin(),
                       [](unsigned char c){ return char(std::tolower(c)); });

        double pitch_v = get_num(calib_params, "pitch", 0.0);
        double yaw_v   = get_num(calib_params, "yaw",   0.0);
        double roll_v  = get_num(calib_params, "roll",  0.0);
        if (angle_lc == "degree") {
            pitch_v = pitch_v * M_PI / 180.0;
            yaw_v   = yaw_v   * M_PI / 180.0;
            roll_v  = roll_v  * M_PI / 180.0;
        }
        obj.pitch = pitch_v;
        obj.yaw   = yaw_v;
        obj.roll  = roll_v;
        obj.angle_type = "rad";

        obj.world_x = get_num(calib_params, "world_x", 0.0);
        obj.world_y = get_num(calib_params, "world_y", 0.0);
        obj.world_z = get_num(calib_params, "world_z", 0.0);

        obj.image_width  = get_int(calib_params, "image_width",  0);
        obj.image_height = get_int(calib_params, "image_height", 0);

        obj.focal_u = get_num(calib_params, "focal_u", 0.0);
        obj.focal_v = get_num(calib_params, "focal_v", 0.0);
        obj.center_u = get_num(calib_params, "center_u", 0.0);
        obj.center_v = get_num(calib_params, "center_v", 0.0);

        obj.fov = get_num(calib_params, "fov", 0.0);

        // R_wc (3x3)
        if (calib_params.contains("R_wc")) {
            const auto& R = calib_params.at("R_wc");
            obj.R_wc = cv::Matx33d(
                R.at(0).at(0).get<double>(), R.at(0).at(1).get<double>(), R.at(0).at(2).get<double>(),
                R.at(1).at(0).get<double>(), R.at(1).at(1).get<double>(), R.at(1).at(2).get<double>(),
                R.at(2).at(0).get<double>(), R.at(2).at(1).get<double>(), R.at(2).at(2).get<double>()
            );
        } else {
            obj.R_wc = cv::Matx33d::eye();
        }

        // t_wc (3)
        if (calib_params.contains("t_wc")) {
            const auto& t = calib_params.at("t_wc");
            obj.t_wc = cv::Vec3d(
                t.at(0).get<double>(),
                t.at(1).get<double>(),
                t.at(2).get<double>()
            );
        } else {
            obj.t_wc = cv::Vec3d(0,0,0);
        }

        obj.pinhole_distort = get_vecd(calib_params, "pinhole_distort");
        obj.fisheye_distort = get_vecd(calib_params, "fisheye_distort");

        return obj;
    }

    void save(const std::string& img_path,
              const std::string& calib_path = std::string()) const {
        if (!image.empty()) cv::imwrite(img_path, image);
        if (!calib_path.empty()) {
            nlohmann::json meta;
            meta["type"] = type;
            meta["world_x"] = world_x; meta["world_y"] = world_y; meta["world_z"] = world_z;
            meta["pitch"] = pitch; meta["yaw"] = yaw; meta["roll"] = roll;
            meta["angle_type"] = "rad";
            meta["image_type"] = image_type;
            meta["image_width"] = image_width;
            meta["image_height"] = image_height;
            meta["focal_u"] = focal_u; meta["focal_v"] = focal_v;
            meta["center_u"] = center_u; meta["center_v"] = center_v;
            meta["fov"] = fov;

            meta["R_wc"] = {
                { R_wc(0,0), R_wc(0,1), R_wc(0,2) },
                { R_wc(1,0), R_wc(1,1), R_wc(1,2) },
                { R_wc(2,0), R_wc(2,1), R_wc(2,2) }
            };
            meta["t_wc"] = { t_wc[0], t_wc[1], t_wc[2] };

            meta["pinhole_distort"] = pinhole_distort;
            meta["fisheye_distort"] = fisheye_distort;

            std::ofstream fout(calib_path);
            if (!fout) throw std::runtime_error("无法写入 calib JSON: " + calib_path);
            fout << meta.dump(4);
        }
    }
};

struct ImageProcessor {
    // 旋转矩阵（右手系，弧度）
    static cv::Matx33d _Rx(double roll) {
        const double c = std::cos(roll), s = std::sin(roll);
        return cv::Matx33d(
            1, 0, 0,
            0, c, -s,
            0, s,  c
        );
    }
    static cv::Matx33d _Ry(double pitch) {
        const double c = std::cos(pitch), s = std::sin(pitch);
        return cv::Matx33d(
             c, 0, s,
             0, 1, 0,
            -s, 0, c
        );
    }
    static cv::Matx33d _Rz(double yaw) {
        const double c = std::cos(yaw), s = std::sin(yaw);
        return cv::Matx33d(
            c, -s, 0,
            s,  c, 0,
            0,  0, 1
        );
    }

    // 主动外参矩阵 [R_act | t_act]，3x4（CV_64F）
    static cv::Mat _Rt_active(const CalibratedImage& cimg) {
        const cv::Matx33d R_passive = _Rz(cimg.yaw) * _Ry(cimg.pitch) * _Rx(cimg.roll);
        const cv::Matx33d R_act = R_passive.t();
        const cv::Vec3d   C(cimg.world_x, cimg.world_y, cimg.world_z);
        const cv::Vec3d   t_act = -(R_act * C);

        cv::Mat Rt(3, 4, CV_64F);
        for (int r=0; r<3; ++r) {
            Rt.at<double>(r,0) = R_act(r,0);
            Rt.at<double>(r,1) = R_act(r,1);
            Rt.at<double>(r,2) = R_act(r,2);
            Rt.at<double>(r,3) = t_act[r];
        }
        return Rt;
    }

    // 相机->像素 投影矩阵 P（3x4，CV_64F）
    static cv::Mat _P_cam2pixel(const CalibratedImage& cimg) {
        const double fu = cimg.focal_u, fv = cimg.focal_v;
        const double u0 = cimg.center_u, v0 = cimg.center_v;
        cv::Mat P = cv::Mat::zeros(3, 4, CV_64F);
        P.at<double>(0,0) = u0;  P.at<double>(0,1) = -fu; P.at<double>(0,2) = 0.0; P.at<double>(0,3)=0.0;
        P.at<double>(1,0) = v0;  P.at<double>(1,1) = 0.0; P.at<double>(1,2) = -fv; P.at<double>(1,3)=0.0;
        P.at<double>(2,0) = 1.0; P.at<double>(2,1) = 0.0; P.at<double>(2,2) = 0.0; P.at<double>(2,3)=0.0;
        return P;
    }

    // K 内参（3x3）
    static cv::Matx33d _K_intrinsic(double f_u, double f_v, double c_u, double c_v) {
        return cv::Matx33d(
            f_u, 0.0, c_u,
            0.0, f_v, c_v,
            0.0, 0.0, 1.0
        );
    }

    // 一致性/健壮性检查（打印结果，不抛错）
    static void check(const CalibratedImage& cimg,
                      double tol_img_px = 0.5,
                      double tol_pp_px  = 2.0,
                      double tol_det    = 1e-3,
                      double tol_orth   = 1e-3,
                      double tol_R      = 1e-2,
                      double tol_t      = 1e-3,
                      double tol_fov_deg= 2.0) {
        using std::cout; using std::endl;
        std::vector<std::string> issues;
        nlohmann::json details;

        // 1) 图像尺寸/类型
        const int H_decl = cimg.image_height, W_decl = cimg.image_width;
        if (cimg.image.empty()) {
            issues.emplace_back("image 为空。");
        } else {
            if (cimg.image.type() != CV_8UC3) {
                issues.emplace_back("image 类型不是 CV_8UC3。");
            }
            const int H_img = cimg.image.rows, W_img = cimg.image.cols;
            if (std::abs(H_img - H_decl) > tol_img_px || std::abs(W_img - W_decl) > tol_img_px) {
                issues.emplace_back("图像尺寸与声明不一致。");
            }
        }
        if (cimg.focal_u <= 0 || cimg.focal_v <= 0) {
            issues.emplace_back("焦距应为正数。");
        }
        if (!( -tol_pp_px <= cimg.center_u && cimg.center_u <= W_decl + tol_pp_px &&
               -tol_pp_px <= cimg.center_v && cimg.center_v <= H_decl + tol_pp_px )) {
            issues.emplace_back("主点 (u0,v0) 超出图像范围（含容忍）。");
        }
        if (cimg.image_type != "RGB" && cimg.image_type != "BGR") {
            issues.emplace_back("image_type 非法（应为 'RGB' 或 'BGR'）。");
        }

        // 3) R 的正交性
        {
            const cv::Matx33d& R = cimg.R_wc;
            cv::Matx33d RtR = R.t()*R;
            cv::Matx33d I = cv::Matx33d::eye();
            double orth_err = 0.0;
            for (int r=0;r<3;++r) for (int c=0;c<3;++c) {
                const double d = RtR(r,c) - I(r,c);
                orth_err += d*d;
            }
            orth_err = std::sqrt(orth_err);
            const double detR =
                R(0,0)*(R(1,1)*R(2,2)-R(1,2)*R(2,1)) -
                R(0,1)*(R(1,0)*R(2,2)-R(1,2)*R(2,0)) +
                R(0,2)*(R(1,0)*R(2,1)-R(1,1)*R(2,0));
            details["R_orth_err"] = orth_err;
            details["detR"] = detR;
            if (orth_err > tol_orth) issues.emplace_back("R 非正交。");
            if (std::abs(detR - 1.0) > tol_det) issues.emplace_back("det(R) 应接近 1。");
        }

        // 4) 与 _Rt_active 一致性
        {
            const cv::Matx33d R_passive = _Rz(cimg.yaw) * _Ry(cimg.pitch) * _Rx(cimg.roll);
            const cv::Matx33d R_act_expected = R_passive.t();
            const cv::Vec3d C(cimg.world_x, cimg.world_y, cimg.world_z);
            const cv::Vec3d t_act_expected = -(R_act_expected * C);

            details["R_act_expected"] = {
                { R_act_expected(0,0), R_act_expected(0,1), R_act_expected(0,2) },
                { R_act_expected(1,0), R_act_expected(1,1), R_act_expected(1,2) },
                { R_act_expected(2,0), R_act_expected(2,1), R_act_expected(2,2) }
            };
            details["t_act_expected"] = { t_act_expected[0], t_act_expected[1], t_act_expected[2] };

            // diff
            double diff_R = 0.0;
            for (int r=0;r<3;++r) for (int c=0;c<3;++c) {
                const double d = cimg.R_wc(r,c) - R_act_expected(r,c);
                diff_R += d*d;
            }
            diff_R = std::sqrt(diff_R);
            details["R_diff_act_fro"] = diff_R;
            if (diff_R > tol_R) issues.emplace_back("R 与期望 R_act 不一致。");

            const cv::Vec3d& t_given = cimg.t_wc;
            const double diff_t = std::sqrt( (t_given[0]-t_act_expected[0])*(t_given[0]-t_act_expected[0]) +
                                             (t_given[1]-t_act_expected[1])*(t_given[1]-t_act_expected[1]) +
                                             (t_given[2]-t_act_expected[2])*(t_given[2]-t_act_expected[2]) );
            details["t_diff_act_l2"] = diff_t;
            if (diff_t > tol_t) issues.emplace_back("t 与期望 t_act 不一致。");
        }

        if (cimg.angle_type != "rad") {
            issues.emplace_back("angle_type 应为 'rad'。");
        }

        // 7) 水平 FOV
        auto theta_distorted = [](double theta, const std::vector<double>& k) {
            double k1 = k.size()>0 ? k[0] : 0.0;
            double k2 = k.size()>1 ? k[1] : 0.0;
            double k3 = k.size()>2 ? k[2] : 0.0;
            double k4 = k.size()>3 ? k[3] : 0.0;
            double t2 = theta*theta;
            return theta * (1.0 + k1*t2 + k2*(t2*t2) + k3*(t2*t2*t2) + k4*(t2*t2*t2*t2));
        };
        auto largest_root_theta = [&](double r_pix, double fu, const std::vector<double>& k)->double {
            if (fu<=0 || r_pix<=0) return -1.0;
            auto g = [&](double th){ return fu*theta_distorted(th,k) - r_pix; };
            const int N = 2048;
            const double lo = 0.0, hi = M_PI - 1e-6;
            double theta_max = -1.0;
            double prev_th = lo, prev_val = g(prev_th);
            for (int i=1;i<=N;++i) {
                double th = lo + (hi-lo)*i/N;
                double val = g(th);
                if (std::abs(val) < 1e-10) theta_max = std::max(theta_max, th);
                if (prev_val * val < 0.0) {
                    double a = prev_th, b = th, fa = prev_val, fb = val;
                    for (int it=0; it<64; ++it) {
                        double m = 0.5*(a+b);
                        double fm = g(m);
                        if (fm == 0.0) { a=b=m; break; }
                        if (fa*fm <= 0) { b=m; fb=fm; } else { a=m; fa=fm; }
                    }
                    theta_max = std::max(theta_max, 0.5*(a+b));
                }
                prev_th = th; prev_val = val;
            }
            return theta_max;
        };

        const double fu = cimg.focal_u;
        const int W = W_decl;
        const double r_left  = cimg.center_u;
        const double r_right = std::max(0.0, (W-1) - cimg.center_u);
        const double r_pix_max = std::min(r_left, r_right);

        if (fu>0 && r_pix_max>0) {
            if (cimg.type == "pinhole") {
                const double fov_h_rad = 2.0 * std::atan((2.0*r_pix_max)/(2.0*fu));
                const double fov_h_deg = fov_h_rad * 180.0 / M_PI;
                details["fov_h_deg_calc"] = fov_h_deg;
                if (!(cimg.fov > 0.0)) {
                    issues.emplace_back("[pinhole] fov 未设置或为 0。");
                } else if (std::abs(fov_h_deg - cimg.fov) > tol_fov_deg) {
                    issues.emplace_back("[pinhole] 计算 FOV 与给定 fov 相差较大。");
                }
            } else if (cimg.type == "fisheye") {
                const double th_max = largest_root_theta(r_pix_max, fu, cimg.fisheye_distort);
                if (th_max < 0.0) {
                    issues.emplace_back("[fisheye] 未找到达到水平边界的 θ 根。");
                } else {
                    const double fov_h_deg = 2.0 * th_max * 180.0 / M_PI;
                    details["theta_max_rad"] = th_max;
                    details["fov_h_deg_calc"] = fov_h_deg;
                    if (!(fov_h_deg > 0.0 && fov_h_deg < 300.0)) {
                        issues.emplace_back("[fisheye] 计算的 FOV 看起来异常（应在 0–300° 之间，允许>180°）。");
                    }
                    if (!(cimg.fov > 0.0)) {
                        issues.emplace_back("[fisheye] fov 未设置或为 0。");
                    } else if (std::abs(fov_h_deg - cimg.fov) > tol_fov_deg) {
                        issues.emplace_back("[fisheye] 计算 FOV 与给定 fov 相差较大。");
                    }
                }
            }
        } else {
            issues.emplace_back("无法计算 FOV：f_u 或 r_pix_max 非法。");
        }

        // 打印
        cout << "=== CalibratedImage Check (严格遵从 _Rt_active / ZYX) ===\n";
        if (issues.empty()) {
            cout << "All good.\n";
        } else {
            cout << "发现 " << issues.size() << " 个问题。\n- Issues:\n";
            for (const auto& s : issues) cout << "  - " << s << "\n";
        }
        if (!details.is_null()) {
            cout << "- Details:\n" << details.dump(2) << "\n";
        }
    }

    // 画面重新定向（基于单应）
    static CalibratedImage reorient(const CalibratedImage& cimg,
                                    double new_yaw = 0.0,
                                    double new_pitch = 0.0,
                                    double new_roll = 0.0,
                                    std::pair<int,int> out_size = {-1,-1}) {
        const int W_new = (out_size.first  > 0) ? out_size.first  : cimg.image_width;
        const int H_new = (out_size.second > 0) ? out_size.second : cimg.image_height;

        const cv::Matx33d K = _K_intrinsic(cimg.focal_u, cimg.focal_v, cimg.center_u, cimg.center_v);
        const cv::Matx33d R_delta = _Rx(new_roll) * _Ry(new_pitch) * _Rz(new_yaw);

        // OpenCV 坐标（光轴 Z，右 X，下 Y）
        const cv::Matx33d Q( 0.0, -1.0, 0.0,
                             0.0,  0.0,-1.0,
                             1.0,  0.0, 0.0 );
        const cv::Matx33d R_rel_cv = Q * R_delta * Q.t();

        cv::Matx33d Kinv = K.inv();
        cv::Matx33d H = K * R_rel_cv * Kinv;

        cv::Mat img_rot;
        cv::warpPerspective(cimg.image, img_rot, cv::Mat(H), cv::Size(W_new, H_new),
                            cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0,0,0));

        CalibratedImage out = cimg;
        out.image = img_rot;
        out.image_width = W_new;
        out.image_height = H_new;
        // 主点此处保持不变（如需居中可改为 W_new/2, H_new/2）
        out.center_u = cimg.center_u;
        out.center_v = cimg.center_v;
        // 欧拉角（简单相对更新；严格姿态合成可按需替换）
        out.yaw   = cimg.yaw   - new_yaw;
        out.pitch = cimg.pitch - new_pitch;
        out.roll  = cimg.roll  - new_roll;
        return out;
    }

    // 基于深度的相机位移
    static CalibratedImage translate(const CalibratedImage& cimg,
                                     const cv::Mat& depth, // 单通道，m
                                     double t_x = 0.0,
                                     double t_y = 0.0,
                                     double t_z = 0.0,
                                     std::pair<int,int> out_size = {-1,-1}) {
        const int H = cimg.image_height, W = cimg.image_width;
        const int W_out = (out_size.first  > 0 ? out_size.first  : W);
        const int H_out = (out_size.second > 0 ? out_size.second : H);

        if (depth.empty() || depth.size() != cv::Size(W,H) || depth.channels()!=1) {
            throw std::invalid_argument("depth 需要与图像同尺寸的单通道 Mat");
        }

        const double u0 = cimg.center_u, v0 = cimg.center_v;
        const double fu = cimg.focal_u,  fv = cimg.focal_v;
        const float  eps = 1e-6f;

        std::vector<float> zbuf(H_out*W_out, std::numeric_limits<float>::infinity());
        cv::Mat new_img = cv::Mat::zeros(H_out, W_out, CV_8UC3);

        // 逐像素
        for (int v=0; v<H; ++v) {
            const uchar* src_row = cimg.image.ptr<uchar>(v);
            for (int u=0; u<W; ++u) {
                float X_old;
                if (depth.type()==CV_32F) X_old = depth.at<float>(v,u);
                else if (depth.type()==CV_64F) X_old = static_cast<float>(depth.at<double>(v,u));
                else continue;

                if (X_old <= eps) continue;

                const double Y_old = (u0 - u) / fu * X_old;
                const double Z_old = (v0 - v) / fv * X_old;

                const double Xn = X_old + t_x;
                const double Yn = Y_old + t_y;
                const double Zn = Z_old + t_z;
                if (Xn <= eps) continue;

                const int j = static_cast<int>(std::round(u0 - fu * (Yn / Xn)));
                const int i = static_cast<int>(std::round(v0 - fv * (Zn / Xn)));
                if (i<0 || i>=H_out || j<0 || j>=W_out) continue;

                const int idx = i*W_out + j;
                if (static_cast<float>(Xn) < zbuf[idx]) {
                    zbuf[idx] = static_cast<float>(Xn);
                    const uchar* psrc = &src_row[u*3];
                    uchar* pdst = new_img.ptr<uchar>(i) + j*3;
                    pdst[0]=psrc[0]; pdst[1]=psrc[1]; pdst[2]=psrc[2];
                }
            }
        }

        // inpaint 孔洞
        cv::Mat mask;
        cv::inRange(new_img, cv::Scalar(0,0,0), cv::Scalar(0,0,0), mask);
        if (cv::countNonZero(mask)>0) {
            cv::inpaint(new_img, mask, new_img, 3.0, cv::INPAINT_NS);
        }

        CalibratedImage out = cimg;
        out.image = new_img;
        out.image_width = W_out;
        out.image_height = H_out;
        out.world_x = cimg.world_x - t_x;
        out.world_y = cimg.world_y - t_y;
        out.world_z = cimg.world_z - t_z;
        return out;
    }

    // 按比例裁剪并更新主点
    static CalibratedImage crop(const CalibratedImage& cimg,
                                double t=0.0, double b=0.0,
                                double l=0.0, double r=0.0) {
        if (!(0.0<=t && t<=1.0 && 0.0<=b && b<=1.0 && 0.0<=l && l<=1.0 && 0.0<=r && r<=1.0)) {
            throw std::invalid_argument("裁剪比例需在 [0,1] 内");
        }
        if (t+b >= 1.0 || l+r >= 1.0) {
            throw std::invalid_argument("上下或左右裁剪比例之和须 < 1");
        }
        const int w=cimg.image_width, h=cimg.image_height;
        const int tp = int(std::round(t*h));
        const int bp = int(std::round(b*h));
        const int lp = int(std::round(l*w));
        const int rp = int(std::round(r*w));

        const int y0 = tp, y1 = h - bp;
        const int x0 = lp, x1 = w - rp;

        cv::Rect roi(x0, y0, x1-x0, y1-y0);
        cv::Mat img2 = cimg.image(roi).clone();

        CalibratedImage out = cimg;
        out.image = img2;
        out.center_u = cimg.center_u - x0;
        out.center_v = cimg.center_v - y0;
        out.image_width  = roi.width;
        out.image_height = roi.height;
        return out;
    }

    // 把主点移动到 (target_u, target_v) 的裁剪
    static CalibratedImage crop_by_point(const CalibratedImage& cimg,
                                         double target_u, double target_v) {
        const int W = cimg.image_width, H = cimg.image_height;
        const double delta_u = cimg.center_u - target_u;
        const double delta_v = cimg.center_v - target_v;

        const double l = std::max(delta_u, 0.0) / W;
        const double r = std::max(-delta_u, 0.0) / W;
        const double t = std::max(delta_v, 0.0) / H;
        const double b = std::max(-delta_v, 0.0) / H;

        if (l+r >= 1.0 || t+b >= 1.0) {
            throw std::invalid_argument("目标主点过近边界，无法裁剪");
        }
        return crop(cimg, t, b, l, r);
    }

    // 缩放图片与内参
    static CalibratedImage scale(const CalibratedImage& cimg, int new_h, int new_w) {
        const int old_w = cimg.image_width,  old_h = cimg.image_height;
        if (new_w<=0 || new_h<=0) throw std::invalid_argument("new_w/new_h 必须为正整数");
        if (new_w==old_w && new_h==old_h) return cimg;

        const double scale_w = double(new_w) / old_w;
        const double scale_h = double(new_h) / old_h;

        // 选择插值
        const bool up_h = new_h > old_h, up_w = new_w > old_w;
        int interp = cv::INTER_LINEAR;
        if (up_h && up_w) {
            const double sf = std::max(scale_w, scale_h);
            interp = (sf >= 1.5) ? cv::INTER_LANCZOS4 : cv::INTER_CUBIC;
        } else if (!up_h && !up_w) {
            interp = cv::INTER_AREA;
        }

        cv::Mat img_resized;
        cv::resize(cimg.image, img_resized, cv::Size(new_w, new_h), 0, 0, interp);

        CalibratedImage out = cimg;
        out.image = img_resized;
        out.focal_u = cimg.focal_u * scale_w;
        out.focal_v = cimg.focal_v * scale_h;
        out.center_u = cimg.center_u * scale_w;
        out.center_v = cimg.center_v * scale_h;
        out.image_width  = new_w;
        out.image_height = new_h;
        return out;
    }

    // 数据打包：HWC(BGR) → (可选)RGB → 标准化 → [1,3,H,W] (CV_32F, [0,1])
    static cv::Mat todata(const CalibratedImage& cimg,
                          bool to_rgb = true,
                          const std::array<float,3>& mean = {0.f,0.f,0.f},
                          const std::array<float,3>& stdv = {1.f,1.f,1.f}) {
        cv::Mat hwc_f32;
        cimg.image.convertTo(hwc_f32, CV_32F, 1.0/255.0);

        if (to_rgb) {
            cv::cvtColor(hwc_f32, hwc_f32, cv::COLOR_BGR2RGB);
        }

        std::vector<cv::Mat> ch(3);
        cv::split(hwc_f32, ch);
        for (int c=0; c<3; ++c) {
            ch[c] = (ch[c] - mean[c]) / stdv[c];
        }

        int sizes[4] = {1, 3, cimg.image_height, cimg.image_width};
        cv::Mat tensor(4, sizes, CV_32F);
        for (int c=0; c<3; ++c) {
            cv::Mat plane(cimg.image_height, cimg.image_width, CV_32F,
                          tensor.ptr(0, c), tensor.step[2]);
            ch[c].copyTo(plane);
        }
        return tensor; // [1,3,H,W]
    }

    // 相机点 → 像素 uv（无绘制）。输入 Nx3 CV_64F，输出 Nx2 CV_32S（-1,-1 表示不可见）
    static cv::Mat cam_points_to_uv(const CalibratedImage& cimg,
                                    const cv::Mat& cam_points) {
        if (cam_points.empty() || cam_points.cols != 3 || cam_points.type()!=CV_64F) {
            throw std::invalid_argument("cam_points 需为 Nx3, CV_64F");
        }
        const int N = cam_points.rows;
        cv::Mat uv(N, 2, CV_32S, cv::Scalar(-1, -1));

        cv::Mat P = _P_cam2pixel(cimg); // 3x4
        for (int i=0; i<N; ++i) {
            const double X = cam_points.at<double>(i,0);
            const double Y = cam_points.at<double>(i,1);
            const double Z = cam_points.at<double>(i,2);
            const double u_h = P.at<double>(0,0)*X + P.at<double>(0,1)*Y + P.at<double>(0,2)*Z + P.at<double>(0,3);
            const double v_h = P.at<double>(1,0)*X + P.at<double>(1,1)*Y + P.at<double>(1,2)*Z + P.at<double>(1,3);
            const double w_h = P.at<double>(2,0)*X + P.at<double>(2,1)*Y + P.at<double>(2,2)*Z + P.at<double>(2,3);
            if (std::isfinite(w_h) && w_h > 0) {
                int u = (int)std::lround(u_h / w_h);
                int v = (int)std::lround(v_h / w_h);
                uv.at<int>(i,0) = u;
                uv.at<int>(i,1) = v;
            }
        }
        return uv;
    }

    // 世界点 → 像素 uv
    static cv::Mat world_points_to_uv(const CalibratedImage& cimg,
                                      const cv::Mat& world_points) {
        if (world_points.empty() || world_points.cols != 3 || world_points.type()!=CV_64F) {
            throw std::invalid_argument("world_points 需为 Nx3, CV_64F");
        }
        const int N = world_points.rows;

        cv::Mat Rt = _Rt_active(cimg); // 3x4
        cv::Mat uv(N, 2, CV_32S, cv::Scalar(-1, -1));

        cv::Mat P = _P_cam2pixel(cimg); // 3x4
        for (int i=0; i<N; ++i) {
            const double Xw = world_points.at<double>(i,0);
            const double Yw = world_points.at<double>(i,1);
            const double Zw = world_points.at<double>(i,2);

            // 相机系点
            const double X = Rt.at<double>(0,0)*Xw + Rt.at<double>(0,1)*Yw + Rt.at<double>(0,2)*Zw + Rt.at<double>(0,3);
            const double Y = Rt.at<double>(1,0)*Xw + Rt.at<double>(1,1)*Yw + Rt.at<double>(1,2)*Zw + Rt.at<double>(1,3);
            const double Z = Rt.at<double>(2,0)*Xw + Rt.at<double>(2,1)*Yw + Rt.at<double>(2,2)*Zw + Rt.at<double>(2,3);

            // 像素
            const double u_h = P.at<double>(0,0)*X + P.at<double>(0,1)*Y + P.at<double>(0,2)*Z + P.at<double>(0,3);
            const double v_h = P.at<double>(1,0)*X + P.at<double>(1,1)*Y + P.at<double>(1,2)*Z + P.at<double>(1,3);
            const double w_h = P.at<double>(2,0)*X + P.at<double>(2,1)*Y + P.at<double>(2,2)*Z + P.at<double>(2,3);

            if (std::isfinite(w_h) && w_h > 0) {
                int u = (int)std::lround(u_h / w_h);
                int v = (int)std::lround(v_h / w_h);
                uv.at<int>(i,0) = u;
                uv.at<int>(i,1) = v;
            }
        }
        return uv;
    }
};
