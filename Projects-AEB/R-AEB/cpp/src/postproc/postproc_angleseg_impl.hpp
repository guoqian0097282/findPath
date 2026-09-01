#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <limits>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include <nlohmann/json.hpp>

#include "common/logger.hpp"
#include "common/ops.hpp"
#include "common/vis.hpp"
#include "postproc/postproc_nms_impl.hpp"

struct AngleFieldPostProcConfig {
    bool enabled = true;
    int column = 6;
    std::string input_unit = "rad";  // "rad" | "deg"
    float scale = 1.0f;
    float offset = 0.0f;
    bool invert = false;
    std::string wrap_mode = "pi";    // "pi" | "2pi" | "half_pi"
    std::optional<float> clip_min;
    std::optional<float> clip_max;

    std::string decode_mode = "auto";  // "auto" | "bins_residual" | "none"
    int angle_bins = 8;
    std::string raw_layout = "auto";   // "auto" | "2k_l" | "l_2k"
    bool raw_logits_first = true;
};

class AngleFieldPostProcessor {
public:
    explicit AngleFieldPostProcessor(AngleFieldPostProcConfig cfg = {}) : cfg_(std::move(cfg)) {}

    static AngleFieldPostProcConfig FromJson(const nlohmann::json& j) {
        AngleFieldPostProcConfig cfg;
        if (!j.is_object()) return cfg;

        auto get_bool = [&](const char* key, bool def) -> bool {
            if (!j.contains(key)) return def;
            const auto& v = j.at(key);
            if (v.is_boolean()) return v.get<bool>();
            if (v.is_number_integer()) return v.get<int>() != 0;
            if (v.is_string()) {
                const std::string s = v.get<std::string>();
                if (s == "1" || s == "true" || s == "TRUE" || s == "on") return true;
                if (s == "0" || s == "false" || s == "FALSE" || s == "off") return false;
            }
            return def;
        };
        auto get_int = [&](const char* key, int def) -> int {
            if (!j.contains(key)) return def;
            const auto& v = j.at(key);
            if (v.is_number_integer()) return v.get<int>();
            return def;
        };
        auto get_float = [&](const char* key, float def) -> float {
            if (!j.contains(key)) return def;
            const auto& v = j.at(key);
            if (v.is_number()) {
                const float f = v.get<float>();
                return std::isfinite(f) ? f : def;
            }
            return def;
        };
        auto get_string = [&](const char* key, const std::string& def) -> std::string {
            if (!j.contains(key)) return def;
            const auto& v = j.at(key);
            if (v.is_string()) return v.get<std::string>();
            return def;
        };

        cfg.enabled = get_bool("enabled", cfg.enabled);
        cfg.column = get_int("column", cfg.column);
        cfg.input_unit = get_string("input_unit", cfg.input_unit);
        cfg.scale = get_float("scale", cfg.scale);
        cfg.offset = get_float("offset", cfg.offset);
        cfg.invert = get_bool("invert", cfg.invert);
        cfg.wrap_mode = get_string("wrap_mode", cfg.wrap_mode);

        cfg.decode_mode = get_string("decode_mode", cfg.decode_mode);
        cfg.angle_bins = std::max(1, get_int("angle_bins", cfg.angle_bins));
        cfg.raw_layout = get_string("raw_layout", cfg.raw_layout);
        cfg.raw_logits_first = get_bool("raw_logits_first", cfg.raw_logits_first);

        if (j.contains("clip_min") && j.at("clip_min").is_number()) {
            const float v = j.at("clip_min").get<float>();
            if (std::isfinite(v)) cfg.clip_min = v;
        }
        if (j.contains("clip_max") && j.at("clip_max").is_number()) {
            const float v = j.at("clip_max").get<float>();
            if (std::isfinite(v)) cfg.clip_max = v;
        }

        if (cfg.input_unit != "rad" && cfg.input_unit != "deg") cfg.input_unit = "rad";
        if (cfg.wrap_mode != "pi" && cfg.wrap_mode != "2pi" && cfg.wrap_mode != "half_pi") cfg.wrap_mode = "pi";
        if (cfg.decode_mode != "auto" && cfg.decode_mode != "bins_residual" && cfg.decode_mode != "none") cfg.decode_mode = "auto";
        if (cfg.raw_layout != "auto" && cfg.raw_layout != "2k_l" && cfg.raw_layout != "l_2k") cfg.raw_layout = "auto";
        if (cfg.clip_min.has_value() && cfg.clip_max.has_value() && *cfg.clip_min > *cfg.clip_max) {
            std::swap(cfg.clip_min, cfg.clip_max);
        }
        return cfg;
    }

    cv::Mat DecodeAngle(const cv::Mat& angle) const {
        if (cfg_.decode_mode == "none") {
            return FlattenToVector(angle);
        }
        if (cfg_.decode_mode == "bins_residual") {
            return DecodeBinsResidual(angle);
        }

        // auto
        if (angle.dims == 2) {
            const int r = angle.rows;
            const int c = angle.cols;
            const int k2 = 2 * std::max(1, cfg_.angle_bins);
            if (r == k2 || c == k2 || (r % 2 == 0 && r >= 4) || (c % 2 == 0 && c >= 4)) {
                return DecodeBinsResidual(angle);
            }
        }
        return FlattenToVector(angle);
    }

    void ProcessInplace(cv::Mat& objs) const {
        constexpr float kPi = 3.14159265358979323846f;
        if (!cfg_.enabled) return;
        if (objs.empty() || objs.dims != 2) return;
        if (cfg_.column < 0 || cfg_.column >= objs.cols) return;
        if (objs.type() != CV_32F && objs.type() != CV_64F) return;

        const bool input_deg = (cfg_.input_unit == "deg");
        const bool wrap_2pi = (cfg_.wrap_mode == "2pi");
        const bool wrap_half_pi = (cfg_.wrap_mode == "half_pi");

        for (int r = 0; r < objs.rows; ++r) {
            float theta = 0.0f;
            if (objs.type() == CV_32F) theta = objs.at<float>(r, cfg_.column);
            else theta = static_cast<float>(objs.at<double>(r, cfg_.column));

            if (input_deg) theta = theta * kPi / 180.0f;
            theta = theta * cfg_.scale + cfg_.offset;
            if (cfg_.invert) theta = -theta;

            if (cfg_.clip_min.has_value()) theta = std::max(theta, *cfg_.clip_min);
            if (cfg_.clip_max.has_value()) theta = std::min(theta, *cfg_.clip_max);
            theta = wrap_half_pi ? WrapToHalfPi(theta) : (wrap_2pi ? WrapTo2Pi(theta) : WrapToPi(theta));

            if (objs.type() == CV_32F) objs.at<float>(r, cfg_.column) = theta;
            else objs.at<double>(r, cfg_.column) = static_cast<double>(theta);
        }
    }

private:
    static float WrapToPi(float a) {
        constexpr float kPi = 3.14159265358979323846f;
        constexpr float kTwoPi = 2.0f * kPi;
        if (!std::isfinite(a)) return 0.0f;
        a = std::fmod(a + kPi, kTwoPi);
        if (a < 0.0f) a += kTwoPi;
        return a - kPi;
    }

    static float WrapTo2Pi(float a) {
        constexpr float kPi = 3.14159265358979323846f;
        constexpr float kTwoPi = 2.0f * kPi;
        if (!std::isfinite(a)) return 0.0f;
        a = std::fmod(a, kTwoPi);
        if (a < 0.0f) a += kTwoPi;
        return a;
    }

    static float WrapToHalfPi(float a) {
        constexpr float kPi = 3.14159265358979323846f;
        constexpr float kHalfPi = 0.5f * kPi;
        if (!std::isfinite(a)) return 0.0f;
        a = std::fmod(a + kHalfPi, kPi);
        if (a < 0.0f) a += kPi;
        return a - kHalfPi;
    }

    cv::Mat FlattenToVector(const cv::Mat& angle) const {
        cv::Mat a;
        if (angle.type() == CV_32F) a = angle;
        else angle.convertTo(a, CV_32F);
        return a.reshape(1, 1).clone();
    }

    cv::Mat DecodeBinsResidual(const cv::Mat& angle) const {
        constexpr float kPi = 3.14159265358979323846f;
        constexpr float kTwoPi = 2.0f * kPi;

        cv::Mat a;
        if (angle.type() == CV_32F) a = angle;
        else angle.convertTo(a, CV_32F);

        if (a.dims != 2) return FlattenToVector(a);

        int rows = a.rows;
        int cols = a.cols;
        int Kcfg = std::max(1, cfg_.angle_bins);

        std::string layout = cfg_.raw_layout;
        if (layout == "auto") {
            if (rows == 2 * Kcfg) layout = "2k_l";
            else if (cols == 2 * Kcfg) layout = "l_2k";
            else if (rows % 2 == 0 && rows >= 4) layout = "2k_l";
            else if (cols % 2 == 0 && cols >= 4) layout = "l_2k";
            else return FlattenToVector(a);
        }

        int K = 0;
        int L = 0;
        cv::Mat logits, res;

        if (layout == "2k_l") {
            K = rows / 2;
            if (K <= 0 || rows != 2 * K) return FlattenToVector(a);
            L = cols;
            if (cfg_.raw_logits_first) {
                logits = a.rowRange(0, K);
                res = a.rowRange(K, 2 * K);
            } else {
                res = a.rowRange(0, K);
                logits = a.rowRange(K, 2 * K);
            }
        } else {
            K = cols / 2;
            if (K <= 0 || cols != 2 * K) return FlattenToVector(a);
            L = rows;
            if (cfg_.raw_logits_first) {
                logits = a.colRange(0, K).t();
                res = a.colRange(K, 2 * K).t();
            } else {
                res = a.colRange(0, K).t();
                logits = a.colRange(K, 2 * K).t();
            }
        }

        cv::Mat out(1, L, CV_32F);
        const float delta = kTwoPi / static_cast<float>(K);
        const float half = 0.5f * delta;

        for (int j = 0; j < L; ++j) {
            int k_star = 0;
            float v_max = logits.at<float>(0, j);
            for (int k = 1; k < K; ++k) {
                float v = logits.at<float>(k, j);
                if (v > v_max) {
                    v_max = v;
                    k_star = k;
                }
            }
            float r = res.at<float>(k_star, j);
            r = std::max(-1.0f, std::min(1.0f, r));
            float c = kPi - static_cast<float>(k_star) * delta;
            out.at<float>(0, j) = WrapToPi(c + r * half);
        }
        return out;
    }

private:
    AngleFieldPostProcConfig cfg_;
};

class AngleSegPostProcessor
{
public:
    struct Result
    {
        cv::Mat objs;  // [N,7] CV_32F
        cv::Mat masks; // [N,H,W] CV_8U
    };

    explicit AngleSegPostProcessor(
        int nc = 80,
        int nm = 32,
        const nlohmann::json::object_t& id2name_json = nlohmann::json::object_t{}, // {"0":"person", ...}
        const nlohmann::json::object_t& whitelist_json = nlohmann::json::object_t{}, // {"0":{...}, "3":{...}}
        float conf_thresh = 0.5f,
        float iou_thresh = 0.3f,
        float mask_thresh = 0.5f,
        int mask_up = 4,
        const nlohmann::json::object_t& angle_postproc_json = nlohmann::json::object_t{}
    )
        : nc_(int(nc))
          , nm_(int(nm))
          , conf_thr_(float(conf_thresh))
          , iou_thr_(float(iou_thresh))
          , mask_thr_(float(mask_thresh))
          , mask_up_(int(mask_up))
          , angle_postproc_(AngleFieldPostProcessor::FromJson(nlohmann::json(angle_postproc_json)))
    {
        if (mask_up_ <= 0) throw std::invalid_argument("mask_up must be positive");

        for (const auto& kv : id2name_json)
        {
            int cid = 0;
            try { cid = std::stoi(kv.first); }
            catch (...) { throw std::runtime_error("id2name_json key must be int-like string"); }
            if (!kv.second.is_string()) throw std::runtime_error("id2name_json value must be string");
            id2name_[cid] = kv.second.get<std::string>();
        }

        std::unordered_map<std::string, int> canonical_name_to_id;
        canonical_name_to_id.reserve(id2name_.size());
        for (const auto& kv : id2name_)
        {
            const std::string target_name = merge_target_name(kv.second);
            if (kv.second == target_name && canonical_name_to_id.find(target_name) == canonical_name_to_id.end())
            {
                canonical_name_to_id[target_name] = kv.first;
            }
        }
        for (const auto& kv : id2name_)
        {
            const std::string target_name = merge_target_name(kv.second);
            auto it = canonical_name_to_id.find(target_name);
            if (it != canonical_name_to_id.end() && it->second != kv.first)
            {
                class_alias_[kv.first] = it->second;
            }
        }

        for (const auto& kv : whitelist_json)
        {
            int cid = 0;
            try { cid = std::stoi(kv.first); }
            catch (...) { throw std::runtime_error("whitelist_json key must be int-like string"); }
            whitelist_ids_.insert(cid);
        }
        std::unordered_set<int> scan_class_ids = whitelist_ids_;
        for (const auto& kv : class_alias_)
        {
            const int src_id = kv.first;
            const int dst_id = kv.second;
            if (whitelist_ids_.find(dst_id) != whitelist_ids_.end())
            {
                scan_class_ids.insert(src_id);
            }
        }

        whitelist_class_ids_.reserve(scan_class_ids.size());
        for (int cid : scan_class_ids)
        {
            if (cid >= 0 && cid < nc_)
            {
                whitelist_class_ids_.push_back(cid);
            }
        }
        std::sort(whitelist_class_ids_.begin(), whitelist_class_ids_.end());
    }


    cv::Mat postprocess(
        const cv::Mat& det_cat, // [C,L], CV_32F
        const cv::Mat& proto, // 3D: [nm,Hm,Wm] or [Hm,Wm,nm], CV_32F
        const cv::Mat& angle, // raw angle tensor (layout decided by angle_postproc config), CV_32F
        bool proto_chw = true // true=[nm,Hm,Wm], false=[Hm,Wm,nm]
    ) const
    {
        return postprocess_impl(det_cat, proto, angle, proto_chw, false).objs;
    }

    Result postprocess_with_masks(
        const cv::Mat& det_cat,
        const cv::Mat& proto,
        const cv::Mat& angle,
        bool proto_chw = true
    ) const
    {
        return postprocess_impl(det_cat, proto, angle, proto_chw, true);
    }

private:
    Result emptyResult() const
    {
        return Result{cv::Mat(0, 7, CV_32F), cv::Mat()};
    }

    Result postprocess_impl(
        const cv::Mat& det_cat,
        const cv::Mat& proto,
        const cv::Mat& angle,
        bool proto_chw,
        bool return_masks
    ) const
    {
        if (!(det_cat.type() == CV_32F && det_cat.dims == 2))
            throw std::runtime_error("det_cat must be [C,L] CV_32F");
        if (!(det_cat.rows > 4 + nm_))
            throw std::runtime_error("invalid det_cat layout: need C > 4 + configured nm");

        const int cls_end = det_cat.rows - nm_;
        return postprocess_det_views(
            det_cat.rowRange(0, 4),
            det_cat.rowRange(4, cls_end),
            det_cat.rowRange(cls_end, det_cat.rows),
            proto,
            angle,
            proto_chw,
            return_masks
        );
    }

    Result postprocess_det_views(
        const cv::Mat& box_rows, // [4,L], CV_32F
        const cv::Mat& cls_rows, // [nc,L], CV_32F
        const cv::Mat& mask_rows, // [nm,L], CV_32F
        const cv::Mat& proto, // 3D: [nm,Hm,Wm] or [Hm,Wm,nm], CV_32F
        const cv::Mat& angle, // raw angle tensor (layout decided by angle_postproc config), CV_32F
        bool proto_chw, // true=[nm,Hm,Wm], false=[Hm,Wm,nm]
        bool return_masks
    ) const
    {
        if (!(box_rows.type() == CV_32F && box_rows.dims == 2 && box_rows.rows == 4))
            throw std::runtime_error("det_cat box rows must be [4,L] CV_32F");
        if (!(cls_rows.type() == CV_32F && cls_rows.dims == 2 && cls_rows.rows > 0))
            throw std::runtime_error("det_cat class rows must be [nc,L] CV_32F");
        if (!(mask_rows.type() == CV_32F && mask_rows.dims == 2 && mask_rows.rows > 0))
            throw std::runtime_error("det_cat mask rows must be [nm,L] CV_32F");
        if (!(box_rows.cols == cls_rows.cols && box_rows.cols == mask_rows.cols))
            throw std::runtime_error("det_cat row views L dimension mismatch");
        if (!(proto.type() == CV_32F && proto.dims == 3))
            throw std::runtime_error("proto must be 3D CV_32F");
        if (angle.type() != CV_32F)
            throw std::runtime_error("angle must be CV_32F");

        const int L = box_rows.cols;
        const int nc = cls_rows.rows;

        if (!proto.isContinuous())
            throw std::runtime_error("proto must be continuous");

        // ---- proto 形状按传入 layout 解读 ----
        bool proto_chw_resolved = proto_chw; // fallback hint
        if (proto.size[0] == nm_)
        {
            proto_chw_resolved = true;
        }
        else if (proto.size[2] == nm_)
        {
            proto_chw_resolved = false;
        }
        else
        {
            proto_chw_resolved = (proto.size[0] <= proto.size[2]);
        }

        int nm = 0, Hm = 0, Wm = 0;
        if (proto_chw_resolved)
        {
            // [nm,Hm,Wm]
            nm = proto.size[0];
            Hm = proto.size[1];
            Wm = proto.size[2];
        }
        else
        {
            // [Hm,Wm,nm]
            Hm = proto.size[0];
            Wm = proto.size[1];
            nm = proto.size[2];
        }

        if (!(nm > 0 && Hm > 0 && Wm > 0))
            throw std::runtime_error("invalid proto shape");
        if (mask_rows.rows != nm)
            throw std::runtime_error("det_cat mask rows must match proto mask channels");

        const int H0 = Hm * mask_up_;
        const int W0 = Wm * mask_up_;
        if (!(H0 > 0 && W0 > 0))
            throw std::runtime_error("invalid output mask size");

        struct ThreadLocalWorkspace {
            cv::Mat box0;                  // (N0,4) CV_32F
            cv::Mat coeff0;                // (N0,nm) CV_32F
            cv::Mat xyxy;                  // (N,4) CV_32F
            cv::Mat coeff;                 // (N,nm) CV_32F
            cv::Mat xyxy_refined;          // (N,4) CV_32F
            std::vector<float> cls_conf;
            std::vector<int> cls_id;
            std::vector<float> angle_full;
            std::vector<int> pre_keep;
            std::vector<const float*> coeff_rows;
            std::vector<float> conf0;
            std::vector<int> id0;
            std::vector<float> ang0;
            std::vector<float> confm;
            std::vector<int> idm;
            std::vector<float> angm;
            std::vector<int> keep_after_lr;
            std::vector<int> x0_map;
            std::vector<int> x1_map;
            std::vector<float> wx0_map;
            std::vector<float> wx1_map;
        };
        static thread_local ThreadLocalWorkspace ws;

        // =========================
        // 1) cls argmax + angle flatten
        // =========================
        std::vector<float>& cls_conf = ws.cls_conf;
        std::vector<int>& cls_id = ws.cls_id;
        cls_conf.resize((size_t)L);
        cls_id.resize((size_t)L);

        // No whitelist: keep the old behavior and find the max class over all nc rows.
        // With whitelist: only scan whitelisted class rows to avoid reading the full class tensor.
        if (whitelist_ids_.empty())
        {
            const float* row0 = cls_rows.ptr<float>(0);
            std::memcpy(cls_conf.data(), row0, (size_t)L * sizeof(float));
            std::fill(cls_id.begin(), cls_id.end(), 0);

            for (int c = 1; c < nc; ++c)
            {
                const float* row = cls_rows.ptr<float>(c);
                for (int i = 0; i < L; ++i)
                {
                    const float v = row[i];
                    if (v > cls_conf[(size_t)i])
                    {
                        cls_conf[(size_t)i] = v;
                        cls_id[(size_t)i] = c;
                    }
                }
            }
        }
        else
        {
            int first_c = -1;
            for (int cid : whitelist_class_ids_)
            {
                if (cid >= 0 && cid < nc)
                {
                    first_c = cid;
                    break;
                }
            }
            if (first_c < 0)
            {
                return emptyResult();
            }

            const float* row0 = cls_rows.ptr<float>(first_c);
            std::memcpy(cls_conf.data(), row0, (size_t)L * sizeof(float));
            std::fill(cls_id.begin(), cls_id.end(), first_c);

            for (int cid : whitelist_class_ids_)
            {
                if (cid == first_c || cid < 0 || cid >= nc)
                {
                    continue;
                }

                const float* row = cls_rows.ptr<float>(cid);
                for (int i = 0; i < L; ++i)
                {
                    const float v = row[i];
                    if (v > cls_conf[(size_t)i])
                    {
                        cls_conf[(size_t)i] = v;
                        cls_id[(size_t)i] = cid;
                    }
                }
            }
        }

        // ---- angle decode + flatten ----
        cv::Mat angle_decoded = angle_postproc_.DecodeAngle(angle);
        if ((int)angle_decoded.total() != L)
            throw std::runtime_error("decoded angle length mismatch with det_cat L");

        cv::Mat angle_vec = angle_decoded.reshape(1, 1);
        if (!angle_vec.isContinuous())
            angle_vec = angle_vec.clone();

        std::vector<float>& angle_full = ws.angle_full;
        angle_full.resize((size_t)L);
        std::memcpy(angle_full.data(), angle_vec.ptr<float>(0), (size_t)L * sizeof(float));

        // ---- conf + whitelist 预筛 ----
        std::vector<int>& pre_keep = ws.pre_keep;
        pre_keep.clear();
        pre_keep.reserve((size_t)L);
        for (int i = 0; i < L; ++i)
        {
            if (!(cls_conf[(size_t)i] > conf_thr_)) continue;
            pre_keep.push_back(i);
        }
        if (pre_keep.empty()) return emptyResult();

        // =========================
        // 2) gather + NMS
        // =========================
        const int N0 = (int)pre_keep.size();

        cv::Mat& box0 = ws.box0;
        cv::Mat& coeff0 = ws.coeff0;
        box0.create(N0, 4, CV_32F);
        coeff0.create(N0, nm, CV_32F);
        std::vector<float>& conf0 = ws.conf0;
        std::vector<int>& id0 = ws.id0;
        std::vector<float>& ang0 = ws.ang0;
        conf0.resize((size_t)N0);
        id0.resize((size_t)N0);
        ang0.resize((size_t)N0);

        // 预取 det_cat 各 row 指针
        const float* row_x = box_rows.ptr<float>(0);
        const float* row_y = box_rows.ptr<float>(1);
        const float* row_w = box_rows.ptr<float>(2);
        const float* row_h = box_rows.ptr<float>(3);

        std::vector<const float*>& coeff_rows = ws.coeff_rows;
        coeff_rows.resize((size_t)nm);
        for (int cch = 0; cch < nm; ++cch)
        {
            coeff_rows[(size_t)cch] = mask_rows.ptr<float>(cch);
        }

        for (int r = 0; r < N0; ++r)
        {
            const int i = pre_keep[(size_t)r];

            float* bdst = box0.ptr<float>(r);
            bdst[0] = row_x[i];
            bdst[1] = row_y[i];
            bdst[2] = row_w[i];
            bdst[3] = row_h[i];

            float* cdst = coeff0.ptr<float>(r);
            for (int cch = 0; cch < nm; ++cch)
            {
                cdst[cch] = coeff_rows[(size_t)cch][i];
            }

            conf0[(size_t)r] = cls_conf[(size_t)i];
            id0[(size_t)r] = canonical_class_id(cls_id[(size_t)i]);
            ang0[(size_t)r] = angle_full[(size_t)i];
        }

        // cxcywh -> xyxy
        cv::Mat xyxy0 = xywh2xyxy(box0);

        // NMS
        cv::Mat scores0(N0, 1, CV_32F, conf0.data()); // view
        cv::Mat keep_mat = NumpyNMS::fast_nms(
            xyxy0,
            scores0,
            iou_thr_,
            /*use_triu=*/true,
            /*exit_early=*/true
        );

        std::vector<int> keep_idx;
        keep_idx.reserve((size_t)keep_mat.total());
        if (!keep_mat.empty())
        {
            if (keep_mat.dims == 2 && keep_mat.type() == CV_32S &&
                (keep_mat.cols == 1 || keep_mat.rows == 1))
            {
                const int K = (int)keep_mat.total();
                for (int k = 0; k < K; ++k)
                {
                    int v = (keep_mat.rows == 1)
                                ? keep_mat.at<int>(0, k)
                                : keep_mat.at<int>(k, 0);
                    keep_idx.push_back(v);
                }
            }
            else
            {
                cv::Mat flat = keep_mat.reshape(1, (int)keep_mat.total());
                for (int k = 0; k < flat.rows; ++k)
                    keep_idx.push_back(flat.at<int>(k, 0));
            }
        }

        if (keep_idx.empty()) return emptyResult();
        constexpr int kMaxKeepAfterNms = 5;
        if ((int)keep_idx.size() > kMaxKeepAfterNms)
        {
            keep_idx.resize((size_t)kMaxKeepAfterNms);
        }

        const int N = (int)keep_idx.size();
        cv::Mat& xyxy = ws.xyxy;
        cv::Mat& coeff = ws.coeff;
        xyxy.create(N, 4, CV_32F);
        coeff.create(N, nm, CV_32F);
        std::vector<float>& confm = ws.confm;
        std::vector<int>& idm = ws.idm;
        std::vector<float>& angm = ws.angm;
        confm.resize((size_t)N);
        idm.resize((size_t)N);
        angm.resize((size_t)N);

        for (int k = 0; k < N; ++k)
        {
            const int i = keep_idx[(size_t)k];
            std::memcpy(xyxy.ptr<float>(k), xyxy0.ptr<float>(i), 4 * sizeof(float));
            std::memcpy(coeff.ptr<float>(k), coeff0.ptr<float>(i), (size_t)nm * sizeof(float));
            confm[(size_t)k] = conf0[(size_t)i];
            idm[(size_t)k] = id0[(size_t)i];
            angm[(size_t)k] = ang0[(size_t)i];
        }

        // =========================
        // 3) 仅为 bbox refine 按采样高度稀疏解码 mask（不全量解码 Hm*Wm）
        // =========================
        const float* proto_data = proto.ptr<float>(0);
        const int HWm = Hm * Wm;
        auto decode_low_mask = [&](const float* coeff_row, int y, int x) -> float
        {
            const int yy = std::max(0, std::min(Hm - 1, y));
            const int xx = std::max(0, std::min(Wm - 1, x));

            float s = 0.0f;
            if (proto_chw_resolved)
            {
                const int idx_hw = yy * Wm + xx;
                for (int c = 0; c < nm; ++c)
                {
                    s += coeff_row[c] * proto_data[(size_t)c * (size_t)HWm + (size_t)idx_hw];
                }
            }
            else
            {
                const float* pix = proto_data + ((size_t)yy * (size_t)Wm + (size_t)xx) * (size_t)nm;
                for (int c = 0; c < nm; ++c)
                {
                    s += coeff_row[c] * pix[c];
                }
            }
            return 1.0f / (1.0f + std::exp(-s));
        };

        // bbox refine：仅采样若干高度行，不构建完整 H0xW0 mask
        cv::Mat& xyxy_refined = ws.xyxy_refined;
        xyxy.copyTo(xyxy_refined);
        constexpr int sample_levels = 9;
        constexpr float top_ratio = 0.1f;
        constexpr float bottom_ratio = 0.9f;
        constexpr int min_pixels_per_row = 2;
        const float sx_scale = (float)Wm / (float)W0;
        const float sy_scale = (float)Hm / (float)H0;
        auto clampf = [](float v, float lo, float hi) -> float
        {
            return std::max(lo, std::min(hi, v));
        };

        for (int i = 0; i < N; ++i)
        {
            const float* coeff_row = coeff.ptr<float>(i);

            const float x1 = xyxy_refined.at<float>(i, 0);
            const float y1 = xyxy_refined.at<float>(i, 1);
            const float x2 = xyxy_refined.at<float>(i, 2);
            const float y2 = xyxy_refined.at<float>(i, 3);
            const int xi1 = std::max(0, std::min(W0 - 1, (int)std::floor(x1)));
            const int yi1 = std::max(0, std::min(H0 - 1, (int)std::floor(y1)));
            const int xi2 = std::max(0, std::min(W0, (int)std::ceil(x2)));
            const int yi2 = std::max(0, std::min(H0, (int)std::ceil(y2)));
            if (xi2 <= xi1 || yi2 <= yi1) continue;

            const int y_lo = yi1;
            const int y_hi = yi2 - 1;
            if (y_hi < y_lo) continue;

            const int bw = xi2 - xi1;
            if (bw <= 0) continue;

            std::vector<int>& x0_map = ws.x0_map;
            std::vector<int>& x1_map = ws.x1_map;
            std::vector<float>& wx0_map = ws.wx0_map;
            std::vector<float>& wx1_map = ws.wx1_map;
            x0_map.resize((size_t)bw);
            x1_map.resize((size_t)bw);
            wx0_map.resize((size_t)bw);
            wx1_map.resize((size_t)bw);
            for (int dx = 0; dx < bw; ++dx)
            {
                const int x = xi1 + dx;
                const float sx = ((float)x + 0.5f) * sx_scale - 0.5f;
                int x0 = (int)std::floor(sx);
                int x1i = x0 + 1;
                const float wx1 = sx - (float)x0;
                const float wx0 = 1.0f - wx1;
                x0 = std::max(0, std::min(Wm - 1, x0));
                x1i = std::max(0, std::min(Wm - 1, x1i));
                x0_map[(size_t)dx] = x0;
                x1_map[(size_t)dx] = x1i;
                wx0_map[(size_t)dx] = wx0;
                wx1_map[(size_t)dx] = wx1;
            }

            const float h = (float)(y_hi - y_lo);
            int v_start = (int)std::lround((float)y_lo + top_ratio * h);
            int v_end = (int)std::lround((float)y_lo + bottom_ratio * h);
            v_start = std::max(y_lo, std::min(y_hi, v_start));
            v_end = std::max(y_lo, std::min(y_hi, v_end));
            if (v_end < v_start) std::swap(v_start, v_end);

            std::array<float, sample_levels> lefts{};
            std::array<float, sample_levels> rights{};
            int valid_rows = 0;
            for (int k = 0; k < sample_levels; ++k)
            {
                const int v = (sample_levels <= 1)
                                  ? (int)std::lround(0.5 * (v_start + v_end))
                                  : (int)std::lround((double)v_start + (double)(v_end - v_start) * (double)k /
                                      (double)(sample_levels - 1));

                const float sy = ((float)v + 0.5f) * sy_scale - 0.5f;
                int y0 = (int)std::floor(sy);
                int y1i = y0 + 1;
                const float wy1 = sy - (float)y0;
                const float wy0 = 1.0f - wy1;
                y0 = std::max(0, std::min(Hm - 1, y0));
                y1i = std::max(0, std::min(Hm - 1, y1i));

                int first = -1;
                int last = -1;
                int cnt = 0;
                for (int dx = 0; dx < bw; ++dx)
                {
                    const int sx0 = x0_map[(size_t)dx];
                    const int sx1 = x1_map[(size_t)dx];
                    const float wx0 = wx0_map[(size_t)dx];
                    const float wx1 = wx1_map[(size_t)dx];
                    const float v00 = decode_low_mask(coeff_row, y0, sx0);
                    const float v01 = decode_low_mask(coeff_row, y0, sx1);
                    const float v10 = decode_low_mask(coeff_row, y1i, sx0);
                    const float v11 = decode_low_mask(coeff_row, y1i, sx1);
                    const float p0 = wx0 * v00 + wx1 * v01;
                    const float p1 = wx0 * v10 + wx1 * v11;
                    const float pv = wy0 * p0 + wy1 * p1;
                    if (pv > mask_thr_)
                    {
                        const int xx = xi1 + dx;
                        if (first < 0) first = xx;
                        last = xx;
                        cnt += 1;
                    }
                }
                if (cnt < min_pixels_per_row)
                {
                    continue;
                }
                lefts[(size_t)valid_rows] = (float)first;
                rights[(size_t)valid_rows] = (float)(last + 1);
                valid_rows += 1;
            }
            if (valid_rows <= 0) continue;

            float sum_l = 0.0f;
            float sum_r = 0.0f;
            for (int t = 0; t < valid_rows; ++t)
            {
                sum_l += lefts[(size_t)t];
                sum_r += rights[(size_t)t];
            }
            const float nx1 = sum_l / (float)valid_rows;
            const float nx2 = sum_r / (float)valid_rows;
            if (!(std::isfinite(nx1) && std::isfinite(nx2))) continue;
            if (nx2 - nx1 < 1.0f) continue;

            xyxy_refined.at<float>(i, 0) = clampf(nx1, 0.0f, (float)(W0 - 1));
            xyxy_refined.at<float>(i, 2) = clampf(nx2, 0.0f, (float)W0);
        }

        // 4) 左右 strip 过滤
        const float left_strip_x = static_cast<float>(W0) * 0.1f;
        const float right_strip_x = static_cast<float>(W0) * 0.9f;
        std::vector<int>& keep_after_lr = ws.keep_after_lr;
        keep_after_lr.clear();
        keep_after_lr.reserve((size_t)N);
        for (int n = 0; n < N; ++n)
        {
            const float* b = xyxy_refined.ptr<float>(n);
            const float cx = 0.5f * (b[0] + b[2]);
            if (cx >= left_strip_x && cx < right_strip_x) keep_after_lr.push_back(n);
        }
        if (keep_after_lr.empty()) return emptyResult();

        // 5) 输出 objs，并按相同顺序可选输出实例分割 mask
        const int K = (int)keep_after_lr.size();
        cv::Mat objs(K, 7, CV_32F);
        int mask_sizes[3] = {K, H0, W0};
        cv::Mat masks;
        if (return_masks)
        {
            masks = cv::Mat(3, mask_sizes, CV_8U, cv::Scalar(0));
        }
        for (int k = 0; k < K; ++k)
        {
            const int src = keep_after_lr[(size_t)k];
            float* p = objs.ptr<float>(k);
            const float* b = xyxy_refined.ptr<float>(src);
            p[0] = b[0];
            p[1] = b[1];
            p[2] = b[2];
            p[3] = b[3];
            p[4] = confm[(size_t)src];
            p[5] = (float)idm[(size_t)src];
            p[6] = angm[(size_t)src];

            if (return_masks)
            {
                const float* coeff_row = coeff.ptr<float>(src);
                const int x1 = std::max(0, std::min(W0 - 1, (int)std::floor(b[0])));
                const int y1 = std::max(0, std::min(H0 - 1, (int)std::floor(b[1])));
                const int x2 = std::max(0, std::min(W0, (int)std::ceil(b[2])));
                const int y2 = std::max(0, std::min(H0, (int)std::ceil(b[3])));

                if (x2 > x1 && y2 > y1)
                {
                    uchar* mask_data = masks.ptr<uchar>(k);
                    const float mask_sx_scale = (float)Wm / (float)W0;
                    const float mask_sy_scale = (float)Hm / (float)H0;

                    for (int yy0 = y1; yy0 < y2; ++yy0)
                    {
                        const float sy = ((float)yy0 + 0.5f) * mask_sy_scale - 0.5f;
                        int py0 = (int)std::floor(sy);
                        int py1 = py0 + 1;
                        const float wy1 = sy - (float)py0;
                        const float wy0 = 1.0f - wy1;
                        py0 = std::max(0, std::min(Hm - 1, py0));
                        py1 = std::max(0, std::min(Hm - 1, py1));

                        uchar* mask_row = mask_data + (size_t)yy0 * (size_t)W0;
                        for (int xx0 = x1; xx0 < x2; ++xx0)
                        {
                            const float sx = ((float)xx0 + 0.5f) * mask_sx_scale - 0.5f;
                            int px0 = (int)std::floor(sx);
                            int px1 = px0 + 1;
                            const float wx1 = sx - (float)px0;
                            const float wx0 = 1.0f - wx1;
                            px0 = std::max(0, std::min(Wm - 1, px0));
                            px1 = std::max(0, std::min(Wm - 1, px1));

                            const float v00 = decode_low_mask(coeff_row, py0, px0);
                            const float v01 = decode_low_mask(coeff_row, py0, px1);
                            const float v10 = decode_low_mask(coeff_row, py1, px0);
                            const float v11 = decode_low_mask(coeff_row, py1, px1);
                            const float p0 = wx0 * v00 + wx1 * v01;
                            const float p1 = wx0 * v10 + wx1 * v11;
                            const float pv = wy0 * p0 + wy1 * p1;
                            mask_row[xx0] = (pv > mask_thr_) ? 1U : 0U;
                        }
                    }
                }
            }
        }

        angle_postproc_.ProcessInplace(objs);

        return Result{objs, masks};
    }

public:

    cv::Mat draw_angleins(
        const cv::Mat& img, // HxWx3 CV_8UC3
        const cv::Mat& objs, // Nx7 CV_32F
        const cv::Mat& masks, // [N,H,W] CV_8U(0/1) or CV_32F
        const std::vector<std::string>* txts = nullptr,
        bool draw_mask = true,
        bool draw_bbox = true,
        bool draw_angle = true
    ) const
    {
        if (objs.empty()) return img.clone();

        cv::Mat out;
        const cv::Mat objs6 = objs.colRange(0, 6);

        if (draw_mask)
        {
            out = draw_2d_instances(
                img, objs6, masks,
                &id2name_,
                txts,
                /*draw_bbox=*/true,
                /*draw_label=*/true,
                /*mask_alpha=*/0.45,
                /*thickness=*/1
            );
        }
        else if (draw_bbox)
        {
            out = draw_2d_boxes(
                img, objs6,
                &id2name_,
                txts,
                /*draw_label=*/true,
                /*thickness=*/1
            );
        } else {
            out = img.clone();
        }

        if (draw_angle)
        {
            const int H = out.rows, W = out.cols;
            const int N = objs.rows;

            std::vector<int> order((size_t)N);
            std::iota(order.begin(), order.end(), 0);
            std::stable_sort(order.begin(), order.end(), [&](int a, int b)
            {
                return objs.at<float>(a, 4) < objs.at<float>(b, 4);
            });

            for (int idx : order)
            {
                float x1 = objs.at<float>(idx, 0);
                float y1 = objs.at<float>(idx, 1);
                float x2 = objs.at<float>(idx, 2);
                float y2 = objs.at<float>(idx, 3);
                int cid = (int)std::lround(objs.at<float>(idx, 5));
                float theta = objs.at<float>(idx, 6);

                cv::Scalar color = color_for_cls(cid);

                int x1i = std::clamp((int)std::lround(x1), 0, W - 1);
                int y1i = std::clamp((int)std::lround(y1), 0, H - 1);
                int x2i = std::clamp((int)std::lround(x2), 0, W - 1);
                int y2i = std::clamp((int)std::lround(y2), 0, H - 1);

                int cx = (x1i + x2i) / 2;
                int cy = (y1i + y2i) / 2;

                float L = 0.35f * (float)std::max(x2i - x1i, y2i - y1i);
                int dx = (int)std::lround(-L * std::sin(theta));
                int dy = (int)std::lround(-L * std::cos(theta));

                cv::arrowedLine(out, {cx, cy}, {cx + dx, cy + dy},
                                color, 1, cv::LINE_AA, 0.2);
            }
        }

        return out;
    }

    cv::Mat draw_angleins_with_track_info(
        const cv::Mat& img, // HxWx3 CV_8UC3
        const cv::Mat& objs, // Nt x 6/7 CV_32F
        const cv::Mat& masks, // [Nt,H,W]
        const cv::Mat& track_info, // Nt x 4 CV_32S [track_id, track_state, track_age, idx]
        bool draw_mask = true,
        bool draw_bbox = true,
        bool draw_angle = true
    ) const
    {
        if (track_info.empty() || objs.empty()) return img.clone();
        if (!(track_info.type() == CV_32S && track_info.dims == 2 && track_info.cols == 4))
            throw std::runtime_error("tracked_info must be Nt x 4 CV_32S");
        if (!(objs.type() == CV_32F && objs.dims == 2 && (objs.cols == 6 || objs.cols ==
            7)))
            throw std::runtime_error("objs must be Nt x 6/7 CV_32F");

        const int Nt = objs.rows;
        if (track_info.rows != Nt) throw std::runtime_error("track_info.rows must match objs.rows");

        cv::Mat objs7;
        if (objs.cols == 6)
        {
            objs7 = cv::Mat(Nt, 7, CV_32F);
            objs.copyTo(objs7.colRange(0, 6));
            objs7.col(6).setTo(0);
        }
        else
        {
            objs7 = objs;
        }

        std::vector<std::string> txts;
        txts.reserve((size_t)Nt);
        for (int i = 0; i < Nt; ++i)
        {
            int track_id = track_info.at<int>(i, 0);
            int track_age = track_info.at<int>(i, 2);
            int cid = (int)std::lround(objs7.at<float>(i, 5));
            float conf = objs7.at<float>(i, 4);

            std::string name;
            auto it = id2name_.find(cid);
            if (it != id2name_.end()) name = it->second;
            else name = std::to_string(cid);

            char buf[160];
            std::snprintf(buf, sizeof(buf), "ID %d %s %.2f A%d", track_id, name.c_str(), (double)conf, track_age);
            txts.emplace_back(buf);
        }

        return draw_angleins(img, objs7, masks, &txts, draw_mask, draw_bbox, draw_angle);
    }

    // 绘制3D边界框的主函数
    cv::Mat draw_3Dbounding(
        const cv::Mat& img_bgr,     // H×W×3 CV_8UC3 原始图像
        const cv::Mat& detections   // N×18 CV_32F 检测结果
    ) {
        // 检查输入
        if (detections.empty() || detections.cols != 18 || detections.type() != CV_32FC1) {
            std::cerr << "Error: detections must be N×18 CV_32FC1" << std::endl;
            return img_bgr.clone();
        }
        
        // 复制图像以便绘制
        cv::Mat out = img_bgr.clone();
        const int H = out.rows;
        const int W = out.cols;
        const int N = detections.rows;
        
        // 按置信度排序（从高到低）
        std::vector<int> order(N);
        std::iota(order.begin(), order.end(), 0);
        std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
            return detections.at<float>(a, 16) > detections.at<float>(b, 16);  // 置信度降序
        });
        
        // 遍历每个检测结果
        for (int idx : order) {
            // 读取8个点的坐标 (p0-p7)
            std::vector<cv::Point2f> points(8);
            for (int i = 0; i < 8; i++) {
                points[i].x = detections.at<float>(idx, i * 2);
                points[i].y = detections.at<float>(idx, i * 2 + 1);
            }
            
            // 地面四个点：p0, p1, p2, p3 (左前, 右前, 右后, 左后)
            cv::Point2f p0 = points[0];  // 地面左前方
            cv::Point2f p1 = points[1];  // 地面右前方
            cv::Point2f p2 = points[2];  // 地面右后方
            cv::Point2f p3 = points[3];  // 地面左后方
            
            // 顶部四个点：p4, p5, p6, p7 (对应地面点的上方)
            cv::Point2f p4 = points[4];  // 顶部左前方
            cv::Point2f p5 = points[5];  // 顶部右前方
            cv::Point2f p6 = points[6];  // 顶部右后方
            cv::Point2f p7 = points[7];  // 顶部左后方
            
            // 读取置信度和类别
            float confidence = detections.at<float>(idx, 16);
            int class_id = (int)std::lround(detections.at<float>(idx, 17));
            
            // 获取颜色
            cv::Scalar color = color_for_cls(class_id);
            cv::Scalar color_dim = color * 0.5;  // 用于背面线条的暗淡颜色
            
            // 确保坐标在图像范围内
            auto clamp_point = [&](cv::Point2f& pt) {
                pt.x = std::clamp(pt.x, 0.0f, (float)(W - 1));
                pt.y = std::clamp(pt.y, 0.0f, (float)(H - 1));
            };
            
            // 对所有点进行裁剪
            for (auto& pt : points) {
                clamp_point(pt);
            }
            
            // 重新赋值裁剪后的点
            p0 = points[0]; p1 = points[1]; p2 = points[2]; p3 = points[3];
            p4 = points[4]; p5 = points[5]; p6 = points[6]; p7 = points[7];
            
            // ========== 绘制3D边界框 ==========
            
            // 绘制地面矩形（四边形）
            std::vector<cv::Point> ground_pts = {
                cv::Point((int)std::lround(p0.x), (int)std::lround(p0.y)),
                cv::Point((int)std::lround(p1.x), (int)std::lround(p1.y)),
                cv::Point((int)std::lround(p2.x), (int)std::lround(p2.y)),
                cv::Point((int)std::lround(p3.x), (int)std::lround(p3.y))
            };
            cv::polylines(out, ground_pts, true, color, 2, cv::LINE_AA);
            
            // 绘制顶部矩形（四边形）
            std::vector<cv::Point> top_pts = {
                cv::Point((int)std::lround(p4.x), (int)std::lround(p4.y)),
                cv::Point((int)std::lround(p5.x), (int)std::lround(p5.y)),
                cv::Point((int)std::lround(p6.x), (int)std::lround(p6.y)),
                cv::Point((int)std::lround(p7.x), (int)std::lround(p7.y))
            };
            cv::polylines(out, top_pts, true, color, 2, cv::LINE_AA);
            
            // 绘制垂直连接线（4条竖边）
            std::vector<std::pair<cv::Point2f, cv::Point2f>> vertical_edges = {
                {p0, p4}, {p1, p5}, {p2, p6}, {p3, p7}
            };
            for (const auto& edge : vertical_edges) {
                cv::line(out, 
                        cv::Point((int)std::lround(edge.first.x), (int)std::lround(edge.first.y)),
                        cv::Point((int)std::lround(edge.second.x), (int)std::lround(edge.second.y)),
                        color, 2, cv::LINE_AA);
            }
            
            // 绘制对角线（为了增强3D效果）
            // 地面对角线
            cv::line(out, 
                    cv::Point((int)std::lround(p0.x), (int)std::lround(p0.y)),
                    cv::Point((int)std::lround(p2.x), (int)std::lround(p2.y)),
                    color_dim, 1, cv::LINE_AA);
            cv::line(out, 
                    cv::Point((int)std::lround(p1.x), (int)std::lround(p1.y)),
                    cv::Point((int)std::lround(p3.x), (int)std::lround(p3.y)),
                    color_dim, 1, cv::LINE_AA);
            
            // 顶部对角线
            cv::line(out, 
                    cv::Point((int)std::lround(p4.x), (int)std::lround(p4.y)),
                    cv::Point((int)std::lround(p6.x), (int)std::lround(p6.y)),
                    color_dim, 1, cv::LINE_AA);
            cv::line(out, 
                    cv::Point((int)std::lround(p5.x), (int)std::lround(p5.y)),
                    cv::Point((int)std::lround(p7.x), (int)std::lround(p7.y)),
                    color_dim, 1, cv::LINE_AA);
            
            // ========== 标识p0-p7点 ==========
            
            // 点的标签名称
            std::vector<std::string> point_names = {"p0", "p1", "p2", "p3", "p4", "p5", "p6", "p7"};
            
            // 每个点的颜色（使用不同颜色以便区分）
            std::vector<cv::Scalar> point_colors = {
                {0, 0, 255},     // p0: 红色
                {0, 255, 0},     // p1: 绿色
                {255, 0, 0},     // p2: 蓝色
                {0, 255, 255},   // p3: 黄色
                {255, 0, 255},   // p4: 品红
                {255, 255, 0},   // p5: 青色
                {128, 0, 128},   // p6: 紫色
                {128, 128, 0}    // p7: 橄榄色
            };
            
            // 绘制每个点
            for (int i = 0; i < 8; i++) {
                cv::Point pt((int)std::lround(points[i].x), (int)std::lround(points[i].y));
                
                // 绘制大圆点
                cv::circle(out, pt, 5, point_colors[i], -1);  // 填充圆
                cv::circle(out, pt, 5, cv::Scalar(255, 255, 255), 1);  // 白色边框
                
                // 绘制标签（带背景）
                std::string label = point_names[i];
                int baseline = 0;
                cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
                
                // 计算标签位置（在点的右上方）
                cv::Point label_pos(pt.x + 8, pt.y - 8);
                
                // 确保标签在图像内
                label_pos.x = std::clamp(label_pos.x, 5, W - text_size.width - 5);
                label_pos.y = std::clamp(label_pos.y, text_size.height + 5, H - 5);
                
                // 绘制半透明背景
                cv::Rect bg_rect(label_pos.x - 2, label_pos.y - text_size.height - 2, 
                                text_size.width + 4, text_size.height + baseline + 4);
                bg_rect.x = std::clamp(bg_rect.x, 0, W - bg_rect.width);
                bg_rect.y = std::clamp(bg_rect.y, 0, H - bg_rect.height);
                
                cv::Mat roi = out(bg_rect);
                roi = roi * 0.6;  // 降低亮度作为背景
                
                // 绘制文本
                cv::putText(out, label, 
                            cv::Point(label_pos.x, label_pos.y + baseline),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                            point_colors[i], 2, cv::LINE_AA);
            }
            
            // ========== 绘制类别和置信度标签 ==========
            
            std::string label = "Class " + std::to_string(class_id) + 
                            " " + std::to_string(confidence).substr(0, 5);
            
            // 计算标签位置（在物体上方中心）
            cv::Point2f top_center = (p4 + p5 + p6 + p7) * 0.25f;
            cv::Point label_pos((int)std::lround(top_center.x), 
                            (int)std::lround(top_center.y) - 20);
            
            // 确保标签位置在图像内
            label_pos.x = std::clamp(label_pos.x, 10, W - 10);
            label_pos.y = std::clamp(label_pos.y, 20, H - 10);
            
            // 绘制文本背景
            int baseline = 0;
            cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &baseline);
            cv::Rect bg_rect(label_pos.x - 8, label_pos.y - text_size.height - 8, 
                            text_size.width + 16, text_size.height + baseline + 16);
            bg_rect.x = std::clamp(bg_rect.x, 0, W - bg_rect.width);
            bg_rect.y = std::clamp(bg_rect.y, 0, H - bg_rect.height);
            
            cv::rectangle(out, bg_rect, color * 0.3, -1);
            cv::rectangle(out, bg_rect, color, 2);
            
            // 绘制文本
            cv::putText(out, label, 
                        cv::Point(label_pos.x, label_pos.y + baseline + 4),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, 
                        cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
            
            // 添加图例说明（可选）
            std::string legend = "p0:left-front, p1:right-front, p2:right-back, p3:left-back";
            cv::putText(out, legend, 
                        cv::Point(10, H - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, 
                        cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
        }
        
        return out;
    }  

    private:
    static std::string merge_target_name(const std::string& name)
    {
        static constexpr const char* kPrefix = "fake ";
        static constexpr size_t kPrefixLen = 5;
        std::string base_name = name;
        if (base_name.size() > kPrefixLen && base_name.compare(0, kPrefixLen, kPrefix) == 0)
        {
            base_name = base_name.substr(kPrefixLen);
        }
        if (base_name == "bike")
        {
            base_name = "bicycle";
        }
        if (base_name == "motorcycle")
        {
            base_name = "bicycle";
        }
        return base_name;
    }

    int canonical_class_id(int cid) const
    {
        auto it = class_alias_.find(cid);
        return it == class_alias_.end() ? cid : it->second;
    }

    int nc_;
    int nm_;
    float conf_thr_;
    float iou_thr_;
    float mask_thr_;
    int mask_up_;

    std::unordered_map<int, std::string> id2name_;
    std::unordered_map<int, int> class_alias_;
    std::unordered_set<int> whitelist_ids_;
    std::vector<int> whitelist_class_ids_;
    AngleFieldPostProcessor angle_postproc_;
};
