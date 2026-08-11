#include "common/visper_internal_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <unordered_set>

#include "common/logger.hpp"

namespace visper::internal {

namespace {

std::unordered_set<std::string> g_CLEARED_SAVE_DIRS;
std::mutex g_CLEARED_SAVE_DIRS_MTX;

} // namespace

std::string NormalizeSaveDirKey(const std::filesystem::path& dir) {
    namespace fs = std::filesystem;
    std::error_code ec;
    const fs::path abs_dir = fs::absolute(dir, ec);
    if (ec) {
        return dir.lexically_normal().string();
    }
    return abs_dir.lexically_normal().string();
}

void PrepareSaveDirOnce(const std::string& save_dir) {
    namespace fs = std::filesystem;
    fs::path dir(save_dir);
    const std::string dir_key = NormalizeSaveDirKey(dir);

    bool need_clear = false;
    {
        std::lock_guard<std::mutex> lk(g_CLEARED_SAVE_DIRS_MTX);
        need_clear = g_CLEARED_SAVE_DIRS.insert(dir_key).second;
    }
    if (need_clear) {
        std::error_code ec_rm;
        fs::remove_all(dir, ec_rm);
        if (ec_rm) {
            LOG_WARNING("Clear dir failed: %s", dir.string().c_str());
        }
    }

    std::error_code ec_mk;
    fs::create_directories(dir, ec_mk);
    if (ec_mk) {
        LOG_WARNING("Create dir failed: %s", dir.string().c_str());
    }
}

void ResetPreparedSaveDirs() {
    std::lock_guard<std::mutex> lk(g_CLEARED_SAVE_DIRS_MTX);
    g_CLEARED_SAVE_DIRS.clear();
}

double AnyToFiniteDouble(const std::any& v, double fallback) {
    if (const auto* p = std::any_cast<double>(&v)) {
        return std::isfinite(*p) ? *p : fallback;
    }
    if (const auto* p = std::any_cast<float>(&v)) {
        const double x = static_cast<double>(*p);
        return std::isfinite(x) ? x : fallback;
    }
    if (const auto* p = std::any_cast<std::int64_t>(&v)) {
        return static_cast<double>(*p);
    }
    if (const auto* p = std::any_cast<std::int32_t>(&v)) {
        return static_cast<double>(*p);
    }
    if (const auto* p = std::any_cast<int>(&v)) {
        return static_cast<double>(*p);
    }
    if (const auto* p = std::any_cast<std::uint64_t>(&v)) {
        return static_cast<double>(*p);
    }
    if (const auto* p = std::any_cast<std::uint32_t>(&v)) {
        return static_cast<double>(*p);
    }
    if (const auto* p = std::any_cast<std::uint16_t>(&v)) {
        return static_cast<double>(*p);
    }
    if (const auto* p = std::any_cast<std::int16_t>(&v)) {
        return static_cast<double>(*p);
    }
    if (const auto* p = std::any_cast<std::uint8_t>(&v)) {
        return static_cast<double>(*p);
    }
    if (const auto* p = std::any_cast<std::int8_t>(&v)) {
        return static_cast<double>(*p);
    }
    if (const auto* p = std::any_cast<bool>(&v)) {
        return *p ? 1.0 : 0.0;
    }
    if (!v.has_value()) {
        return fallback;
    }
    return fallback;
}

double GetEgoYawFromExtra(const std::unordered_map<std::string, std::any>& extra) {
    const auto it = extra.find("ego_yaw");
    if (it == extra.end()) {
        return 0.0;
    }
    return AnyToFiniteDouble(it->second, 0.0);
}

} // namespace visper::internal
