#include "vis/vis_api.h"

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

#include <fcntl.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <unistd.h>

#include "common/logger.hpp"
#include "common/visper_internal_utils.hpp"
#include "cuboids/cuboids_api.h"
#include "postproc/postproc_api.h"

namespace {

bool HasVislTriggerFileInCwd() {
    namespace fs = std::filesystem;
    std::error_code ec;
    const fs::path cwd = fs::current_path(ec);
    if (ec) {
        LOG_WARNING("Read current path failed when checking vis trigger file");
        return false;
    }

    const fs::path p = cwd / "visl";
    const bool exists = fs::exists(p, ec);
    if (ec) {
        LOG_WARNING("Check vis trigger file exists failed: %s", p.string().c_str());
        return false;
    }
    if (!exists) {
        return false;
    }

    const bool is_file = fs::is_regular_file(p, ec);
    if (ec) {
        LOG_WARNING("Check vis trigger file type failed: %s", p.string().c_str());
        return false;
    }
    return is_file;
}

void SyncFilesystemByPathOrThrow(const std::filesystem::path& p) {
    const std::string path = p.string();
    const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
    if (fd < 0) {
        throw std::runtime_error(
            "open for syncfs failed: " + path + ", errno=" + std::to_string(errno)
        );
    }

#if defined(__linux__)
    if (::syncfs(fd) != 0) {
        const int e = errno;
        ::close(fd);
        throw std::runtime_error(
            "syncfs failed: " + path + ", errno=" + std::to_string(e)
        );
    }
#else
    if (::fsync(fd) != 0) {
        const int e = errno;
        ::close(fd);
        throw std::runtime_error(
            "fsync failed: " + path + ", errno=" + std::to_string(e)
        );
    }
#endif

    if (::close(fd) != 0) {
        throw std::runtime_error(
            "close after sync failed: " + path + ", errno=" + std::to_string(errno)
        );
    }
}

} // namespace

bool vis_HasRaebStageBTrigger() {
    return HasVislTriggerFileInCwd();
}

void vis_RunRaebStageBModule(
    std::int64_t timestamp,
    const cv::Mat& img_bgr,
    const cv::Mat& objs_in,
    const cv::Mat& masks_in,
    const cv::Mat& track_info,
    const cv::Mat& tracked_cuboids_in,
    const cv::Mat& tracked_cuboids_vel_in
) {
    if (!HasVislTriggerFileInCwd()) {
        return;
    }
    if (img_bgr.empty()) {
        LOG_WARNING("RAEB StageB vis skipped: image is empty");
        return;
    }

    namespace fs = std::filesystem;
    const std::string save_dir = "vis";
    const fs::path dir(save_dir);
    visper::internal::PrepareSaveDirOnce(save_dir);

    const fs::path path_ori = dir / ("raeb_vis_ori_" + std::to_string(timestamp) + ".jpg");
    const fs::path path_2d = dir / ("raeb_vis_2d_" + std::to_string(timestamp) + ".jpg");
    const fs::path path_3d = dir / ("raeb_vis_3d_" + std::to_string(timestamp) + ".jpg");

    bool ok_ori = false;
    {
        std::error_code ec;
        const std::string abs_ori = fs::absolute(path_ori, ec).string();
        ok_ori = cv::imwrite(path_ori.string(), img_bgr);
        if (ok_ori) {
            LOG_INFO("Save image: %s", abs_ori.c_str());
        } else {
            LOG_WARNING("Save image failed: %s", abs_ori.c_str());
        }
    }

    cv::Mat objs = objs_in;
    cv::Mat masks = masks_in;
    cv::Mat cuboids = tracked_cuboids_in;
    cv::Mat cuboids_vel = tracked_cuboids_vel_in;

    if (objs.empty()) {
        objs = cv::Mat(0, 7, CV_32F);
    }
    if (cuboids.empty()) {
        cuboids = cv::Mat(0, 9, CV_32F);
    }
    if (cuboids_vel.empty()) {
        cuboids_vel = cv::Mat(0, 4, CV_32F);
    }

    if (!masks.empty() &&
        !(masks.dims == 3 && masks.type() == CV_8U &&
          masks.size[0] == objs.rows &&
          masks.size[1] == img_bgr.rows &&
          masks.size[2] == img_bgr.cols)) {
        LOG_WARNING(
            "RAEB StageB vis masks ignored: masks_dims=%d masks_type=%d objs_rows=%d img=%dx%d",
            masks.dims,
            masks.type(),
            objs.rows,
            img_bgr.cols,
            img_bgr.rows
        );
        masks.release();
    }

    cv::Mat vis_img_2d = postproc_VisInstances(img_bgr, objs, masks);
    std::error_code ec;
    const std::string abs_2d = fs::absolute(path_2d, ec).string();
    const bool ok_2d = cv::imwrite(path_2d.string(), vis_img_2d);
    if (ok_2d) {
        LOG_INFO("Save image: %s", abs_2d.c_str());
    } else {
        LOG_WARNING("Save image failed: %s", abs_2d.c_str());
    }

    std::vector<std::string> labels_3d;
    if (!cuboids.empty() && cuboids.dims == 2 && cuboids.type() == CV_32F && cuboids.cols >= 2) {
        labels_3d.reserve(cuboids.rows);
        auto state_to_text = [](int st) -> const char* {
            switch (st) {
            case 0: return "N";
            case 1: return "T";
            case 2: return "L";
            case 3: return "R";
            default: return "?";
            }
        };
        const bool has_track_info =
            !track_info.empty() &&
            track_info.dims == 2 &&
            track_info.type() == CV_32S &&
            track_info.cols >= 3;
        const bool has_vel =
            !cuboids_vel.empty() &&
            cuboids_vel.dims == 2 &&
            cuboids_vel.type() == CV_32F &&
            cuboids_vel.cols >= 4;

        for (int i = 0; i < cuboids.rows; ++i) {
            const float x = cuboids.at<float>(i, 0);
            const float y = cuboids.at<float>(i, 1);
            const bool row_has_track = has_track_info && i < track_info.rows;
            const bool row_has_vel = has_vel && i < cuboids_vel.rows;

            char line1[128];
            if (row_has_track) {
                const int id = track_info.at<int>(i, 0);
                const int st = track_info.at<int>(i, 1);
                const int age = track_info.at<int>(i, 2);
                std::snprintf(
                    line1,
                    sizeof(line1),
                    "ID %d %s A%d x=%.2f y=%.2f",
                    id,
                    state_to_text(st),
                    age,
                    x,
                    y
                );
            } else {
                std::snprintf(line1, sizeof(line1), "x=%.2f y=%.2f", x, y);
            }

            if (row_has_vel) {
                const float vx = cuboids_vel.at<float>(i, 2);
                const float vy = cuboids_vel.at<float>(i, 3);
                char line2[64];
                std::snprintf(line2, sizeof(line2), "vx=%.2f vy=%.2f", vx, vy);
                labels_3d.emplace_back(std::string(line1) + "\n" + line2);
            } else {
                labels_3d.emplace_back(line1);
            }
        }
    }

    cv::Mat vis_img_3d = cuboids_VisCuboids(
        vis_img_2d,
        cuboids,
        labels_3d.empty() ? nullptr : &labels_3d
    );

    const std::string abs_3d = fs::absolute(path_3d, ec).string();
    const bool ok_3d = cv::imwrite(path_3d.string(), vis_img_3d);
    if (ok_3d) {
        LOG_INFO("Save image: %s", abs_3d.c_str());
    } else {
        LOG_WARNING("Save image failed: %s", abs_3d.c_str());
    }

    if (ok_ori || ok_2d || ok_3d) {
        SyncFilesystemByPathOrThrow(ok_3d ? path_3d : (ok_2d ? path_2d : path_ori));
    }
}
