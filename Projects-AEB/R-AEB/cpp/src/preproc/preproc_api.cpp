#include "preproc_api.h"
#include "preproc_impl.hpp"

#include <cctype>
#include <memory>
#include <stdexcept>

/**
 * @file preproc_api.cpp
 * @brief 预处理模块实现文件（全局单例 + 接口包装）。
 *
 * 内部全局对象：
 *   - g_convert：指向 ImageConvert 的全局唯一实例
 *
 * 设计目的：
 *   - 模拟 Python 版本的全局 g_convert 行为
 *   - 提供 C 接口风格的简单调用方式（但这里仍使用 C++ 类型）
 *
 * 注意：
 *   - g_convert 的创建/替换不是线程安全的
 *   - 初始化和使用的并发访问需调用方自行保证
 */

static std::unique_ptr<ImageConvert> g_convert;


void preproc_Init(int width, int height, const std::string& input_type, const std::string& target_type) {
    /**
     * @brief 初始化全局 ImageConvert 实例。
     * 若多次调用，会替换旧实例（旧实例自动释放）。
     */
    g_convert = std::make_unique<ImageConvert>(width, height, input_type, target_type);
}

cv::Mat preproc_ToMat(const uint8_t* addr, std::size_t length)
{
    if (!g_convert)
    {
        throw std::runtime_error("ImageConvert 未初始化：请先调用 preproc_Init");
    }
    return g_convert->to_mat(addr, length);
}


std::size_t preproc_InputBytes() {
    if (!g_convert) {
        throw std::runtime_error("ImageConvert 未初始化：请先调用 preproc_Init");
    }
    return g_convert->input_bytes();
}


cv::Mat preproc_Convert(const cv::Mat& img, const std::string& target_type) {
    /**
     * @brief 对输入图像执行颜色空间/格式转换。
     */
    if (!g_convert) {
        throw std::runtime_error("ImageConvert 未初始化：请先调用 preproc_Init");
    }
    return g_convert->convert(img, target_type);
}

cv::Mat preproc_ToData(const cv::Mat& img,
                       int expand_axis,
                       const std::vector<int>& transpose_axes,
                       int cv_dtype) {
    /**
     * @brief 执行数据后处理（expand_dims / transpose / dtype 转换）。
     */
    if (!g_convert) {
        throw std::runtime_error("ImageConvert 未初始化：请先调用 preproc_Init");
    }

    return ImageConvert::to_data(img, expand_axis, transpose_axes, cv_dtype);
}