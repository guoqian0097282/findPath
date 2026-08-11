#pragma once

#include <cstddef>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

/**
 * @file preproc_api.h
 * @brief 预处理模块的 C++ 接口封装（对应 Python 的 preproc_Init / preproc_Convert / preproc_ToData）。
 *
 * 该模块内部维护一个全局 ImageConvert 实例（类似 Python 里的 g_convert）。
 * 使用顺序：
 *   1) 必须先调用 preproc_Init(...)
 *   2) 再调用 preproc_Convert(...) / preproc_ToData(...)
 *
 * 线程安全说明：
 *   - 全局单例 g_convert 的创建/替换不是线程安全的；
 *   - 多线程并发调用前建议在单线程完成初始化；
 *   - 若要在运行时替换 g_convert，需自行加锁。
 */

/**
 * @brief 初始化图像预处理器（等价于 Python 的 preproc_Init）。
 *
 * 初始化完成后，模块内部会持有一个全局 ImageConvert 实例，用于后续转换。
 *
 * @param width      图像宽度（像素）。
 * @param height     图像高度（像素）。
 * @param input_type 输入类型，支持："NV12" / "BGR" / "RGB"（大小写不敏感）。
 * @param target_type 默认目标类型，支持："NV12" / "BGR" / "RGB"（大小写不敏感）。
 *
 * @throws std::invalid_argument 当类型字符串不在支持范围内时抛出。
 *
 * @note
 * - 当 input_type 为 NV12 时，宽高必须是偶数（NV12 格式要求）。
 * - 该检查也可能在实际转换时触发（由 ImageConvert 内部保证）。
 */
void preproc_Init(int width, int height, const std::string& input_type, const std::string& target_type);


/**
 * @brief 从裸 uint8 数据构造只读 cv::Mat 视图（不拷贝）（对应 ImageConvert::to_mat）。
 *
 * 该函数会根据 preproc_Init(...) 时配置的 input_type / width / height，
 * 将一段连续内存“包装”为 OpenCV Mat：
 * - input_type = "NV12"：返回 (height*3/2, width) 的 CV_8UC1（单平面 NV12：Y + UV 交错）
 * - input_type = "BGR"/"RGB"：返回 (height, width) 的 CV_8UC3
 *
 * @param data   指向输入缓冲区的指针（连续内存，uint8_t）。
 * @param length 缓冲区字节长度。
 *
 * @return 构造出的 cv::Mat 视图（不拥有内存，不负责释放）。
 *
 * @throws std::runtime_error
 * - 若未先调用 preproc_Init，会抛出固定错误信息："ImageConvert 未初始化：请先调用 preproc_Init"
 * - 若 data 为空或 length 为 0，会抛出 "empty data"
 * - 若 input_type 为 NV12 且 width/height 不是偶数，会抛出 "NV12 requires even width and height"
 * - 若 length 与期望字节数不匹配会抛出：
 *     - NV12： "invalid NV12 buffer length"
 *     - RGB/BGR： "invalid RGB/BGR buffer length"
 *
 * @note
 * - 本函数不做任何拷贝，返回 Mat 的 data 指针直接指向传入的 data；
 *   因此 data 的生命周期必须覆盖 Mat 的使用周期，否则会发生悬空引用。
 * - OpenCV 的 Mat 构造接口对 data 参数是非 const 指针；实现中会对 const_cast，
 *   语义上仍应将返回 Mat 视为只读，避免对其写入。
 * - 该函数只负责“包装视图”，不验证内容是否真的是合法图像数据（只验证长度/形状）。
 */
cv::Mat preproc_ToMat(const uint8_t* data, std::size_t length);

/**
 * @brief 返回 preproc_Init(...) 当前配置对应的紧凑输入图像字节数。
 *
 * 对 NV12 为 width*height*3/2，对 RGB/BGR 为 width*height*3。
 */
std::size_t preproc_InputBytes();


/**
 * @brief 执行图像颜色空间/格式转换（等价于 Python 的 preproc_Convert）。
 *
 * 将输入 cv::Mat 转换到指定目标类型：
 * - 若 target_type 为空字符串，则使用 preproc_Init 初始化时配置的默认 target_type；
 * - 若 target_type 非空，则本次调用临时覆盖目标类型（不修改初始化默认值）。
 *
 * 输入形状约束：
 * - input_type = "BGR"/"RGB"：
 *     - img 必须为 CV_8UC3，尺寸必须为 (height, width)
 * - input_type = "NV12"：
 *     - img 必须为 CV_8UC1，尺寸必须为 (height*3/2, width)（单平面）
 *
 * 输出约定（与 OpenCV cvtColor 行为一致）：
 * - 目标为 "BGR"/"RGB"：返回 CV_8UC3，尺寸为 (height, width)
 * - 目标为 "NV12"：返回 CV_8UC1，尺寸为 (height*3/2, width)
 *
 * @param img         输入图像（cv::Mat）。
 * @param target_type 可选目标类型（空字符串表示使用默认目标类型）。
 *
 * @return 转换后的 cv::Mat。
 *
 * @throws std::runtime_error
 * - 若未先调用 preproc_Init，会抛出固定错误信息："ImageConvert 未初始化：请先调用 preproc_Init"
 * - 若输入形状/类型与初始化配置不匹配，会抛出对应错误
 * - 若遇到不支持的转换路径，会抛出错误
 *
 * @throws std::invalid_argument 如果传入 target_type 不在支持范围内可能抛出（由 ImageConvert 解析/校验）。
 */
cv::Mat preproc_Convert(const cv::Mat& img, const std::string& target_type = "");

/**
 * @brief 数据后处理工具：按需执行 expand_dims / transpose / dtype 转换（等价于 Python 的 preproc_ToData）。
 *
 * 执行顺序固定为：
 *   1) expand_dims（可选）
 *   2) transpose（可选）
 *   3) astype / convertTo（可选）
 *
 * 参数语义：
 * - expand_axis:
 *     - 传入 -1 表示不扩维
 *     - 其它值表示在该轴位置插入 size=1 的新维度（类似 numpy.expand_dims）
 * - transpose_axes:
 *     - 为空表示不转置
 *     - 非空时必须是一个 0..(ndims-1) 的排列，用于重排维度（类似 numpy.transpose）
 * - out_dtype:
 *     - 为空字符串表示不改变 dtype
 *     - 支持示例（大小写不敏感，支持带前缀）：
 *         "uint8", "float32", "float64", "CV_32F", "np.float32", "numpy.uint8"
 *     - 注意：这里只改变 depth（CV_8U/CV_32F 等），不改变通道数
 *
 * 关于输入 Mat 的形态：
 * - 若输入是 2D 单通道（常见图像/平面数据），直接按 ND single-channel 处理。
 * - 若输入是 2D 多通道（例如 HxW 的 CV_8UC3），内部会将其重解释为 ND：
 *     (H, W, C) 且通道数变为 1（等价于把通道维“搬到维度里”）。
 *   这样才能像 numpy 一样对 “最后一个维度是通道维” 做 transpose。
 *
 * @param img            输入数据（cv::Mat）。
 * @param expand_axis    维度扩展轴；-1 表示不扩维。
 * @param transpose_axes 维度置换；空表示不置换。
 * @param cv_dtype
 *
 * @return 处理后的 cv::Mat（可能为 ND Mat，channels() == 1）。
 *
 * @throws std::runtime_error
 * - 若未先调用 preproc_Init，会抛出固定错误信息："ImageConvert 未初始化：请先调用 preproc_Init"
 * - 若 transpose_axes 非法（长度不匹配/有重复/越界）会抛出
 * - 若输入为不支持的通道数/维度类型会抛出
 *
 * @throws std::invalid_argument 若 out_dtype 无法解析为支持的 depth，会抛出。
 *
 * @note
 * - 该函数不依赖 g_convert 的宽高/颜色配置，但为对齐 Python 行为，这里仍要求初始化后才可调用。
 * - 若你不希望 preproc_ToData 依赖初始化，可自行移除该检查。
 */
cv::Mat preproc_ToData(const cv::Mat& img,
                       int expand_axis = -1,
                       const std::vector<int>& transpose_axes = {},
                       int cv_dtype = -1);
