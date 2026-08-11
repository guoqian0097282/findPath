#pragma once
/**
 * @file VisPer_c.h
 * @brief VisPer 管线 C ABI。
 *
 * 该头文件提供一层 C 可调用的轻量封装，底层仍复用 C++ 版 VisPer API。
 * 接口中不会暴露 std::string、std::unordered_map、std::any、
 * std::function、cv::Mat 等 C++ 类型。
 *
 * RAEB 典型调用流程：
 *
 * @code
 * VisPer_InitTask_C("RAEB", config_path, model_path, NULL);
 * // TI 编译版本如需指定 C7x 核：
 * // VisPer_InitTask_C("RAEB", config_path, model_path, "DSP_C7-2");
 *
 * VisPer_RunInfer_C(data, length, frame_timestamp);
 *
 * VisPerRaebResult_C result;
 * VisPer_GetResult_C("RAEB", &result);
 *
 * for (int32_t i = 0; i < result.tracked_cuboids_rows; ++i) {
 *     const float cx = result.tracked_cuboids[i][0];
 *     const float cy = result.tracked_cuboids[i][1];
 *     const float vx = result.tracked_cuboids_vel[i][2];
 *     const float vy = result.tracked_cuboids_vel[i][3];
 * }
 * @endcode
 */

#include <stddef.h>
#include <stdint.h>
#include <VisPer.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct VisPerTiPerfHandles_C {
    void* graph;
    void* total_perf;
    void* graph_perf;
} VisPerTiPerfHandles_C;

/**
 * @defgroup VisPer_C_RAEB_Result RAEB 任务 C 结果格式说明
 * @{
 *
 * `VisPerRaebResult_C` 是 C ABI 对外返回的 RAEB 结果快照。
 *
 * 所有数组均为行优先排列，内存由 libvisper 持有，调用方只读访问。
 * 当某个数组行数为 0 时，对应指针可能为 NULL。数组指针在下一次调用
 * VisPer_GetResult_C() 前有效；如果需要长期保存结果，调用方应自行拷贝。
 *
 * ------------------------------------------------------------
 * VisPerRaebResult_C 的结构（以当前实现为准）
 * ------------------------------------------------------------
 *
 * typedef struct VisPerRaebResult_C {
 *     int64_t timestamp;                       // 当前帧用户时间戳
 *
 *     int32_t objs_rows;                       // objs 行数 N
 *     const float   (*objs)[7];                // N x 7，2D 检测结果
 *
 *     int32_t track_info_rows;                 // track_info 行数 M
 *     const int32_t (*track_info)[4];          // M x 4，轨迹基础信息
 *
 *     int32_t tracked_cuboids_rows;            // tracked_cuboids 行数 M
 *     const float   (*tracked_cuboids)[9];     // M x 9，最终跟踪 3D 框
 *
 *     int32_t tracked_cuboids_vel_rows;        // tracked_cuboids_vel 行数 M
 *     const float   (*tracked_cuboids_vel)[4]; // M x 4，速度估计结果
 * } VisPerRaebResult_C;
 *
 * 该 C 结构对应 C++ `raeb_result` 中的以下 key：
 *
 *   - "timestamp"            -> timestamp
 *   - "objs"                 -> objs_rows / objs
 *   - "track_info"           -> track_info_rows / track_info
 *   - "tracked_cuboids"      -> tracked_cuboids_rows / tracked_cuboids
 *   - "tracked_cuboids_vel"  -> tracked_cuboids_vel_rows / tracked_cuboids_vel
 *
 * C ABI 当前不导出以下 C++ 侧历史字段或中间字段：
 *
 *   - "masks"
 *   - "tracked_objs"
 *   - "tracked_masks"
 *   - "tracked_cuboids_raw"
 *
 * @}
 */
typedef struct VisPerRaebResult_C {
    /**
     * @brief 当前帧时间戳。
     *
     * - 类型：int64_t
     * - 含义：当前这帧 RAEB 结果对应的时间戳。
     * - 来源：数值来源于 VisPer_RunInfer_C(...) 调用时传入的 timestamp 参数。
     * - 用途：用于多路任务之间对齐帧、延时统计或回放。
     * - 单位：由调用方统一约定，常见为毫秒或微秒。
     */
    int64_t timestamp;

    /**
     * @brief objs 的行数。
     *
     * - 类型：int32_t
     * - 含义：objs 中 2D 检测结果的数量。
     * - 关系：若 objs_rows 为 0，objs 指针可能为 NULL。
     */
    int32_t objs_rows;

    /**
     * @brief 2D 检测结果。
     *
     * - 类型：const float (*)[7]
     * - 形状：objs_rows x 7
     * - 含义：每一行表示一个 2D 检测框、类别和相对朝向角。
     *
     * 每行字段：
     *
     *   [x1, y1, x2, y2, conf, cls, theta_rel]
     *
     * 字段含义：
     *   - x1, y1, x2, y2：像素坐标系下左上角与右下角。
     *   - conf：目标置信度，范围通常为 0 到 1。
     *   - cls：类别 id，以 float 形式存储。
     *   - theta_rel：相对朝向角，单位为弧度。
     */
    const float (*objs)[7];

    /**
     * @brief track_info 的行数。
     *
     * - 类型：int32_t
     * - 含义：成功绑定轨迹的目标数量。
     * - 关系：track_info_rows、tracked_cuboids_rows、tracked_cuboids_vel_rows
     *         正常情况下应保持一致。
     */
    int32_t track_info_rows;

    /**
     * @brief 轨迹基础信息。
     *
     * - 类型：const int32_t (*)[4]
     * - 形状：track_info_rows x 4
     * - 含义：当前帧中所有成功绑定轨迹的目标基础信息。
     *
     * 每行字段：
     *
     *   [TrackID, TrackState, TrackAge, Idx]
     *
     * 字段含义：
     *   - TrackID：轨迹唯一 ID，跨帧稳定。
     *   - TrackState：轨迹状态。
     *       - 0: New
     *       - 1: Tracked
     *       - 2: Lost
     *       - 3: Removed
     *   - TrackAge：轨迹持续帧数。
     *   - Idx：该轨迹对应的原始检测索引，可用于索引 objs[Idx]。
     *
     * 注意：
     *   - track_info 仅包含成功关联轨迹的目标，行数 M 可能小于 objs_rows。
     *   - tracked_cuboids 和 tracked_cuboids_vel 已按 track_info 行顺序排列。
     *   - 不应使用 Idx 去索引 tracked_cuboids 或 tracked_cuboids_vel。
     */
    const int32_t (*track_info)[4];

    /**
     * @brief tracked_cuboids 的行数。
     *
     * - 类型：int32_t
     * - 含义：最终跟踪 3D 框数量。
     * - 关系：与 track_info 按行一一对应。
     */
    int32_t tracked_cuboids_rows;

    /**
     * @brief 最终跟踪 3D 框。
     *
     * - 类型：const float (*)[9]
     * - 形状：tracked_cuboids_rows x 9
     * - 含义：经过跟踪和位置补偿后的 3D 目标结果。
     *
     * 每行字段：
     *
     *   [cx, cy, cz, l, w, h, conf, cls, theta_abs]
     *
     * 字段含义：
     *   - cx, cy, cz：目标三维中心。
     *   - l, w, h：目标长宽高尺寸。
     *   - conf：3D 检测置信度。
     *   - cls：类别 id。
     *   - theta_abs：绝对朝向角，单位为弧度。
     *
     * 对于无法解算 3D 几何的目标：
     *   - cx, cy, cz = 0
     *   - l, w, h = -1
     *   - theta_abs = 0
     */
    const float (*tracked_cuboids)[9];

    /**
     * @brief tracked_cuboids_vel 的行数。
     *
     * - 类型：int32_t
     * - 含义：速度估计结果数量。
     * - 关系：与 track_info 按行一一对应。
     */
    int32_t tracked_cuboids_vel_rows;

    /**
     * @brief 速度估计结果。
     *
     * - 类型：const float (*)[4]
     * - 形状：tracked_cuboids_vel_rows x 4
     * - 含义：每个轨迹对应的速度估计结果。
     *
     * 每行字段：
     *
     *   [Mode, MotionState, VelX, VelY]
     *
     * 字段含义：
     *   - Mode：速度坐标系标识。
     *       - 0: Ego，车辆坐标系。
     *       - 1: World，世界坐标系。
     *   - MotionState：运动状态。
     *       - 0: Unknown
     *       - 1: Static
     *       - 2: Moving
     *   - VelX, VelY：平面速度分量，单位为 m/s。
     *
     * 注意：
     *   - 当外部未提供有效 ego 位姿时，MotionState 可能为 Unknown。
     *   - 速度数值可能仍会输出，但调用方不应在 Unknown 状态下依赖其语义。
     */
    const float (*tracked_cuboids_vel)[4];
} VisPerRaebResult_C;

/**
 * @brief 初始化一个 VisPer 任务。
 *
 * @param task        任务名，目前使用 "RAEB"。
 * @param config_path RAEB 配置文件路径。
 * @param model_path  模型或 artifact 目录路径。
 * @param ti_target   TI OpenVX target 字符串。
 *                    - NULL 或 ""：使用默认推理目标，TI 编译版本默认 C7_1；
 *                    - "DSP_C7-1"：TI 编译版本绑定到 C7_1；
 *                    - "DSP_C7-2"：TI 编译版本绑定到 C7_2。
 *
 * 同一任务重复初始化时，行为与底层 C++ API 保持一致：首次成功初始化后，
 * 后续重复调用会被忽略。
 *
 * 在 x86/SGS 编译版本中，ti_target 参数会被忽略。
 */
void VisPer_InitTask_C(const char* task,
                       const char* config_path,
                       const char* model_path,
                       const char* ti_target);

/**
 * @brief 通过裸指针输入接口执行一帧推理。
 *
 * @param data      输入缓冲区地址。
 * @param length    输入缓冲区字节数。
 * @param timestamp 用户时间戳，会写入内部上下文，并在结果的 timestamp 中返回。
 */
void VisPer_RunInfer_C(const uint8_t* data,
                       size_t length,
                       int64_t timestamp);

/**
 * @brief 获取最近一次 RAEB 结果快照。
 *
 * @param task       任务名，目前仅支持 "RAEB"。
 * @param out_result 输出结构体。若没有可用结果或 task 不支持，会被清零。
 */
void VisPer_GetResult_C(const char* task,
                        VisPerRaebResult_C* out_result);

/**
 * @brief 获取 TI 板端资源统计所需的 OpenVX graph 和 app perf point 句柄。
 *
 * @return 1 表示成功，0 表示当前不是 TI 后端、RAEB 未初始化或参数无效。
 *
 * 返回的指针由 libvisper 内部持有，调用方只读使用，不负责释放；
 * 在 VisPer_CleanUp() 或重新初始化/释放模型后失效。
 */
int VisPer_GetTiPerfHandles_C(VisPerTiPerfHandles_C* out_handles);

void PostProcessRAEB_C(const uint8_t* data,size_t length,int64_t timestamp,const TensorInfo* infos, size_t count,VisPerRaebResult_C* out_result);

#ifdef __cplusplus
}
#endif
