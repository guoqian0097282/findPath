#pragma once
/**
 * @file    VisPer.h
 * @brief   VisPer 管线对外 API（C++ 版）
 *
 *
 * ------------------------------------------------------------
 * RAEB pipeline 典型调用流程（从初始化到逐帧处理）
 * ------------------------------------------------------------
 *
 * // ===== 1) 初始化（只需初始化一次；重复调用会被忽略） =====
 * VisPer_InitTask("RAEB", raeb_config_path, raeb_model_path);
 *
 * // ===== 2) 可选：注册回调（传空函数对象可取消回调） =====
 * VisPer_RegCallback("RAEB",
 *     [](const std::unordered_map<std::string, std::any>& res) {
 *         // res 结构见 VisPer_RAEB_Result
 *         // 示例：
 *         // auto ts   = std::any_cast<std::int64_t>(res.at("timestamp"));
 *         // auto objs = std::any_cast<cv::Mat>(res.at("objs"));
 *     });
 *
 * // ===== 3) 推入 extra（支持多线程并发调用；内部会做时序对齐）=====
 * std::unordered_map<std::string, std::any> extra;
 * extra["ego_timestamp"] = static_cast<std::int64_t>(ego_ts);
 * extra["ego_x"]         = static_cast<float>(ego_x);
 * extra["ego_y"]         = static_cast<float>(ego_y);
 * extra["ego_yaw"]       = static_cast<float>(ego_yaw);
 * VisPer_PushExtraData("RAEB", extra);
 *
 * // ===== 4) 执行推理 =====
 * VisPer_RunInfer(data_ptr, data_len, frame_ts);
 * // SGS 零拷贝输入建议直接传 MI_SYS_FrameData_s：
 * // VisPer_RunInfer(frame_data, frame_ts);
 *
 * // ===== 5) 退出清理 =====
 * VisPer_CleanUp();
 *
 * 线程安全：
 *   所有对外 API 内部都通过互斥锁串行化访问全局上下文，可在多线程中安全调用，
 *   但同一时刻仅有一个线程在执行实际逻辑。
 */

#include <string>
#include <unordered_map>
#include <any>
#include <cstdint>
#include <cstddef>
#include <functional>

namespace cv {
    class Mat;
}

struct MI_SYS_FrameData_s;

struct VisPerTiPerfHandles {
    void* graph = nullptr;
    void* total_perf = nullptr;
    void* graph_perf = nullptr;
};
// ========== 张量信息结构体 ==========
typedef struct {
    // 形状信息
    const int64_t* shape_data;      // 指向形状数据的指针
    size_t shape_length;            // 形状维度数
    
    // 输出数据
    float* output_data;             // 输出数据指针（CV_32F）
    size_t output_size;             // 输出数据总元素数
    
    // 张量元信息
    int tensor_id;                  // 张量ID
    int data_type;                  // 原始数据类型 (TensorDataType)
    int batch_index;                // 批次索引
    uint64_t frame_id;              // 帧ID
    
    // 步长信息
    const size_t* strides;          // 步长数组指针
    size_t stride_count;            // 步长数量
    
    // 原始buffer（用于调试）
    const void* raw_buffer;         // 原始数据buffer指针
    size_t raw_buffer_size;         // 原始buffer大小
} TensorInfo;
/**
 * @defgroup VisPer_RAEB_Result RAEB 任务结果格式说明
 * @{
 *
 * 当任务名为 "RAEB" 时，VisPer 管线内部维护一个
 * `std::unordered_map<std::string, std::any>` 作为当前帧的结果快照，
 * 在以下两个场景中会对外暴露：
 *
 *   1. VisPer_GetResult("RAEB") 的返回值；
 *   2. 通过 VisPer_RegCallback("RAEB", cb) 注册的回调函数 cb 的入参。
 *
 * 这两个地方获取到的是同一套数据结构，即 `raeb_result`。
 *
 * ------------------------------------------------------------
 * raeb_result 的结构（以当前实现为准）
 * ------------------------------------------------------------
 *
 * std::unordered_map<std::string, std::any> raeb_result{
 *     {"task",               std::any{std::string("RAEB")}}, // 显式放一个 std::string
 *     {"timestamp",          std::any{}},   // int64_t：该次推理的时间戳
 *
 *     {"objs",               std::any{}},   // cv::Mat (N x 7, CV_32F)
 *     {"track_info",         std::any{}},   // cv::Mat (M x 4, CV_32S)
 *     {"tracked_cuboids_raw",std::any{}},   // cv::Mat (M x 9, CV_32F)
 *     {"tracked_cuboids",    std::any{}},   // cv::Mat (M x 9, CV_32F)
 *     {"tracked_cuboids_vel",std::any{}},   // cv::Mat (M x 4, CV_32F)
 * };
 *
 * 当前约定的键及其含义如下：
 *
 * ============================================================
 * 基础检测结果
 * ============================================================
 *
 * - "task"
 *   - 类型：`std::string`
 *   - 含义：当前结果所属的任务名。对 RAEB 管线，该字段固定为 "RAEB"，
 *           便于区分不同任务类型。
 *           注意在 `std::any` 中存放的是 `std::string` 而不是字符串字面量，
 *           调用方可直接使用 `std::any_cast<std::string>` 取出。
 *
 * - "timestamp"
 *   - 类型：`std::int64_t`
 *   - 含义：当前这帧 RAEB 结果对应的时间戳，
 *           数值来源于 VisPer_RunInfer(...) 调用时传入的 timestamp 参数，
 *           用于多路任务之间对齐帧、延时统计或回放。
 *           时间单位（毫秒、微秒等）由调用方统一约定。
 *
 * - "objs"
 *   - 类型：`cv::Mat`（CV_32F），形状：N x 7
 *   - 含义：2D 检测框 + 类别 + 相对朝向角，每行格式：
 *
 *       [x1, y1, x2, y2, conf, cls, theta_rel]
 *
 *     其中：
 *       - (x1, y1, x2, y2)：像素坐标系下左上角与右下角；
 *       - conf：目标置信度（0 ~ 1）；
 *       - cls ：类别 id；
 *       - theta_rel：相对朝向角（单位：弧度）。
 *
 * ============================================================
 * 跟踪相关结果
 * ============================================================
 *
 * - "track_info" TODO 传给MCU
 *   - 类型：`cv::Mat`（CV_32S），形状：M x 4
 *   - 含义：当前帧中所有成功绑定轨迹的目标，其基础轨迹信息。
 *
 *     注意：
 *       - track_info 仅包含成功关联轨迹的目标（M ≤ N）；
 *       - Idx 用于索引原始 objs；
 *       - tracked_cuboids_raw/tracked_cuboids/tracked_cuboids_vel 均按轨迹顺序排列，并与 track_info 按行一一对应；
 *       - 不应使用 Idx 去索引 tracked_cuboids_raw/tracked_cuboids/tracked_cuboids_vel。

 *     每一行字段说明：
 *
 *       [TrackID, TrackState, TrackAge, Idx]
 *
 *       - TrackID    ：轨迹唯一 ID（跨帧稳定）
 *       - TrackState ：轨迹状态
 *           0: New
 *           1: Tracked
 *           2: Lost
 *           3: Removed
 *       - TrackAge   ：轨迹持续帧数
 *       - Idx        ：该轨迹对应的原始检测索引
 *
 *     映射关系（按约定）：
 *       - objs.row(Idx) 为该轨迹对应的原始 2D 检测框（N x 7 中的第 Idx 行）
 *
 *
 * - "tracked_cuboids" TODO 传给MCU
 *   - 类型：`cv::Mat`（CV_32F），形状：M x 9
 *   - 含义：与 "track_info" 按行一一对应的 3D 立方体解算结果。
 *          按轨迹顺序排列，每行格式：
 *
 *       [cx, cy, cz, l, w, h, conf, cls, theta_abs]
 *
 *     其中：
 *       - (cx, cy, cz)：目标在车辆坐标系 / 世界坐标系下的三维中心；
 *       - (l, w, h)：长宽高尺寸；
 *       - conf：3D 检测置信度；
 *       - cls ：类别 id；
 *       - theta_abs：绝对朝向角（单位：弧度）。
 *
 *     对于无法解算 3D 几何的目标：
 *       - cx, cy, cz = 0
 *       - l, w, h = -1
 *       - theta_abs = 0
 *
 * - "tracked_cuboids_vel" TODO 传给MCU
 *   - 类型：`cv::Mat`（CV_32F），形状：M x 4
 *   - 含义：每个轨迹对应的速度估计结果（对齐 Python VelocityEstimator 输出）。
 *
 *     每一行字段说明：
 *
 *       [Mode, MotionState, VelX, VelY]
 *
 *       - Mode       ：速度坐标系标识
 *           0: Ego   （车辆坐标系）
 *           1: World （世界坐标系）
 *       - MotionState：运动状态
 *           0: Unknown
 *           1: Static
 *           2: Moving
 *       - VelX / VelY：平面速度分量（单位：m/s）
 *
 * ============================================================
 * 补充说明
 * ============================================================
 *
 * - 所有字段均通过 `std::any` 保存，使用前需通过 `std::any_cast<T>` 转出；
 * - 大部分语义字段以 float 形式存储（如 objs / tracked_cuboids_raw / tracked_cuboids / tracked_cuboids_vel），
 *   track_info 中的 ID / 状态等离散值采用 CV_32S 整型存储；
 * - 当前实现仅保证上述键存在。
 *
 * @}
 */


/**
 * @brief 初始化单个任务，使用默认推理目标。
 *
 * @param task        任务名，当前典型值为 "RAEB" 或 "OP"
 * @param config_path 配置文件路径（JSON，可含注释）
 * @param model_path  模型路径
 *
 * 调用说明：
 *   - 相同 task 只会执行一次真正的初始化，后续重复调用会被忽略；
 *   - 初始化过程中会按 task 类型完成配置读取、预处理初始化、模型加载、
 *     后处理初始化以及跟踪/3D 相关模块初始化；
 *   - 在 TI 编译版本中，本接口默认把 RAEB TIDL 模型放到 C7_1 上运行；
 *   - 在 x86/SGS 编译版本中，本接口行为与原先一致，不涉及 TI C7x 目标核选择。
 */
void VisPer_InitTask(const std::string& task,
                     const std::string& config_path,
                     const std::string& model_path);

/**
 * @brief 初始化单个任务，并在 TI 编译版本中指定 TIDL 模型运行的 C7x 核。
 *
 * @param task        任务名，当前典型值为 "RAEB" 或 "OP"
 * @param config_path 配置文件路径（JSON，可含注释）
 * @param model_path  模型路径
 * @param ti_target   TI OpenVX target 字符串。TI 编译版本中调用方只传以下两个值之一：
 *                    - "DSP_C7-1"：把 TIDL 模型节点绑定到 C7_1；
 *                    - "DSP_C7-2"：把 TIDL 模型节点绑定到 C7_2。
 *
 * 调用说明：
 *   - 这个重载用于需要显式选择 TI C7x 核的场景；
 *   - "DSP_C7-1" / "DSP_C7-2" 是 TI TIOVX 中
 *     TIVX_TARGET_DSP_C7_1 / TIVX_TARGET_DSP_C7_2 宏展开后的实际 target 字符串；
 *   - 如果 ti_target 不是上述两个字符串，TI 编译版本会在初始化阶段抛出异常；
 *   - C7_2 能否成功运行取决于板端 C7_2 firmware、remoteproc 和 TIDL kernel 是否已经就绪；
 *   - 在 x86/SGS 编译版本中，ti_target 参数会被忽略。
 */
void VisPer_InitTask(const std::string& task,
                     const std::string& config_path,
                     const std::string& model_path,
                     const std::string& ti_target);


/**
 * @brief 注册回调。task="RAEB"/"OP"
 *
 * @param task  任务名（当前支持 "RAEB" / "OP"）
 * @param cb    回调函数。若传入空函数对象，相当于取消该任务的回调。
 *
 * 回调参数是一帧结果的快照：
 *
 *   - 对于 task == "RAEB"：
 *       传入的是 RAEB 任务对应的结果 map，
 *       具体键值见 @ref VisPer_RAEB_Result。
 *
 *   - 对于 task == "OP"：
 *       当前实现中 map 中典型键为：
 *         - "preds" : cv::Mat
 *       （后续可根据 OP 的具体定义扩展更多字段）
 *
 * 典型调用方式：
 *   @code
 *   VisPer_RegCallback("RAEB",
 *       [](const std::unordered_map<std::string, std::any>& res) {
 *           const auto& objs_any  = res.at("objs");   // cv::Mat
 *           // ...
 *       }
 *   );
 *   @endcode
 */
void VisPer_RegCallback(const std::string& task,
                        const std::function<void(const std::unordered_map<std::string, std::any>&)>& cb);


/**
 * @brief 为指定任务推入一份“额外输入数据”（extra），供后续 VisPer_RunInfer 使用
 *
 * @details
 *   - 用途：为推理 pipeline 提供除图像之外的补充信息（extra）。
 *
 *   - 调用时机：应在本帧对应的 VisPer_RunInfer(...) 之前调用。
 *       示例：
 *         std::unordered_map<std::string, std::any> extra;
 *         // 按上层与任务实现的约定填充 extra（可为空，也可包含任意键）
 *         VisPer_PushExtraData("RAEB", extra);
 *         VisPer_RunInfer(img, frame_ts);
 *
 *   - 更新策略（整体覆盖 / replace）：
 *       - 每次调用都会用本次传入的 extra 整体替换内部保存的上一份 extra；
 *
 *   - 线程安全：
 *       - 函数内部会加锁保护全局上下文（g_CTX），可与 VisPer_RunInfer /
 *         VisPer_GetResult 等接口并发调用；
 *       - 但请保证同一帧的数据流调用顺序合理：先 PushExtraData，再 RunInfer。
 *
* ------------------------------------------------------------
 * RAEB extra 数据结构与键约定（显式约定；字段不同则视为无效）TODO SOC传给算法
 * ------------------------------------------------------------
 * 对于 task == "RAEB"，推理管线启用了速度估计/世界坐标输出等功能，
 * extra 需要提供自车位姿相关信息。VisPer 内部对外只约定以下键的语义，
 * 其余自定义字段允许存在，但 RAEB 模块不保证使用。
 *
 * raeb_extra 的结构（示例；以 std::any 显式存放具体类型）
 * std::unordered_map<std::string, std::any> raeb_extra{
 *     //自车位姿来源时间戳，需保证和 VisPer_RunInfer 使用同源时间戳
 *     {"ego_timestamp", std::any{std::int64_t{0}}},
 *     {"ego_x",         std::any{float{0.0f}}},   // m
 *     {"ego_y",         std::any{float{0.0f}}},   // m
 *     {"ego_yaw",       std::any{float{0.0f}}},   // rad
 * };
 *
 * 键约定（RAEB 明确约定；缺失或类型不匹配将被视为无效）
 * ------------------------------------------------------------
 * - "ego_timestamp"
 *   - 类型：std::int64_t（或可安全转换为 int64 的整型）
 *   - 含义：自车位姿来源的时间戳（如定位/里程计/融合定位时间戳）。
 *          是否参与与图像帧时间对齐由实现决定。
 *          时间单位（毫秒、微秒等）由调用方统一约定。
 *
 * - "ego_x"
 *   - 类型：float / double（或可安全转换为 float 的数值）
 *   - 含义：自车在世界坐标系下的平面位置 X（单位：m）。
 *
 * - "ego_y"
 *   - 类型：float / double（或可安全转换为 float 的数值）
 *   - 含义：自车在世界坐标系下的平面位置 Y（单位：m）。
 *
 * - "ego_yaw"
 *   - 类型：float / double（或可安全转换为 float 的数值）
 *   - 含义：自车航向角 yaw（单位：rad，绕 Z 轴）。
 *
 * 失效/退化策略
 * ------------------------------------------------------------
 * - 若 "ego_x" / "ego_y" / "ego_yaw" 任意缺失或无效：
 *     - 世界坐标使用上一次传入内容；
 *     - 运动状态（MotionState）标记为 Unknown；
 *     - 速度数值可能仍会输出（取决于速度估计器实现），但不应依赖其语义。
 *
 * @param task   任务名："RAEB" / "OP"
 * @param extra  本次推理所需的额外输入数据键值对集合
 */
void VisPer_PushExtraData(const std::string& task, const std::unordered_map<std::string, std::any>& extra);


/**
 * @brief 推一帧并做完这帧的全部处理（预处理 → 推理 → 后处理）
 *
 * @param img       输入图像，BGR, CV_8UC3
 * @param timestamp 用户时间戳，会写进内部上下文（可用于上层对齐）
 *
 * 调用完成后：
 *   - 内部全局上下文会被更新为该帧的状态；
 *   - 对应任务（如 "RAEB"）的结果 map 会被填充；
 *   - 对于 "RAEB"，若已注册回调，会在 StageB 末尾自动触发回调。
 */
void VisPer_RunInfer(const cv::Mat& img, std::int64_t timestamp);


/**
 * @brief 裸指针输入兼容接口。
 *
 * @param data      输入缓冲区地址
 * @param length    缓冲区实际字节数
 * @param timestamp 用户时间戳，会写进内部上下文
 *
 * SGS 新代码应优先使用 MI_SYS_FrameData_s 重载。
 */
void VisPer_RunInfer(const uint8_t* data, std::size_t length, std::int64_t timestamp);

/**
 * @brief 获取 TI 板端资源统计所需的 OpenVX graph 和 app perf point 句柄。
 *
 * @details
 *   - 仅 TI 编译版本且 RAEB 初始化成功后返回 true；
 *   - 返回的指针由 libvisper 内部持有，调用方只读使用，不负责释放；
 *   - 句柄在 VisPer_CleanUp() 或重新初始化/释放模型后失效；
 *   - 对外使用 void* 是为了避免业务头文件强依赖 TI OpenVX/vision_apps 头。
 *
 * SOC 侧如已包含 TI 头文件，可按需转换：
 *
 * @code
 * VisPerTiPerfHandles h;
 * if (VisPer_GetTiPerfHandles(&h)) {
 *     tivx_utils_graph_perf_print(reinterpret_cast<vx_graph>(h.graph));
 *     appPerfPointPrint(reinterpret_cast<app_perf_point_t*>(h.graph_perf));
 *     appPerfPointPrint(reinterpret_cast<app_perf_point_t*>(h.total_perf));
 *     appPerfPointPrintFPS(reinterpret_cast<app_perf_point_t*>(h.total_perf));
 * }
 * @endcode
 */
bool VisPer_GetTiPerfHandles(VisPerTiPerfHandles* out_handles);


/**
 * @brief SGS 零拷贝输入：直接使用 MI_SYS_FrameData_s 里的虚拟地址和 MIU 物理地址。
 *
 * 该接口会使用 frame.pVirAddr[0] 作为 CPU/SDK 侧地址，使用 frame.phyAddr[0]
 * 作为 IPU/DMA 侧地址；必要时会同时绑定 NV12 第二平面地址。
 * 非 SGS 架构调用该接口会抛出异常。
 */
void VisPer_RunInfer(const MI_SYS_FrameData_s& frame, std::int64_t timestamp);


/**
 * @brief 只负责“拿结果”，不做新的推理/后处理
 *
 * @param task 任务名："RAEB" / "OP"
 * @return 该任务的结果快照（按值返回一份 map；拿不到则返回空 map）
 *
 * 典型用途：
 *   - 在你已经完成一帧推理（VisPer_RunInfer）之后，主动拉取最新结果；
 *   - 在回调机制之外，做轮询式查询。
 *
 * 注意：
 *   - 对于 "RAEB"，返回格式见 @ref VisPer_RAEB_Result；
 *   - 返回的是一个拷贝，修改它不会影响内部上下文。
 */
std::unordered_map<std::string, std::any> VisPer_GetResult(const std::string& task);


/**
 * @brief 清理资源
 *
 * 当前实现为空，预留接口：
 *   - 若后续需要显式释放模型、缓存、句柄等，会在此处补充。
 */
void VisPer_CleanUp();

std::unordered_map<std::string, std::any> PostProcessRAEB(const uint8_t* data, std::size_t length, std::int64_t timestamp, const TensorInfo* infos, std::size_t count);
