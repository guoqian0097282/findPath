# visper.pyi
from __future__ import annotations
from typing import Any, Callable

"""
@defgroup VisPer_RAEB_Result RAEB 任务结果格式说明
@{

当任务名为 "RAEB" 时，VisPer 管线内部维护一个
`dict[str, Any]` 作为当前帧的结果快照，
在以下两个场景中会对外暴露：

  1. VisPer_GetResult("RAEB") 的返回值；
  2. 通过 VisPer_RegCallback("RAEB", cb) 注册的回调函数 cb 的入参。

这两个地方获取到的是同一套数据结构，即 `raeb_result`。

------------------------------------------------------------
raeb_result 的结构（以当前实现为准）
------------------------------------------------------------

raeb_result: dict[str, Any] = {
    "task": "RAEB",                 # str：固定为 "RAEB"
    "timestamp": 0,                 # int：该次推理的时间戳（对应 C++ int64_t）

    "objs": np.ndarray(...),        # ndarray, shape: (N, 7), dtype: float32
    "track_info": np.ndarray(...),  # ndarray, shape: (M, 4), dtype: int32
    "tracked_cuboids_raw": np.ndarray(...), # ndarray, shape: (M, 9), dtype: float32
    "tracked_cuboids": np.ndarray(...), # ndarray, shape: (M, 9), dtype: float32
    "tracked_cuboids_vel": np.ndarray(...), # ndarray, shape: (M, 4), dtype: float32
}

当前约定的键及其含义如下：

============================================================
基础检测结果
============================================================

- "task"
  - 类型：str
  - 含义：当前结果所属的任务名。对 RAEB 管线，该字段固定为 "RAEB"，
          便于区分不同任务类型。

- "timestamp"
  - 类型：int（对应 C++ std::int64_t）
  - 含义：当前这帧 RAEB 结果对应的时间戳，
          数值来源于 VisPer_RunInfer(...) 调用时传入的 timestamp 参数，
          用于多路任务之间对齐帧、延时统计或回放。
          时间单位（毫秒、微秒等）由调用方统一约定。

- "objs"
  - 类型：numpy.ndarray，dtype=float32，形状：N x 7
  - 含义：2D 检测框 + 类别 + 相对朝向角，每行格式：

      [x1, y1, x2, y2, conf, cls, theta_rel]

    其中：
      - (x1, y1, x2, y2)：像素坐标系下左上角与右下角；
      - conf：目标置信度（0 ~ 1）；
      - cls ：类别 id；
      - theta_rel：相对朝向角（单位：弧度）。

============================================================
跟踪相关结果
============================================================

- "track_info" TODO 传给MCU
  - 类型：numpy.ndarray，dtype=int32，形状：M x 4
  - 含义：当前帧中所有成功绑定轨迹的目标，其基础轨迹信息。

    注意：
      - track_info 仅包含成功关联轨迹的目标（M ≤ N）；
      - Idx 用于索引原始 objs；
      - tracked_* 均按轨迹顺序排列，并与 track_info 按行一一对应；
      - 不应使用 Idx 去索引 tracked_*。

    每一行字段说明：

      [TrackID, TrackState, TrackAge, Idx]

      - TrackID    ：轨迹唯一 ID（跨帧稳定）
      - TrackState ：轨迹状态
          0: New
          1: Tracked
          2: Lost
          3: Removed
      - TrackAge   ：轨迹持续帧数
      - Idx        ：该轨迹对应的原始检测索引

    映射关系（按约定）：
      - objs[Idx] 为该轨迹对应的原始 2D 检测框（N x 7 中的第 Idx 行）

- "tracked_cuboids" TODO 传给MCU
  - 类型：numpy.ndarray，dtype=float32，形状：M x 9
  - 含义：与 "track_info" 按行一一对应的 3D 立方体解算结果，
         按轨迹顺序排列，每行格式：

      [cx, cy, cz, l, w, h, conf, cls, theta_abs]

    其中：
      - (cx, cy, cz)：目标在车辆坐标系 / 世界坐标系下的三维中心；
      - (l, w, h)：长宽高尺寸；
      - conf：3D 检测置信度；
      - cls ：类别 id；
      - theta_abs：绝对朝向角（单位：弧度）。

    对于无法解算 3D 几何的目标：
      - cx, cy, cz = 0
      - l, w, h = -1
      - theta_abs = 0

- "tracked_cuboids_vel"
  - 类型：numpy.ndarray，dtype=float32，形状：M x 4
  - 含义：每个轨迹对应的速度输出信息（包含坐标系标记）。

    每一行字段说明：

      [Mode, MotionState, VelX, VelY]

      - Mode ：速度所在坐标系
          0: Ego（车辆坐标系）
          1: World（世界坐标系）
      - MotionState ：运动状态
          0: Unknown
          1: Static
          2: Moving
      - VelX / VelY ：对应坐标系下的平面速度分量（单位：m/s）

    备注：
      - 当外部未提供有效 ego 位姿（ego_x/ego_y/ego_yaw 缺失或为 None）时，
        仍会输出 VelX/VelY 的估算值，但 MotionState 被置为 Unknown（0）。

============================================================
补充说明
============================================================

- 所有字段都通过 Python dict 保存，取用时通过 key 访问；
- 大部分语义字段以 float32 形式存储（如 objs / tracked_cuboids / tracked_cuboids_vel），
  track_info 中的 ID / 状态等离散值采用 int32 存储；
- 当前实现仅保证上述键存在。

@}
"""


def VisPer_InitTask(task: str, config_path: str, model_path: str) -> None:
    """初始化单个任务。

    对齐 C++:
        void VisPer_InitTask(const std::string& task,
                             const std::string& config_path,
                             const std::string& model_path);

    说明：
        - 相同 task 只会真正初始化一次，重复调用会被忽略。
        - task 典型值："RAEB" / "OP"
    """
    ...


def VisPer_RegCallback(
        task: str,
        cb: Callable[[dict[str, Any]], None],
) -> None:
    """注册回调。

    对齐 C++:
        void VisPer_RegCallback(const std::string& task,
                                const std::function<void(const std::unordered_map<std::string, std::any>&)>& cb);

    Args:
        task: 任务名，典型为 "RAEB" / "OP"
        cb: 回调函数。入参是该 task 最近一帧结果的快照（dict[str, Any]）。
            若传入空/None 的语义在 C++ 中是“空函数对象取消回调”，
            Python 侧通常建议传一个空 lambda 或在绑定层提供取消能力。
    """
    ...


def VisPer_PushExtraData(task: str, extra: dict[str, Any]) -> None:
    """为指定任务推入一份额外输入数据 extra（整体覆盖）。

    对齐 C++:
        void VisPer_PushExtraData(const std::string& task,
                                  const std::unordered_map<std::string, std::any>& extra);

    Args:
        task: 任务名，"RAEB" / "OP"
        extra: 额外输入数据（键值对）。每次调用会整体替换上一份 extra。
    """
    ...


def VisPer_RunInfer(img: Any, timestamp: int) -> None:
    """推一帧并完成该帧全部处理（预处理 → 推理 → 后处理）。

    对齐 C++:
        void VisPer_RunInfer(const cv::Mat& img, std::int64_t timestamp);

    Args:
        img: 输入图像（Python 绑定层可接受 ndarray / cv2.Mat 等）。
        timestamp: 时间戳（对应 C++ 的 int64_t）。
    """
    ...


def VisPer_GetResult(task: str) -> dict[str, Any]:
    """只负责拿结果，不触发推理/后处理。

    对齐 C++:
        std::unordered_map<std::string, std::any> VisPer_GetResult(const std::string& task);

    Returns:
        dict[str, Any]: 该任务最近一帧结果快照（拷贝语义），拿不到则返回空 dict。

    说明：
        - 对 task=="RAEB"：返回字段见 C++ 头文件 @defgroup VisPer_RAEB_Result
        - 对 task=="OP"：典型包含 "preds" 等
    """
    ...


def VisPer_GetVis(task: str, save_dir: str = "") -> dict[str, Any]:
    """获取指定任务的可视化结果，可选保存到磁盘。

    对齐 C++:
        std::unordered_map<std::string, std::any> VisPer_GetVis(const std::string& task,
                                                                const std::string& save_dir = {});

    Args:
        task: 任务名，"RAEB" / "OP"
        save_dir: 保存目录。
            - 为空字符串（默认）：只返回内存中的可视化结果，不保存文件
            - 非空：保存可视化图片到该目录（文件名含 timestamp）

    Returns:
        dict[str, Any]:
            - task=="RAEB"：典型键 "vis_img_2d", "vis_img_3d"
            - task=="OP"  ：典型键 "vis_img"
            - 无结果/任务不存在：返回空 dict
    """
    ...


def VisPer_RunCallBack() -> None:
    """基于当前内部结果触发回调（如果注册过）。

    对齐 C++:
        void VisPer_RunCallBack();

    说明：
        - 回调顺序：先 RAEB，再 OP（若两者都已启用/注册）
        - 回调在调用该函数的线程上下文执行
    """
    ...


def VisPer_CleanUp() -> None:
    """清理资源（预留接口）。

    对齐 C++:
        void VisPer_CleanUp();
    """
    ...
