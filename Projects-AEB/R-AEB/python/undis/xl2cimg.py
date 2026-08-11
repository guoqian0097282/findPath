import numpy as np
import math

CM_TO_M = 0.01


# TODO 这份代码是将肖亮的代码转换为cimg的定义

# ============================================================
# 1. 基础旋转矩阵：与你的 ImageProcessor 保持一致
# ============================================================

def Rx(roll: float) -> np.ndarray:
    return np.array([
        [1.0, 0.0, 0.0],
        [0.0, math.cos(roll), -math.sin(roll)],
        [0.0, math.sin(roll), math.cos(roll)]
    ], dtype=np.float64)


def Ry(pitch: float) -> np.ndarray:
    return np.array([
        [math.cos(pitch), 0.0, math.sin(pitch)],
        [0.0, 1.0, 0.0],
        [-math.sin(pitch), 0.0, math.cos(pitch)]
    ], dtype=np.float64)


def Rz(yaw: float) -> np.ndarray:
    return np.array([
        [math.cos(yaw), -math.sin(yaw), 0.0],
        [math.sin(yaw), math.cos(yaw), 0.0],
        [0.0, 0.0, 1.0]
    ], dtype=np.float64)


# ============================================================
# 2. 坐标系定义
# ============================================================

def get_camera_cv_to_new() -> np.ndarray:
    """
    OpenCV 相机系:
        x: 右
        y: 下
        z: 前

    新相机系:
        x: 前
        y: 左
        z: 上

    因此:
        x_new = z_cv
        y_new = -x_cv
        z_new = -y_cv
    """
    return np.array([
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ], dtype=np.float64)


def get_world_old_to_new() -> np.ndarray:
    """
    世界坐标系变换:

        x_new = y_old
        y_new = -x_old
        z_new = z_old

    即:
        X_w_new = S_w @ X_w_old
    """
    return np.array([
        [0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)


def to_homo_4x4(S: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = S
    return T


# ============================================================
# 3. 原始 Rt 转新 Rt
# ============================================================

def convert_old_opencv_rt_to_new_rt(T_old_world_to_cv_cam: np.ndarray) -> np.ndarray:
    """
    输入:
        T_old_world_to_cv_cam

    含义:
        X_c_cv = R_old @ X_w_old + t_old

    输出:
        T_new_world_to_new_cam

    含义:
        X_c_new = R_new @ X_w_new + t_new

    公式:
        X_c_new = S_c @ X_c_cv
        X_w_new = S_w @ X_w_old
        X_w_old = S_w.T @ X_w_new

        T_new = S_c_4x4 @ T_old @ S_w_4x4.T
    """
    S_c = get_camera_cv_to_new()
    S_w = get_world_old_to_new()

    S_c_4 = to_homo_4x4(S_c)
    S_w_4 = to_homo_4x4(S_w)

    T_new_world_to_cam = S_c_4 @ T_old_world_to_cv_cam @ S_w_4.T
    return T_new_world_to_cam


# ============================================================
# 4. world -> camera 求逆得到 camera -> world
# ============================================================

def invert_rt(T_world_to_cam: np.ndarray) -> np.ndarray:
    """
    输入:
        T_world_to_cam = [R | t]

    输出:
        T_cam_to_world = [R.T | -R.T @ t]
    """
    R = T_world_to_cam[:3, :3]
    t = T_world_to_cam[:3, 3]

    T_cam_to_world = np.eye(4, dtype=np.float64)
    T_cam_to_world[:3, :3] = R.T
    T_cam_to_world[:3, 3] = -R.T @ t

    return T_cam_to_world


def scale_rt_translation(T_world_to_cam: np.ndarray, scale: float) -> np.ndarray:
    T_scaled = np.array(T_world_to_cam, dtype=np.float64, copy=True)
    T_scaled[:3, 3] *= scale
    return T_scaled


# ============================================================
# 5. 按你的 CalibratedImage / ImageProcessor 约定分解 Rt
# ============================================================

def decompose_rt_for_calibrated_image(T_world_to_cam: np.ndarray):
    """
    你的代码中:

        R_passive = Rz(yaw) @ Ry(pitch) @ Rx(roll)
        R_act = R_passive.T
        t_act = -R_act @ C

    其中:
        C = [world_x, world_y, world_z]

    因此:
        C = -R_act.T @ t_act

    输入:
        T_world_to_cam，也就是主动外参 [R_act | t_act]

    输出:
        world_x, world_y, world_z
        pitch, yaw, roll
    """
    R_act = T_world_to_cam[:3, :3]
    t_act = T_world_to_cam[:3, 3]

    # 相机中心在世界系下的位置
    C = -R_act.T @ t_act
    world_x, world_y, world_z = C.tolist()

    # 你的代码中 R_act = R_passive.T
    R_passive = R_act.T

    # R_passive = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    #
    # 对 ZYX 顺序反解:
    #   pitch = asin(-R[2, 0])
    #   yaw   = atan2(R[1, 0], R[0, 0])
    #   roll  = atan2(R[2, 1], R[2, 2])
    pitch = math.asin(-R_passive[2, 0])
    yaw = math.atan2(R_passive[1, 0], R_passive[0, 0])
    roll = math.atan2(R_passive[2, 1], R_passive[2, 2])

    return {
        "world_x": world_x,
        "world_y": world_y,
        "world_z": world_z,

        "pitch_rad": pitch,
        "yaw_rad": yaw,
        "roll_rad": roll,

        "pitch_deg": math.degrees(pitch),
        "yaw_deg": math.degrees(yaw),
        "roll_deg": math.degrees(roll),

        "R_act": R_act,
        "t_act": t_act,
    }


# ============================================================
# 6. 用分解结果重新生成 Rt，用于验证
# ============================================================

def build_rt_from_calibrated_image_params(
        world_x: float,
        world_y: float,
        world_z: float,
        pitch: float,
        yaw: float,
        roll: float,
) -> np.ndarray:
    """
    完全对应你的 ImageProcessor._Rt_active():

        R_passive = Rz(yaw) @ Ry(pitch) @ Rx(roll)
        R_act = R_passive.T
        t_act = -R_act @ C
    """
    R_passive = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    R_act = R_passive.T

    C = np.array([world_x, world_y, world_z], dtype=np.float64).reshape(3, 1)
    t_act = -R_act @ C

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R_act
    T[:3, 3] = t_act.reshape(3)

    return T


# ============================================================
# 7. 打印工具
# ============================================================

def print_matrix(name: str, M: np.ndarray):
    print(f"\n{name} =")
    np.set_printoptions(precision=10, suppress=True)
    print(M)


# ============================================================
# 8. 主流程
# ============================================================

if __name__ == "__main__":
    # ------------------------------------------------------------
    # 原始 Rt:
    # 旧世界坐标系 -> OpenCV 相机坐标系
    # 注意：这里输入平移单位是厘米；输出给 CalibratedImage/config 的单位是米。
    # ------------------------------------------------------------
    T_old_world_to_cv_cam = np.array([
        [0.9997288, 0.016541021, -0.016391674, -5.158205],
        [-0.0052490332, -0.52571559, -0.85064417, 269.0246],
        [-0.022687882, 0.85049957, -0.52548623, -278.00623],
        [0, 0, 0, 1]
    ], dtype=np.float64)
    # ------------------------------------------------------------
    # Step 1:
    # 转成新坐标系下的 world -> camera
    # ------------------------------------------------------------
    T_new_world_to_cam_cm = convert_old_opencv_rt_to_new_rt(
        T_old_world_to_cv_cam
    )
T_new_world_to_cam = scale_rt_translation(T_new_world_to_cam_cm, CM_TO_M)

print_matrix("T_new_world_to_cam_m", T_new_world_to_cam)

# ------------------------------------------------------------
# Step 2:
# 新坐标系下的 camera -> world
# ------------------------------------------------------------
T_new_cam_to_world = invert_rt(T_new_world_to_cam)

print_matrix("T_new_cam_to_world_m", T_new_cam_to_world)

# ------------------------------------------------------------
# Step 3:
# 按你的代码约定分解为位置 + 欧拉角
# ------------------------------------------------------------
params = decompose_rt_for_calibrated_image(T_new_world_to_cam)

print("\n===== CalibratedImage params =====")
print(f"world_x = {params['world_x']:.10f}")
print(f"world_y = {params['world_y']:.10f}")
print(f"world_z = {params['world_z']:.10f}")

print("\n--- radians ---")
print(f"pitch = {params['pitch_rad']:.10f}")
print(f"yaw   = {params['yaw_rad']:.10f}")
print(f"roll  = {params['roll_rad']:.10f}")

print("\n--- degrees ---")
print(f"pitch = {params['pitch_deg']:.10f}")
print(f"yaw   = {params['yaw_deg']:.10f}")
print(f"roll  = {params['roll_deg']:.10f}")

print("\n--- R_act ---")
print(params["R_act"])

print("\n--- t_act ---")
print(params["t_act"])

# ------------------------------------------------------------
# Step 4:
# 重新构造 Rt 验证
# ------------------------------------------------------------
T_rebuild = build_rt_from_calibrated_image_params(
    world_x=params["world_x"],
    world_y=params["world_y"],
    world_z=params["world_z"],
    pitch=params["pitch_rad"],
    yaw=params["yaw_rad"],
    roll=params["roll_rad"],
)

print_matrix("T_rebuild_from_decomposed_params", T_rebuild)

diff = np.linalg.norm(T_new_world_to_cam - T_rebuild)
print(f"\nrebuild diff Frobenius norm = {diff:.12e}")
