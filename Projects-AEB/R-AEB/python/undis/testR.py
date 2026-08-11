import numpy as np
import math

def euler_to_rotation_matrix(roll, pitch, yaw):
    """
    将 RPY 角转换为旋转矩阵
    参数:
        roll: X轴旋转角 (弧度)
        pitch: Y轴旋转角 (弧度)  
        yaw: Z轴旋转角 (弧度)
    返回:
        3x3 旋转矩阵
    """
    # 分别计算三个轴旋转矩阵
    # X轴旋转 (Roll)
    R_x = np.array([
        [1, 0, 0],
        [0, math.cos(roll), -math.sin(roll)],
        [0, math.sin(roll), math.cos(roll)]
    ])
    
    # Y轴旋转 (Pitch)
    R_y = np.array([
        [math.cos(pitch), 0, math.sin(pitch)],
        [0, 1, 0],
        [-math.sin(pitch), 0, math.cos(pitch)]
    ])
    
    # Z轴旋转 (Yaw)
    R_z = np.array([
        [math.cos(yaw), -math.sin(yaw), 0],
        [math.sin(yaw), math.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    # 旋转顺序: R = R_z * R_y * R_x (外旋)
    rotation_matrix = R_z @ R_y @ R_x
    return rotation_matrix.T

# 示例：使用角度制
# roll_deg, pitch_deg, yaw_deg =  -2.3165611410,31.0428831960,176.0649693732  #<pitchangle>3.1010080337524414e+01</pitchangle>
                                                                              #<headangle>1.7412255859375000e+02</headangle>
                                                                              #<yawangle>-2.3364446163177490e+00</yawangle>
   

roll_deg, pitch_deg, yaw_deg =  0.23961384594440,31.56802558898,1.4755083322525   #<pitchangle>3.1568025588989258e+01</pitchangle>
#                                                                                 <headangle>1.4755083322525024e+00</headangle>
#                                                                                 <yawangle>2.3961384594440460e-01</yawangle>                                                                         
roll = math.radians(roll_deg)
pitch = math.radians(pitch_deg)
yaw = math.radians(yaw_deg)

R_matrix = euler_to_rotation_matrix(roll, pitch, yaw)
print("手动计算的旋转矩阵:")
print(R_matrix)
# 原始向量
# v = np.array([-1.0, 0.13, 1.2])
v = np.array([3.7303263, 0.0052569, 0.82389717])
# 旋转矩阵 (R)
# R = np.array([
#     [-0.85476172, 0.05879686, -0.51567948],
#     [-0.04777424, -0.99825762, -0.03463163],
#     [-0.5168172,  -0.0049656,   0.85608138]
# ])
R = np.array([
    [0.85173671,  0.02193918, -0.52351051],
    [-0.02356079,  0.99971606,  0.00356318],
    [0.52344004 , 0.00929943,  0.85201177]
])
# 矩阵左乘向量: v' = R @ v
v_rotated = (-R) @ v
print("旋转后的向量:")
print(v_rotated)
print(f"\n结果: {v_rotated}")