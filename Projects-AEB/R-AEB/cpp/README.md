q---

# VisPer 模块说明与使用指南

VisPer 模块提供完整的任务初始化、图像预处理、推理、后处理、跟踪、立体结构分析以及可视化能力，并支持 ARM 设备部署。

---

## 1. 发布内容

### **核心库**

| 文件                     | 描述                        |
|------------------------|---------------------------|
| **libvisper.so**       | 主共享库                      |
| **libvisper.so.x.y.z** | 含版本号的共享库文件（x.y.z 为语义化版本号） |

### **头文件**

| 头文件          | 描述                                     |
|--------------|----------------------------------------|
| **VisPer.h** | VisPer API 入口，包含初始化、推理、取结果、可视化、回调注册等接口 |

### **测试程序**

| 可执行文件               | 描述                      |
|---------------------|-------------------------|
| **TEST_model_arm**  | 最小冒烟测试（验证模型加载与基本推理流程）   |
| **TEST_visper_arm** | 完整功能测试（含预处理/推理/后处理/可视化） |

---

## 2. 功能概述

### **RAEB（前向碰撞预警）管线**

VisPer 对 RAEB 提供完整 C++ 推理链路：

* **预处理（Fisheye → Cylindrical）**
* **推理执行（离线模型 / ONNX / ARM 固件模型）**
* **检测结果解析（objs / masks / angles）**
* **三维立方体生成（Cuboids）**
* **多目标跟踪（ID 分配、重排序）**
* **可视化绘制（2D 分割叠加、3D Cuboid 投影）**

返回结构与 Python SDK 对齐，便于跨语言使用。

### **OP（其他任务）扩展**

保留统一上下文结构，可按相同 API 接入其他模型任务。

### **回调机制**

支持用户注册 RAEB / OP 推理完成后的回调，便于实时处理或外部日志记录。

---

## 3. 目录结构说明

```
VisPer/
├── VisPer.h                   —— 对外统一 API 头文件
├── libvisper.so              —— 主共享库（符号链接）
├── libvisper.so.x.y.z        —— 带版本号的共享库文件（实际文件）
│
├── assets/
│   └── RAEB/
│       ├── config.jsonc                  —— RAEB 配置（相机内参、后处理阈值、类别信息等）
│       ├── <MODEL>_fixed.sim_sgsimg.img  —— ARM 平台离线模型
│       ├── visper_640x320_u.bin          —— 畸变校正表（U）
│       ├── visper_640x320_v.bin          —— 畸变校正表（V）
│
├── test_data/                —— 测试图片数据集（可用作验证流程输入）
│
└── libopencv_world.so       —— OpenCV 运行时依赖（与目标平台 ABI 一致）
```

---

## 4. 冒烟测试（TEST_model_arm）

用于快速验证以下能力：

* 模型能否正常加载
* 推理是否可正常运行
* 多进程情况下模型句柄是否稳定

### **运行参数**

```
Usage: TEST_model_arm
       -m <model_path> -p <process_count> -n <iters_per_process>

  -m <model_path>       RAEB 离线模型路径
  -p <process_count>    进程数量 (>0)
  -n <iters_per_proc>   每个进程的推理迭代次数 (>0)
```

### **示例**

```
./TEST_model_arm -m assets/RAEB/Donut_fixed.sim_sgsimg.img -p 1 -n 20
```

---

## 5. 完整测试（TEST_visper_arm）

测试 RAEB 全流程，包括：

* Task 初始化
* 配置与模型加载
* 图像预处理与推理
* 后处理（检测、mask、cuboid、追踪）
* 可视化生成与保存

### **运行参数**

```
Usage:
  TEST_visper_arm
  -t <task> -c <config_path> -m <model_path> -i <image_path> [-o <out_dir>] [-r] [-l]

  -t, --task        任务名（如 RAEB）
  -c, --config      配置文件路径（json/jsonc）
  -m, --model       模型文件路径（如 .onnx / .img）
  -i, --input       输入图片文件或文件夹
  -o, --outdir      可视化输出目录（默认：vis）
  -r, --recursive   递归读取输入目录
  -l, --loop        循环执行，直到手动中断
  -h, --help        显示帮助信息
```

### **示例**

```
./TEST_visper_arm \
    -t RAEB \
    -c ./assets/RAEB/config.jsonc \
    -m ./assets/RAEB/Donut_fixed.sim_sgsimg.img \
    -i test_data/
```

生成的可视化图片会保存至 `vis/` 或指定目录。

---

## 6. 运行依赖

| 库                              | 说明                 |
|--------------------------------|--------------------|
| **OpenCV（libopencv_world.so）** | 图像处理与可视化           |
| ARM Toolchain                  | 运行环境需支持相应 glibc 版本 |
| 设备需加载畸变表（U/V bin 文件）           | 用于 Cylindrical 展开  |

---
