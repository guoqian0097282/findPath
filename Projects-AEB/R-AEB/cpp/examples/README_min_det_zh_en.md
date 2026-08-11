# RAEB Minimal Detection/Segmentation Example / RAEB 最小检测分割示例

## 中文说明

### 1. 目录内容

本示例代码只放在 `cpp/examples` 下，核心目录是：

```bash
examples/min_det_x86/
├── CMakeLists.txt
├── main.cpp
├── min_det.cpp
├── min_det.hpp
└── assets/
    ├── raeb/
    │   ├── config.jsonc
    │   └── TI_lyl.onnx
    ├── artifacts/
    │   ├── allowedNode.txt
    │   ├── onnxrtMetaData.txt
    │   ├── subgraph_0_tidl_io_1.bin
    │   └── subgraph_0_tidl_net.bin
    └── inputs/
        └── raeb_vis_ori_1767254411991.jpg
```

功能：

- 读取 RAEB YOLOv11n 量化模型输出。
- 只做最小后处理：置信度过滤、可选 NMS、画框、叠加分割 mask。
- 不做完整 RAEB 的 cuboid、track、angle 可视化等后处理。

打包给别人时可以直接打包 `examples` 目录：

```bash
cd /opt_disk3/rd234421/Projects-SGS/R-AEB/cpp
tar -czf raeb_min_det_examples.tar.gz examples
```

注意：`examples` 目录包含示例源码和输入资产，但编译仍依赖完整的 `R-AEB/cpp` 工程环境，包括 `src/`、`libs/`、顶层 `CMakeLists.txt` 和对应 TI SDK/ONNXRuntime 依赖。接收方应把该目录放回同结构工程的 `cpp/examples` 下再编译。

### 2. x86 TI 量化仿真环境

在本机 x86 上运行 TI TIDL/ONNXRuntime 仿真，需要先进入 R-AEB 的 `cpp` 目录：

```bash
cd /opt_disk3/rd234421/Projects-SGS/R-AEB/cpp
```

如果使用当前机器已有环境，先设置：

```bash
export SOC=J722S
export TIDL_TOOLS_PATH=/opt_disk3/rd234421/Projects-SGS/edgeai-tidl-tools-2/tools/J722S/tidl_tools
export ORT_LIB_DIR=/opt/conda/envs/tidl2/lib/python3.10/site-packages/onnxruntime/capi
export LD_LIBRARY_PATH="$PWD/cmake-build-release-ti-x86-ort123/src:$PWD/cmake-build-release-ti-x86-ort123/ort_libs:$ORT_LIB_DIR:$TIDL_TOOLS_PATH:${LD_LIBRARY_PATH:-}"
```

如果对方机器路径不同，需要对应修改 `TIDL_TOOLS_PATH` 和 `ORT_LIB_DIR`。

### 3. 编译 x86 TI 仿真版本

当前工程如果已经有 `cmake-build-release-ti-x86-ort123`，直接编译：

```bash
cmake --build cmake-build-release-ti-x86-ort123 --target MIN_det_ti_x86 --parallel 8
```

可执行文件位置：

```bash
cmake-build-release-ti-x86-ort123/examples/min_det_x86/MIN_det_ti_x86
```

### 4. 运行 x86 TI 仿真

默认会画检测框并叠加分割 mask：

```bash
cmake-build-release-ti-x86-ort123/examples/min_det_x86/MIN_det_ti_x86 \
  -c examples/min_det_x86/assets/raeb/config.jsonc \
  -m examples/min_det_x86/assets/raeb/TI_lyl.onnx \
  -a examples/min_det_x86/assets/artifacts \
  -i examples/min_det_x86/assets/inputs/raeb_vis_ori_1767254411991.jpg \
  -o min_det_ti_x86_result \
  --conf 0.5 \
  --max-det 50 \
  --dump-outputs
```

输出图片在：

```bash
min_det_ti_x86_result/
```

如果只想画框，不叠加 mask：

```bash
cmake-build-release-ti-x86-ort123/examples/min_det_x86/MIN_det_ti_x86 \
  -c examples/min_det_x86/assets/raeb/config.jsonc \
  -m examples/min_det_x86/assets/raeb/TI_lyl.onnx \
  -a examples/min_det_x86/assets/artifacts \
  -i examples/min_det_x86/assets/inputs/raeb_vis_ori_1767254411991.jpg \
  -o min_det_box_only_result \
  --conf 0.5 \
  --max-det 50 \
  --no-mask
```

### 5. 编译板端版本

板端版本在 x86 主机上交叉编译，使用 R-AEB 工程已有的 TI build 目录：

```bash
cd /opt_disk3/rd234421/Projects-SGS/R-AEB/cpp
cmake --build cmake-build-release-ti --target MIN_det_ti --parallel 8
```

可执行文件位置：

```bash
cmake-build-release-ti/examples/min_det_x86/MIN_det_ti
```

检查是不是 ARM aarch64：

```bash
file cmake-build-release-ti/examples/min_det_x86/MIN_det_ti
```

### 6. 板端运行文件组织

在板子上建议组织成：

```bash
min_det_ti_run/
├── MIN_det_ti
├── config.jsonc
├── artifacts/
│   ├── onnxrtMetaData.txt
│   ├── subgraph_0_tidl_io_1.bin
│   └── subgraph_0_tidl_net.bin
├── inputs/
│   └── raeb_vis_ori_1767254411991.jpg
├── outputs/
└── libs/
    └── libvisper.so.2.8.7
```

说明：

- 板端优先使用系统 `/usr/lib/libtivision_apps.so.11.0.0`，不要优先抢本地打包的 `libtivision_apps`。
- 如果带了本地 `libtivision_apps.so*` 或 `libvx_tidl_rt.so*`，建议先改名禁用，除非确认版本和板子一致。
- `libvisper.so.2.8.7` 来自 `cmake-build-release-ti/src/`。

### 7. 板端运行命令

在板子上：

```bash
cd /media/record/min_det_ti_run

export LD_LIBRARY_PATH=/usr/lib:/app/lib:$PWD/libs:$PWD:${LD_LIBRARY_PATH:-}

ldd ./MIN_det_ti | egrep "tivision|visper|opencv"
```

期望 `libtivision_apps` 指向系统库：

```bash
libtivision_apps.so.11.0.0 => /usr/lib/libtivision_apps.so.11.0.0
```

运行：

```bash
./MIN_det_ti \
  -c ./config.jsonc \
  -m ./artifacts \
  -d DSP_C7-1 \
  -i ./inputs/raeb_vis_ori_1767254411991.jpg \
  -o ./outputs \
  --conf 0.5 \
  --max-det 50 \
  --dump-outputs
```

如果 `DSP_C7-1` 不可用，可换成：

```bash
-d DSP_C7-2
```

只画框、不叠加分割：

```bash
./MIN_det_ti \
  -c ./config.jsonc \
  -m ./artifacts \
  -d DSP_C7-1 \
  -i ./inputs/raeb_vis_ori_1767254411991.jpg \
  -o ./outputs_box_only \
  --conf 0.5 \
  --max-det 50 \
  --no-mask
```

### 8. 常用参数

```bash
-c, --config <path>       RAEB config.jsonc
-m, --model <path>        x86/ti_x86 传 ONNX；板端传 artifacts 目录
-a, --artifacts <dir>     TI x86 仿真 artifacts 目录
-d, --ti-target <target>  板端目标核，DSP_C7-1 或 DSP_C7-2
-i, --input <path>        输入图片或目录
-o, --outdir <dir>        输出目录
--conf <value>            检测置信度阈值
--nms <value>             NMS IoU 阈值
--mask-thresh <value>     mask 二值化阈值
--max-det <num>           最多保留检测数量
--no-mask                 不叠加分割 mask
--no-nms                  不做 NMS
--dump-outputs            打印模型输出 shape
```

### 9. 已知现象

部分 TI SDK 环境下，程序推理结束、`APP: Deinit ... Done !!!` 后可能出现 `Segmentation fault`。如果输出图片已经生成，这通常是运行时清理阶段的问题，先以 `outputs/` 下结果为准。

---

## English Guide

### 1. Directory Contents

This minimal example is self-contained under `cpp/examples`. Main directory:

```bash
examples/min_det_x86/
├── CMakeLists.txt
├── main.cpp
├── min_det.cpp
├── min_det.hpp
└── assets/
    ├── raeb/
    │   ├── config.jsonc
    │   └── TI_lyl.onnx
    ├── artifacts/
    │   ├── allowedNode.txt
    │   ├── onnxrtMetaData.txt
    │   ├── subgraph_0_tidl_io_1.bin
    │   └── subgraph_0_tidl_net.bin
    └── inputs/
        └── raeb_vis_ori_1767254411991.jpg
```

What it does:

- Runs the RAEB YOLOv11n-derived quantized model outputs.
- Applies only minimal post-processing: confidence filter, optional NMS, box drawing, and segmentation mask overlay.
- Does not run the full RAEB cuboid, tracking, angle visualization, or other production post-processing.

Package command:

```bash
cd /opt_disk3/rd234421/Projects-SGS/R-AEB/cpp
tar -czf raeb_min_det_examples.tar.gz examples
```

Note: `examples` contains the example source code and input assets, but it is not a fully standalone project. Building still depends on the full `R-AEB/cpp` tree, including `src/`, `libs/`, the top-level `CMakeLists.txt`, and the configured TI SDK/ONNXRuntime dependencies. The receiver should place this directory back under `cpp/examples` in the same project layout before building.

### 2. x86 TI Emulation Environment

On the x86 host:

```bash
cd /opt_disk3/rd234421/Projects-SGS/R-AEB/cpp
```

For the current machine:

```bash
export SOC=J722S
export TIDL_TOOLS_PATH=/opt_disk3/rd234421/Projects-SGS/edgeai-tidl-tools-2/tools/J722S/tidl_tools
export ORT_LIB_DIR=/opt/conda/envs/tidl2/lib/python3.10/site-packages/onnxruntime/capi
export LD_LIBRARY_PATH="$PWD/cmake-build-release-ti-x86-ort123/src:$PWD/cmake-build-release-ti-x86-ort123/ort_libs:$ORT_LIB_DIR:$TIDL_TOOLS_PATH:${LD_LIBRARY_PATH:-}"
```

If the receiver uses different paths, update `TIDL_TOOLS_PATH` and `ORT_LIB_DIR`.

### 3. Build x86 TI Emulation Binary

If the existing build directory is available:

```bash
cmake --build cmake-build-release-ti-x86-ort123 --target MIN_det_ti_x86 --parallel 8
```

Binary:

```bash
cmake-build-release-ti-x86-ort123/examples/min_det_x86/MIN_det_ti_x86
```

### 4. Run x86 TI Emulation

By default, the output image includes boxes and segmentation mask overlay:

```bash
cmake-build-release-ti-x86-ort123/examples/min_det_x86/MIN_det_ti_x86 \
  -c examples/min_det_x86/assets/raeb/config.jsonc \
  -m examples/min_det_x86/assets/raeb/TI_lyl.onnx \
  -a examples/min_det_x86/assets/artifacts \
  -i examples/min_det_x86/assets/inputs/raeb_vis_ori_1767254411991.jpg \
  -o min_det_ti_x86_result \
  --conf 0.5 \
  --max-det 50 \
  --dump-outputs
```

Results are written to:

```bash
min_det_ti_x86_result/
```

Box-only mode:

```bash
cmake-build-release-ti-x86-ort123/examples/min_det_x86/MIN_det_ti_x86 \
  -c examples/min_det_x86/assets/raeb/config.jsonc \
  -m examples/min_det_x86/assets/raeb/TI_lyl.onnx \
  -a examples/min_det_x86/assets/artifacts \
  -i examples/min_det_x86/assets/inputs/raeb_vis_ori_1767254411991.jpg \
  -o min_det_box_only_result \
  --conf 0.5 \
  --max-det 50 \
  --no-mask
```

### 5. Build Target Board Binary

Cross-compile on the x86 host using the existing TI build directory:

```bash
cd /opt_disk3/rd234421/Projects-SGS/R-AEB/cpp
cmake --build cmake-build-release-ti --target MIN_det_ti --parallel 8
```

Binary:

```bash
cmake-build-release-ti/examples/min_det_x86/MIN_det_ti
```

Check architecture:

```bash
file cmake-build-release-ti/examples/min_det_x86/MIN_det_ti
```

### 6. Target Board Runtime Layout

Recommended layout on the board:

```bash
min_det_ti_run/
├── MIN_det_ti
├── config.jsonc
├── artifacts/
│   ├── onnxrtMetaData.txt
│   ├── subgraph_0_tidl_io_1.bin
│   └── subgraph_0_tidl_net.bin
├── inputs/
│   └── raeb_vis_ori_1767254411991.jpg
├── outputs/
└── libs/
    └── libvisper.so.2.8.7
```

Notes:

- Prefer the board system `/usr/lib/libtivision_apps.so.11.0.0`.
- Do not override it with packaged `libtivision_apps.so*` or `libvx_tidl_rt.so*` unless versions are confirmed.
- `libvisper.so.2.8.7` comes from `cmake-build-release-ti/src/`.

### 7. Run on Target Board

On the board:

```bash
cd /media/record/min_det_ti_run

export LD_LIBRARY_PATH=/usr/lib:/app/lib:$PWD/libs:$PWD:${LD_LIBRARY_PATH:-}

ldd ./MIN_det_ti | egrep "tivision|visper|opencv"
```

Expected `libtivision_apps` resolution:

```bash
libtivision_apps.so.11.0.0 => /usr/lib/libtivision_apps.so.11.0.0
```

Run:

```bash
./MIN_det_ti \
  -c ./config.jsonc \
  -m ./artifacts \
  -d DSP_C7-1 \
  -i ./inputs/raeb_vis_ori_1767254411991.jpg \
  -o ./outputs \
  --conf 0.5 \
  --max-det 50 \
  --dump-outputs
```

If needed, use the second C7x target:

```bash
-d DSP_C7-2
```

Box-only mode:

```bash
./MIN_det_ti \
  -c ./config.jsonc \
  -m ./artifacts \
  -d DSP_C7-1 \
  -i ./inputs/raeb_vis_ori_1767254411991.jpg \
  -o ./outputs_box_only \
  --conf 0.5 \
  --max-det 50 \
  --no-mask
```

### 8. Useful Options

```bash
-c, --config <path>       RAEB config.jsonc
-m, --model <path>        ONNX for x86/ti_x86; artifacts directory for target board
-a, --artifacts <dir>     TIDL artifacts directory for x86 TI emulation
-d, --ti-target <target>  Target board C7x target, DSP_C7-1 or DSP_C7-2
-i, --input <path>        Input image or directory
-o, --outdir <dir>        Output directory
--conf <value>            Confidence threshold
--nms <value>             NMS IoU threshold
--mask-thresh <value>     Mask binarization threshold
--max-det <num>           Maximum detections per image
--no-mask                 Disable segmentation mask overlay
--no-nms                  Disable NMS
--dump-outputs            Print model output tensor shapes
```

### 9. Known Behavior

On some TI SDK environments, the program may print `Segmentation fault` after inference finishes and `APP: Deinit ... Done !!!`. If the output image has already been generated, this is likely a runtime cleanup issue. Check the `outputs/` directory first.
