# C++ Build and Run Guide

本文档说明 `cpp` 工程的四种编译版本：

```text
ARCH=x86  -> x86 本机版本，推理后端 ONNXRuntime，模型为 .onnx
ARCH=sgs  -> SGS 板端版本，推理后端 SGS IPU，模型为 .sim_sgsimg.img
ARCH=ti   -> TI J722S 板端版本，推理后端 TI TIDL/OpenVX，模型为 TIDL artifact .bin
ARCH=ti_x86 -> TI x86 仿真版本，推理后端 TI 封装 ONNXRuntime + TIDL EP，模型为 .onnx + TIDL artifacts
```

以下命令默认在工程目录执行：

```bash
cd /opt_disk3/rd234421/Projects-SGS/R-AEB/cpp
```

## 1. x86 本机版本

### 编译

已有构建目录时，直接重新编译：

```bash
cmake --build cmake-build-release-x86 --parallel 8
```

从零配置并编译：

```bash
cmake -S . -B cmake-build-release-x86 \
  -DARCH=x86 \
  -DCMAKE_BUILD_TYPE=Release \
  -G Ninja

cmake --build cmake-build-release-x86 --parallel 8
```

主要产物：

```text
cmake-build-release-x86/src/libvisper_x86.so
cmake-build-release-x86/tests/TEST_visper_x86
```

### 运行

当前 `TEST_visper_x86` 的模型路径在测试代码中硬编码：

```text
config: /opt_disk3/rd234421/Projects-SGS/R-AEB/assets/RAEB/config.jsonc
model : /opt_disk3/rd234421/Projects-SGS/R-AEB/assets/RAEB/SGS_miao.onnx
input : /opt_disk3/rd234421/Projects-SGS/R-AEB/cpp/tests/vis-bike1
```

运行默认输入：

```bash
./cmake-build-release-x86/tests/TEST_visper_x86
```

指定输入图片目录：

```bash
VISPER_INPUT_DIR=/opt_disk3/rd234421/Projects-SGS/R-AEB/cpp/tests/vis-bike1 \
./cmake-build-release-x86/tests/TEST_visper_x86
```

如需换成其它 `.onnx`，修改：

```text
tests/test_visper_x86.cpp
```

中的 `kModelPath`，然后重新编译 x86 版本。

## 2. SGS 板端版本

### 编译

已有构建目录时，直接重新编译：

```bash
cmake --build cmake-build-release-sgs --parallel 8
```

从零配置并编译：

```bash
cmake -S . -B cmake-build-release-sgs \
  -DARCH=sgs \
  -DCMAKE_BUILD_TYPE=Release \
  -DVISPER_OPENCV_ARM_DIR=/opt_disk3/rd234421/Projects-SGS/R-AEB/cpp/libs/arm/opencv-armhf-split-static-4.9.0 \
  -G Ninja

cmake --build cmake-build-release-sgs --parallel 8
```

当前 SGS 编译使用的环境在 CMake 中配置为：

```text
/opt_disk3/rd234421/Projects-SGS/R-AEB/SGS-Environment/Iford_IMD00V2.1.0
```

主要产物：

```text
cmake-build-release-sgs/src/libvisper.so
cmake-build-release-sgs/tests/TEST_visper_sgs
cmake-build-release-sgs/tests/TEST_model_sgs
```

### 板端运行

把以下文件放到 SGS 板端同一个目录，例如 `/opt/VisPer_sgs`：

```text
libvisper.so
TEST_visper_sgs
TEST_model_sgs
assets/config.jsonc
assets/<model>.sim_sgsimg.img
img/<input images>
```

进入板端目录：

```bash
cd /opt/VisPer_sgs
chmod +x TEST_visper_sgs TEST_model_sgs
export LD_LIBRARY_PATH=$PWD:$LD_LIBRARY_PATH
```

模型冒烟测试：

```bash
./TEST_model_sgs \
  -m ./assets/lyl_fixed.sim_sgsimg.img \
  -p 1 \
  -n 20
```

完整 RAEB 图片流程：

```bash
./TEST_visper_sgs \
  -t RAEB \
  -c ./assets/config.jsonc \
  -m ./assets/lyl_fixed.sim_sgsimg.img \
  -i ./img \
  -o ./vis \
  -r
```

## 3. TI J722S 板端版本

### 编译

已有构建目录时，直接重新编译：

```bash
cmake --build cmake-build-release-ti --parallel 8
```

从零配置并编译：

```bash
cmake -S . -B cmake-build-release-ti \
  -DARCH=ti \
  -DCMAKE_BUILD_TYPE=Release \
  -DPSDK_RTOS_ROOT=/opt_disk3/rd234421/Projects-SGS/R-AEB/TI-Environment/ti-processor-sdk-rtos-j722s-evm-11_02_00_10 \
  -DTARGET_FS=/opt_disk3/rd234421/Projects-SGS/R-AEB/TI-Environment/tisdk-adas-image-j722s-evm \
  -DTI_DEP_LIB_DIR=/opt_disk3/rd234421/Projects-SGS/R-AEB/TI-Environment/lzq_lib \
  -DTOOLCHAIN_ROOT=/opt_disk3/rd234421/Projects-SGS/R-AEB/TI-Environment/arm-gnu-toolchain-13.2.Rel1-x86_64-aarch64-none-linux-gnu \
  -G Ninja

cmake --build cmake-build-release-ti --parallel 8
```

当前 TI 编译使用的是 `R-AEB/TI-Environment`，不是下面两个 TIDL 转换工具目录：

```text
/opt_disk3/rd234421/Projects-SGS/edgeai-tidl-tools
/opt_disk3/rd234421/Projects-SGS/edgeai-tidl-tools-2
```

主要产物：

```text
cmake-build-release-ti/src/libvisper.so
cmake-build-release-ti/src/libvisper.so.2.8.7
cmake-build-release-ti/tests/TEST_visper_ti
cmake-build-release-ti/tests/TEST_model_ti
```

### 板端部署目录

当前已整理好的部署目录：

```text
/opt_disk3/rd234421/Projects-SGS/R-AEB/VisPer_ti
```

建议拷到板端 `/opt/VisPer_ti`，目录结构保持：

```text
/opt/VisPer_ti/
├── TEST_visper_ti
├── TEST_model_ti
├── libvisper.so
├── libvisper.so.2.8.7
├── assets/
│   ├── config.jsonc
│   ├── raeb_tidl_net.bin
│   └── raeb_tidl_io.bin
├── img/
└── vis/
```

注意：TI 版本 `-m` 传的是 TIDL artifact，不是 `.onnx`。`raeb_tidl_net.bin` 和 `raeb_tidl_io.bin` 必须在同一个目录。

### 板端运行

进入板端目录：

```bash
cd /opt/VisPer_ti
chmod +x TEST_visper_ti TEST_model_ti
ln -sf libvisper.so.2.8.7 libvisper.so
export LD_LIBRARY_PATH=$PWD:/usr/lib:$LD_LIBRARY_PATH
```

模型冒烟测试：

```bash
./TEST_model_ti \
  -m ./assets/raeb_tidl_net.bin \
  -p 1 \
  -n 20 \
  -d DSP_C7-2
```

完整 RAEB 图片流程：

```bash
./TEST_visper_ti \
  -t RAEB \
  -c ./assets/config.jsonc \
  -m ./assets/raeb_tidl_net.bin \
  -i ./img \
  -o ./vis \
  -r \
  -d DSP_C7-2
```

如果板端 C7_2 firmware 或 remoteproc 没有就绪，可以改用：

```bash
-d DSP_C7-1
```

## 4. TI x86 TIDL 仿真版本

这个版本运行在 x86 主机，但不是普通 CPU ONNXRuntime。它使用 TI 封装过的 ONNXRuntime TIDL Execution Provider，加载 `.onnx` 模型和已经量化/编译好的 TIDL artifacts，在 x86 上做 TIDL 仿真运行。

### 编译

推荐直接用脚本：

```bash
cd /opt_disk3/rd234421/Projects-SGS/R-AEB/cpp
scripts/build_ti_x86.sh
```

脚本默认使用：

```text
TIDL tools : /opt_disk3/rd234421/Projects-SGS/edgeai-tidl-tools-2/tools/J722S/tidl_tools
ORT headers: /opt_disk3/rd234421/Projects-SGS/R-AEB/TI-Environment/ti-processor-sdk-rtos-j722s-evm-11_00_00_06/targetfs/usr/include/onnxruntime/include
ORT library: /opt/conda/envs/tidl2/lib/python3.10/site-packages/onnxruntime/capi/libonnxruntime.so.1.23.0
build dir  : /opt_disk3/rd234421/Projects-SGS/R-AEB/cpp/cmake-build-release-ti-x86-ort123
```

主要产物：

```text
cmake-build-release-ti-x86-ort123/src/libvisper_ti_x86.so
cmake-build-release-ti-x86-ort123/tests/TEST_visper_ti_x86
cmake-build-release-ti-x86-ort123/ort_libs/libonnxruntime.so.1
```

等价的手动 CMake 命令：

```bash
cmake -S . -B cmake-build-release-ti-x86-ort123 \
  -DARCH=ti_x86 \
  -DCMAKE_BUILD_TYPE=Release \
  -DTI_TIDL_TOOLS_ROOT=/opt_disk3/rd234421/Projects-SGS/edgeai-tidl-tools-2 \
  -DTI_X86_SOC=J722S \
  -DTI_X86_TIDL_TOOLS_PATH=/opt_disk3/rd234421/Projects-SGS/edgeai-tidl-tools-2/tools/J722S/tidl_tools \
  -DTI_X86_ONNXRT_ROOT=/opt/conda/envs/tidl2/lib/python3.10/site-packages/onnxruntime/capi \
  -DTI_X86_ONNXRT_INCLUDE_ROOT=/opt_disk3/rd234421/Projects-SGS/R-AEB/TI-Environment/ti-processor-sdk-rtos-j722s-evm-11_00_00_06/targetfs/usr/include/onnxruntime/include \
  -DTI_X86_ONNXRT_LIB_DIR=/opt/conda/envs/tidl2/lib/python3.10/site-packages/onnxruntime/capi \
  -DTI_X86_ONNXRT_LIB=/opt/conda/envs/tidl2/lib/python3.10/site-packages/onnxruntime/capi/libonnxruntime.so.1.23.0 \
  -DTI_X86_TIDL_PROVIDER_API=options \
  -G Ninja

cmake --build cmake-build-release-ti-x86-ort123 --parallel 8
```

### 运行

推荐直接用脚本：

```bash
cd /opt_disk3/rd234421/Projects-SGS/R-AEB/cpp
scripts/run_ti_x86.sh
```

脚本默认使用：

```text
model    : /opt_disk3/rd234421/Projects-SGS/R-AEB/assets/RAEB/TI_lyl.onnx
config   : /opt_disk3/rd234421/Projects-SGS/R-AEB/assets/RAEB/config.jsonc
input    : /opt_disk3/rd234421/Projects-SGS/R-AEB/cpp/tests/vis-bike1_ti_x86
artifacts: /opt_disk3/rd234421/Projects-SGS/edgeai-tidl-tools-2/runtimes/examples/model-artifacts/TI_lyl/artifacts
```

输出不会覆盖旧结果。每次运行都会生成新的目录：

```text
/opt_disk3/rd234421/Projects-SGS/R-AEB/cpp/ti_x86_runs/run_YYYYmmdd_HHMMSS_PID/vis
```

指定单张图片运行：

```bash
INPUT=/opt_disk3/rd234421/Projects-SGS/R-AEB/cpp/tests/vis-bike1_ti_x86/raeb_vis_ori_1767254559855.jpg \
scripts/run_ti_x86.sh
```

指定自定义输出根目录：

```bash
OUT_ROOT=/opt_disk3/rd234421/Projects-SGS/R-AEB/cpp/ti_x86_runs \
RUN_NAME=my_tidl_x86_test_001 \
scripts/run_ti_x86.sh
```

手动运行命令示例：

```bash
cd /opt_disk3/rd234421/Projects-SGS/R-AEB/cpp

export SOC=J722S
export TIDL_TOOLS_PATH=/opt_disk3/rd234421/Projects-SGS/edgeai-tidl-tools-2/tools/J722S/tidl_tools
export VISPER_TIDL_ARTIFACTS_DIR=/opt_disk3/rd234421/Projects-SGS/edgeai-tidl-tools-2/runtimes/examples/model-artifacts/TI_lyl/artifacts
export LD_LIBRARY_PATH=$PWD/cmake-build-release-ti-x86-ort123/src:$PWD/cmake-build-release-ti-x86-ort123/ort_libs:/opt/conda/envs/tidl2/lib/python3.10/site-packages/onnxruntime/capi:$TIDL_TOOLS_PATH:$LD_LIBRARY_PATH

RUN_DIR=$PWD/ti_x86_runs/manual_$(date +%Y%m%d_%H%M%S)
mkdir -p "$RUN_DIR"
cd "$RUN_DIR"

/opt_disk3/rd234421/Projects-SGS/R-AEB/cpp/cmake-build-release-ti-x86-ort123/tests/TEST_visper_ti_x86 \
  -t RAEB \
  -c /opt_disk3/rd234421/Projects-SGS/R-AEB/assets/RAEB/config.jsonc \
  -m /opt_disk3/rd234421/Projects-SGS/R-AEB/assets/RAEB/TI_lyl.onnx \
  -i /opt_disk3/rd234421/Projects-SGS/R-AEB/cpp/tests/vis-bike1_ti_x86 \
  -o vis
```

### 注意

`ARCH=ti_x86` 和 `ARCH=ti` 用的是同一套上层 C++ 业务逻辑、预处理、后处理、跟踪和可视化；差别在推理后端：

```text
ARCH=ti     -> 板端 OpenVX/TIDLRT，输入是 TIDL .bin artifact
ARCH=ti_x86 -> x86 TI ONNXRuntime + TIDL EP，输入是 .onnx + artifacts 目录
```

`TI_lyl.onnx` 必须和 `TI_lyl/artifacts` 对应，否则可能出现 provider 初始化失败或输出 shape 不匹配。

## 4. 常用检查命令

确认 TI 产物是 aarch64：

```bash
file cmake-build-release-ti/src/libvisper.so.2.8.7 \
     cmake-build-release-ti/tests/TEST_visper_ti \
     cmake-build-release-ti/tests/TEST_model_ti
```

确认构建目录配置：

```bash
rg -n "^(ARCH|CMAKE_BUILD_TYPE|PSDK_RTOS_ROOT|TARGET_FS|TI_DEP_LIB_DIR|TOOLCHAIN_ROOT|OpenCV_DIR|onnxruntime_DIR|VISPER_OPENCV_ARM_DIR):" \
  cmake-build-release-x86/CMakeCache.txt \
  cmake-build-release-sgs/CMakeCache.txt \
  cmake-build-release-ti/CMakeCache.txt
```
