# x86 仿真版本
# 编译
cd /opt_disk3/rd234421/Projects-SGS/R-AEB/cpp

  rm -rf cmake-build-release-ti-x86
  cmake -S . -B cmake-build-release-ti-x86 \
    -DARCH=ti_x86 \
    -DCMAKE_BUILD_TYPE=Release

  cmake --build cmake-build-release-ti-x86 \
    --target TEST_visper_ti_x86 \
    --parallel 8

# 运行
export VISPER_TIDL_ARTIFACTS_DIR=/opt_disk3/rd234421/Projects-SGS/edgeai-tidl-tools-2/runtimes/examples/model-artifacts/ti_lyl_user_16bit_layers_rerun_20260703/artifacts
export TIDL_TOOLS_PATH=/opt_disk3/rd234421/Projects-SGS/edgeai-tidl-tools-2/tools/J722S/tidl_tools
export LD_LIBRARY_PATH=/opt_disk3/rd234421/Projects-SGS/edgeai-tidl-tools-2/tools/J722S/tidl_tools:$LD_LIBRARY_PATH

./cmake-build-release-ti-x86/tests/TEST_visper_ti_x86 \
-t RAEB \
-c /opt_disk3/rd234421/Projects-SGS/R-AEB/assets/RAEB/config.jsonc \
-m /opt_disk3/rd234421/Projects-SGS/edgeai-tidl-tools-2/runtimes/examples/model-artifacts/ti_lyl_user_16bit_layers_rerun_20260703/TI_lyl.onnx \
-i /opt_disk3/rd234421/Projects-SGS/R-AEB/cpp/tests/vis-bike1/raeb_vis_ori_1767254560813.jpg \
-o ./ti_x86_runs/one \
-d DSP_C7-1

# x86 跑 ONNX 的编译命令：
cd /opt_disk3/rd234421/Projects-SGS/R-AEB/cpp

  cmake -S . -B cmake-build-release-x86 \
    -DARCH=x86 \
    -DCMAKE_BUILD_TYPE=Debug

  cmake --build cmake-build-release-x86 \
    --target TEST_visper_x86 \
    --parallel 8
产物位置：

  cmake-build-release-x86/tests/TEST_visper_x86
  cmake-build-release-x86/src/libvisper_x86.so  

运行命令示例：

  ./cmake-build-release-x86/tests/TEST_visper_x86 \
    -t RAEB \
    -c /home/gq/guoqian/Projects-AEB/R-AEB/assets/RAEB/config.jsonc \
    -m /home/gq/guoqian/Projects-AEB/R-AEB/assets/RAEB/TI_lyl.onnx \
    -i /home/gq/guoqian/Projects-AEB/R-AEB/cpp/tests/vis-bike1/raeb_vis_ori_1767254560813.jpg \
    -o ./x86_runs/one

  
# 1100-patch版本板端编译
  # 从零重新配置编译：
  cd /opt_disk3/rd234421/Projects-SGS/R-AEB/cpp

  cmake -S . -B cmake-build-release-ti-1100-patch \
    -DARCH=ti \
    -DTI_SDK_VERSION=1100-patch \
    -DCMAKE_BUILD_TYPE=Release

  cmake --build cmake-build-release-ti-1100-patch --target clean

  cmake --build cmake-build-release-ti-1100-patch \
    --target TEST_visper_ti \
    --parallel 8

  # 快速编译
  
  cd /opt_disk3/rd234421/Projects-SGS/R-AEB/cpp

  cmake --build cmake-build-release-ti-1100-patch \
    --target TEST_visper_ti \
    --parallel 8

# 整理拷贝板端运行需要的文件

  cp -a cmake-build-release-ti-1100-patch/TEST_visper_ti \
    /opt_disk3/rd234421/Projects-SGS/R-AEB/VisPer_ti/

  cp -a cmake-build-release-ti-1100-patch/src/libvisper.so* \
    /opt_disk3/rd234421/Projects-SGS/R-AEB/VisPer_ti/

  cp -a include/VisPer.h include/VisPer_c.h \
    /opt_disk3/rd234421/Projects-SGS/R-AEB/VisPer_ti/

# 板端运行

  # 跑单张图（不可视化）
  
    cd /media/record/VisPer_ti
    ./TEST_visper_ti \
        -t RAEB \
        -c ./assets/config.jsonc \
        -m ./assets/raeb_tidl_net.bin \
        -i ./img/raeb_vis_ori_1767254400804.jpg \
        -o ./vis \
        -d DSP_C7-2 2>&1 | tee run_board_one.log
    sync

  # 跑单张图（可视化）
    cd /media/record/VisPer_ti
    export LD_LIBRARY_PATH=$PWD:$PWD/libs:/usr/lib:/app/lib:$LD_LIBRARY_PATH
    touch visl
    mkdir -p vis

    ./TEST_visper_ti \
        -t RAEB \
        -c ./assets/config.jsonc \
        -m ./assets/raeb_tidl_net.bin \
        -i ./img/raeb_vis_ori_1767254400804.jpg \
        -o ./vis \
        -d DSP_C7-2 2>&1 | tee run_board_one.log

    sync

  # 批量跑
  
  cd /media/record/VisPer_ti
  export LD_LIBRARY_PATH=$PWD:$PWD/libs:/usr/lib:/app/lib:$LD_LIBRARY_PATH
  chmod +x TEST_visper_ti TEST_model_ti run_visper_ti.sh

  # 只看 TIDL perf，不打印每帧结果
  export VISPER_LOG_LEVEL=0
  export VISPER_TI_PRINT_PERF=1  # 打印资源耗时的情况

  # 用命令跑
  ./TEST_visper_ti \
    -t RAEB \
    -c ./assets/config.jsonc \
    -m ./assets/raeb_tidl_net.bin \
    -i ./img \
    -o ./vis \
    -r \
    -d DSP_C7-1 2>&1 | tee run_perf.log

  # 用脚本跑  
  ./run_visper_ti.sh DSP_C7-1 2>&1 | tee run_board.log



  # 临时测试关掉stage B之后能不能让c7x_1 load：DSP 接近跑满

  cd /media/record/VisPer_ti

    ./TEST_visper_ti \
    -t RAEB \
    -c ./assets/config.jsonc \
    -m ./assets/raeb_tidl_net.bin \
    -i ./img/raeb_vis_ori_1767254400804.jpg \
    -d DSP_C7-1 \
    --direct-tidl \
    --workers 1 \
    --stress-frames 300 \
    2>&1 | tee single_workers1_load.log

  cd /media/record/VisPer_ti
    ./TEST_visper_ti \
    -t RAEB \
    -c ./assets/config.jsonc \
    -m ./assets/raeb_tidl_net.bin \
    -i ./img/raeb_vis_ori_1767254400804.jpg \
    -d DSP_C7-1 \
    --direct-tidl \
    --workers 2 \
    --stress-frames 300 \
    2>&1 | tee single_workers2_load.log

