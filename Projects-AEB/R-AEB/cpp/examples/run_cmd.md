# MIN_det Quick Commands

完整中英文说明见：

```bash
examples/README_min_det_zh_en.md
```

## x86 TI emulation

```bash
cd /opt_disk3/rd234421/Projects-SGS/R-AEB/cpp

export SOC=J722S
export TIDL_TOOLS_PATH=/opt_disk3/rd234421/Projects-SGS/edgeai-tidl-tools-2/tools/J722S/tidl_tools
export ORT_LIB_DIR=/opt/conda/envs/tidl2/lib/python3.10/site-packages/onnxruntime/capi
export LD_LIBRARY_PATH="$PWD/cmake-build-release-ti-x86-ort123/src:$PWD/cmake-build-release-ti-x86-ort123/ort_libs:$ORT_LIB_DIR:$TIDL_TOOLS_PATH:${LD_LIBRARY_PATH:-}"

cmake --build cmake-build-release-ti-x86-ort123 --target MIN_det_ti_x86 --parallel 8

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

## Target board

```bash
cd /media/record/min_det_ti_run

export LD_LIBRARY_PATH=/usr/lib:/app/lib:$PWD/libs:$PWD:${LD_LIBRARY_PATH:-}

ldd ./MIN_det_ti | egrep "tivision|visper|opencv"

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

Box-only mode:

```bash
--no-mask
```
