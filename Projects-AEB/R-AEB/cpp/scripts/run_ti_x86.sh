#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CPP_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
PROJECTS_ROOT="$(cd "${CPP_DIR}/../.." && pwd)"
RAEB_ROOT="$(cd "${CPP_DIR}/.." && pwd)"

TIDL_ROOT="${TIDL_ROOT:-${PROJECTS_ROOT}/edgeai-tidl-tools-2}"
SOC="${SOC:-J722S}"
TIDL_TOOLS_PATH="${TIDL_TOOLS_PATH:-${TIDL_ROOT}/tools/${SOC}/tidl_tools}"
ONNXRT_LIB_DIR="${ONNXRT_LIB_DIR:-/opt/conda/envs/tidl2/lib/python3.10/site-packages/onnxruntime/capi}"
ONNXRT_LIB="${ONNXRT_LIB:-${ONNXRT_LIB_DIR}/libonnxruntime.so.1.23.0}"

BUILD_DIR="${BUILD_DIR:-${CPP_DIR}/cmake-build-release-ti-x86-ort123}"
EXE="${EXE:-${BUILD_DIR}/tests/TEST_visper_ti_x86}"
MODEL="${MODEL:-${RAEB_ROOT}/assets/RAEB/TI_lyl.onnx}"
CONFIG="${CONFIG:-${RAEB_ROOT}/assets/RAEB/config.jsonc}"
DEFAULT_INPUT="${CPP_DIR}/tests/vis-bike1"
if [[ ! -e "${DEFAULT_INPUT}" && -e "${CPP_DIR}/tests/vis-bike1_ti_x86" ]]; then
  DEFAULT_INPUT="${CPP_DIR}/tests/vis-bike1_ti_x86"
fi
INPUT="${INPUT:-${DEFAULT_INPUT}}"
ARTIFACTS="${ARTIFACTS:-${TIDL_ROOT}/runtimes/examples/model-artifacts/TI_lyl/artifacts}"
OUT_ROOT="${OUT_ROOT:-${CPP_DIR}/ti_x86_runs}"
RUN_NAME="${RUN_NAME:-run_$(date +%Y%m%d_%H%M%S)_$$}"
RUN_DIR="${RUN_DIR:-${OUT_ROOT}/${RUN_NAME}}"

if [[ ! -x "${EXE}" ]]; then
  echo "[ERROR] executable not found: ${EXE}" >&2
  echo "        run scripts/build_ti_x86.sh first" >&2
  exit 1
fi

ORT_SHIM_DIR="${BUILD_DIR}/ort_libs"
cmake -E make_directory "${ORT_SHIM_DIR}"
if [[ ! -e "${ORT_SHIM_DIR}/libonnxruntime.so.1" && ! -L "${ORT_SHIM_DIR}/libonnxruntime.so.1" ]]; then
  cmake -E create_symlink "${ONNXRT_LIB}" "${ORT_SHIM_DIR}/libonnxruntime.so.1"
fi
if [[ ! -e "${ORT_SHIM_DIR}/libonnxruntime.so" && ! -L "${ORT_SHIM_DIR}/libonnxruntime.so" ]]; then
  cmake -E create_symlink "${ONNXRT_LIB}" "${ORT_SHIM_DIR}/libonnxruntime.so"
fi

if [[ -e "${RUN_DIR}" ]]; then
  echo "[ERROR] run dir already exists, refusing to overwrite: ${RUN_DIR}" >&2
  exit 1
fi

mkdir -p "${RUN_DIR}"
touch "${RUN_DIR}/visl"

export SOC
export TIDL_TOOLS_PATH
export VISPER_TIDL_ARTIFACTS_DIR="${ARTIFACTS}"
export LD_LIBRARY_PATH="${BUILD_DIR}/src:${ORT_SHIM_DIR}:${ONNXRT_LIB_DIR}:${TIDL_TOOLS_PATH}:${LD_LIBRARY_PATH:-}"

echo "TI x86 TIDL ONNXRuntime run config:"
echo "  exe       : ${EXE}"
echo "  model     : ${MODEL}"
echo "  config    : ${CONFIG}"
echo "  input     : ${INPUT}"
echo "  artifacts : ${ARTIFACTS}"
echo "  run dir   : ${RUN_DIR}"

cd "${RUN_DIR}"

"${EXE}" \
  -t RAEB \
  -c "${CONFIG}" \
  -m "${MODEL}" \
  -i "${INPUT}" \
  -o vis

echo "TI x86 TIDL ONNXRuntime run done:"
echo "  run dir : ${RUN_DIR}"
echo "  output  : ${RUN_DIR}/vis"
