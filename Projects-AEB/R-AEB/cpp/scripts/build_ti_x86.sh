#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CPP_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
PROJECTS_ROOT="$(cd "${CPP_DIR}/../.." && pwd)"
RAEB_ROOT="$(cd "${CPP_DIR}/.." && pwd)"

TIDL_ROOT="${TIDL_ROOT:-${PROJECTS_ROOT}/edgeai-tidl-tools-2}"
SOC="${SOC:-J722S}"
TIDL_TOOLS_PATH="${TIDL_TOOLS_PATH:-${TIDL_ROOT}/tools/${SOC}/tidl_tools}"
ONNXRT_INCLUDE_ROOT="${ONNXRT_INCLUDE_ROOT:-${RAEB_ROOT}/TI-Environment/ti-processor-sdk-rtos-j722s-evm-11_00_00_06/targetfs/usr/include/onnxruntime/include}"
ONNXRT_LIB_DIR="${ONNXRT_LIB_DIR:-/opt/conda/envs/tidl2/lib/python3.10/site-packages/onnxruntime/capi}"
ONNXRT_LIB="${ONNXRT_LIB:-${ONNXRT_LIB_DIR}/libonnxruntime.so.1.23.0}"
ONNXRT_ROOT="${ONNXRT_ROOT:-${ONNXRT_LIB_DIR}}"
TIDL_PROVIDER_API="${TIDL_PROVIDER_API:-options}"
BUILD_DIR="${BUILD_DIR:-${CPP_DIR}/cmake-build-release-ti-x86-ort123}"
BUILD_TYPE="${BUILD_TYPE:-Release}"
JOBS="${JOBS:-8}"

export SOC
export TIDL_TOOLS_PATH
export LD_LIBRARY_PATH="${ONNXRT_LIB_DIR}:${TIDL_TOOLS_PATH}:${LD_LIBRARY_PATH:-}"

cmake -S "${CPP_DIR}" -B "${BUILD_DIR}" \
  -DARCH=ti_x86 \
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
  -DTI_TIDL_TOOLS_ROOT="${TIDL_ROOT}" \
  -DTI_X86_SOC="${SOC}" \
  -DTI_X86_TIDL_TOOLS_PATH="${TIDL_TOOLS_PATH}" \
  -DTI_X86_ONNXRT_ROOT="${ONNXRT_ROOT}" \
  -DTI_X86_ONNXRT_INCLUDE_ROOT="${ONNXRT_INCLUDE_ROOT}" \
  -DTI_X86_ONNXRT_LIB_DIR="${ONNXRT_LIB_DIR}" \
  -DTI_X86_ONNXRT_LIB="${ONNXRT_LIB}" \
  -DTI_X86_TIDL_PROVIDER_API="${TIDL_PROVIDER_API}"

cmake --build "${BUILD_DIR}" --parallel "${JOBS}"

ORT_SHIM_DIR="${BUILD_DIR}/ort_libs"
cmake -E make_directory "${ORT_SHIM_DIR}"
if [[ ! -e "${ORT_SHIM_DIR}/libonnxruntime.so.1" && ! -L "${ORT_SHIM_DIR}/libonnxruntime.so.1" ]]; then
  cmake -E create_symlink "${ONNXRT_LIB}" "${ORT_SHIM_DIR}/libonnxruntime.so.1"
fi
if [[ ! -e "${ORT_SHIM_DIR}/libonnxruntime.so" && ! -L "${ORT_SHIM_DIR}/libonnxruntime.so" ]]; then
  cmake -E create_symlink "${ONNXRT_LIB}" "${ORT_SHIM_DIR}/libonnxruntime.so"
fi

echo "TI x86 TIDL ONNXRuntime build done:"
echo "  ${BUILD_DIR}/src/libvisper_ti_x86.so"
echo "  ${BUILD_DIR}/tests/TEST_visper_ti_x86"
echo "  ${ORT_SHIM_DIR}/libonnxruntime.so.1 -> ${ONNXRT_LIB}"
