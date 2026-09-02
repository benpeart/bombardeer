#!/usr/bin/env bash
set -euo pipefail

# ==============================================================================
# Automated Hailo-8 Model Build Pipeline: MegaDetector v6 (YOLOv9-c)
# Run inside the Hailo AI Software Suite Docker container:
#   cd /local/shared_with_docker && bash build_mdv6_hef.sh
# ==============================================================================

WORK_DIR="/local/shared_with_docker"
ONNX_URL="https://zenodo.org/records/15398270/files/MDV6-yolov9-c.onnx?download=1"
ONNX_FILE="MDV6-yolov9-c.onnx"
BASE_HAR="MDV6_yolov9_c.har"
QUANT_HAR="MDV6_yolov9_c_quantized.har"
ALLS_FILE="mdv6.alls"
CALIB_FILE="calib_data.npy"
FINAL_HEF="MDV6-yolov9-c.hef"

cd "${WORK_DIR}"

echo "=== [1/6] Setting Up GPU & cuDNN Environment ==="
if python3 -c "import nvidia.cudnn" &>/dev/null; then
    CUDNN_PATH=$(python3 -c "import nvidia.cudnn; print(list(nvidia.cudnn.__path__)[0] + '/lib')")
    export LD_LIBRARY_PATH="${CUDNN_PATH}:${LD_LIBRARY_PATH:-}"
fi
export CUDA_VISIBLE_DEVICES=0
export HAILO_GPU_ID=0

echo "=== [2/6] Checking Model Asset ==="
if [ ! -f "${ONNX_FILE}" ]; then
    echo "Downloading ${ONNX_FILE}..."
    wget -q --show-progress -O "${ONNX_FILE}" "${ONNX_URL}"
else
    echo "Using existing ${ONNX_FILE}."
fi

echo "=== [3/6] Generating Calibration Dataset ==="
if [ ! -f "${CALIB_FILE}" ]; then
    echo "Generating ${CALIB_FILE} (64 frames, 640x640x3, uint8)..."
    python3 -c "
import numpy as np
calib = np.random.randint(0, 256, size=(64, 640, 640, 3), dtype=np.uint8)
np.save('${CALIB_FILE}', calib)
"
else
    echo "Using existing ${CALIB_FILE}."
fi

echo "=== [4/6] Parsing ONNX to HAR (Non-Interactive) ==="
# Pipe 'y\ny\n' to auto-accept YOLO end nodes and NMS post-process script injections
printf "y\ny\n" | hailo parser onnx "${ONNX_FILE}" \
    --hw-arch hailo8 \
    --start-node-names images \
    --tensor-shapes [1,3,640,640] \
    --har-path "${BASE_HAR}"

echo "=== [5/6] Writing Model Script & Quantizing Model ==="
cat << 'EOF' > "${ALLS_FILE}"
normalization1 = normalization([0.0, 0.0, 0.0], [255.0, 255.0, 255.0])
performance_param(compiler_optimization_level=max)
EOF

hailo optimize "${BASE_HAR}" \
    --calib-set-path "${CALIB_FILE}" \
    --model-script "${ALLS_FILE}" \
    --output-har-path "${QUANT_HAR}"

echo "=== [6/6] Compiling HAR to HEF Binary ==="
# Partitioning and cluster mapping (CPU-bound branch & cut solver)
hailo compiler "${QUANT_HAR}"

echo "=== Validating HEF Binary ==="
if [ -f "${FINAL_HEF}" ]; then
    hailortcli parse-hef "${FINAL_HEF}"
    echo "BUILD SUCCESSFUL: ${WORK_DIR}/${FINAL_HEF}"
else
    echo "ERROR: ${FINAL_HEF} not generated."
    exit 1
fi