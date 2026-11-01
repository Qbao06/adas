#!/usr/bin/env bash
# Cài một lần trên Jetson Orin (JetPack 6.x). Không dùng trên PC / Jetson Nano đời cũ.
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJECT_DIR"
PYTHON_BIN="${PYTHON:-python3}"

if [ ! -f /etc/nv_tegra_release ]; then
    echo "[setup_jetson] Không phát hiện Jetson Linux (/etc/nv_tegra_release)." >&2
    echo "Jetson Nano đời cũ (JetPack 4) không chạy được dự án này. Cần Orin + JetPack 6." >&2
    exit 1
fi
if ! "$PYTHON_BIN" -c 'import sys; assert sys.version_info >= (3,10)' 2>/dev/null; then
    echo "[setup_jetson] Cần Python 3.10+ (JetPack 6.x)." >&2
    exit 1
fi

echo "[setup_jetson] JetPack: $(head -n1 /etc/nv_tegra_release)"

sudo_cmd() {
    if [ "$(id -u)" -eq 0 ]; then
        "$@"
    elif command -v sudo >/dev/null 2>&1; then
        sudo "$@"
    else
        echo "[setup_jetson] Cần quyền root cho: $*" >&2
        return 1
    fi
}

echo "[setup_jetson] Cài gói apt (OpenCV hệ thống, công cụ camera/UART)..."
sudo_cmd apt-get update -y
sudo_cmd apt-get install -y \
    python3-venv python3-dev python3-pip python3-opencv \
    v4l-utils libopenblas-dev libjpeg-dev pkg-config git
sudo_cmd apt-get install -y python3-libnvinfer python3-libnvinfer-dev || true

CURRENT_USER="$(id -un)"
sudo_cmd usermod -aG dialout,video,plugdev "$CURRENT_USER" || true

RULES_SRC="$PROJECT_DIR/deploy/99-adas-hardware.rules"
if [ -f "$RULES_SRC" ]; then
    sudo_cmd cp "$RULES_SRC" /etc/udev/rules.d/99-adas-hardware.rules
    if command -v udevadm >/dev/null 2>&1; then
        sudo_cmd udevadm control --reload-rules || true
        sudo_cmd udevadm trigger || true
    fi
fi

# venv phải thấy OpenCV apt + PyTorch NVIDIA + TensorRT JetPack
"$PYTHON_BIN" -m venv --system-site-packages "$PROJECT_DIR/.venv"
VENV_PY="$PROJECT_DIR/.venv/bin/python"
"$VENV_PY" -m pip install --upgrade pip wheel

# Gỡ OpenCV/ONNX Runtime pip (phá JetPack). Không gỡ torch — wheel NVIDIA.
"$VENV_PY" -m pip uninstall -y \
    opencv-python opencv-python-headless opencv-contrib-python \
    onnxruntime onnxruntime-gpu \
    2>/dev/null || true

"$VENV_PY" -m pip install -r "$PROJECT_DIR/requirements-jetson.txt"

if ! "$VENV_PY" -c 'import cv2' 2>/dev/null; then
    echo "[setup_jetson] Thiếu OpenCV (cv2)." >&2
    echo "Cài: sudo apt install -y python3-opencv" >&2
    echo "Không chạy: pip install opencv-python" >&2
    exit 1
fi
if ! "$VENV_PY" -c 'import torch; assert torch.cuda.is_available()' 2>/dev/null; then
    echo "[setup_jetson] Thiếu PyTorch CUDA tương thích JetPack." >&2
    echo "Cài wheel NVIDIA (không pip install torch từ PyPI) rồi chạy lại script." >&2
    echo "Xem: https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/" >&2
    exit 1
fi
if ! "$VENV_PY" -c 'import torchvision' 2>/dev/null; then
    echo "[setup_jetson] Thiếu torchvision (wheel NVIDIA cùng bản với torch)." >&2
    exit 1
fi

"$VENV_PY" -m pip install --no-deps 'ultralytics>=8.2,<9'

mkdir -p logs output data models drivers
chmod +x run.sh start_jetson.sh setup_jetson.sh \
    deploy/install_service.sh deploy/create_shortcut.sh

echo "[setup_jetson] Kiểm tra thư viện..."
"$VENV_PY" "$PROJECT_DIR/tools/verify_install.py" --strict

"$PROJECT_DIR/deploy/create_shortcut.sh" || true

echo
echo "[setup_jetson] Thư viện xong."
echo "Nếu vừa thêm nhóm dialout/video: đăng xuất rồi đăng nhập lại."
echo "Tiếp theo: cài wheel torch NVIDIA (nếu script vừa dừng vì CUDA),"
echo "điền config/jetson_live.yaml, export models/best.engine, rồi ./start_jetson.sh"
echo "Hướng dẫn đầy đủ: HUONG_DAN_CAI_DAT_JETSON.md"
