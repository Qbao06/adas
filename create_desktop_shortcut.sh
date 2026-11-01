#!/usr/bin/env bash
# Tự động tạo biểu tượng (Desktop Shortcut) trên màn hình nền và trong menu ứng dụng Ubuntu.

set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ICON_PATH="$PROJECT_DIR/assets/adas_icon.png"
RUN_SCRIPT="$PROJECT_DIR/run.sh"

chmod +x "$RUN_SCRIPT"

DESKTOP_CONTENT="[Desktop Entry]
Version=1.0
Type=Application
Name=ADAS LiDAR Camera
GenericName=ADAS Perception Pipeline
Comment=Khoi dong he thong canh bao va cham ADAS LiDAR-Camera
Exec=$RUN_SCRIPT
Icon=$ICON_PATH
Path=$PROJECT_DIR
Terminal=true
Categories=Utility;Science;Development;
StartupNotify=true
"

DESKTOP_DIR="${XDG_DESKTOP_DIR:-$HOME/Desktop}"
mkdir -p "$DESKTOP_DIR"

DESKTOP_FILE="$DESKTOP_DIR/adas.desktop"
echo "$DESKTOP_CONTENT" > "$DESKTOP_FILE"
chmod +x "$DESKTOP_FILE"

if command -v gio >/dev/null 2>&1; then
    gio set "$DESKTOP_FILE" metadata::trusted true || true
fi

APP_DIR="$HOME/.local/share/applications"
mkdir -p "$APP_DIR"

APP_FILE="$APP_DIR/adas.desktop"
echo "$DESKTOP_CONTENT" > "$APP_FILE"
chmod +x "$APP_FILE"

if command -v update-desktop-database >/dev/null 2>&1; then
    update-desktop-database "$APP_DIR" || true
fi

echo "============================================================"
echo " ĐÃ TẠO ICON THÀNH CÔNG!"
echo " 1. Trên màn hình nền Desktop: $DESKTOP_FILE"
echo " 2. Trong Menu ứng dụng:        $APP_FILE"
echo "============================================================"
