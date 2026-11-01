#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

DESKTOP_DIR=""
if command -v xdg-user-dir >/dev/null 2>&1; then
    DESKTOP_DIR="$(xdg-user-dir DESKTOP 2>/dev/null || true)"
fi
if [ -z "$DESKTOP_DIR" ]; then
    USER_DIR="$(getent passwd "$(id -un)" | cut -d: -f6)"
    DESKTOP_DIR="$USER_DIR/Desktop"
fi

mkdir -p "$DESKTOP_DIR"
SHORTCUT="$DESKTOP_DIR/ADAS-Live.desktop"
sed "s|@PROJECT_DIR@|$PROJECT_DIR|g" \
    "$PROJECT_DIR/deploy/ADAS.desktop.in" > "$SHORTCUT"
chmod +x "$SHORTCUT"

# GNOME-based Jetson desktops may require marking a launcher trusted.
if command -v gio >/dev/null 2>&1; then
    gio set "$SHORTCUT" metadata::trusted true >/dev/null 2>&1 || true
fi

echo "Đã tạo shortcut: $SHORTCUT"
echo "Nhấp đúp 'ADAS Live' để chạy phần cứng thật."
