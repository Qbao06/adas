#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DESKTOP_DIR="$(xdg-user-dir DESKTOP 2>/dev/null || true)"
if [ -z "$DESKTOP_DIR" ]; then
    USER_DIR="$(getent passwd "$(id -un)" | cut -d: -f6)"
    DESKTOP_DIR="$USER_DIR/Desktop"
fi
mkdir -p "$DESKTOP_DIR"
SHORTCUT="$DESKTOP_DIR/ADAS-Web-Demo.desktop"
sed "s|@PROJECT_DIR@|$PROJECT_DIR|g" "$PROJECT_DIR/deploy/ADAS-Web-Demo.desktop.in" > "$SHORTCUT"
chmod +x "$SHORTCUT"
if command -v gio >/dev/null 2>&1; then
    gio set "$SHORTCUT" metadata::trusted true >/dev/null 2>&1 || true
fi
echo "Đã tạo shortcut: $SHORTCUT"
