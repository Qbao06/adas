#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SERVICE_TMP="$(mktemp)"
trap 'rm -f "$SERVICE_TMP"' EXIT
sed "s|@PROJECT_DIR@|$PROJECT_DIR|g" "$PROJECT_DIR/deploy/adas.service.in" > "$SERVICE_TMP"
sudo install -m 0644 "$SERVICE_TMP" /etc/systemd/system/adas.service
sudo systemctl daemon-reload
sudo systemctl enable adas.service
echo "Đã cài adas.service. Khởi động bằng: sudo systemctl start adas.service"
