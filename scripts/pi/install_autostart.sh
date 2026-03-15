#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SERVICE_DIR="/etc/systemd/system"

if [[ $EUID -ne 0 ]]; then
  echo "Please run as root (e.g. sudo scripts/pi/install_autostart.sh)."
  exit 1
fi

for template in bicycle-blind-spot.service bicycle-power-switch.service; do
  src="${REPO_DIR}/scripts/pi/systemd/${template}"
  dst="${SERVICE_DIR}/${template}"
  sed "s|__REPO_DIR__|${REPO_DIR}|g" "${src}" > "${dst}"
  echo "Installed ${dst}"
done

systemctl daemon-reload
systemctl enable bicycle-blind-spot.service bicycle-power-switch.service
systemctl restart bicycle-blind-spot.service bicycle-power-switch.service

echo "Done. Check status with:"
echo "  systemctl status bicycle-blind-spot.service"
echo "  systemctl status bicycle-power-switch.service"