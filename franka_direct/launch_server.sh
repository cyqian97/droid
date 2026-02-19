#!/usr/bin/env bash
# Launch franka_server inside the Docker container.
# This replaces Polymetis — do NOT run launch_robot.sh at the same time.
#
# Run from the host:
#   docker exec <container> bash /app/droid/franka_direct/launch_server.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BINARY="${SCRIPT_DIR}/build/franka_server"

ROBOT_IP="${ROBOT_IP:-192.168.1.11}"
GRPC_ADDR="${GRPC_ADDR:-0.0.0.0:50052}"

if [ ! -f "${BINARY}" ]; then
    echo "[ERROR] Binary not found: ${BINARY}"
    echo "Run build.sh first."
    exit 1
fi

# Kill any leftover franka_panda_client / polymetis server that might hold
# the robot connection
pkill -9 franka_panda_cl 2>/dev/null || true
pkill -9 run_server      2>/dev/null || true
sleep 1

source /root/miniconda3/etc/profile.d/conda.sh
conda activate polymetis-local

# Make libfranka findable at runtime
find /root/miniconda3 -type d -name "lib" | sudo tee /etc/ld.so.conf.d/conda-polymetis.conf > /dev/null
sudo ldconfig

echo "[franka_server] Starting: robot_ip=${ROBOT_IP}  grpc=${GRPC_ADDR}"
exec "${BINARY}" "${ROBOT_IP}" "${GRPC_ADDR}" 2>&1 | tee /tmp/franka_direct.log
