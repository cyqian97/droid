#!/usr/bin/env bash
# Launch franka_server_cartesian inside the Docker container.
# This replaces Polymetis — do NOT run launch_robot.sh at the same time.
#
# Usage (from the host):
#   docker exec <container> bash /app/droid/franka_direct/launch_server_cartesian.sh
#
# All parameters are optional environment variables:
#   ROBOT_IP     Robot IP address              (default: 192.168.1.11)
#   GRPC_ADDR    gRPC listen address           (default: 0.0.0.0:50052)
#   CONFIG_FILE  Path to controller YAML       (default: config/controller_cartesian.yaml)
#
# Examples:
#   # Use all defaults
#   docker exec <container> bash /app/droid/franka_direct/launch_server_cartesian.sh
#
#   # Custom robot IP
#   docker exec <container> env ROBOT_IP=192.168.1.12 \
#       bash /app/droid/franka_direct/launch_server_cartesian.sh
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BINARY="${SCRIPT_DIR}/build/franka_server_cartesian"

ROBOT_IP="${ROBOT_IP:-192.168.1.11}"
GRPC_ADDR="${GRPC_ADDR:-0.0.0.0:50052}"
CONFIG_FILE="${CONFIG_FILE:-${SCRIPT_DIR}/config/controller_cartesian.yaml}"

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

echo "[cartesian_server] Starting: robot_ip=${ROBOT_IP}  grpc=${GRPC_ADDR}  config=${CONFIG_FILE}"
exec "${BINARY}" "${ROBOT_IP}" "${GRPC_ADDR}" "${CONFIG_FILE}" 2>&1 | tee /tmp/franka_direct_cartesian.log
