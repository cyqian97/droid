#!/usr/bin/env bash
# Generate Python gRPC stubs from franka_control.proto.
# Run this on the laptop (or inside the container) before using the Python client.
#
#   bash franka_direct/python/generate_stubs.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROTO_FILE="${SCRIPT_DIR}/../proto/franka_control.proto"
OUT_DIR="${SCRIPT_DIR}"

python -m grpc_tools.protoc \
    -I "${SCRIPT_DIR}/../proto" \
    --python_out="${OUT_DIR}" \
    --grpc_python_out="${OUT_DIR}" \
    "${PROTO_FILE}"

echo "Generated stubs in ${OUT_DIR}:"
ls "${OUT_DIR}"/*.py
