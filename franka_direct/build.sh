#!/usr/bin/env bash
# Build franka_server inside the Docker container.
# Run from the host:
#   docker exec <container> bash /app/droid/franka_direct/build.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"

echo "=== Activating conda environment ==="
source /root/miniconda3/etc/profile.d/conda.sh
conda activate polymetis-local

# ldconfig so libfranka .so is discoverable at link time
find /root/miniconda3 -type d -name "lib" | sudo tee /etc/ld.so.conf.d/conda-polymetis.conf > /dev/null
sudo ldconfig

echo "=== Configuring CMake ==="
mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"
cmake .. -DCMAKE_BUILD_TYPE=Release

echo "=== Building ==="
make -j"$(nproc)"

echo ""
echo "=== Build complete ==="
echo "Binary: ${BUILD_DIR}/franka_server"
echo ""
echo "Generate Python stubs (run on laptop or inside container):"
echo "  bash ${SCRIPT_DIR}/python/generate_stubs.sh"
