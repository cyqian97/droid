#!/bin/bash
set -e

# Initialize conda for bash
source ~/miniconda3/etc/profile.d/conda.sh

# Activate the robot environment
conda activate robot

# Run the user's command
exec "$@"