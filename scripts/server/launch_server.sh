#!/bin/bash
source /root/miniconda3/etc/profile.d/conda.sh
conda activate polymetis-local
cd $( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
# python run_server.py

# Keep container alive
exec sleep infinity
