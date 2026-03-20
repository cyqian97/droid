source ~/miniconda3/etc/profile.d/conda.sh
conda activate robot
bash franka_direct/python/generate_stubs.sh 
python scripts/test_vr_readout.py&
