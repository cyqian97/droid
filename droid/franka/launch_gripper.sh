# Find all conda lib directories
find /root/miniconda3 -type d -name "lib" | sudo tee /etc/ld.so.conf.d/conda-polymetis.conf

# Update cache
sudo ldconfig

source ~/miniconda3/etc/profile.d/conda.sh
conda activate polymetis-local
pkill -9 gripper
chmod a+rw /dev/ttyUSB0
launch_gripper.py gripper=robotiq_2f gripper.comport=/dev/ttyUSB0
