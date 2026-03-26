#!/bin/bash

set -o pipefail

mkdir -p log/lab2
timestamp=$(TZ=Asia/Singapore date +%Y%m%d_%H%M%S)
log_file="log/lab2/proj2_sim_${timestamp}.log"

echo "Writing proj2 log to ${log_file} (Asia/Singapore)"

source install/setup.bash
ros2 launch ee4308_bringup proj2_sim.launch.py libgl:=False 2>&1 | tee "${log_file}"
./kill_gz.sh
