#!/bin/bash

set -euo pipefail

duration_sec="${1:-20}"
shift || true

if [ "$#" -gt 0 ]; then
    sonar_values=("$@")
else
    sonar_values=(0.0003 0.001 0.003 0.01 0.03 0.05)
fi

timestamp="$(TZ=Asia/Singapore date +%Y%m%d_%H%M%S)"
log_dir="tmp/lab2_var_sonar_sweep/${timestamp}"
param_file="src/ee4308_bringup/params/proj2.yaml"
backup_file="$(mktemp)"

cleanup() {
    cp "$backup_file" "$param_file"
    rm -f "$backup_file"
    ./kill_gz.sh >/dev/null 2>&1 || true
}

trap cleanup EXIT

cp "$param_file" "$backup_file"
mkdir -p "$log_dir"

set +u
source install/setup.bash
set -u

for sonar in "${sonar_values[@]}"; do
    safe_name="${sonar//./p}"
    log_path="${log_dir}/imu_20p0__sonar_${safe_name}.log"

    sed -i -E "s/^(\\s*var_imu_z:\\s*).*/\\120.0/" "$param_file"
    sed -i -E "s/^(\\s*var_sonar:\\s*).*/\\1${sonar}/" "$param_file"

    echo
    echo "=== var_imu_z=20.0 var_sonar=${sonar} ==="
    echo "Log: ${log_path}"

    colcon build --symlink-install >/dev/null

    (
        set +e
        timeout "${duration_sec}s" ros2 launch ee4308_bringup proj2_sim.launch.py libgl:=False \
            >"${log_path}" 2>&1
        exit 0
    )

    ./kill_gz.sh >/dev/null 2>&1 || true
    sleep 2
done

echo
echo "Sweep complete."
echo "Logs written to ${log_dir}"
echo "Parameter file restored to its original contents."
