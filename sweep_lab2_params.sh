#!/bin/bash

set -euo pipefail

duration_s="${1:-20}"

# Second-round fine sweep centered around the best coarse result:
#   var_imu_z = 10.0
#   var_sonar = 0.1
imu_values=("6.0" "10.0" "15.0" "20.0")
sonar_values=("0.03" "0.05" "0.1" "0.2")

workspace_dir="$(cd "$(dirname "$0")" && pwd)"
param_file="${workspace_dir}/src/ee4308_bringup/params/proj2.yaml"
backup_file="$(mktemp)"
timestamp="$(TZ=Asia/Singapore date +%Y%m%d_%H%M%S)"
run_dir="${workspace_dir}/log/lab2/sweep_${timestamp}"

mkdir -p "${run_dir}"
cp "${param_file}" "${backup_file}"

restore_param_file() {
    cp "${backup_file}" "${param_file}"
    rm -f "${backup_file}"
}

trap restore_param_file EXIT

write_param_file() {
    local imu_z="$1"
    local sonar="$2"

    awk -v imu_z="${imu_z}" -v sonar="${sonar}" '
        /^[[:space:]]*var_imu_z:/ {
            sub(/:.*/, ": " imu_z)
        }
        /^[[:space:]]*var_sonar:/ {
            sub(/:.*/, ": " sonar)
        }
        { print }
    ' "${backup_file}" > "${param_file}"
}

sanitize_value() {
    printf "%s" "$1" | sed 's/-/m/g; s/\./p/g'
}

source_workspace() {
    # colcon-generated setup scripts may read unset variables such as COLCON_TRACE
    # and are not always compatible with `set -u`.
    set +u
    source install/setup.bash
    set -u
}

source_workspace

echo "Running lab2 parameter sweep"
echo "Duration per run: ${duration_s}s"
echo "Logs: ${run_dir}"

for imu_z in "${imu_values[@]}"; do
    for sonar in "${sonar_values[@]}"; do
        imu_tag="$(sanitize_value "${imu_z}")"
        sonar_tag="$(sanitize_value "${sonar}")"
        log_file="${run_dir}/imu_${imu_tag}__sonar_${sonar_tag}.log"

        write_param_file "${imu_z}" "${sonar}"

        {
            echo "# imu_z=${imu_z}"
            echo "# sonar=${sonar}"
            echo "# duration_s=${duration_s}"
            echo "# started_at_sgt=$(TZ=Asia/Singapore date '+%Y-%m-%d %H:%M:%S %Z')"
        } > "${log_file}"

        echo
        echo "=== imu_z=${imu_z} sonar=${sonar} ==="
        timeout --signal=INT "${duration_s}s" \
            ros2 launch ee4308_bringup proj2_sim.launch.py libgl:=False \
            >> "${log_file}" 2>&1 || true
        ./kill_gz.sh >/dev/null 2>&1 || true
        sleep 2
    done
done

restore_param_file
trap - EXIT

echo
echo "Sweep complete."
echo "Logs written to ${run_dir}"
echo "Next step:"
echo "  ./analyze_lab2_logs.sh ${run_dir}"
