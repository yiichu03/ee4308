#!/bin/bash

set -euo pipefail

target_dir="${1:-log/lab2}"
output_dir="${2:-tmp/lab2_analysis}"

if [ ! -d "${target_dir}" ]; then
    echo "Directory not found: ${target_dir}" >&2
    exit 1
fi

mkdir -p "${output_dir}"

target_tag="$(basename "${target_dir%/}")"
summary_file="${output_dir%/}/${target_tag}_summary.csv"
ranking_file="${output_dir%/}/${target_tag}_ranking.txt"

tmp_file="$(mktemp)"

{
    echo "log_file,imu_z,sonar,samples,mean_abs_errpose_z,mean_abs_errtwis_z,mean_abs_pose_minus_sonar,mean_abs_twist_z,final_pose_z,final_twist_z,final_errpose_z,final_errtwis_z,final_sonar_z,score"

    find "${target_dir}" -maxdepth 1 -type f -name '*.log' | sort | while read -r log_file; do
        awk '
            BEGIN {
                imu_z = ""
                sonar = ""
                errpose_sum = 0
                errtwis_sum = 0
                twist_sum = 0
                pose_sonar_sum = 0
                errpose_count = 0
                errtwis_count = 0
                twist_count = 0
                pose_sonar_count = 0
            }
            /^# imu_z=/ {
                sub(/^# imu_z=/, "", $0)
                imu_z = $0
                next
            }
            /^# sonar=/ {
                sub(/^# sonar=/, "", $0)
                sonar = $0
                next
            }
            $3 == "Pose" {
                pose[$2] = $6
                final_pose = $6
                next
            }
            $3 == "Twist" {
                twist = $6 + 0
                twist_sum += (twist < 0 ? -twist : twist)
                twist_count++
                final_twist = $6
                next
            }
            $3 == "ErrPose" {
                err = $6 + 0
                errpose_sum += (err < 0 ? -err : err)
                errpose_count++
                final_errpose = $6
                next
            }
            $3 == "ErrTwis" {
                err = $6 + 0
                errtwis_sum += (err < 0 ? -err : err)
                errtwis_count++
                final_errtwis = $6
                next
            }
            $3 == "Sonar" && $6 != "nan" && $6 != "--" {
                sonar_value = $6 + 0
                if ($2 in pose) {
                    diff = pose[$2] - sonar_value
                    pose_sonar_sum += (diff < 0 ? -diff : diff)
                    pose_sonar_count++
                }
                final_sonar = $6
                next
            }
            END {
                samples = errpose_count
                mean_abs_errpose = (errpose_count ? errpose_sum / errpose_count : 9999)
                mean_abs_errtwis = (errtwis_count ? errtwis_sum / errtwis_count : 9999)
                mean_abs_pose_sonar = (pose_sonar_count ? pose_sonar_sum / pose_sonar_count : 9999)
                mean_abs_twist = (twist_count ? twist_sum / twist_count : 9999)
                score = mean_abs_errpose + mean_abs_errtwis + mean_abs_pose_sonar + mean_abs_twist

                printf "%s,%s,%s,%d,%.6f,%.6f,%.6f,%.6f,%s,%s,%s,%s,%s,%.6f\n",
                    FILENAME,
                    imu_z,
                    sonar,
                    samples,
                    mean_abs_errpose,
                    mean_abs_errtwis,
                    mean_abs_pose_sonar,
                    mean_abs_twist,
                    final_pose,
                    final_twist,
                    final_errpose,
                    final_errtwis,
                    final_sonar,
                    score
            }
        ' "${log_file}"
    done
} > "${summary_file}"

{
    head -n 1 "${summary_file}"
    tail -n +2 "${summary_file}" | sort -t, -k14,14g
} > "${tmp_file}"

mv "${tmp_file}" "${summary_file}"

{
    echo "Best runs first:"
    echo
    if command -v column >/dev/null 2>&1; then
        column -s, -t "${summary_file}"
    else
        cat "${summary_file}"
    fi
} > "${ranking_file}"

echo "Summary written to ${summary_file}"
echo "Readable ranking written to ${ranking_file}"
echo
sed -n '1,20p' "${ranking_file}"
