#!/bin/bash

set -euo pipefail

samples="${1:-120}"
timestamp="$(TZ=Asia/Singapore date +%Y%m%d_%H%M%S)"
out_dir="tmp/lab2_sonar/${timestamp}"
out_csv="${out_dir}/sonar_samples.csv"

set +u
source install/setup.bash
set -u
mkdir -p "${out_dir}"

echo "Recording ${samples} valid sonar samples to ${out_csv}"
python3 tools/record_sonar_samples.py --output "${out_csv}" --samples "${samples}"
echo "Done."
