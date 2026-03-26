#!/bin/bash

set -euo pipefail

if [ "$#" -lt 1 ]; then
    echo "Usage: $0 <sonar_samples.csv>"
    exit 1
fi

input_csv="$1"
base_dir="$(dirname "$input_csv")"
summary_txt="${base_dir}/variance_summary.txt"
residual_csv="${base_dir}/sonar_residuals.csv"

set +u
source install/setup.bash
set -u

python3 tools/estimate_sonar_variance.py \
    "$input_csv" \
    --summary "$summary_txt" \
    --residual-csv "$residual_csv"

echo
echo "Summary: ${summary_txt}"
echo "Residual CSV: ${residual_csv}"
