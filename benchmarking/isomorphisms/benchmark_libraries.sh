# Copyright (c) 2026 IBM
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

#!/bin/bash
set -euo pipefail



SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DATA_ROOT="${SCRIPT_DIR}/../si"
OUTPUT_DIR="${SCRIPT_DIR}/results"
START_INDEX=${1:-0}
STRIDE=${2:-1}
N=10

mkdir -p "${OUTPUT_DIR}"

if [[ ${STRIDE} -le 0 ]]; then
    echo "Stride must be a positive integer."
    exit 1
fi

profilers=(
    "ghl_profiler.py"
    "graphtool_profiler.py"
    "igraph_profiler_vf2.py"
    "networkx_profiler_vf2.py"
)

mapfile -t DIRS < <(find "${DATA_ROOT}" -mindepth 1 -maxdepth 1 -type d | sort)

for ((idx=START_INDEX; idx<${#DIRS[@]}; idx+=STRIDE)); do
    dir="${DIRS[idx]}"
    for subdir in "${dir}"/*; do
        files=("${subdir}"/*)
        pattern="${files[0]}"
        target="${files[1]}"
        for profiler in "${profilers[@]}"; do
            code_path="${SCRIPT_DIR}/${profiler}"
            output_path="${OUTPUT_DIR}/${profiler%.py}.csv"
            "${SCRIPT_DIR}/run_profiler.sh" "${code_path}" "${target}" "${pattern}" "${N}" "${output_path}"
        done
    done
done

echo "Submitted all jobs."
