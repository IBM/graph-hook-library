# Copyright (c) 2025 IBM
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
EXECUTABLE="${SCRIPT_DIR}/vf3_vs_vf2"
DATA_ROOT="${SCRIPT_DIR}/../si"
START_INDEX=${1:-0}
STRIDE=${2:-1}

if [[ ${STRIDE} -le 0 ]]; then
    echo "Stride must be a positive integer."
    exit 1
fi

if [[ ! -x "${EXECUTABLE}" ]]; then
    echo "Executable ${EXECUTABLE} not found or not executable."
    exit 1
fi

mapfile -t DIRS < <(find "${DATA_ROOT}" -mindepth 1 -maxdepth 1 -type d | sort)

for ((idx=START_INDEX; idx<${#DIRS[@]}; idx+=STRIDE)); do
    dir="${DIRS[idx]}"
    for subdir in "${dir}"/*; do
        files=("${subdir}"/*)
        pattern="${files[0]}"
        target="${files[1]}"
        perf_csv="${SCRIPT_DIR}/results/perf_results_${START_INDEX}.csv"
        mismatch_csv="${SCRIPT_DIR}/results/mismatches_${START_INDEX}.csv"
        aborted_csv="${SCRIPT_DIR}/results/aborted_${START_INDEX}.csv"
        echo "Running ${EXECUTABLE} with pattern=${pattern}, target=${target}"
        ./vf3_vs_vf2 "${target}" "${pattern}" "${perf_csv}" "${mismatch_csv}" "${aborted_csv}"
    done
done
