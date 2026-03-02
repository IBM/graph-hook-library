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

#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARCHIVE_URL="https://liris.cnrs.fr/csolnon/newSIPbenchmarks.tgz"
ARCHIVE_PATH="${SCRIPT_DIR}/newSIPbenchmarks.tgz"
WORK_DIR="$(mktemp -d "${SCRIPT_DIR}/si_download.XXXXXX")"

cleanup() {
  rm -f "${ARCHIVE_PATH}"
  rm -rf "${WORK_DIR}"
}
trap cleanup EXIT

wget -O "${ARCHIVE_PATH}" "${ARCHIVE_URL}"
tar -xzf "${ARCHIVE_PATH}" -C "${WORK_DIR}"

SI_DIR="$(find "${WORK_DIR}" -type d -name "si" -print -quit)"
if [[ -z "${SI_DIR}" ]]; then
  echo "Unable to locate si directory in extracted archive" >&2
  exit 1
fi

find "${SI_DIR}" -mindepth 1 -maxdepth 1 -type d -exec cp -R {} "${SCRIPT_DIR}/" \;

echo "SI benchmarks copied to ${SCRIPT_DIR}"
