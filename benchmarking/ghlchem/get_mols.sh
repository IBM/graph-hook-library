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

#!/usr/bin/env bash
set -euo pipefail

# Download RDKit sample SDFs and concatenate into molecules.sdf
urls=(
  "https://raw.githubusercontent.com/rdkit/rdkit/2d25752fe0319309886a595d08c6f379c8c6acfb/Docs/Book/data/actives_5ht3.sdf"
  "https://raw.githubusercontent.com/rdkit/rdkit/2d25752fe0319309886a595d08c6f379c8c6acfb/Docs/Book/data/cdk2.sdf"
  "https://raw.githubusercontent.com/rdkit/rdkit/2d25752fe0319309886a595d08c6f379c8c6acfb/Docs/Book/data/5ht3ligs.sdf"
)

rm -f molecules.sdf
for url in "${urls[@]}"; do
  fname=$(basename "$url")
  echo "Fetching $fname..."
  curl -L -o "$fname" "$url"
  cat "$fname" >> molecules.sdf
done

echo "Cleaning up individual SDFs..."
for url in "${urls[@]}"; do
  rm -f "$(basename "$url")"
done

echo "Combined molecules written to molecules.sdf"
