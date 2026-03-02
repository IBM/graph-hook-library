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

#! /bin/bash

CODE=$1
TARGET=$2
PATTERN=$3
N=$4
OUTPUT_PATH=$5

echo "=== Debug Information ==="
echo "Using Python from: $(which python3)"
echo "Python version: $(python3 --version)"
echo "Current directory: $(pwd)"
echo "Script arguments:"
echo "  CODE: $CODE"
echo "  TARGET: $TARGET"
echo "  PATTERN: $PATTERN"
echo "  N: $N"
echo "  OUTPUT_PATH: $OUTPUT_PATH"
echo "  DEBUG: $DEBUG"

# Check if files exist
if [ ! -f "$CODE" ]; then
    echo "ERROR: Code file not found: $CODE"
    exit 1
fi

if [ ! -f "$TARGET" ]; then
    echo "ERROR: Target file not found: $TARGET"
    exit 1
fi

if [ ! -f "$PATTERN" ]; then
    echo "ERROR: Pattern file not found: $PATTERN"
    exit 1
fi

echo "=== Starting Python script ==="

timeout --kill-after=30s 3h python3 $CODE $TARGET $PATTERN $N $OUTPUT_PATH

exit_code=$?
echo "=== Python script finished with exit code: $exit_code ==="