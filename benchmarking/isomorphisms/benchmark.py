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

import timeit
import statistics
import time


def benchmark(stmt, n, globals, debug=False):
    timer = time.process_time
    times = timeit.repeat(stmt, number=1, globals=globals, repeat=n, timer=timer)
    print(f"Times: {times}")
    stats_obj = {}
    stats_obj["min"] = min(times)
    stats_obj["max"] = max(times)
    stats_obj["num_samples"] = len(times)
    stats_obj["median"] = statistics.median(times)
    stats_obj["mean"] = statistics.mean(times)
    stats_obj["stdev"] = statistics.stdev(times) if len(times) > 1 else "N.A."
    if debug:
        print("Function:", stmt)
        print("  --------------")
        print(f"  Min:      {min(times)}")
        print(f"  Median:   {statistics.median(times)}")
        print(f"  Mean:     {statistics.mean(times)}")
        print(f'  Stdev:    {statistics.stdev(times) if len(times) > 1 else "N.A."}')
        print(f"  Max:      {max(times)}")
        print("  --------------")
        print(f"  samples:  {len(times)}")
    return stats_obj
