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

import csv
import os
import sys
import psutil
import threading
import time

fieldnames = [
    "target_graph_num_vertices",
    "target_graph_num_edges",
    "target_graph_path",
    "subgraph_num_vertices",
    "subgraph_num_edges",
    "subgraph_path",
    "min",
    "max",
    "num_samples",
    "median",
    "mean",
    "stdev",
    "num_found_matchings",
]


fieldnames_mem = [
    "target_graph_num_vertices",
    "target_graph_num_edges",
    "target_graph_path",
    "subgraph_num_vertices",
    "subgraph_num_edges",
    "subgraph_path",
    "num_found_matchings",
    "memray_peak_bytes",
]


def check_for_precomputed(target_graph_path, subgraph_path, file_path):
    if not os.path.exists(file_path):
        return False
    import pandas

    df = pandas.read_csv(file_path)
    mask = (df["target_graph_path"] == target_graph_path) & (
        df["subgraph_path"] == subgraph_path
    )
    return mask.any()


def create_edge_list(filename):
    edges = []
    with open(filename, "r") as f:
        lines = f.readlines()
        for i, line in enumerate(lines):
            if i == 0:
                continue
            else:
                vertex_index = i - 1
                line = line.strip()
                neighbors_str = line.split(" ")
                for j, neigh in enumerate(neighbors_str):
                    if j == 0:
                        continue
                    else:
                        neigh = int(neigh)
                        edge = (vertex_index, neigh)
                        edges.append(edge)
    return edges


def create_edge_list_labeled_graphs(filename):
    with open(filename, "r") as f:
        lines = f.readlines()

    n = int(lines[0].strip())
    edge_list = []
    vertex_labels = {}
    edge_labels = {}

    for i in range(1, n + 1):
        line = lines[i].strip()
        if "|" not in line:
            raise ValueError(
                f"Separation sign | does not exist in {filename} on line {i}"
            )
        edge_part, vertex_label = line.split("|")
        vertex_id = i - 1
        vertex_labels[vertex_id] = vertex_label

        edge_tokens = edge_part.strip().split()
        for token in edge_tokens:
            if ":" not in token:
                continue
            target_str, label = token.split(":")
            target = int(target_str)
            edge_list.append((vertex_id, target))
            edge_labels[(vertex_id, target)] = label.strip()

    return edge_list, vertex_labels, edge_labels


def write_to_csv(data, filename):
    write_header = True
    if os.path.exists(filename):
        write_header = False
    with open(filename, "a+", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        if write_header:
            writer.writeheader()
        writer.writerow(data)


def write_to_csv_mem(data, filename):
    write_header = True
    if os.path.exists(filename):
        write_header = False
    with open(filename, "a+", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames_mem)
        if write_header:
            writer.writeheader()
        writer.writerow(data)


def parse_arguments():
    target_graph_path = sys.argv[1]
    subgraph_path = sys.argv[2]
    n = int(sys.argv[3])
    output_path = sys.argv[4]
    debug = True if len(sys.argv) > 5 else False

    return (target_graph_path, subgraph_path, n, output_path, debug)


class MemoryMonitor:
    """
    Monitors the peak memory usage of the process it's running in.
    """

    def __init__(self, interval=0.01):
        self.interval = interval
        self.peak_memory = 0
        self.monitoring = False
        self.thread = None
        self.process = psutil.Process()

    def _monitor(self):
        while self.monitoring:
            try:
                mem_info = self.process.memory_info()
                self.peak_memory = max(mem_info.rss, self.peak_memory)
            except Exception as e:
                print(f"EXCEPTION: {e}")
                pass
        time.sleep(self.interval)

    def start(self):
        self.monitoring = True
        self.peak_memory = 0
        self.thread = threading.Thread(target=self._monitor, daemon=True)
        self.thread.start()

    def stop(self):
        self.monitoring = False
        if self.thread:
            self.thread.join(timeout=1.0)
        return self.peak_memory

    def get_peak_bytes(self):
        return self.peak_memory
