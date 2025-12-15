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

from graph_tool.all import Graph, subgraph_isomorphism
from benchmark import benchmark
import utils
import sys


target_graph_path, subgraph_path, n, output_path, debug = utils.parse_arguments()
if utils.check_for_precomputed(target_graph_path, subgraph_path, output_path):
    if debug:
        print("Already exists, stopping")
    sys.exit()

prof_obj = {}
if debug:
    print(
        f"Profiling target graph {target_graph_path} with pattern graph {subgraph_path}"
    )

    print("Loading Graphs")
    print("=" * 100)

target_graph_edges = utils.create_edge_list(target_graph_path)
target_graph = Graph(target_graph_edges)

if debug:
    print(f"Target Graph: Total number of Vertices: {len(target_graph.get_vertices())}")
    print(f"Target Graph: Total number of Edges: {len(target_graph.get_edges())}")
    print(f"Loaded Target Graph: {target_graph}")
    print("=" * 20)

prof_obj["target_graph_num_vertices"] = len(target_graph.get_vertices())
prof_obj["target_graph_num_edges"] = len(target_graph.get_edges())
prof_obj["target_graph_path"] = target_graph_path


subgraph_edges = utils.create_edge_list(subgraph_path)
subgraph = Graph(subgraph_edges)

if debug:
    print(f"Subgraph: Total number of Vertices: {len(subgraph.get_vertices())}")
    print(f"Subgraph: Total number of Edges: {len(subgraph.get_edges())}")
    print(f"Loaded Subgraph: {subgraph}")
    print("=" * 100)
    print("Profiling Subgraph Isomorphism")
    print("=" * 100)

prof_obj["subgraph_num_vertices"] = len(subgraph.get_vertices())
prof_obj["subgraph_num_edges"] = len(subgraph.get_edges())
prof_obj["subgraph_path"] = subgraph_path

stats_obj = benchmark(
    "subgraph_isomorphism(subgraph, target_graph)", globals=globals(), n=n
)
prof_obj.update(stats_obj)

returned_val = subgraph_isomorphism(subgraph, target_graph)
num_matchings = len(returned_val)
prof_obj["num_found_matchings"] = num_matchings

if debug:
    print(f"Found {num_matchings} match{'es' if num_matchings > 1 else ''}")

utils.write_to_csv(prof_obj, output_path)

if debug:
    print("DONE")
    print("=" * 100)
