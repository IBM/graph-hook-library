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

import graph_hook_library.ghl_bindings as ghl
from graph_hook_library.ghl_bindings import DirectedGraph, BaseNode, DirectedIsomorphism
from benchmark import benchmark
import utils
import sys

target_graph_path = sys.argv[1]
subgraph_path = sys.argv[2]
n = int(sys.argv[3])
output_path = sys.argv[4]


def ghl_construct_graph_from_edges(edges):
    graph = DirectedGraph()
    vertex_ids = set()
    for src, dst in edges:
        vertex_ids.add(src)
        vertex_ids.add(dst)

    for vid in vertex_ids:
        vertex = BaseNode(vid)
        ghl.add_vertex(vertex, graph)

    for i, (src, dst) in enumerate(edges):
        edge = ghl.PrimitiveEdge(i)
        ghl.add_edge(src, dst, edge, graph)
    return graph


prof_obj = {}

target_graph_edges = utils.create_edge_list(target_graph_path)
target_graph = ghl_construct_graph_from_edges(target_graph_edges)

prof_obj["target_graph_num_vertices"] = len(ghl.vertices(target_graph))
prof_obj["target_graph_num_edges"] = len(ghl.edges(target_graph))
prof_obj["target_graph_path"] = target_graph_path

subgraph_edges = utils.create_edge_list(subgraph_path)
subgraph = ghl_construct_graph_from_edges(subgraph_edges)

prof_obj["subgraph_num_vertices"] = len(ghl.vertices(subgraph))
prof_obj["subgraph_num_edges"] = len(ghl.edges(subgraph))
prof_obj["subgraph_path"] = subgraph_path

iso = DirectedIsomorphism(target_graph)

stats_obj = benchmark(
    "iso.discover(subgraph, target_graph, first=False, use_vf3=True)",
    globals=globals(),
    n=n,
)
prof_obj.update(stats_obj)

returned_val = iso.discover(subgraph, target_graph, first=False, use_vf3=True)
prof_obj["num_found_matchings"] = len(returned_val)

utils.write_to_csv(prof_obj, output_path)

print("=" * 100)
