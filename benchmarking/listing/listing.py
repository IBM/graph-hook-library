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

import cppimport.import_hook  # noqa: F401 # isort: skip # Import cppimport
import listing  # listing.cpp will be transparently recompiled here if previously modified
from graph_hook_library.ghl_bindings import (
    UndirectedExtendedGraph,
    PrimitiveEdge,
    apply_isomorphism,
)

target_graph = (
    UndirectedExtendedGraph()
)  # Create graph on which isomorphism will be run
v0 = target_graph.add_vertex(listing.City(0, 100))  # Add city
v1 = target_graph.add_vertex(listing.City(1, 100))  # Add city
e = target_graph.add_edge(v0, v1, PrimitiveEdge(0))  # Add edge
target_graph[v0].population = 500  # Modify city population
pattern_graph = UndirectedExtendedGraph()  # Create pattern graph
pattern_graph.add_vertex(listing.City(0, 100))  # Add city
lanes_iso = listing.LanesIso(
    pattern_graph.graph()
)  # Create isomorphism with pattern graph
print(
    "City populations:",
    ", ".join(str(target_graph[v].population) for v in target_graph.vertices()),
)
# "City populations: 500, 100"
apply_isomorphism(target_graph, lanes_iso, True, True)  # Apply isomorphism to graph
print(
    "City populations:",
    ", ".join(str(target_graph[v].population) for v in target_graph.vertices()),
)
# "City populations: 500, 10000"
