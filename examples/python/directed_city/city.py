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

import os
from pathlib import Path

import cppimport.import_hook  # noqa: F401, E402
from ghl_city import Capitals, City, MapGraph, Road

from graph_hook_library.ghl_bindings import DirectedGraph, apply_isomorphism
import graph_hook_library.ghl_bindings as ghl


def make_small_map():
    """
    Creates a test graph with known structure for binding tests.

    Creates:
        - Two vertices with IDs 0 and 1
        - One edge connecting them with ID 2

    Returns:
        Graph: The constructed test graph
    """
    graph = MapGraph()

    v0 = graph.add_vertex(City("Chicago", 1000, True))
    v1 = graph.add_vertex(City("New York", 10000, True))
    v2 = graph.add_vertex(City("Buffalo", 100, False))
    graph.add_edge(v0, v1, Road(2))
    graph.add_edge(v1, v2, Road(2))
    graph.add_edge(v2, v0, Road(2))
    graph.write_graph(os.path.join("results", "small_graph"), False)
    print("Small map printed to results")


def make_map():
    """
    Mirrors the C++ make_map example by constructing a graph of Italian
    and French cities along with the same road layout.

    Returns:
        MapGraph: Graph populated with cities and roads described in
        02_isomorphism.cpp.
    """
    graph = MapGraph()

    city_specs = [
        ("Rome", 100, True),
        ("Florence", 100, True),
        ("Naples", 1000, True),
        ("Venice", 100, True),
        ("Milan", 1000, True),
        ("Paris", 100, True),
        ("Marsailles", 100, True),
        ("Lyon", 1000, True),
        ("Stuttgart", 100, True),
        ("Dijon", 1000, True),
    ]
    city_vertices = [
        graph.add_vertex(City(name, population, has_post_office))
        for name, population, has_post_office in city_specs
    ]

    for index in range(1, 5):
        graph.add_edge(city_vertices[index], city_vertices[0], Road(2))

    for index in range(6, 10):
        graph.add_edge(city_vertices[index], city_vertices[5], Road(2))

    graph.add_edge(city_vertices[4], city_vertices[7], Road(2))
    graph.add_edge(city_vertices[7], city_vertices[4], Road(2))

    return graph


def perform_isomorphism():
    """
    Replicates the C++ isomorphism example: builds the map, applies the
    Capitals isomorphism, and writes both the original and transformed graphs.
    """
    map_graph = make_map()
    map_graph.write_graph(os.path.join("results", "original_graph"), False)

    isomorphism_graph = DirectedGraph()
    ghl.add_vertex(City("Capital", -1, False), isomorphism_graph)
    capitals = {"Paris", "Rome"}

    isomorphism = Capitals(isomorphism_graph, capitals)
    discovered_matches = isomorphism.discover(
        isomorphism_graph, map_graph.graph(), first=False, use_vf3=True
    )
    print(f"Discovered {len(discovered_matches)} matching capitals")
    apply_isomorphism(map_graph, isomorphism, True, False)

    map_graph.write_graph(os.path.join("results", "transformed_graph"), False)
    print("Isomorphism applied; results stored in results")


if __name__ == "__main__":
    os.chdir(Path(__file__).parent)
    os.makedirs("results", exist_ok=True)
    make_small_map()
    perform_isomorphism()
