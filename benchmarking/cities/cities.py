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

import os
from pathlib import Path

import cppimport.import_hook  # noqa: F401

import cities
from graph_hook_library.ghl_bindings import apply_isomorphism


def build_city_map():
    city_map = cities.MapGraph()

    a = city_map.add_vertex(cities.City("A", 6))
    b = city_map.add_vertex(cities.City("B", 7))
    c = city_map.add_vertex(cities.City("C", 6))
    d = city_map.add_vertex(cities.City("D", 12))
    e = city_map.add_vertex(cities.City("E", 5))
    f = city_map.add_vertex(cities.City("F", 9))

    city_map.add_edge(a, b, cities.Road(2))
    city_map.add_edge(a, c, cities.Road(2))
    city_map.add_edge(a, d, cities.Road(2))
    city_map.add_edge(b, f, cities.Road(2))
    city_map.add_edge(b, c, cities.Road(2))
    city_map.add_edge(b, d, cities.Road(2))
    city_map.add_edge(b, e, cities.Road(2))
    city_map.add_edge(c, f, cities.Road(2))

    return city_map


def run_insert_hub(output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)

    city_map = build_city_map()
    # Build pattern graph directly (triangle)
    pattern_graph = cities.MapGraph()
    p0 = pattern_graph.add_vertex(cities.City("any", 0, False))
    p1 = pattern_graph.add_vertex(cities.City("any", 0, False))
    p2 = pattern_graph.add_vertex(cities.City("any", 0, False))
    pattern_graph.add_edge(p0, p1, cities.Road(0))
    pattern_graph.add_edge(p0, p2, cities.Road(0))
    pattern_graph.add_edge(p1, p2, cities.Road(0))

    iso = cities.InsertHub(pattern_graph.graph())

    city_map.write_graph(str(output_dir / "cities_no_hub"), True)
    apply_isomorphism(city_map, iso, True, True)
    city_map.write_graph(str(output_dir / "cities_inserted_hub"), True)

    print(f"Wrote DOT/PDF graphs to {output_dir}")


if __name__ == "__main__":
    here = Path(__file__).parent
    os.chdir(here)
    run_insert_hub(here / "output")
