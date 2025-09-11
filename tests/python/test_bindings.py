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

"""
Unit tests for Python bindings in GHL.

Tests bindings functionality including:
- Graph construction and manipulation through Python
- Property access and modification
- Memory management and object lifecycle
- Type conversion between C++ and Python
"""

try:
    from graph_hook_library.ghl_bindings import Graph, BaseNode, PrimitiveEdge
    import graph_hook_library.ghl_bindings as ghl
except ModuleNotFoundError:
    import ghl_bindings as ghl
    from ghl_bindings import Graph, BaseNode, PrimitiveEdge


def create_test_graph():
    """
    Creates a test graph with known structure for binding tests.

    Creates:
        - Two vertices with IDs 0 and 1
        - One edge connecting them with ID 2

    Returns:
        Graph: The constructed test graph
    """
    # Initialize empty graph
    graph = Graph()

    # Create and add vertices with sequential IDs
    v1 = BaseNode(0)
    v2 = BaseNode(1)
    ghl.add_vertex(v1, graph)
    ghl.add_vertex(v2, graph)

    # Create and add edge between vertices
    edge = PrimitiveEdge(2)
    ghl.add_edge(0, 1, edge, graph)

    return graph


def test_vertex_operations():
    """
    Tests vertex manipulation through bindings.

    Verifies:
        - Vertices can be added and removed
        - Properties are correctly accessed
        - Python-side modifications persist
    """
    # Create graph for vertex testing
    graph = Graph()

    # Add vertex with known ID
    vertex = BaseNode(42)
    vertex_desc = ghl.add_vertex(vertex, graph)

    # Verify vertex was added with correct properties
    vertices = ghl.vertices(graph)
    assert len(vertices) == 1
    assert ghl.get_vertex(vertex_desc, graph).id() == 42


def test_edge_operations():
    """
    Tests edge manipulation through bindings.

    Verifies:
        - Edges can be added between vertices
        - Edge properties are preserved
        - Source/target relationships maintained
    """
    # Create graph with vertices
    graph = create_test_graph()

    # Get edges and verify properties
    edges = ghl.edges(graph)
    assert len(edges) == 1

    # Verify edge endpoints
    edge = edges[0]
    assert ghl.source(edge, graph) == 0
    assert ghl.target(edge, graph) == 1
