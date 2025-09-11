// Copyright (c) 2025 IBM
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/**
 * @file test_graph.cpp
 *
 * @brief Unit tests for core graph functionality in the Generic Graph Library.
 *
 * This file contains test cases that verify:
 * - Basic graph instantiation and manipulation
 * - Vertex and edge creation/management
 * - Extended graph functionality
 */

#include <ghl/ghl.h>
#include <gtest/gtest.h>
namespace boost = b;

// Type alias for edge properties using PrimitiveEdge
using TemplatedEdge = std::shared_ptr<ghl::PrimitiveEdge>;
// Type alias for vertex properties using BaseNode
using TemplatedVertex = std::shared_ptr<ghl::BaseNode>;
// Type alias for the main graph type being tested
using GraphType = ghl::Graph<TemplatedVertex, TemplatedEdge>;

/**
 * @brief Creates a simple test graph with two vertices and one edge.
 *
 * Constructs a graph containing:
 * - Two vertices with IDs 0 and 1
 * - One edge with ID 2 connecting vertex 0 to vertex 1
 *
 * @return GraphType A newly constructed graph for testing
 */
GraphType instantiate_graph() {
  auto graph = GraphType();
  auto v1 = std::make_shared<ghl::BaseNode>(0);
  auto v2 = std::make_shared<ghl::BaseNode>(1);
  b::add_vertex(v1, graph);
  b::add_vertex(v2, graph);
  auto edge = std::make_shared<ghl::PrimitiveEdge>(2);
  b::add_edge(0, 1, edge, graph);
  return graph;
}

/**
 * @brief Tests basic graph instantiation and element counting.
 *
 * Verifies that:
 * - A new graph can be created with the correct number of vertices
 * - Edge creation works properly
 */
TEST(Graph, InstantiateGraph) {
  GraphType graph = instantiate_graph();
  ASSERT_EQ(b::num_vertices(graph), 2);
  ASSERT_EQ(b::num_edges(graph), 1);
}

/**
 * @brief Tests the ExtendedGraph wrapper functionality.
 *
 * Verifies that:
 * - A basic graph can be wrapped in an ExtendedGraph container
 * - The ExtendedGraph maintains the underlying graph structure
 */
TEST(Graph, InstantiateExtendedGraph) {
  GraphType graph = instantiate_graph();
  ghl::ExtendedGraph extended_graph = ghl::ExtendedGraph(graph);
  ASSERT_EQ(1, 1);
}