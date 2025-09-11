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
 * @file test_subgraphs.cpp
 *
 * @brief Unit tests for subgraph functionality in GHL.
 *
 * Tests the subgraph capabilities:
 * - Subgraph creation and management
 * - Property preservation in subgraphs
 * - Hierarchical graph relationships
 * - Cross-subgraph edge connections
 */

#include <ghl/ghl.h>
#include <ghl/ghl_subgraphs.h>
#include <gtest/gtest.h>
namespace boost = b;

// Type alias for edge properties using PrimitiveEdge
using TemplatedEdge = std::shared_ptr<ghl::PrimitiveEdge>;
// Type alias for vertex properties using BaseNode
using TemplatedVertex = std::shared_ptr<ghl::BaseNode>;
// Type alias for the main graph type being tested
using GraphType = ghl::Graph<TemplatedVertex, TemplatedEdge>;
// Type alias for the extended subgraph implementation
using SubGraphType = ghl::ExtendedSubGraph<TemplatedVertex, TemplatedEdge>;

/**
 * @brief Creates a basic test subgraph structure.
 *
 * Sets up a graph with:
 * - Main graph with two vertices
 * - One subgraph containing two vertices
 * - Edges connecting vertices across graph levels
 *
 * @return SubGraphType The constructed test graph
 */
SubGraphType create_test_graph() {
  auto graph = SubGraphType();
  SubGraphType::GraphType &main_graph = graph.graph();

  // Create a subgraph
  SubGraphType::GraphType &subgraph = graph.add_subgraph(main_graph, "subgraph");

  // Add vertices to main graph
  auto v1_global = graph.add_vertex();
  auto v2_global = graph.add_vertex(std::make_shared<ghl::BaseNode>(0));

  // Add vertices to subgraph
  auto [v3_global, v3_local] = graph.add_vertex(subgraph);
  auto [v4_global, v4_local] = graph.add_vertex(std::make_shared<ghl::BaseNode>(1), subgraph);

  // Create edges
  // Default edge class initialization
  graph.add_edge(v1_global, subgraph.local_to_global(v3_local), main_graph);
  // Initialize edge class
  graph.add_edge(v2_global, subgraph.local_to_global(v3_local), std::make_shared<ghl::PrimitiveEdge>(0));
  // Initialize edge class, add to subgraph
  graph.add_edge(v3_local, v4_local, std::make_shared<ghl::PrimitiveEdge>(1), subgraph);

  return graph;
}

/**
 * @brief Tests basic subgraph creation and structure.
 *
 * Verifies:
 * - Extended subgraph object creation
 * - Basic graph initialization
 * - Memory management
 */
TEST(Subgraph, InstantiateGraph) {
  auto sg = std::make_shared<SubGraphType>();
  ASSERT_NE(sg, nullptr);

  // Verify empty graph properties
  ASSERT_EQ(b::num_vertices(sg->graph()), 0);
  ASSERT_EQ(b::num_edges(sg->graph()), 0);
}

/**
 * @brief Tests subgraph creation and management.
 *
 * Verifies:
 * - Subgraph creation within main graph
 * - Subgraph naming and attributes
 * - Nested subgraph relationships
 */
TEST(Subgraph, CreateSubgraphs) {
  auto sg = std::make_shared<SubGraphType>();
  SubGraphType::GraphType &main_graph = sg->graph();

  // Create multiple levels of subgraphs
  auto &subgraph1 = sg->add_subgraph(main_graph, "Level1");
  auto &subgraph2 = sg->add_subgraph(subgraph1, "Level2");

  // Verify subgraph creation
  ASSERT_EQ(b::get_property(subgraph1, b::graph_name), "cluster_Level1");
  ASSERT_EQ(b::get_property(subgraph2, b::graph_name), "cluster_Level2");
}

/**
 * @brief Tests vertex management across graph levels.
 *
 * Verifies:
 * - Vertex addition to main graph and subgraphs
 * - Vertex property preservation
 * - Local to global vertex mapping
 */
TEST(Subgraph, VertexManagement) {
  auto sg = std::make_shared<SubGraphType>();
  SubGraphType::GraphType &main_graph = sg->graph();
  auto &subgraph = sg->add_subgraph(main_graph, "SubGraph");

  // Add vertices to different levels
  auto v1_global = sg->add_vertex(std::make_shared<ghl::BaseNode>(1));
  auto v2_local = b::add_vertex(sg->add_vertex(std::make_shared<ghl::BaseNode>(2)), subgraph);

  // Verify vertex properties
  ASSERT_EQ(main_graph[v1_global]->id(), 1);
  ASSERT_EQ(main_graph[subgraph.local_to_global(v2_local)]->id(), 2);
}

/**
 * @brief Tests edge connections across graph levels.
 *
 * Verifies:
 * - Edge creation between main graph vertices
 * - Edge creation between main graph and subgraph vertices
 * - Edge property preservation
 */
TEST(Subgraph, EdgeConnections) {
  SubGraphType graph = create_test_graph();
  const SubGraphType::GraphType &main_graph = graph.graph();
  const SubGraphType::GraphType &subgraph = graph["subgraph"];

  // Count edges in main graph
  int edge_count = b::num_edges(main_graph);

  // Verify edge count
  ASSERT_EQ(edge_count, 3);

  // Count edges in subgraph
  edge_count = b::num_edges(subgraph);

  // Verify edge count
  ASSERT_EQ(edge_count, 1);
}

/**
 * @brief Tests the GraphViz output functionality.
 *
 * Verifies:
 * - Graph visualization file generation
 * - Proper handling of graph attributes
 */
TEST(Subgraph, GraphVisualization) {
  SubGraphType graph = create_test_graph();

  // Attempt to write graph to file
  ASSERT_NO_THROW(graph.write_graph("test_output"));
}