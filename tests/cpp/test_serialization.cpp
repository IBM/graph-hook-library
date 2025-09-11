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
 * @file test_serialization.cpp
 *
 * @brief Unit tests for serialization functionality in GHL.
 *
 * Tests the serialization capabilities:
 * - Primitive edge serialization/deserialization
 * - Base node serialization/deserialization
 * - Complete graph structure serialization/deserialization
 */

#ifdef GHL_SERIALIZATION

#include <ghl/ghl.h>
#include <gtest/gtest.h>
namespace boost = b;

/**
 * @brief Tests serialization of PrimitiveEdge objects.
 *
 * Verifies:
 * - Edge properties can be saved to JSON format
 * - Edge properties can be restored from JSON
 * - ID values are preserved through serialization
 */
TEST(Serialization, PrimitiveEdge) {
  // Create test edge with known ID
  auto edge = std::make_shared<ghl::PrimitiveEdge>(5);

  // Serialize edge to JSON file
  std::ofstream os("out.json", std::ios::binary);
  {
    cereal::JSONOutputArchive oar(os);
    oar(cereal::make_nvp("primitive_edge", edge));
  }

  // Clear edge from memory to ensure complete reload
  edge.reset();

  // Deserialize edge from JSON file
  std::ifstream is("out.json", std::ios::binary);
  cereal::JSONInputArchive iar(is);
  iar(cereal::make_nvp("primitive_edge", edge));

  // Verify ID was preserved through serialization cycle
  ASSERT_EQ(5, edge->id());
}

/**
 * @brief Tests serialization of BaseNode objects.
 *
 * Verifies:
 * - Node properties can be saved to JSON format
 * - Node properties can be restored from JSON
 * - ID values are preserved through serialization
 */
TEST(Serialization, BaseNode) {
  // Create test node with known ID
  auto node = std::make_shared<ghl::BaseNode>(5);

  // Serialize node to JSON file
  std::ofstream os("out.json", std::ios::binary);
  {
    cereal::JSONOutputArchive oar(os);
    oar(cereal::make_nvp("base_node", node));
  }

  // Clear node from memory to ensure complete reload
  node.reset();

  // Deserialize node from JSON file
  std::ifstream is("out.json", std::ios::binary);
  cereal::JSONInputArchive iar(is);
  iar(cereal::make_nvp("base_node", node));

  // Verify ID was preserved through serialization cycle
  ASSERT_EQ(5, node->id());
}

/**
 * @brief Tests serialization of complete graph structures.
 *
 * Verifies:
 * - Complete graphs can be serialized with vertices and edges
 * - Graph structure is preserved through serialization cycle
 * - Vertex and edge counts remain correct after deserialization
 */
TEST(Serialization, Graph) {
  // Type aliases for graph components
  using TemplatedEdge = std::shared_ptr<ghl::PrimitiveEdge>;
  using TemplatedVertex = std::shared_ptr<ghl::BaseNode>;
  using GraphType = ghl::Graph<TemplatedVertex, TemplatedEdge>;

  // Construct test graph with known structure
  auto graph = GraphType();
  auto v1 = std::make_shared<ghl::BaseNode>(0);
  auto v2 = std::make_shared<ghl::BaseNode>(1);
  b::add_vertex(v1, graph);
  b::add_vertex(v2, graph);
  auto edge = std::make_shared<ghl::PrimitiveEdge>(2);
  b::add_edge(0, 1, edge, graph);

  // Serialize complete graph to JSON file
  std::ofstream os("out.json", std::ios::binary);
  {
    cereal::JSONOutputArchive oar(os);
    oar(cereal::make_nvp("graph", graph));
  }

  // Clear graph from memory to ensure complete reload
  graph.clear();

  // Deserialize graph from JSON file
  std::ifstream is("out.json", std::ios::binary);
  cereal::JSONInputArchive iar(is);
  iar(cereal::make_nvp("graph", graph));

  // Verify graph structure was preserved
  ASSERT_EQ(b::num_vertices(graph), 2);
  ASSERT_EQ(b::num_edges(graph), 1);
}

#endif