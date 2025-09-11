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
 * @file test_vf3_isomorphisms.cpp
 *
 * @brief Unit tests for graph isomorphism functionality in GHL.
 *
 * Tests the graph isomorphism detection and transformation capabilities:
 * - Custom vertex and edge matching
 * - Isomorphism validation
 * - Graph transformation based on pattern matching
 */

#include <ghl/ghl.h>
#include <ghl/ghl_isomorphisms.h>
#include <gtest/gtest.h>
namespace boost = b;

// Type alias for edge properties using PrimitiveEdge
using TemplatedEdge = std::shared_ptr<ghl::PrimitiveEdge>;
// Type alias for vertex properties using BaseNode
using TemplatedVertex = std::shared_ptr<ghl::BaseNode>;
// Type alias for the main graph type being tested
using GraphType = ghl::Graph<TemplatedVertex, TemplatedEdge>;
// Type alias for the base isomorphism implementation
using ImplementedIsomorphism = ghl::Isomorphism<TemplatedVertex, TemplatedEdge>;

/**
 * @class Vertex
 * @brief Test vertex class extending BaseNode with additional property.
 */
class Vertex : public ghl::BaseNode {
public:
  bool added; //< Flag indicating if vertex was added by transformation

#ifdef GHL_SERIALIZATION
  friend class cereal::access;
  template <class Archive> void serialize(Archive &ar);
#endif

  /**
   * @brief Constructs a vertex with specified ID and added status.
   * @param id Unique identifier for the vertex
   * @param added Whether the vertex was added by transformation
   */
  explicit Vertex(int id, bool added) : ghl::BaseNode(id), added(added) {};

  /**
   * @brief Default constructor required for serialization.
   */
  Vertex() = default;
};

/**
 * @class InsertVertexAfterVertexID
 * @brief Test isomorphism implementation that inserts vertices after matches.
 *
 * Demonstrates pattern-based graph transformation by:
 * - Matching vertices by ID
 * - Validating potential transformations
 * - Inserting new vertices after matched patterns
 */
class InsertVertexAfterVertexID : public ImplementedIsomorphism {
public:
  int id; //< ID to match for vertex insertion

  /**
   * @brief Constructs the isomorphism with target vertex ID.
   * @param input_graph Graph to transform
   * @param id Vertex ID to match
   */
  InsertVertexAfterVertexID(const GraphType &input_graph, int id) : ImplementedIsomorphism(input_graph), id(id) {};

  /**
   * @brief Defines vertex matching criteria based on ID.
   * @return Function implementing vertex comparison logic
   */
  VertexCompFunction vertex_comp_function() final {
    return [this](const GraphType &, const GraphType &g2, const TemplatedVertex, const TemplatedVertex v2) {
      if (auto v = std::dynamic_pointer_cast<ghl::BaseNode>(g2[v2])) {
        bool debugme = v->id() == this->id;
        return v->id() == this->id;
      }
      return false;
    };
  }

  /**
   * @brief Defines edge matching criteria (accepts all edges).
   * @return Function implementing edge comparison logic
   */
  EdgeCompFunction edge_comp_function() final {
    return [](const GraphType &, const GraphType &, const TemplatedEdge, const TemplatedEdge) { return true; };
  }

  /**
   * @brief Validates potential transformations.
   *
   * Ensures vertices aren't added after already transformed vertices.
   *
   * @param graph The graph being transformed
   * @param isomorphism The discovered isomorphism mapping
   * @return true if the transformation is valid, false otherwise
   */
  bool is_isomorphism_valid(const GraphType &graph, const IsoMap &isomorphism) final {
    if (auto v = isomorphism.at(0)[0]; boost::out_degree(v, graph) == 1) {
      if (auto [out_edge_begin, out_edge_end] = b::out_edges(v, graph); out_edge_begin != out_edge_end) {
        if (auto v2 = std::dynamic_pointer_cast<Vertex>(graph[boost::target(*out_edge_begin, graph)])) {
          if (v2->added) {
            return false;
          }
        }
      }
    }
    return true;
  }

  /**
   * @brief Constructs the transformed graph.
   *
   * Creates a new graph with an additional vertex inserted after matches.
   *
   * @param input_graph Original graph to transform
   * @param isomorphism Isomorphism mapping guiding the transformation
   * @return The transformed graph
   */
  GraphType construct_desired_graph(const GraphType &input_graph, IsoMap &isomorphism) final {
    GraphType ret_graph = input_graph;
    auto v = std::dynamic_pointer_cast<ghl::BaseNode>(ret_graph[0]);
    auto v2 = std::make_shared<Vertex>(-1, true);
    boost::add_vertex(v2, ret_graph);
    boost::add_edge(0, 1, std::make_shared<ghl::PrimitiveEdge>(-1), ret_graph);
    return ret_graph;
  }
};

/**
 * @brief Tests vertex insertion transformation.
 *
 * Verifies that:
 * - Pattern matching correctly identifies vertices by ID
 * - New vertices are properly inserted after matches
 * - Graph structure is maintained during transformation
 */
TEST(VF3Isomorphisms, InsertVertexAfterID) {
  auto graph = GraphType();
  auto v1 = std::make_shared<ghl::BaseNode>(0);
  auto v2 = std::make_shared<ghl::BaseNode>(1);
  b::add_vertex(v1, graph);
  b::add_vertex(v2, graph);
  auto edge = std::make_shared<ghl::PrimitiveEdge>(2);
  b::add_edge(0, 1, edge, graph);
  auto extended_graph = ghl::ExtendedGraph(graph);
  auto isomorphism_graph = GraphType();
  auto v3 = std::make_shared<ghl::BaseNode>(-1);
  b::add_vertex(v3, isomorphism_graph);
  auto isomorphism = std::make_shared<InsertVertexAfterVertexID>(isomorphism_graph, 0);
  extended_graph = ghl::ExtendedGraph(graph);
  ghl::apply_isomorphism<TemplatedVertex, TemplatedEdge>(extended_graph, isomorphism, false, true);
  ASSERT_EQ(3, b::num_vertices(extended_graph.graph()));
}