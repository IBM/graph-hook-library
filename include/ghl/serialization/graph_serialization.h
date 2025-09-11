/*
 * Copyright (c) 2025 IBM
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file graph_serialization.h
 *
 * @brief Provides serialization support for graph structures in the GHL.
 *
 * This file implements serialization and deserialization functionality for GHL
 * graph structures using the cereal library. It enables graphs to be saved to
 * and loaded from various formats including JSON and binary. The implementation
 * handles both vertex and edge properties, maintaining the complete graph
 * structure during serialization.
 *
 * Dependencies:
 * - cereal (>= 1.3.0)
 */

#ifndef GRAPH_SERIALIZATION_H
#define GRAPH_SERIALIZATION_H

#include "../cereal/cereal.h"
#include "../graph/graph.h"

namespace ghl {
// Registers base node class serialization with cereal
SERIALIZE_BASE(BaseNode, CEREAL_NVP(id_))

// Registers primitive edge class serialization with cereal
SERIALIZE_BASE(PrimitiveEdge, CEREAL_NVP(id_))
} // namespace ghl
namespace b = boost;

namespace cereal {

/**
 * @brief Serializes a GHL graph to an archive.
 *
 * Saves the complete graph structure including vertices, edges, and their
 * associated properties. The serialization process preserves the graph topology
 * and all property data.
 *
 * @tparam Archive The type of archive to serialize to
 * @tparam VertexProperties The type of vertex properties in the graph
 * @tparam EdgeProperties The type of edge properties in the graph
 *
 * @param ar The archive to save to
 * @param graph The graph to serialize
 */
template <class Archive, typename VertexProperties, typename EdgeProperties = b::no_property>
void save(Archive &ar, ghl::Graph<VertexProperties, EdgeProperties> const &graph) {
  using serializable_edge = std::tuple<unsigned long int, unsigned long int, EdgeProperties>;

  auto n_vertices = num_vertices(graph); //< Get total vertex count
  ar(cereal::make_nvp("n_vertices", n_vertices));

  std::vector<VertexProperties> vertices_;
  for (auto v : b::make_iterator_range(b::vertices(graph))) {
    vertices_.push_back(graph[v]);
  }
  ar(cereal::make_nvp("vertices", vertices_));

  auto n_edges = num_edges(graph); //< Get total edge count
  ar(cereal::make_nvp("n_edges", n_edges));

  std::vector<serializable_edge> edges_;
  for (auto e : b::make_iterator_range(b::edges(graph))) {
    unsigned long int source = b::source(e, graph);
    unsigned long int target = b::target(e, graph);
    edges_.push_back({source, target, graph[e]});
  }
  ar(cereal::make_nvp("edges", edges_));
}

/**
 * @brief Deserializes a GHL graph from an archive.
 *
 * Reconstructs a complete graph structure from serialized data, including
 * vertices, edges, and their associated properties. The deserialization process
 * restores the original graph topology and all property data.
 *
 * @tparam Archive The type of archive to deserialize from
 * @tparam VertexProperties The type of vertex properties in the graph
 * @tparam EdgeProperties The type of edge properties in the graph
 *
 * @param ar The archive to load from
 * @param graph The graph to deserialize into
 */
template <class Archive, typename VertexProperties, typename EdgeProperties = b::no_property>
void load(Archive &ar, ghl::Graph<VertexProperties, EdgeProperties> &graph) {
  using serializable_edge = std::tuple<unsigned long int, unsigned long int, EdgeProperties>;

  int n_vertices; //< Number of vertices to restore
  ar(cereal::make_nvp("n_vertices", n_vertices));

  std::vector<VertexProperties> vertices_;
  ar(cereal::make_nvp("vertices", vertices_));
  for (auto v : vertices_) {
    b::add_vertex(v, graph);
  }

  int n_edges; //< Number of edges to restore
  ar(cereal::make_nvp("n_edges", n_edges));

  std::vector<serializable_edge> edges_;
  ar(cereal::make_nvp("edges", edges_));
  for (auto e : edges_) {
    b::add_edge(std::get<0>(e), std::get<1>(e), std::get<2>(e), graph);
  }
}

} // namespace cereal

#endif // GRAPH_SERIALIZATION_H