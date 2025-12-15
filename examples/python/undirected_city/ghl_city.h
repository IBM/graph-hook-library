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
 * @file city.h
 *
 * @brief Defines city and road classes for graph-based map examples.
 *
 * This file contains two classes that demonstrate custom vertex and edge
 * property implementations in the GHL:
 * - City: A vertex property class representing urban centers
 * - Road: An edge property class representing connections between cities
 *
 * These classes are used in examples to show how to create domain-specific
 * graph structures using the GHL's extension mechanisms. They demonstrate
 * proper inheritance from BaseNode and PrimitiveEdge respectively.
 */

#ifndef EXAMPLES_CITY_H
#define EXAMPLES_CITY_H

#include <format>
#include <ghl/ghl.h>
#include <ghl/ghl_isomorphisms.h>

// namespace b = boost;

/**
 * @class City
 * @brief Represents a city as a vertex property in a map graph.
 *
 * This class extends BaseNode to represent cities in a map graph. Each city
 * has properties including:
 * - Name
 * - Presence of a post office
 * - Population
 *
 * Used in examples to demonstrate custom vertex properties and graph
 * visualization. See examples/01_graph.cpp and examples/02_isomorphism.cpp
 * and examples/04_insert_hub.cpp for usage examples.
 */
class City : public ghl::BaseNode {
  std::string name_;     //< Name of the city
  int population_;       //< Population of the city
  bool has_post_office_; //< Whether the city has a post office

public:
  /**
   * @brief Constructs a City with specified properties.
   *
   * @param name Name of the city
   * @param has_post_office Whether the city has a post office (default: true)
   * @param population Population of the city (default: 100)
   */
  explicit City(const std::string &name, int population = 100, bool has_post_office = true)
      : name_(name), population_(population), has_post_office_(has_post_office) {}

  /**
   * @brief Creates a clone of the city.
   *
   * @return std::shared_ptr<BaseNode> Smart pointer to a new City instance
   *         with the same properties
   */
  std::shared_ptr<BaseNode> clone() final { return std::make_shared<City>(name_, population_, has_post_office_); }

  /**
   * @brief Gets the city's name.
   * @return The name of the city
   */
  std::string name() const { return name_; }

  /**
   * @brief Checks if the city has a post office.
   * @return true if the city has a post office, false otherwise
   */
  bool hasPostOffice() const { return has_post_office_; }

  /**
   * @brief Gets the city's population.
   * @return The population of the city
   */
  int population() const { return population_; }

  /**
   * @brief Set city graph attributes. Necessary for subgraph DOT file
   * generation.
   * @return A map of attribute strings to their values
   */
  std::map<std::string, std::string, std::less<>> graphAttributes() final {
    return std::map<std::string, std::string, std::less<>>{
        {"label", std::format("City: {} Population: {}", name_, population_)}};
  }
};

/**
 * @class HUB
 * @brief Represents a hub as a vertex property in a map graph.
 *
 * This class extends BaseNode to represent hubs in a map graph.
 * HUB stands for an efficient road connection.
 *
 * Used in examples to demonstrate custom vertex properties and graph
 * visualization. See examples/04_insert_hub.cpp
 */
class HUB : public ghl::BaseNode {

public:
  explicit HUB() {}

  /**
   * @brief Creates a clone of the hub.
   *
   * @return std::shared_ptr<BaseNode> Smart pointer to a new HUB instance
   *         with the same properties
   */
  std::shared_ptr<BaseNode> clone() final { return std::make_shared<HUB>(); }

  /**
   * @brief Set city graph attributes. Necessary for subgraph DOT file
   * generation.
   * @return A map of attribute strings to their values
   */
  std::map<std::string, std::string, std::less<>> graphAttributes() final {
    return std::map<std::string, std::string, std::less<>>{{"label", "HUB"}};
  }
};

/**
 * @class Road
 * @brief Represents a road as an edge property in a map graph.
 *
 * This class extends PrimitiveEdge to represent roads connecting cities
 * in a map graph. Each road has a property specifying the number of lanes,
 * representing its capacity.
 *
 * Used in examples to demonstrate custom edge properties and graph
 * visualization. See examples/01_graph.cpp and examples/04_insert_hub.cpp for usage examples.
 */
class Road : public ghl::PrimitiveEdge {
  // Number of lanes in the road
  int num_lanes_;

public:
  /**
   * @brief Constructs a Road with specified number of lanes.
   *
   * @param num_lanes Number of lanes in the road
   */
  explicit Road(int num_lanes) : num_lanes_(num_lanes) {}

  /**
   * @brief Gets the number of lanes in the road.
   * @return The number of lanes
   */
  int numLanes() const { return num_lanes_; }
};

// Type alias for vertex properties in the map graph
using VertexProperty = std::shared_ptr<ghl::BaseNode>;

// Type alias for edge properties in the map graph
using EdgeProperty = std::shared_ptr<ghl::PrimitiveEdge>;

/**
 * @class MapGraph
 * @brief ExtendedGraph specialization with custom DOT writers.
 */
class MapGraph : public ghl::UndirectedExtendedGraph<VertexProperty, EdgeProperty> {
  using Base = ghl::UndirectedExtendedGraph<VertexProperty, EdgeProperty>;

public:
  using Base::Base;

  void vertex_property_writer(std::ostream &out, const VertexDescriptor &v) const override {
    auto city = std::dynamic_pointer_cast<City>(this->graph()[v]);
    if (city) {
      out << std::format("[label=\"City: {}\nPopulation: {}\"]", city->name(), city->population());
    } else {
      out << std::format("[label=\"HUB \"]");
    }
  }

  void edge_property_writer(std::ostream &out, const EdgeDescriptor &e) const override {
    auto road = std::dynamic_pointer_cast<Road>(this->graph()[e]);
    out << std::format("[label=\"num lanes: {}\"]", road->numLanes());
  }
};

using ImplementedIsomorphism = ghl::UndirectedIsomorphism<VertexProperty, EdgeProperty>;

/**
 * @class Capitals
 * @brief Isomorphism class that disconnects all cities only connecting to
 * a capital and connects them to each other, leaving cities connected to a capital
 * and another city untouched.
 *
 * This class implements a graph transformation that:
 * - Identifies vertices (cities) with roads leading to a capital
 * - Removes those roads and creates new connections between the cities
 * - Preserves the original city properties while modifying the road structure
 */
class Capitals : public ImplementedIsomorphism {
  std::set<std::string> capitals_;

public:
  /**
   * @brief Constructs the isomorphism with a pattern graph.
   * @param iso_graph Graph pattern to match for transformation
   */
  explicit Capitals(const MapGraph::GraphType &iso_graph, std::set<std::string> capitals)
      : ImplementedIsomorphism(iso_graph), capitals_(capitals) {};

  /**
   * @brief Defines vertex matching criteria based on city names.
   * @return Function implementing vertex comparison logic
   */
  VertexCompFunction vertex_comp_function() final {
    return [this](const GraphType &iso_graph, const GraphType &target_graph, const VertexDescriptor iso_vertex,
                  const VertexDescriptor target_vertex) {
      auto target_city = std::dynamic_pointer_cast<City>(target_graph[target_vertex]);
      return target_city && this->capitals_.find(target_city->name()) != this->capitals_.end();
    };
  }

  /**
   * @brief Defines edge matching criteria (accepts all edges).
   * @return Function implementing edge comparison logic
   */
  EdgeCompFunction edge_comp_function() final {
    return [](const GraphType &, const GraphType &, const EdgeDescriptor, const EdgeDescriptor) { return true; };
  }

  /**
   * @brief Specializes the isomorphism by identifying cities with single roads.
   *
   * Refines the isomorphism mapping to consist of cities connected to the
   * initially matched city that have a single outgoing road (all cities leading
   * only to Rome.)
   *
   * @param graph The graph being transformed
   * @param isomorphism Initial isomorphism mapping
   * @return Specialized isomorphism mapping
   */
  IsoMap specialize_isomorphism(const GraphType &graph, const IsoMap &isomorphism) final {
    IsoMap ret_iso;
    VertexDescriptor iso_vertex = 0;
    for (auto edge : b::make_iterator_range(b::in_edges(isomorphism.at(0)[0], graph))) {
      if (b::out_degree(b::source(edge, graph), graph) == 1) {
        ret_iso[iso_vertex].push_back(b::source(edge, graph));
        iso_vertex++;
      }
    }
    return ret_iso;
  }

  /**
   * @brief Validates the isomorphism based on city population.
   *
   * Only allows transformation of cities with non-zero population.
   *
   * @param graph The graph being transformed
   * @param isomorphism The isomorphism mapping to validate
   * @return true if the transformation is valid, false otherwise
   */
  bool is_isomorphism_valid(const GraphType &graph, const IsoMap &isomorphism) final {
    if (isomorphism.empty())
      return false;
    for (auto [iso_vertex, target_vertex_vec] : isomorphism) {
      auto target_vertex = target_vertex_vec[0];
      if (auto city = std::dynamic_pointer_cast<City>(graph[target_vertex]); city && city->population() == 0)
        return false;
    }
    return true;
  }

  /**
   * @brief Constructs the transformed graph structure.
   *
   * Creates a new graph where roads no longer lead to Rome, instead connecting
   * the matched cities to each other.
   *
   * @param input_graph Original graph to transform
   * @param isomorphism Isomorphism mapping guiding the transformation
   * @return The transformed graph
   */
  GraphType construct_desired_graph(const GraphType &input_graph, IsoMap &isomorphism) final {
    GraphType ret_graph = input_graph;

    // Remove all existing edges from matched vertices
    for (const auto &[iso_vertex, _] : isomorphism) {
      b::clear_vertex(iso_vertex, ret_graph);
    }

    // Create new edges between all pairs of matched vertices
    for (const auto &[source_iso_vertex, source_vertex_vec] : isomorphism) {
      for (const auto &[target_iso_vertex, target_vertex_vec] : isomorphism) {
        if (source_iso_vertex == target_iso_vertex)
          continue;
        auto road = std::make_shared<Road>(2);
        b::add_edge(source_iso_vertex, target_iso_vertex, road, ret_graph);
      }
    }
    return ret_graph;
  }

  /**
   * @brief Defines rules for reconstructing external edges after
   * transformation.
   *
   * Controls how edges connecting to non-transformed vertices are handled:
   * - Preserves incoming edges to transformed vertices
   * - Only maintains outgoing edges from cities with large populations
   * (connect cities with large populations to Rome)
   *
   * @return Function implementing edge reconstruction logic
   */
  ReconstructEdgesFunction reconstruct_edges_function() final {
    return [](const std::vector<TemplatedEdgeReplacementStruct> &external_incoming_edges,
              const std::vector<TemplatedEdgeReplacementStruct> &external_outgoing_edges,
              const std::vector<VertexDescriptor> &, const std::vector<VertexDescriptor> &, GraphType &graph) {
      // Reconstruct incoming edges
      for (auto &[src, _old_trg, new_trg, i_edge_properties] : external_incoming_edges) {
        boost::add_edge(src, new_trg, i_edge_properties, graph);
      }
      // Reconstruct outgoing edges only for high-population cities
      for (auto &[trg, _old_src, new_src, o_edge_properties] : external_outgoing_edges) {
        auto city = std::dynamic_pointer_cast<City>(graph[new_src]);
        if (city->population() > 100)
          boost::add_edge(new_src, trg, o_edge_properties, graph);
      }
    };
  }
};

#endif
