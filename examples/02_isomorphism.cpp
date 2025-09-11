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
 * @file 02_isomorphism.cpp
 *
 * @brief Example demonstrating graph isomorphism and transformation
 * capabilities.
 *
 * This example showcases:
 * - Creating a graph representing Italian cities and their connections
 * - Implementing a custom isomorphism class to transform the graph
 * - Using property-based pattern matching for vertices
 * - Applying graph transformations based on discovered patterns
 * - Reconnecting incoming and outgoing edges dependent on vertex properties
 *
 * The example creates a graph where all roads lead to Rome, then transforms it
 * using an isomorphism that redirects roads away from Rome.
 */

#include "city.h"
#include "map_graph.h"
#include <ghl/ghl.h>
#include <ghl/ghl_isomorphisms.h>

using ImplementedIsomorphism = ghl::Isomorphism<VertexProperty, EdgeProperty>;

/**
 * @brief Creates a sample map graph of Italian cities and their connecting
 * roads.
 *
 * Constructs a graph with several European cities (Rome, Florence, Naples,
 * etc.) connected by roads. Most cities have roads leading to Rome, except
 * Paris and Marseilles which are connected to each other.
 *
 * @return MapGraphType A graph containing the cities and their connecting roads
 */
MapGraphType make_map() {
  // Initialize empty map graph
  auto map = MapGraphType();
  std::vector<MapGraphType::TemplatedVertex> city_vertices;

  // Add vertices (cities) with properties
  city_vertices.push_back(map.add_vertex(std::make_shared<City>("Rome")));
  city_vertices.push_back(map.add_vertex(std::make_shared<City>("Florence")));
  city_vertices.push_back(map.add_vertex(std::make_shared<City>("Naples", 1000)));
  city_vertices.push_back(map.add_vertex(std::make_shared<City>("Venice")));
  city_vertices.push_back(map.add_vertex(std::make_shared<City>("Milan", 1000)));

  city_vertices.push_back(map.add_vertex(std::make_shared<City>("Paris")));
  city_vertices.push_back(map.add_vertex(std::make_shared<City>("Marsailles")));
  city_vertices.push_back(map.add_vertex(std::make_shared<City>("Lyon", 1000)));
  city_vertices.push_back(map.add_vertex(std::make_shared<City>("Stuttgart")));
  city_vertices.push_back(map.add_vertex(std::make_shared<City>("Dijon", 1000)));

  // Add roads - connect all Italian cities to Rome
  for (auto city_vertex : std::ranges::subrange(map.vertex_range().begin() + 1, map.vertex_range().begin() + 5)) {
    map.add_edge(city_vertex, city_vertices[0], std::make_shared<Road>(2));
  }
  // Add roads - connect all French cities to Paris
  for (auto city_vertex : std::ranges::subrange(map.vertex_range().begin() + 6, map.vertex_range().begin() + 10)) {
    map.add_edge(city_vertex, city_vertices[5], std::make_shared<Road>(2));
  }
  // Connect Milan to Lyon
  map.add_edge(city_vertices[4], city_vertices[7], std::make_shared<Road>(2));
  map.add_edge(city_vertices[7], city_vertices[4], std::make_shared<Road>(2));

  return map;
}

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
  explicit Capitals(const MapGraphType::GraphType &iso_graph, std::set<std::string> capitals)
      : ImplementedIsomorphism(iso_graph), capitals_(capitals) {};

  /**
   * @brief Defines vertex matching criteria based on city names.
   * @return Function implementing vertex comparison logic
   */
  VertexCompFunction vertex_comp_function() final {
    return [this](const GraphType &iso_graph, const GraphType &target_graph, const TemplatedVertex iso_vertex,
                  const TemplatedVertex target_vertex) {
      auto target_city = std::dynamic_pointer_cast<City>(target_graph[target_vertex]);
      return target_city && this->capitals_.find(target_city->name()) != this->capitals_.end();
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
    TemplatedVertex iso_vertex = 0;
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
              const std::vector<TemplatedVertex> &, const std::vector<TemplatedVertex> &, GraphType &graph) {
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

/**
 * @brief Main function demonstrating graph transformation.
 *
 * Creates an initial graph where all roads lead to Rome, then applies
 * an isomorphism to transform it into a graph where cities are directly
 * connected to each other instead.
 *
 * @param argc Number of command line arguments
 * @param argv Array of command line argument strings
 * @return int Exit status code
 */
int main(int argc, char *argv[]) {
  // Create initial graph with roads leading to Rome
  MapGraphType map = make_map();
  std::filesystem::create_directories("output");
  map.write_graph("output/original_graph");

  // Create and apply the transformation
  auto isomorphism_graph = ghl::Graph<VertexProperty, EdgeProperty>();
  b::add_vertex(std::make_shared<City>("Capital", false, -1), isomorphism_graph);
  std::set<std::string> capitals = {"Paris", "Rome"};
  auto isomorphism = std::make_shared<Capitals>(isomorphism_graph, capitals);
  ghl::apply_isomorphism<VertexProperty, EdgeProperty>(map, isomorphism, true, false);

  // Output the transformed graph
  map.write_graph("output/transformed_graph");
  std::cout << std::format("Isomorphism run successfully and results stored in {}/output\n",
                           std::filesystem::current_path().string());
  return 0;
}