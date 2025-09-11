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

#include "city.h"
#include "map_graph.h"
#include <ghl/ghl.h>
#include <ghl/ghl_isomorphisms.h>

using ImplementedIsomorphism = ghl::Isomorphism<VertexProperty, EdgeProperty>;

/**
 * @brief Creates a sample map graph of cities and connecting
 * roads.
 *
 * @return MapGraphType A graph containing the cities and their connecting roads
 */
MapGraphType make_map() {
  // Initialize empty map graph
  auto map = MapGraphType();
  std::vector<MapGraphType::TemplatedVertex> city_vertices;

  // Add vertices (cities) with properties
  city_vertices.push_back(map.add_vertex(std::make_shared<City>("A", 6)));
  city_vertices.push_back(map.add_vertex(std::make_shared<City>("B", 7)));
  city_vertices.push_back(map.add_vertex(std::make_shared<City>("C", 6)));
  city_vertices.push_back(map.add_vertex(std::make_shared<City>("D", 12)));
  city_vertices.push_back(map.add_vertex(std::make_shared<City>("E", 5)));
  city_vertices.push_back(map.add_vertex(std::make_shared<City>("F", 9)));

  // Edges connect cities with roads with 2 lanes
  map.add_edge(city_vertices[0], city_vertices[1], std::make_shared<Road>(2));
  map.add_edge(city_vertices[1], city_vertices[0], std::make_shared<Road>(2));
  map.add_edge(city_vertices[0], city_vertices[2], std::make_shared<Road>(2));
  map.add_edge(city_vertices[2], city_vertices[0], std::make_shared<Road>(2));
  map.add_edge(city_vertices[0], city_vertices[3], std::make_shared<Road>(2));
  map.add_edge(city_vertices[3], city_vertices[0], std::make_shared<Road>(2));
  map.add_edge(city_vertices[1], city_vertices[5], std::make_shared<Road>(2));
  map.add_edge(city_vertices[5], city_vertices[1], std::make_shared<Road>(2));
  map.add_edge(city_vertices[1], city_vertices[2], std::make_shared<Road>(2));
  map.add_edge(city_vertices[2], city_vertices[1], std::make_shared<Road>(2));
  map.add_edge(city_vertices[1], city_vertices[3], std::make_shared<Road>(2));
  map.add_edge(city_vertices[3], city_vertices[1], std::make_shared<Road>(2));
  map.add_edge(city_vertices[1], city_vertices[4], std::make_shared<Road>(2));
  map.add_edge(city_vertices[4], city_vertices[1], std::make_shared<Road>(2));
  map.add_edge(city_vertices[2], city_vertices[5], std::make_shared<Road>(2));
  map.add_edge(city_vertices[5], city_vertices[2], std::make_shared<Road>(2));

  return map;
}


/**
 * @brief Creates the pattern graph to match for isomorphism
 * @return A GHL graph with the topoplogy to be matched
 */
ghl::Graph<VertexProperty, EdgeProperty> create_isomorphism_graph() {
  auto isomorphism_graph = ghl::Graph<VertexProperty, EdgeProperty>();
  b::add_vertex(std::make_shared<City>("any", 0, false), isomorphism_graph);
  b::add_vertex(std::make_shared<City>("any", 0, false), isomorphism_graph);
  b::add_vertex(std::make_shared<City>("any", 0, false), isomorphism_graph);
  b::add_edge(0, 1, isomorphism_graph);
  b::add_edge(1, 0, isomorphism_graph);
  b::add_edge(0, 2, isomorphism_graph);
  b::add_edge(2, 0, isomorphism_graph);
  b::add_edge(1, 2, isomorphism_graph);
  b::add_edge(2, 1, isomorphism_graph);
  return isomorphism_graph;
}

/**
 * @class InsertHub
 * @brief Isomorphism class that inserts a hub connection between specific cities
 *
 * This class implements a graph transformation that:
 * - Identifies vertices (cities) with their populations
 * - Removes some roads and creates hub connections between the cities
 * - Preserves the original city properties while modifying the road structure
 */
class InsertHub : public ImplementedIsomorphism {
public:
  /**
   * @brief Constructs the isomorphism with a pattern graph.
   * @param iso_graph Graph pattern to match for transformation
   */
  explicit InsertHub(const MapGraphType::GraphType &iso_graph) : ImplementedIsomorphism(iso_graph) {};

  /**
   * @brief Defines vertex matching criteria for a detected isomorphism
   *
   * Isomorphism should include cities with population less than 8
   * @return Function implementing vertex comparison logic
   */
  VertexCompFunction vertex_comp_function() final {
    return [](const GraphType &iso_graph, const GraphType &target_graph, const TemplatedVertex iso_vertex,
              const TemplatedVertex target_vertex) {
      auto target_city = std::dynamic_pointer_cast<City>(target_graph[target_vertex]);
      return target_city && target_city->population() < 8;
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
   * set of matched cities with a single road
   *
   * @param graph The graph being transformed
   * @param isomorphism Initial isomorphism mapping
   * @return Specialized isomorphism mapping
   */
  IsoMap specialize_isomorphism(const GraphType &graph, const IsoMap &isomorphism) final {
    IsoMap ret_iso = isomorphism;
    TemplatedVertex iso_vertex = isomorphism.size();

    for (int i = 0; i < isomorphism.size(); i++) {
      for (auto edge : b::make_iterator_range(b::in_edges(isomorphism.at(i)[0], graph))) {
        // if the connected city has outdegree of 1, (modeling a single road connection)
        if (b::out_degree(b::source(edge, graph), graph) == 1) {
          ret_iso[iso_vertex].push_back(b::source(edge, graph));
          iso_vertex++;
        }
      }
    }
    return ret_iso;
  }

  /**
   * @brief Validates the isomorphism based on city population.
   *
   * Only allows the hub if the total population of cities is >= 20
   *
   * @param graph The graph being transformed
   * @param isomorphism The isomorphism mapping to validate
   * @return true if the transformation is valid, false otherwise
   */
  bool is_isomorphism_valid(const GraphType &graph, const IsoMap &isomorphism) final {
    std::cout << "Validating isomorphism..." << std::endl;
    if (isomorphism.empty())
      return false;

    int total_population = 0;
    for (auto [iso_vertex, target_vertex_vec] : isomorphism) {
      auto target_vertex = target_vertex_vec[0];
      if (auto city = std::dynamic_pointer_cast<City>(graph[target_vertex]); city)
        total_population += city->population();
    }
    return total_population >= 20;
  }

  /**
   * @brief Constructs the transformed graph structure.
   *
   * Creates a new graph where the matched cities are connected
   * with a HUB instead of pairwise road connections
   * If a matched city does not have a population more than 5, it is not connected to the HUB,
   * its road connections are preserved.
   *
   * @param input_graph Original graph to transform
   * @param isomorphism Isomorphism mapping guiding the transformation
   * @return The transformed graph
   */
  GraphType construct_desired_graph(const GraphType &input_graph, IsoMap &isomorphism) final {
    std::cout << "Constructing desired graph..." << std::endl;
    GraphType ret_graph = input_graph;

    // Creating the hub object
    auto hub = std::make_shared<HUB>();
    auto hub_vertex = b::add_vertex(hub, ret_graph);

    // Connecting matched vertices to the hub with roads with 5 lanes
    for (const auto &[iso_vertex, vertex_vec] : isomorphism) {
      auto city = std::dynamic_pointer_cast<City>(ret_graph[iso_vertex]);
      if (!city) // then it is a hub
        continue;
      b::clear_vertex(iso_vertex, ret_graph);
      if (city->population() <= 5)
        continue;
      auto road = std::make_shared<Road>(5);
      b::add_edge(iso_vertex, hub_vertex, road, ret_graph);
      b::add_edge(hub_vertex, iso_vertex, road, ret_graph);
    }

    // Restoring the road connections of the cities with <= 5 population
    for (const auto &[iso_vertex, vertex_vec] : isomorphism) {
      auto city = std::dynamic_pointer_cast<City>(input_graph[iso_vertex]);
      if (!city) // then it is a hub
        continue;
      if (city->population() <= 5) { // need to restore its edge
        for (TemplatedEdge edge : b::make_iterator_range(b::edges(input_graph))) {
          auto src = b::source(edge, input_graph);
          if (src == iso_vertex) {
            auto trg = b::target(edge, input_graph);
            auto road = std::dynamic_pointer_cast<Road>(input_graph[edge]);
            b::add_edge(src, trg, road, ret_graph);
            b::add_edge(trg, src, road, ret_graph);
            break;
          }
        }
      }
    }

    return ret_graph;
  }

  /**
   * @brief Defines rules for reconstructing external edges after
   * transformation.
   *
   * Controls how edges connecting to non-transformed vertices are handled:
   * - If a non-transformed vertex (city) has population > 10, it is 
   * also connected to the hub and loses its other road connections.
   * - If a non-transformed vertex (city) has population <= 10, it preserves
   * its road connections and is not connected to the hub. 
   *
   * @return Function implementing edge reconstruction logic
   */
  ReconstructEdgesFunction reconstruct_edges_function() final {
    return [](const std::vector<TemplatedEdgeReplacementStruct> &external_incoming_edges,
              const std::vector<TemplatedEdgeReplacementStruct> &external_outgoing_edges,
              const std::vector<TemplatedVertex> &, const std::vector<TemplatedVertex> &, GraphType &graph) {
      std::cout << "Reconstructing edges to the isomorphism..." << std::endl;

      // Detect the hub
      TemplatedVertex hub_vertex;
      for (TemplatedVertex vertex : b::make_iterator_range(b::vertices(graph))) {
        auto hub = std::dynamic_pointer_cast<HUB>(graph[vertex]);
        if (hub) {
          hub_vertex = vertex;
          break;
        }
      }

      std::vector<std::string> connected_hub;

      // Reconstruct incoming edges
      for (auto &[src, _old_trg, new_trg, i_edge_properties] : external_incoming_edges) {
        auto city = std::dynamic_pointer_cast<City>(graph[src]);
        if (!city) // then it is a hub
          continue;
        auto it = std::find(connected_hub.begin(), connected_hub.end(), city->name());
        if (city->population() > 10 && (connected_hub.empty() || it == connected_hub.end())) {
          b::clear_vertex(src, graph);
          auto road = std::make_shared<Road>(5);
          boost::add_edge(src, hub_vertex, road, graph);
          connected_hub.push_back(city->name());
        } else {
          if (city->population() <= 10)
            boost::add_edge(src, new_trg, i_edge_properties, graph);
        }
      }

      connected_hub.clear();
      // Reconstruct outgoing edges
      for (auto &[trg, _old_src, new_src, o_edge_properties] : external_outgoing_edges) {
        auto city = std::dynamic_pointer_cast<City>(graph[trg]);
        if (!city) // then it is a hub
          continue;
        auto it = std::find(connected_hub.begin(), connected_hub.end(), city->name());
        if (city->population() > 10 && (connected_hub.empty() || it == connected_hub.end())) {
          auto road = std::make_shared<Road>(5);
          boost::add_edge(hub_vertex, trg, road, graph);
          connected_hub.push_back(city->name());
        } else {
          if (city->population() <= 10)
            boost::add_edge(new_src, trg, o_edge_properties, graph);
        }
      }
    };
  }
};

/**
 * @brief Main function demonstrating graph transformation. Implements
 * DATE2026 Figure 1 example.
 *
 * Creates an initial graph of interconnected cities, then applies
 * an isomorphism that inserts a hub between certain busy cities.
 *
 * @param argc Number of command line arguments
 * @param argv Array of command line argument strings
 * @return int Exit status code
 */
int main(int argc, char *argv[]) {

  auto isomorphism_graph = create_isomorphism_graph();
  auto isomorphism = std::make_shared<InsertHub>(isomorphism_graph);
  std::cout << "Isomorphism created" << std::endl;

  MapGraphType map = make_map();
  map.write_graph("output/cities_no_hub");
  ghl::apply_isomorphism<VertexProperty, EdgeProperty>(map, isomorphism, true, true);
  map.write_graph("output/cities_inserted_hub");

  return 0;
}