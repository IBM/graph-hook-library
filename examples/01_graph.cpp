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
 * @file 01_graph.cpp
 *
 * @brief Example demonstrating basic usage of the Generic Graph Library (GHL).
 *
 * This example showcases:
 * - Creating and manipulating a simple city/road graph structure
 * - Working with custom vertex (City) and edge (Road) properties
 * - Basic graph traversal and property access
 * - Graph visualization using GraphViz output
 *
 * The example creates a small graph representing cities connected by roads,
 * demonstrates how to access and print vertex/edge properties, and shows
 * how to output the graph structure for visualization.
 */

#include "city.h"
#include "map_graph.h"
#include <ghl/ghl.h>

/**
 * @brief Creates a sample map graph with cities and connecting roads.
 *
 * Constructs a graph with three cities (Chicago, New York, and Buffalo)
 * connected by roads. Each city has properties including name, post office
 * presence, and population. Roads between cities have a property indicating
 * number of lanes.
 *
 * @return MapGraphType A graph containing the cities and their connecting roads
 */
MapGraphType make_map() {
  auto map = MapGraphType();
  // Add vertices (cities) with properties
  auto v1 = map.add_vertex(std::make_shared<City>("Chicago", 1000, true));
  auto v2 = map.add_vertex(std::make_shared<City>("New York", 10000, true));
  auto v3 = map.add_vertex(std::make_shared<City>("Buffalo", 100, false));

  // Add edges (roads) connecting the cities
  map.add_edge(v1, v2,
               std::make_shared<Road>(2)); //< 2-lane road Chicago to NY
  map.add_edge(v2, v3,
               std::make_shared<Road>(4)); //< 4-lane road NY to Buffalo
  map.add_edge(v3, v1,
               std::make_shared<Road>(10)); //< 10-lane road Buffalo to Chicago
  return map;
}

/**
 * @brief Main function demonstrating graph creation and traversal.
 *
 * Creates a map graph using make_map(), then demonstrates:
 * - Iterating through vertices to print city information
 * - Iterating through edges to print road information
 * - Writing the graph to a visualization file
 *
 * @param argc Number of command line arguments
 * @param argv Array of command line argument strings
 * @return int Exit status code
 */
int main(int argc, char *argv[]) {
  MapGraphType map = make_map();
  std::filesystem::create_directories("output");
  // Print information about each city (vertex)
  for (MapGraphType::TemplatedVertex vertex : map.vertex_range()) {
    auto city = std::dynamic_pointer_cast<City>(map[vertex]);
    std::cout << std::format("{} has a population of {} and has a post office: {}", city->name(), city->population(),
                             city->hasPostOffice())
              << std::endl;
  }

  // Print information about each road (edge)
  for (MapGraphType::TemplatedEdge edge : map.edge_range()) {
    auto road = std::dynamic_pointer_cast<Road>(map[edge]);
    auto src_city = std::dynamic_pointer_cast<City>(map[map.edge_source(edge)]);
    auto trg_city = std::dynamic_pointer_cast<City>(map[map.edge_target(edge)]);

    std::cout << std::format("The road between {} and {} has {} lanes", src_city->name(), trg_city->name(),
                             road->numLanes())
              << std::endl;
  }

  // Write the graph to a file for visualization
  map.write_graph("output/map");
  std::cout << std::format("Graph built and stored in {}/output\n", std::filesystem::current_path().string());
  return 0;
}