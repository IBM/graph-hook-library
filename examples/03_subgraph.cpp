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
 * @file 03_subgraph.cpp
 *
 * @brief Example demonstrating hierarchical subgraph functionality in GHL.
 *
 * This example showcases:
 * - Creating a hierarchical world map using subgraphs
 * - Managing geographic regions as nested subgraphs
 * - Connecting cities across different geographic regions
 * - Visualizing hierarchical graph structures
 *
 * The example creates a world map with regions (USA, Europe) and subregions
 * (Italy), demonstrating how to organize and connect vertices across different
 * levels of the graph hierarchy.
 */

#include "city.h"
#include "map_graph.h"
#include <ghl/ghl_subgraphs.h>

using MapSubGraphType = ghl::ExtendedSubGraph<VertexProperty, EdgeProperty>;

/**
 * @brief Creates a hierarchical world map using subgraphs.
 *
 * Constructs a graph with three levels of hierarchy:
 * - Global level containing all cities
 * - Region level (USA, Europe) grouping cities by continent
 * - Subregion level (Italy) for more specific geographic grouping
 *
 * Cities are connected both within and across regions to demonstrate
 * cross-hierarchy edge creation.
 *
 * @return MapSubGraphType A hierarchical graph representing the world map
 */
MapSubGraphType make_map() {
  // Initialize the main graph and create regional subgraphs
  auto map = MapSubGraphType();
  MapSubGraphType::GraphType &main_graph = map.graph();
  MapSubGraphType::GraphType &usa = map.add_subgraph(main_graph, "USA");
  MapSubGraphType::GraphType &europe = map.add_subgraph(main_graph, "Europe");
  map.add_subgraph(europe, "Italy");
  MapSubGraphType::GraphType &italy = map["Italy"];

  // Add New York to both global and USA graphs. Vertex initialization optional
  auto ny_global = map.add_vertex();
  auto ny_local = b::add_vertex(ny_global, usa);
  usa[ny_local] = std::make_shared<City>("New York");

  // Necessary to update graph attributes if vertex class changes after vertex
  // instantiation, or stale data will be in the printed graph
  map.update_graph_attributes();

  // Add Paris to both global and Europe graphs
  auto [paris_global, paris_local] = map.add_vertex(std::make_shared<City>("Paris"), europe);

  // Add Italian cities to both global and Italy subgraphs
  auto milan_local = b::add_vertex(map.add_vertex(std::make_shared<City>("Milan")), italy);
  auto rome_local = b::add_vertex(map.add_vertex(std::make_shared<City>("Rome")), italy);

  // Create intercontinental connection (NY to Paris)
  // Vertices may be connected via their global vertices
  map.add_edge(ny_global, paris_global);

  // Create inter-region connection (Paris to Milan). Initialization optional
  // Global vertices may be found from their subgraph
  map.add_edge(europe.local_to_global(paris_local), italy.local_to_global(milan_local),
               std::make_shared<ghl::PrimitiveEdge>(1));

  // Create local connections within Italy. Initialization optional
  map.add_edge(milan_local, rome_local, italy);
  map.add_edge(rome_local, milan_local, std::make_shared<Road>(2), italy);
  return map;
}

/**
 * @brief Main function demonstrating subgraph creation and visualization.
 *
 * Creates a hierarchical world map and outputs it in GraphViz format,
 * demonstrating how subgraphs are visualized as clusters in the resulting
 * graph.
 *
 * @param argc Number of command line arguments
 * @param argv Array of command line argument strings
 * @return int Exit status code
 */
int main(int argc, char *argv[]) {
  MapSubGraphType map = make_map();
  std::filesystem::create_directories("output");
  map.write_graph("output/world_map");
  std::cout << std::format("Subgraph built and stored in {}/output\n", std::filesystem::current_path().string());
  return 0;
}