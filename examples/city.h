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
#include <ghl/graph/base_node.h>
#include <ghl/graph/primitive_edge.h>

namespace b = boost;

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
  std::shared_ptr<BaseNode> clone() final {
    return std::make_shared<HUB>();
  }


  /**
   * @brief Set city graph attributes. Necessary for subgraph DOT file
   * generation.
   * @return A map of attribute strings to their values
   */
  std::map<std::string, std::string, std::less<>> graphAttributes() final {
    return std::map<std::string, std::string, std::less<>>{
        {"label", "HUB"}};
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

#endif