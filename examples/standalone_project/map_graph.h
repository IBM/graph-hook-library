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
 * @file map_graph.h
 *
 * @brief Defines specialized graph type and visualization settings for map
 * examples.
 *
 * This file provides:
 * - Type definitions for a map-based graph using cities and roads
 * - Specialized vertex property writer for city visualization
 * - Specialized edge property writer for road visualization
 *
 * These specializations customize how cities and roads are displayed in
 * GraphViz output, making the resulting visualizations more informative and
 * domain-specific.
 */

#ifndef EXAMPLES_MAP_GRAPH_H
#define EXAMPLES_MAP_GRAPH_H

#include "city.h"
#include <ghl/ghl.h>
#include <memory>

// Type alias for vertex properties in the map graph
using VertexProperty = std::shared_ptr<ghl::BaseNode>;

// Type alias for edge properties in the map graph
using EdgeProperty = std::shared_ptr<ghl::PrimitiveEdge>;

// Type alias for the specialized map graph implementation
using MapGraphType = ghl::ExtendedGraph<VertexProperty, EdgeProperty>;

/**
 * @brief Writes vertex properties to graph DOT file.
 *
 * This template specialization overrides the default BaseNode
 * vertex_property_writer to provide map-specific vertex visualization. It
 * formats each city's name and population for display in the GraphViz output.
 *
 * @param out Output stream for the DOT file
 * @param v Vertex descriptor whose properties should be written
 */
template <> void MapGraphType::vertex_property_writer(std::ostream &out, const TemplatedVertex &v) const {
  auto city = std::dynamic_pointer_cast<City>(graph()[v]);
  out << std::format("[label=\"City: {}\nPopulation: {}\"]", city->name(), city->population());
};

/**
 * @brief Writes edge properties to graph DOT file.
 *
 * This template specialization overrides the default PrimitiveEdge
 * edge_property_writer to provide map-specific edge visualization. It formats
 * each road's number of lanes for display in the GraphViz output.
 *
 * @param out Output stream for the DOT file
 * @param e Edge descriptor whose properties should be written
 */
template <> void MapGraphType::edge_property_writer(std::ostream &out, const TemplatedEdge &e) const {
  auto road = std::dynamic_pointer_cast<Road>(graph()[e]);
  out << std::format("[label=\"num lanes: {}\"]", road->numLanes());
};

#endif