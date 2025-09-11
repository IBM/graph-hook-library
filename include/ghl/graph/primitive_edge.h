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
 * @file primitive_edge.h
 *
 * @brief Defines the base class for edge properties in the Generic Graph
 * Library.
 *
 * This file contains the PrimitiveEdge class which serves as the foundation for
 * all edge property classes in the GHL. It extends boost::no_property to
 * maintain compatibility with the Boost Graph Library while adding unique
 * identification capabilities for GHL's extended functionality.
 *
 * The PrimitiveEdge class parallels BaseNode in providing core property
 * management for graph elements, but specifically for edges rather than
 * vertices. It supports optional serialization when the GHL_SERIALIZATION flag
 * is defined.
 */

#ifndef PRIMITIVE_EDGE_H
#define PRIMITIVE_EDGE_H
#include <boost/graph/adjacency_list.hpp>
#include <memory>
#ifdef GHL_SERIALIZATION
#include "../cereal/forward_decl.h"
#endif

namespace ghl {

/**
 * @class PrimitiveEdge
 * @brief Base class for all edge property classes in the GHL.
 *
 * PrimitiveEdge extends boost::no_property to provide:
 * - Unique identification through ID management
 * - Cloning capability for graph operations
 * - Optional serialization support
 *
 * This class is designed to be inherited from when creating custom edge
 * property types. See the Road class in examples/city.h for an example of
 * proper inheritance and extension.
 *
 * @note Unlike BaseNode, PrimitiveEdge inherits from boost::no_property to
 * maintain compatibility with Boost Graph Library's edge property system while
 * adding GHL-specific functionality.
 */
class PrimitiveEdge : public boost::no_property {
  int id_; //< Unique identifier for the edge
#ifdef GHL_SERIALIZATION
  friend class cereal::access;
  template <class Archive> void serialize(Archive &ar);
#endif

public:
  /**
   * @brief Constructs a PrimitiveEdge with the specified ID.
   *
   * @param id The unique identifier for the edge. Defaults to -1 if not
   * specified.
   */
  explicit PrimitiveEdge(int id = -1) : id_(id) {}

  /**
   * @brief Creates a clone of the edge.
   *
   * This virtual method enables polymorphic cloning of edges, which is
   * essential for graph operations that require copying edges, such as subgraph
   * extraction or graph transformation.
   *
   * @return std::shared_ptr<PrimitiveEdge> A smart pointer to a new instance
   * with the same properties.
   */
  virtual std::shared_ptr<PrimitiveEdge> clone() { return std::make_shared<PrimitiveEdge>(id_); }

  /**
   * @brief Virtual destructor to ensure proper cleanup of derived classes.
   */
  virtual ~PrimitiveEdge() = default;

  /**
   * @brief Gets the edge's ID.
   *
   * @return The unique identifier of the edge.
   */
  int id() const { return id_; }

  /**
   * @brief Sets the edge's ID.
   *
   * @param id The new unique identifier for the edge.
   */
  void id(int id) { id_ = id; }

  /**
   * @brief Generates GraphViz attributes for visualization.
   *
   * This virtual method can be overridden by derived classes to customize the
   * appearance of edges in GraphViz output. The base implementation results in
   * no output.
   *
   * @return A map of GraphViz attribute names to values.
   */
  virtual std::map<std::string, std::string, std::less<>> graphAttributes() {
    return std::map<std::string, std::string, std::less<>>();
  }
};

} // namespace ghl
#endif