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
 * @file base_node.h
 *
 * @brief Defines the base class for vertex properties in the Generic Graph
 * Library.
 *
 * This file contains the BaseNode class which serves as the foundation for all
 * vertex property classes in the GHL. It provides basic functionality such as
 * unique identification and GraphViz attribute generation that can be extended
 * by derived classes.
 *
 * The BaseNode class is designed to work seamlessly with the Boost Graph
 * Library while providing additional functionality specific to GHL's needs. It
 * supports optional serialization capabilities when the GHL_SERIALIZATION flag
 * is defined.
 */

#ifndef BASE_NODE_H
#define BASE_NODE_H

#include <format>
#include <map>
#include <memory>
#ifdef GHL_SERIALIZATION
#include "../cereal/forward_decl.h"
#endif
// #ifdef GHL_PYBIND11
#include "pybind11_fwd_decl.h"
// #endif
namespace ghl {

/**
 * @class BaseNode
 * @brief Base class for all vertex property classes in the GHL.
 *
 * BaseNode provides core functionality for vertex properties, including:
 * - Unique identification through ID management
 * - Cloning capability for graph operations
 * - GraphViz attribute generation for visualization
 * - Optional serialization support
 *
 * This class is designed to be inherited from when creating custom vertex
 * property types. See the City class in examples/city.h for an example of
 * proper inheritance and extension.
 */
class PYBIND11_EXPORT BaseNode {
  int id_ = INT_MIN; //< Unique identifier for the node
#ifdef GHL_SERIALIZATION
  friend class cereal::access;
  template <class Archive> void serialize(Archive &ar);
#endif

public:
  /**
   * @brief Constructs a BaseNode with the specified ID.
   *
   * @param id The unique identifier for the node. Defaults to -1 if not
   * specified.
   */
  explicit BaseNode(int id = -1) : id_(id) {}

  /**
   * @brief Creates a clone of the node.
   *
   * This virtual method enables polymorphic cloning of nodes, which is
   * essential for graph operations that require copying vertices.
   *
   * @return std::shared_ptr<BaseNode> A smart pointer to a new instance with
   * the same properties.
   */
  virtual std::shared_ptr<BaseNode> clone() { return std::make_shared<BaseNode>(id_); }

  /**
   * @brief Virtual destructor to ensure proper cleanup of derived classes.
   */
  virtual ~BaseNode() = default;

  /**
   * @brief Gets the node's ID.
   *
   * @return The unique identifier of the node.
   */
  int id() const { return id_; }

  /**
   * @brief Sets the node's ID.
   *
   * @param id The new unique identifier for the node.
   */
  void id(int id) { id_ = id; }

  /**
   * @brief Equality comparison operator.
   *
   * Nodes are considered equal if they have the same ID. This is used in graph
   * algorithms that need to compare vertices.
   *
   * @param other The node to compare with.
   * @return true if the nodes have the same ID, false otherwise.
   */
  bool operator==(const BaseNode &other) const { return (id() == other.id()); }

  /**
   * @brief Generates GraphViz attributes for visualization.
   *
   * This virtual method can be overridden by derived classes to customize the
   * appearance of nodes in GraphViz output. The base implementation provides
   * a simple label with the node's ID.
   *
   * @return A map of GraphViz attribute names to values.
   */
  virtual std::map<std::string, std::string, std::less<>> graphAttributes() {
    return std::map<std::string, std::string, std::less<>>{{"label", std::format("id: {}", id_)}};
  }
};

// Concept for some SFINAE guards, requires the type
// to dereference to a BaseNode or derived class. This
// allows the use of pointers and smart-pointers.
template <typename T>
concept NodeLike = requires(T t) { requires std::derived_from<std::remove_reference_t<decltype(*t)>, BaseNode>; };

} // namespace ghl
#endif