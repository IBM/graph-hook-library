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
 * @file subgraph.h
 *
 * @brief Provides subgraph functionality for the Generic Graph Library.
 *
 * This file defines the ExtendedSubGraph class which extends GHL's graph
 * functionality to support subgraph operations. It builds upon Boost's subgraph
 * capabilities while adding GHL-specific features such as vertex object ID
 * management and GraphViz attribute handling.
 *
 * The implementation uses Boost's property system to maintain visualization
 * attributes and supports hierarchical graph structures through subgraph
 * relationships.
 */

#ifndef SUBGRAPH_H
#define SUBGRAPH_H
#include "graph.h"
#include <format>
#include <stdexcept>
#include <type_traits>

template <class T> struct is_shared_ptr : std::false_type {};

template <class T> struct is_shared_ptr<std::shared_ptr<T>> : std::true_type {};

namespace b = boost;

namespace ghl {

/**
 * @class ExtendedSubGraph
 * @brief Extends graph functionality with subgraph operations.
 *
 * This class provides support for creating and managing subgraphs within a
 * larger graph structure. It includes functionality for:
 * - Managing vertex and edge properties with GraphViz attributes
 * - Creating and tracking hierarchical subgraph relationships
 * - Maintaining consistent vertex object IDs across the graph hierarchy
 * - Generating visual representations through GraphViz
 *
 * @tparam VertexProperties The type of properties stored in vertices.
 * @tparam EdgeProperties The type of properties stored in edges (defaults to
 * boost::no_property).
 */
template <typename VertexProperties, typename EdgeProperties = b::no_property> class ExtendedSubGraph {
  /** @brief Type alias for GraphViz attribute maps */
  using GraphvizAttributes = std::map<std::string, std::string, std::less<>>;

  /** @brief Type for vertex properties extended with GraphViz attributes */
  using ExtendedVertexProperties = b::property<b::vertex_attribute_t, GraphvizAttributes, VertexProperties>;

  /** @brief Type for edge properties extended with GraphViz attributes and
   * indices */
  using ExtendedEdgeProperties =
      b::property<b::edge_index_t, int, b::property<b::edge_attribute_t, GraphvizAttributes, EdgeProperties>>;

  /**
   * @brief Type definition for the underlying graph structure.
   *
   * Combines the extended property types with Boost's adjacency list and adds
   * graph-level attributes for GraphViz visualization.
   */
  using UnderlyingGraph = b::adjacency_list<
      b::vecS, b::vecS, b::bidirectionalS, ExtendedVertexProperties, ExtendedEdgeProperties,
      b::property<b::graph_name_t, std::string,
                  b::property<b::graph_graph_attribute_t, GraphvizAttributes,
                              b::property<b::graph_vertex_attribute_t, GraphvizAttributes,
                                          b::property<b::graph_edge_attribute_t, GraphvizAttributes>>>>>;

  /** @brief The underlying Boost subgraph instance */
  b::subgraph<UnderlyingGraph> graph_;

  /** @brief Maps subgraphs to their names */
  std::map<std::string, b::subgraph<UnderlyingGraph> *> subgraphs_;

public:
  /** @brief Type alias for the graph implementation */
  using GraphType = b::subgraph<UnderlyingGraph>;

  /** @brief Type alias for vertex descriptors */
  using Vertex = b::graph_traits<GraphType>::vertex_descriptor;

  /** @brief Type alias for edge descriptors */
  using Edge = b::graph_traits<GraphType>::edge_descriptor;

  /**
   * @brief Accesses the underlying graph instance.
   * @return Reference to the underlying graph.
   */
  GraphType &graph() { return graph_; }

  /**
   * @brief Accesses the underlying graph instance (const version).
   * @return Const reference to the underlying graph.
   */
  const GraphType &graph() const { return graph_; }

  /**
   * @brief Creates a vertex in both global and subgraph contexts.
   *
   * Creates a vertex with the specified properties in both the global graph and
   * the provided subgraph. The vertex's GraphViz attributes are initialized
   * based on the vertex properties.
   *
   * @param v Properties to assign to the new vertex
   * @param subgraph The subgraph to add the vertex to
   * @return A pair containing the global and local vertex descriptors
   */
  std::pair<Vertex, Vertex> add_vertex(VertexProperties v, GraphType &subgraph) {
    if constexpr (is_shared_ptr<VertexProperties>::value == true) {
      if (v == nullptr)
        v = std::make_shared<typename VertexProperties::element_type>();
    }
    // Create vertex in global graph
    Vertex v_global = b::add_vertex(graph_);
    graph_[v_global] = v;
    // Update graph attributes. Call changes depending on if VertexProperties
    // is a shared_ptr or a base class
    if constexpr (is_shared_ptr<VertexProperties>::value == true)
      b::put(b::get(b::vertex_attribute, graph_), v_global, v->graphAttributes());
    else
      b::put(b::get(b::vertex_attribute, graph_), v_global, v.graphAttributes());

    // Create corresponding vertex in subgraph if different from global
    Vertex v_local = (&subgraph != &graph_) ? b::add_vertex(v_global, subgraph) : v_global;
    return std::make_pair(v_global, v_local);
  }

  /**
   * @brief Creates a vertex in the global graph.
   *
   * @param v Properties to assign to the new vertex (default constructed if not
   * provided)
   * @return The descriptor of the newly created vertex
   */
  Vertex add_vertex(VertexProperties v = VertexProperties()) { return this->add_vertex(v, graph_).first; }

  /**
   * @brief Creates a vertex with default properties in both contexts.
   *
   * Creates a vertex with default-constructed properties in both the global
   * graph and specified subgraph.
   *
   * @param subgraph The subgraph to add the vertex to
   * @return A pair containing the global and local vertex descriptors
   */
  std::pair<Vertex, Vertex> add_vertex(GraphType &subgraph) { return this->add_vertex(VertexProperties(), subgraph); }

  /**
   * @brief Creates an edge with properties in a specific graph context.
   *
   * Adds an edge between specified vertices with associated properties. The
   * edge is created in the specified graph context and its GraphViz attributes
   * are initialized.
   *
   * @param source Source vertex descriptor
   * @param target Target vertex descriptor
   * @param e Properties to assign to the new edge
   * @param subgraph The graph or subgraph to add the edge to
   * @return A pair containing the edge descriptor and a boolean indicating
   * success
   */
  std::pair<Edge, bool> add_edge(Vertex source, Vertex target, EdgeProperties e, GraphType &subgraph) {
    if constexpr (is_shared_ptr<EdgeProperties>::value == true) {
      if (e == nullptr)
        e = std::make_shared<typename EdgeProperties::element_type>();
    }
    // Attempt to add edge in specified graph
    auto [edge, added] = b::add_edge(source, target, subgraph);
    if (!added)
      return std::make_pair(edge, added);

    // Update graph attributes. Call changes depending on if EdgeProperties
    // is a shared_ptr or a base class
    subgraph[edge] = e;
    if constexpr (is_shared_ptr<EdgeProperties>::value == true)
      b::put(b::get(b::edge_attribute, graph_), edge, e->graphAttributes());
    else
      b::put(b::get(b::edge_attribute, graph_), edge, e.graphAttributes());
    return std::make_pair(edge, added);
  }

  /**
   * @brief Creates an edge with properties in the global graph.
   *
   * @param source Source vertex descriptor
   * @param target Target vertex descriptor
   * @param e Properties to assign to the new edge (default constructed if not
   * provided)
   * @return A pair containing the edge descriptor and a boolean indicating
   * success
   */
  std::pair<Edge, bool> add_edge(Vertex source, Vertex target, EdgeProperties e = EdgeProperties()) {
    return this->add_edge(source, target, e, graph_);
  }

  /**
   * @brief Creates an edge with default properties in a specific graph context.
   *
   * Creates an edge with default-constructed properties between specified
   * vertices in the given graph context.
   *
   * @param source Source vertex descriptor
   * @param target Target vertex descriptor
   * @param subgraph The graph or subgraph to add the edge to
   * @return A pair containing the edge descriptor and a boolean indicating
   * success
   */
  std::pair<Edge, bool> add_edge(Vertex source, Vertex target, GraphType &subgraph) {
    return this->add_edge(source, target, EdgeProperties(), subgraph);
  }

  /**
   * @brief Creates a new subgraph within the current graph.
   *
   * Creates and initializes a new subgraph with the given name, setting up
   * appropriate GraphViz attributes for visualization.
   *
   * @param curr_graph The parent graph to create the subgraph in.
   * @param name The name to assign to the new subgraph.
   * @return Reference to the newly created subgraph.
   */
  GraphType &add_subgraph(GraphType &curr_graph, const std::string &name) {
    GraphType &subgraph = curr_graph.create_subgraph();
    if (subgraphs_.contains(name))
      throw std::out_of_range("Subgraph " + name + " already exists in ExtendedSubgraph");
    subgraphs_[name] = &subgraph;
    b::get_property(subgraph, b::graph_name) = "cluster_" + name;
    b::get_property(subgraph, b::graph_graph_attribute) = {{"label", std::format(" {} ", name)}, {"style", "rounded"}};
    return subgraph;
  }

  GraphType &add_subgraph(const std::string &name) {
    GraphType &subgraph = graph_.create_subgraph();
    subgraphs_[name] = &subgraph;
    b::get_property(subgraph, b::graph_name) = "cluster_" + name;
    b::get_property(subgraph,
                    b::graph_graph_attribute) = {{"label", name}, {"labeljust", "left"}, {"style", "rounded"}};
    return subgraph;
  }

  /**
   * @brief Accesses a subgraph by name.
   *
   * Retrieves a reference to a previously created subgraph using its assigned
   * name. Throws std::out_of_range if the subgraph doesn't exist.
   *
   * @param name The name of the subgraph to retrieve
   * @return Reference to the requested subgraph
   * @throws std::out_of_range if no subgraph exists with the given name
   */
  GraphType &operator[](const std::string &name) { return *(subgraphs_.at(name)); }

  /**
   * @brief Updates GraphViz attributes for all vertices.
   *
   * Refreshes the GraphViz attributes for each vertex in the graph by calling
   * their graphAttributes() method. This is useful after modifying vertex
   * properties that affect visualization.
   */
  void update_graph_attributes() {
    for (Vertex v : b::make_iterator_range(b::vertices(graph_))) {
      b::put(b::get(b::vertex_attribute, graph_), v, graph_[v]->graphAttributes());
    }
    for (Edge e : b::make_iterator_range(b::edges(graph_))) {
      b::put(b::get(b::edge_attribute, graph_), e, graph_[e]->graphAttributes());
    }
  }

  /**
   * @brief Updates vertex object IDs to match their position in the graph.
   *
   * Iterates through all vertices and sets their internal ID to match their
   * vertex descriptor value, ensuring consistent identification across the
   * graph.
   *
   * @pre The graph must not be empty and vertices must have an 'id' member.
   */
  void fix_id_vertices() {
    for (Vertex v : b::make_iterator_range(b::vertices(graph_))) {
      graph_[v]->id(v);
    }
    update_graph_attributes();
  }

  /**
   * @brief Writes the graph to GraphViz format files.
   *
   * Generates both a DOT file and a PDF visualization of the graph using
   * GraphViz.
   *
   * @param filename Base name for the output files (without extension).
   */
  void write_graph(const std::string &filename) {
    std::string filename_dot = filename + ".dot";
    std::ofstream dotFile(filename_dot);
    if (!dotFile)
      throw std::runtime_error(std::format("Could not open {}", filename_dot));
    ;
    b::write_graphviz(dotFile, graph_);
    ghl::convert_dot_to_pdf(filename_dot, filename + ".pdf");
  }
};

} // namespace ghl

#endif