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
 * @file graph_bindings.h
 *
 * @brief Provides Python bindings for the Generic Graph Library core
 * functionality.
 *
 * This file implements pybind11-based Python bindings for GHL's core graph
 * types and operations. It enables seamless integration between C++ and Python
 * by exposing graph manipulation functions, property access, and traversal
 * methods.
 *
 * Dependencies:
 * - pybind11 (when GHL_BINDINGS is defined)
 * - Boost Graph Library
 */

#ifndef GRAPH_BINDINGS_H
#define GRAPH_BINDINGS_H

// Try to keep this file out of other h files where possible, as it pulls in the
// pybind11 library.
#include "../graph/graph.h"
#include "../isomorphism/graph_isomorphisms.h"
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>);

namespace py = pybind11;

namespace ghl {

/**
 * @brief Prints the structure of a graph to standard output.
 *
 * Outputs all vertices and edges in the graph for debugging and visualization
 * purposes.
 *
 * @tparam VertexProperties Type of vertex properties
 * @tparam EdgeProperties Type of edge properties
 * @param graph The graph to print
 */
template <typename VertexProperties, typename EdgeProperties = b::no_property>
void print_graph(const Graph<VertexProperties, EdgeProperties> &graph) {
  std::cout << "Vertices:" << std::endl;
  for (Vertex<VertexProperties, EdgeProperties> vertex : b::make_iterator_range(b::vertices(graph))) {
    std::cout << vertex << std::endl;
  }
  std::cout << "Edges:" << std::endl;
  for (Edge<VertexProperties, EdgeProperties> edge : b::make_iterator_range(b::edges(graph))) {
    std::cout << source(edge, graph) << " -> " << target(edge, graph) << std::endl;
  }
}

/**
 * @brief Returns a vector of all vertices in the graph.
 *
 * @tparam VertexProperties Type of vertex properties
 * @tparam EdgeProperties Type of edge properties
 * @param graph The graph to extract vertices from
 * @return Vector containing all vertex descriptors
 */
template <typename VertexProperties, typename EdgeProperties = b::no_property>
std::vector<Vertex<VertexProperties, EdgeProperties>> vertices_(Graph<VertexProperties, EdgeProperties> graph) {
  std::vector<Vertex<VertexProperties, EdgeProperties>> vertex_vector;
  for (Vertex<VertexProperties, EdgeProperties> vertex : b::make_iterator_range(b::vertices(graph))) {
    vertex_vector.push_back(vertex);
  }
  return vertex_vector;
}

/**
 * @brief Returns a vector of all edges in the graph.
 *
 * @tparam VertexProperties Type of vertex properties
 * @tparam EdgeProperties Type of edge properties
 * @param graph The graph to extract edges from
 * @return Vector containing all edge descriptors
 */
template <typename VertexProperties, typename EdgeProperties = b::no_property>
std::vector<Edge<VertexProperties, EdgeProperties>> edges_(Graph<VertexProperties, EdgeProperties> graph) {
  std::vector<Edge<VertexProperties, EdgeProperties>> edge_vector;
  for (Edge<VertexProperties, EdgeProperties> edge : b::make_iterator_range(b::edges(graph))) {
    edge_vector.push_back(edge);
  }
  return edge_vector;
}

/**
 * @brief Returns a vector of all incoming edges for a vertex.
 *
 * @tparam VertexProperties Type of vertex properties
 * @tparam EdgeProperties Type of edge properties
 * @param graph The graph containing the vertex
 * @param vertex The vertex to get incoming edges for
 * @return Vector of incoming edge descriptors
 */
template <typename VertexProperties, typename EdgeProperties = b::no_property>
std::vector<Edge<VertexProperties, EdgeProperties>> in_edges(Graph<VertexProperties, EdgeProperties> graph,
                                                             Vertex<VertexProperties, EdgeProperties> vertex) {
  std::vector<Edge<VertexProperties, EdgeProperties>> edge_vector;
  for (Edge<VertexProperties, EdgeProperties> edge : b::make_iterator_range(b::in_edges(vertex, graph))) {
    edge_vector.push_back(edge);
  }
  return edge_vector;
}

/**
 * @brief Returns a vector of all outgoing edges for a vertex.
 *
 * @tparam VertexProperties Type of vertex properties
 * @tparam EdgeProperties Type of edge properties
 * @param graph The graph containing the vertex
 * @param vertex The vertex to get outgoing edges for
 * @return Vector of outgoing edge descriptors
 */
template <typename VertexProperties, typename EdgeProperties = b::no_property>
std::vector<Edge<VertexProperties, EdgeProperties>> out_edges(Graph<VertexProperties, EdgeProperties> graph,
                                                              Vertex<VertexProperties, EdgeProperties> vertex) {
  std::vector<Edge<VertexProperties, EdgeProperties>> edge_vector;
  for (Edge<VertexProperties, EdgeProperties> edge : b::make_iterator_range(b::out_edges(vertex, graph))) {
    edge_vector.push_back(edge);
  }
  return edge_vector;
}

// Type alias for Boost's edge descriptor implementation
using EdgeDescImpl = b::detail::edge_desc_impl<b::bidirectional_tag, unsigned long>;

/**
 * @brief Declares Python bindings for a graph type.
 *
 * Creates Python bindings for graph operations including vertex/edge
 * manipulation, property access, and traversal methods.
 *
 * @tparam VertexProperties Type of vertex properties
 * @tparam EdgeProperties Type of edge properties
 * @param m The pybind11 module to add bindings to
 * @param pyclass_name Name of the Python class to create
 */
template <typename VertexProperties, typename EdgeProperties = b::no_property>
void declare_graph_bindings(py::module_ &m, const std::string &pyclass_name) {
  using TemplatedGraph = Graph<VertexProperties, EdgeProperties>;
  using TemplatedVertex = Vertex<VertexProperties, EdgeProperties>;
  using TemplatedEdge = Edge<VertexProperties, EdgeProperties>;
  py::class_<TemplatedGraph>(m, pyclass_name.c_str(), py::buffer_protocol(), py::dynamic_attr())
      .def(py::init<>())
      .def("in_edges", &in_edges<VertexProperties, EdgeProperties>)
      .def("out_edges", &out_edges<VertexProperties, EdgeProperties>);
  m.def("add_vertex",
        [](const VertexProperties &p, TemplatedGraph &g) { return (TemplatedVertex)b::add_vertex(p, g); });
  m.def("remove_vertex", [](TemplatedVertex u, TemplatedGraph &g) { return b::remove_vertex(u, g); });
  m.def("add_edge", [](TemplatedVertex u, TemplatedVertex v, const EdgeProperties &p, TemplatedGraph &g) {
    std::pair<TemplatedEdge, bool> result = b::add_edge(u, v, p, g);
    return result.second;
  });
  m.def("remove_edge", [](TemplatedVertex u, TemplatedVertex v, TemplatedGraph &g) { return b::remove_edge(u, v, g); });
  m.def("vertices", &vertices_<VertexProperties, EdgeProperties>);
  m.def("get_vertex", [](TemplatedVertex v, TemplatedGraph &g) { return g[v]; });
  m.def("edges", &edges_<VertexProperties, EdgeProperties>);
  m.def("get_edge", [](TemplatedEdge e, TemplatedGraph &g) { return g[e]; });
  m.def("source", [](TemplatedEdge e, TemplatedGraph &g) { return b::source(e, g); });
  m.def("target", [](TemplatedEdge e, TemplatedGraph &g) { return b::target(e, g); });
  m.def("print_graph", &print_graph<VertexProperties, EdgeProperties>);
}

template <typename VertexProperties, typename EdgeProperties = b::no_property>
void declare_generic_edge_descriptor(py::module_ &m, const std::string &pyclass_name) {
  using TemplatedEdgeDescriptor = generic_edge_descriptor<VertexProperties, EdgeProperties>;
  py::class_<TemplatedEdgeDescriptor>(m, pyclass_name.c_str(), py::buffer_protocol(), py::dynamic_attr())
      .def(py::init<VertexProperties, VertexProperties, EdgeProperties>());
}

/**
 * @brief Declares Python bindings for an isomorphism type.
 *
 * Creates Python bindings for isomorphism initialization and discover function.
 *
 * @tparam VertexProperties Type of vertex properties
 * @tparam EdgeProperties Type of edge properties
 * @param m The pybind11 module to add bindings to
 * @param pyclass_name Name of the Python class to create
 */
template <typename VertexProperties, typename EdgeProperties = b::no_property>
void declare_isomorphism_bindings(py::module_ &m, const std::string &pyclass_name) {
  using Isomorphism = Isomorphism<VertexProperties, EdgeProperties>;
  using GraphType = Isomorphism::GraphType;
  using IsoMap = Isomorphism::IsoMap;
  using IsIsomorphismValidFunction = Isomorphism::IsIsomorphismValidFunction;
  using SpecializeIsoFunction = Isomorphism::SpecializeIsoFunction;

  py::class_<Isomorphism>(m, pyclass_name.c_str(), py::buffer_protocol(), py::dynamic_attr())
      .def(py::init<GraphType>(), py::arg("init_graph"),
           "Construct an Isomorphism instance with the given input graph.")
      .def(
          "discover",
          [](Isomorphism &self, GraphType &sg, const GraphType &g, bool first,
             IsIsomorphismValidFunction is_isomorphism_valid, SpecializeIsoFunction specialize_isomorphism,
             bool nonoverlapping, bool use_vf3) {
            return self.discover(sg, g, first, nonoverlapping, use_vf3, is_isomorphism_valid, specialize_isomorphism);
          },
          py::arg("sg"), py::arg("g"), py::arg("first") = false, py::arg("nonoverlapping") = false,
          py::arg("use_vf3") = true, py::arg("is_isomorphism_valid") = nullptr,
          py::arg("specialize_isomorphism") = nullptr,
          "Discover the isomorphisms in the graph g that match the subgraph sg.");
}

/**
 * @brief Creates Python bindings for the BaseNode class.
 *
 * @param m The pybind11 module to add bindings to
 */
void base_node_bindings(const py::module_ &m);

/**
 * @brief Creates Python bindings for the PrimitiveEdge class.
 *
 * @param m The pybind11 module to add bindings to
 */
void primitive_edge_bindings(const py::module_ &m);

/**
 * @brief Creates Python bindings for core graph functionality.
 *
 * Initializes bindings for base classes and common graph operations.
 *
 * @param m The pybind11 module to add bindings to
 */
void graph_bindings(py::module_ &m);

/**
 * @brief Creates Python bindings for core isomorphism functionality.
 *
 * Initializes bindings for base classes and common isomorphism operations.
 *
 * @param m The pybind11 module to add bindings to
 */
void isomorphism_bindings(py::module_ &m);

} // namespace ghl

#endif
