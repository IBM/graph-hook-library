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
#include "../isomorphism/apply_isomorphism.h"
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
 * @tparam GraphT Boost graph type exposing vertex/edge iterators and descriptors.
 * @param graph The graph to print
 */
template <typename GraphT> void print_graph(const GraphT &graph) {
  std::cout << "Vertices:" << std::endl;
  for (auto vertex : b::make_iterator_range(b::vertices(graph))) {
    std::cout << vertex << std::endl;
  }
  std::cout << "Edges:" << std::endl;
  for (auto edge : b::make_iterator_range(b::edges(graph))) {
    std::cout << source(edge, graph) << " -> " << target(edge, graph) << std::endl;
  }
}

// Type alias for Boost's edge descriptor implementation
using DirectedEdgeDescImpl = b::detail::edge_desc_impl<b::bidirectional_tag, unsigned long>;
using UndirectedEdgeDescImpl = b::detail::edge_desc_impl<b::undirected_tag, unsigned long>;

/**
 * @brief Declares Python bindings for a graph type.
 *
 * Creates Python bindings for graph operations including vertex/edge
 * manipulation, property access, and traversal methods.
 *
 * @tparam GraphT Boost graph type to expose to Python.
 * @param m The pybind11 module to add bindings to
 * @param pyclass_name Name of the Python class to create
 */
template <typename GraphT> void declare_graph_bindings(py::module_ &m, const std::string &pyclass_name) {
  using VertexDescriptor = typename GraphT::vertex_descriptor;
  using EdgeDescriptor = typename GraphT::edge_descriptor;
  using VertexProperties = typename GraphT::vertex_property_type;
  using EdgeProperties = typename GraphT::edge_property_type;
  py::class_<GraphT>(m, pyclass_name.c_str(), py::buffer_protocol(), py::dynamic_attr())
      .def(py::init<>())
      .def("in_edges",
           [](GraphT graph, VertexDescriptor vertex) {
             std::vector<EdgeDescriptor> edge_vector;
             for (auto edge : b::make_iterator_range(b::in_edges(vertex, graph))) {
               edge_vector.push_back(edge);
             }
             return edge_vector;
           })
      .def("out_edges", [](GraphT graph, VertexDescriptor vertex) {
        std::vector<EdgeDescriptor> edge_vector;
        for (auto edge : b::make_iterator_range(b::out_edges(vertex, graph))) {
          edge_vector.push_back(edge);
        }
        return edge_vector;
      });
  m.def("add_vertex", [](const VertexProperties &p, GraphT &g) { return (VertexDescriptor)b::add_vertex(p, g); });
  m.def("remove_vertex", [](VertexDescriptor u, GraphT &g) { return b::remove_vertex(u, g); });
  m.def("add_edge", [](VertexDescriptor u, VertexDescriptor v, const EdgeProperties &p, GraphT &g) {
    std::pair<EdgeDescriptor, bool> result = b::add_edge(u, v, p, g);
    return result.second;
  });
  m.def("remove_edge", [](VertexDescriptor u, VertexDescriptor v, GraphT &g) { return b::remove_edge(u, v, g); });
  m.def("vertices", [](GraphT graph) {
    std::vector<VertexDescriptor> vertex_vector;
    for (auto vertex : b::make_iterator_range(b::vertices(graph))) {
      vertex_vector.push_back(vertex);
    }
    return vertex_vector;
  });
  m.def("get_vertex", [](VertexDescriptor v, GraphT &g) { return g[v]; });
  m.def("edges", [](GraphT graph) {
    std::vector<EdgeDescriptor> edge_vector;
    for (auto edge : b::make_iterator_range(b::edges(graph))) {
      edge_vector.push_back(edge);
    }
    return edge_vector;
  });
  m.def("get_edge", [](EdgeDescriptor e, GraphT &g) { return g[e]; });
  m.def("source", [](EdgeDescriptor e, GraphT &g) { return b::source(e, g); });
  m.def("target", [](EdgeDescriptor e, GraphT &g) { return b::target(e, g); });
  m.def("print_graph", &print_graph<GraphT>);
}

/**
 * @brief Declares Python bindings for an ExtendedGraph wrapper.
 *
 * @tparam ExtendedGraphT The extended graph wrapper type.
 * @param m The pybind11 module to add bindings to.
 * @param pyclass_name Name of the Python class to create.
 */
template <typename ExtendedGraphT>
void declare_extended_graph_bindings(py::module_ &m, const std::string &pyclass_name) {
  using EdgeDescriptor = typename ExtendedGraphT::EdgeDescriptor;
  using VertexDescriptor = typename ExtendedGraphT::VertexDescriptor;
  using GraphType = typename ExtendedGraphT::GraphType;
  using VertexProperties = typename GraphType::vertex_property_type;
  using EdgeProperties = typename GraphType::edge_property_type;

  py::class_<ExtendedGraphT, std::shared_ptr<ExtendedGraphT>>(m, pyclass_name.c_str(), py::buffer_protocol(),
                                                              py::dynamic_attr())
      .def(py::init<>())
      .def("add_vertex", &ExtendedGraphT::add_vertex)
      .def("add_edge", &ExtendedGraphT::add_edge)
      .def(
          "__getitem__", [](ExtendedGraphT &self, VertexDescriptor v) -> VertexProperties & { return self.graph()[v]; },
          py::return_value_policy::reference_internal)
      .def(
          "__getitem__", [](ExtendedGraphT &self, EdgeDescriptor e) -> EdgeProperties & { return self.graph()[e]; },
          py::return_value_policy::reference_internal)
      .def(
          "get_vertex", [](ExtendedGraphT &self, VertexDescriptor v) -> VertexProperties & { return self.graph()[v]; },
          py::return_value_policy::reference_internal)
      .def(
          "get_edge", [](ExtendedGraphT &self, EdgeDescriptor e) -> EdgeProperties & { return self.graph()[e]; },
          py::return_value_policy::reference_internal)
      .def("edge_source", &ExtendedGraphT::edge_source)
      .def("edge_target", &ExtendedGraphT::edge_target)
      .def("vertices",
           [](ExtendedGraphT &self) {
             std::vector<VertexDescriptor> verts;
             for (auto v : self.vertex_range()) {
               verts.push_back(v);
             }
             return verts;
           })
      .def("edges",
           [](ExtendedGraphT &self) {
             std::vector<EdgeDescriptor> edges;
             for (auto e : self.edge_range()) {
               edges.push_back(e);
             }
             return edges;
           })
      .def("write_graph", &ExtendedGraphT::write_graph)
      .def("num_vertices", &ExtendedGraphT::num_vertices)
      .def("graph", static_cast<typename ExtendedGraphT::GraphType &(ExtendedGraphT::*)()>(&ExtendedGraphT::graph),
           py::return_value_policy::reference_internal);
}

/**
 * @brief Declares Python bindings for an isomorphism type.
 *
 * Creates Python bindings for isomorphism initialization and discover function.
 *
 * @tparam Isomorphism The isomorphism class to expose.
 * @param m The pybind11 module to add bindings to
 * @param pyclass_name Name of the Python class to create
 */
template <typename Isomorphism> void declare_isomorphism_bindings(py::module_ &m, const std::string &pyclass_name) {
  using GraphType = Isomorphism::GraphType;
  using VertexProperties = Isomorphism::VertexProperties;
  using DirectionProperty = Isomorphism::DirectionProperty;
  using EdgeProperties = Isomorphism::EdgeProperties;
  using IsIsomorphismValidFunction = Isomorphism::IsIsomorphismValidFunction;
  using SpecializeIsoFunction = Isomorphism::SpecializeIsoFunction;
  using GraphWrapper = ExtendedGraph<VertexProperties, DirectionProperty, EdgeProperties>;

  py::class_<Isomorphism, std::shared_ptr<Isomorphism>>(m, pyclass_name.c_str(), py::buffer_protocol(),
                                                        py::dynamic_attr())
      .def(py::init<GraphType>(), py::arg("init_graph"),
           "Construct an Isomorphism instance with the given input graph.")
      .def(
          "discover",
          [](Isomorphism &self, GraphType &sg, const GraphType &g, bool first, bool nonoverlapping, bool use_vf3,
             IsIsomorphismValidFunction is_isomorphism_valid, SpecializeIsoFunction specialize_isomorphism) {
            return self.discover(sg, g, first, nonoverlapping, use_vf3, is_isomorphism_valid, specialize_isomorphism);
          },
          py::arg("sg"), py::arg("g"), py::arg("first") = false, py::arg("nonoverlapping") = false,
          py::arg("use_vf3") = true, py::arg("is_isomorphism_valid") = nullptr,
          py::arg("specialize_isomorphism") = nullptr,
          "Discover the isomorphisms in the graph g that match the subgraph sg.");
  m.def("apply_isomorphism", &ghl::apply_isomorphism<GraphWrapper>);
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
void register_base_types(py::module_ &m);
} // namespace ghl

#endif
