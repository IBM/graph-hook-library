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

#include "graph_bindings.h"

void ghl::base_node_bindings(const py::module_ &m) {
  py::class_<BaseNode, std::shared_ptr<BaseNode>>(m, "BaseNode")
      .def(py::init<int>(), py::arg("id"))
      .def("id", py::overload_cast<int>(&BaseNode::id))
      .def("id", py::overload_cast<>(&BaseNode::id, py::const_));
}

void ghl::primitive_edge_bindings(const py::module_ &m) {
  py::class_<PrimitiveEdge, std::shared_ptr<PrimitiveEdge>>(m, "PrimitiveEdge")
      .def(py::init<int>(), py::arg("id"))
      .def("id", py::overload_cast<int>(&PrimitiveEdge::id))
      .def("id", py::overload_cast<>(&PrimitiveEdge::id, py::const_));
}

void ghl::graph_bindings(py::module_ &m) {
  py::class_<b::no_property>(m, "boost::no_property").def(py::init<>());
  py::class_<EdgeDescImpl>(m, "EdgeDescImpl")
      .def(py::init<>())
      .def_readwrite("m_source", &EdgeDescImpl::m_source)
      .def_readwrite("m_target", &EdgeDescImpl::m_target)
      .def("__repr__", [](const EdgeDescImpl &e) {
        return std::format("<EdgeDescImpl source={} target={}>", std::to_string(e.m_source),
                           std::to_string(e.m_target));
      });
  declare_graph_bindings<std::shared_ptr<ghl::BaseNode>, std::shared_ptr<ghl::PrimitiveEdge>>(m, "Graph");
}

void ghl::isomorphism_bindings(py::module_ &m) {
  declare_isomorphism_bindings<std::shared_ptr<ghl::BaseNode>, std::shared_ptr<ghl::PrimitiveEdge>>(m, "Isomorphism");
}

/**
 * @brief Defines the Python module for GHL bindings.
 *
 * Creates and initializes the Python module containing all GHL functionality.
 *
 * @param m The pybind11 module to initialize
 */
PYBIND11_MODULE(ghl_bindings, m) {
  m.doc() = "Pybind11 bindings that expose the GHL to python.";
  ghl::base_node_bindings(m);
  ghl::primitive_edge_bindings(m);
  ghl::graph_bindings(m);
  ghl::isomorphism_bindings(m);
}