// cppimport
/*
<%
# Copyright (c) 2025 IBM
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from graph_hook_library import cppimport
cfg.update(cppimport.default_cfg())
cfg["dependencies"] = ["ghl_city.h"]
%>
*/

#include <memory>

#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "ghl_city.h"
#include <ghl/bindings/graph_bindings.h>

namespace py = pybind11;

PYBIND11_MODULE(ghl_city, m) {
  py::module_::import("graph_hook_library.ghl_bindings");
  py::class_<MapGraph, ghl::DirectedExtendedGraph<VertexProperty, EdgeProperty>, std::shared_ptr<MapGraph>>(m,
                                                                                                            "MapGraph")
      .def(py::init<>());
  py::class_<City, ghl::BaseNode, std::shared_ptr<City>>(m, "City")
      .def(py::init<const std::string &, int, bool>(), py::arg("name"), py::arg("population"),
           py::arg("has_post_office"))
      .def("name", &City::name)
      .def("clone", &City::clone);
  py::class_<Road, ghl::PrimitiveEdge, std::shared_ptr<Road>>(m, "Road")
      .def(py::init<int>(), py::arg("num_lanes"))
      .def("numLanes", &Road::numLanes);
  py::class_<Capitals, ImplementedIsomorphism, std::shared_ptr<Capitals>>(m, "Capitals")
      .def(py::init<const MapGraph::GraphType &, std::set<std::string>>(), py::arg("iso_graph"), py::arg("capitals"));
}
