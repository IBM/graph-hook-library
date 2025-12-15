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
cfg["dependencies"] = ["ghl_chem.h"]
%>
*/

#include <memory>

#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "ghl_chem.h"
#include <ghl/bindings/graph_bindings.h>

namespace py = pybind11;

PYBIND11_MODULE(ghl_chem, m) {
  py::module_::import("graph_hook_library.ghl_bindings");
  py::class_<ChemGraph, ghl::UndirectedExtendedGraph<VertexProperty, EdgeProperty>, std::shared_ptr<ChemGraph>>(
      m, "ChemGraph")
      .def(py::init<>());

  py::class_<Atom, ghl::BaseNode, std::shared_ptr<Atom>>(m, "Atom")
      .def(py::init<int, std::string, int, int, bool>(), py::arg("id"), py::arg("symbol"), py::arg("atomic_number") = 0,
           py::arg("formal_charge") = 0, py::arg("aromatic") = false)
      .def("symbol", py::overload_cast<>(&Atom::symbol, py::const_))
      .def("symbol", py::overload_cast<const std::string &>(&Atom::symbol))
      .def("atomic_number", py::overload_cast<>(&Atom::atomic_number, py::const_))
      .def("atomic_number", py::overload_cast<int>(&Atom::atomic_number))
      .def("formal_charge", py::overload_cast<>(&Atom::formal_charge, py::const_))
      .def("formal_charge", py::overload_cast<int>(&Atom::formal_charge))
      .def("aromatic", py::overload_cast<>(&Atom::aromatic, py::const_))
      .def("aromatic", py::overload_cast<bool>(&Atom::aromatic))
      .def("matched", py::overload_cast<>(&Atom::matched, py::const_))
      .def("matched", py::overload_cast<bool>(&Atom::matched));

  py::class_<Bond, ghl::PrimitiveEdge, std::shared_ptr<Bond>>(m, "Bond")
      .def(py::init<int>(), py::arg("order"))
      .def("order", &Bond::order)
      .def("matched", py::overload_cast<>(&Bond::matched, py::const_))
      .def("matched", py::overload_cast<bool>(&Bond::matched));

  py::class_<SubStruct, ImplementedIsomorphism, std::shared_ptr<SubStruct>>(m, "SubStruct")
      .def(py::init<const ChemGraph::GraphType &>(), py::arg("pattern"));
}
