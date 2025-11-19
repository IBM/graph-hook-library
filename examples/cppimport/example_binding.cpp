//cppimport
/*
<%
import cppimport_shared
cfg.update(cppimport_shared.default_cfg())
%>
*/
#include <ghl/ghl.h>
#include <pybind11/pybind11.h>
#include <iostream>

using TemplatedEdge = std::shared_ptr<ghl::PrimitiveEdge>;
using TemplatedVertex = std::shared_ptr<ghl::BaseNode>;
using GraphType = ghl::ExtendedGraph<TemplatedVertex, TemplatedEdge>;

void empty_graph() {
    auto graph = GraphType();
    graph.print_dfs_graph();
}

namespace py = pybind11;

PYBIND11_MODULE(example_binding, m) {
    py::module_::import("ghl_bindings");
    m.doc() = "Example external GHL extension";
    m.def("empty_graph", [](){
        return empty_graph();
    });
}
