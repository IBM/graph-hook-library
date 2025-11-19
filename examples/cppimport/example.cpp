// cppimport
/*
<%
import os
import pybind11
import importlib
import glob

pkg_name = "graph_hook_library.ghl_bindings"
mod = importlib.import_module(pkg_name)
include_dir = None
if getattr(mod, "__file__", None):
    so_path = os.path.abspath(mod.__file__)
    include_dir = os.path.dirname(so_path)
else:
    # fallback to __path__ (package dir)
    p = getattr(mod, "__path__", None)
    if p:
        include_dir = os.path.abspath(next(iter(p)))
    else:
        raise RuntimeError(f"Couldn't locate package path for {pkg_name!r}")

so_candidates = []
for entry in os.listdir(include_dir):
    if entry.endswith(".so") or entry.endswith(".pyd") or entry.endswith(".dylib"):
        if "ghl_bindings" in entry or pkg_name.split(".")[-1] in entry:
            so_candidates.append(os.path.join(include_dir, entry))

if not so_candidates:
    so_path = getattr(mod, "__file__", None)
    if so_path:
        so_candidates = [os.path.abspath(so_path)]

if not so_candidates:
    raise RuntimeError("No .so found next to package. Build the extension in-place first.")

so_fullpath = os.path.abspath(so_candidates[0])

print(include_dir)
print(so_fullpath)

cfg["compiler_args"] = ["-std=c++20"]
cfg["include_dirs"] = [
    include_dir,
    pybind11.get_include(),
    pybind11.get_include(user=True),
    "/mnt/graph-hook-library/.py-build-cmake_cache/cp310-cp310-linux_aarch64/editable/graph_hook_library/include",
]
cfg["extra_link_args"] = [so_fullpath]
cfg["define_macros"] = [("GHL_BINDINGS", None)]
%>
*/
#include <ghl/ghl.h>
#include <pybind11/pybind11.h>
#include <iostream>

using TemplatedEdge = std::shared_ptr<ghl::PrimitiveEdge>;
using TemplatedVertex = std::shared_ptr<ghl::BaseNode>;
using GraphType = ghl::Graph<TemplatedVertex, TemplatedEdge>;

int get_magic_value_capi() {
    auto graph = GraphType();
    return 42;
}

namespace py = pybind11;

PYBIND11_MODULE(example, m) {
    py::module_::import("ghl_bindings");
    m.doc() = "Example external GHL extension";
    m.def("get_magic_value", [](){
        return get_magic_value_capi();
    });
}