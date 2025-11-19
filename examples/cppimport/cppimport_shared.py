import os
import pybind11
import importlib
import glob

def default_cfg():
    pkg_name = "graph_hook_library.ghl_bindings"
    mod = importlib.import_module(pkg_name)
    include_dir = None
    if getattr(mod, "__file__", None):
        so_path = os.path.abspath(mod.__file__)
        include_dir = os.path.dirname(so_path)
    else:
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
    return {
        "extra_compile_args": ["-std=c++20"],
        "include_dirs": [
            include_dir,
            pybind11.get_include(),
            pybind11.get_include(user=True),
            os.path.join(include_dir, "include"),
        ],
        "extra_link_args": [so_fullpath],
        "define_macros": [("GHL_BINDINGS", None)]
    }
