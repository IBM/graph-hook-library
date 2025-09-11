<p align="center">
  <img src="./logo.svg" width="25%">
</p>

<div align="center">
  <h1>Graph Hook Library (GHL)</h1>
</div>
Is a standalone C++ header-only generic graph library built upon the Boost Graph Library.
<p></p>


> The primary feature of the GHL is to provide a robust environment for developing and applying graph transformations or isomorphisms. These isomorphisms can either be contingent on the properties of the graph, such as the edge in or out degree of vertices, or con the subclassed properties of the vertex, such as class type or the value of a class member.

## Authors
The GHL was developed with Corey Lammie, Hadjer Benmeziane, Tunaberk Almaci, and William Simon, as the initial core authors.

## Setup/Installation
The GHL can be setup/installed in two different ways:

#### 1. As a Python-based library:
For sake of convenience, we provide a Python package with bindings, that can be installed using:

`pip install git+https://github.com/IBM/graph-hook-library.git`

It can then be used as follows:

```
from graph_hook_library import ghl_bindings
g = ghl_bindings.Graph()
```

Python-based tests can then be run using `pytest tests/python -s`.

#### 2. As a header-only library:
Alternatively, it can be used as a header-only library. To use it in your project, after including the mandatory dependencies, simply include `#include <ghl/include/ghl.h>`. If you require isomorphism support, include `#include <ghl/include/ghl_isomorphisms.h>`, and if you require subgraph support, include `#include <ghl/include/ghl_subgraphs.h>`.

An example of how to correctly build a C++ project with these dependencies using CMake is provided [here](CMakeLists.txt).

## Optional features
When building a C++ project including the GHL, a number of optional features can be enabled by adding compile defitions:
1. `GHL_SERIALIZATION`: Enables serialization using the [cereal](https://uscilab.github.io/cereal/) library;
2. `GHL_BINDINGS`: Enables python bindings using [pybind11](https://github.com/pybind/pybind11).

To set these directly using CMake, the following commands can be used: `add_compile_definitions(GHL_SERIALIZATION)` and `add_compile_definitions(GHL_BINDINGS)`.

## Documentation
Some documentation for the C++ code is avaliable at [https://ibm.github.io/graph-hook-library/](https://ibm.github.io/graph-hook-library/). We plan to improve this and to add documentation for the Python bindings in due course.

## Isomorphism Framework Overview
The isomorphism framework provides methods for:
  - Providing an initial isomorphism for locating one or all candidate subgraphs,
  - interrogating the vertices and edges of these subgraphs to filter for matching subgraphs,
  - optionally extend the original isomorphism to capture further surrounding edges and vertices,
  - dynamically constructing the replacement subgraph optionally using information from the original subgraph, and inserting said subgraph, and finally,
  - providing logical default as well as customizable edge reattachment rules for connecting the new subgraph to the original graph.

## Step-by-Step Isomorphism Workflow

1. Define Pattern Matching Class
   - Create a class inheriting from `ghl::Isomorphism`
   - Override virtual methods to modify isomorphism behavior. Every method mentioned
   below is overridable.

2. Pattern Recognition
   - Create an initial isomorphism graph representing the pattern to match.
   - Use `vertex_comp_function()` to define vertex matching criteria.
   - Use `edge_comp_function()` to define edge matching criteria.
   - The pattern becomes the template for finding matching subgraphs.

3. Pattern Discovery and augmentation
   - The GHL uses Boost's VF2 algorithm to find initial matching subgraphs
   - Valid matches are further refined using `specialize_isomorphism()` to expand the
   subgraph to surrounding relevant edges and vertices.
   - The refined isomorphism is validated using `is_isomorphism_valid()`

   - Matches can be found until the first valid one or continue for all matches. If matching
   the first pattern, the search will be reinitiated after the replacement, enabling recursive
   matching.

4. Graph Transformation
   - For each valid match, `construct_desired_graph()` builds the replacement structure.
   - The replacement can modify vertices, edges, or both.
   - The replacement can be smaller, larger, or the same size as the matched subgraph.
   - Properties from the original subgraph can be preserved or modified.

5. Edge Reconstruction
   - External edges (connecting the subgraph to the rest of the graph) are identified.
   - Default edge reconstruction connects all incoming edges to root vertices.
   - Default edge reconstruction connects all outgoing edges to leaf vertices.
   - Custom reconstruction can be defined using `reconstruct_edges_function()`

6. Graph Update
   - The matched subgraph is removed from the original graph.
   - The replacement subgraph is inserted.
   - Edges are reconstructed according to the defined rules.
   - The process repeats until no more matches are found.

This process enables powerful graph transformations based on both structural and property-based pattern matching. See `examples/02_isomorphism.cpp` for a complete example implementation.

## Tests
If not using the Python-based library, to build and run the test suite for this project, run the following commands:
```
mkdir build && cd build
cmake .. -DGHL_SERIALIZATION=ON -DGHL_BINDINGS=ON -DCMAKE_INSTALL_PREFIX:PATH=install
cmake --build . --target install
ctest
pytest tests/python -s # If bindings are enabled.
```

## Contributing
Before contributing, please lint code using the following commands (clang-format and black are required):
1. `./run-clang-format.py -r include -i && ./run-clang-format.py -r tests/cpp -i`
2. `black . --check`
