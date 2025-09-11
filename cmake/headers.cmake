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

set(GHL_HEADERS_DIR "${CMAKE_CURRENT_LIST_DIR}/../include/ghl")

set(GHL_TOP_HEADERS "${GHL_HEADERS_DIR}/ghl_isomorphisms.h"
                    "${GHL_HEADERS_DIR}/ghl_subgraphs.h"
                    "${GHL_HEADERS_DIR}/ghl.h"
)
set(GHL_BINDINGS_HEADERS "${GHL_HEADERS_DIR}/bindings/graph_bindings.h")
set(GHL_GRAPH_HEADERS "${GHL_HEADERS_DIR}/graph/graph.h"
                            "${GHL_HEADERS_DIR}/graph/base_node.h"
                            "${GHL_HEADERS_DIR}/graph/primitive_edge.h"
                            "${GHL_HEADERS_DIR}/graph/pybind11_fwd_decl.h"
                            "${GHL_HEADERS_DIR}/graph/subgraph.h"
)
set(GHL_ISOMORPHISM_HEADERS "${GHL_HEADERS_DIR}/isomorphism/apply_isomorphism.h"
                            "${GHL_HEADERS_DIR}/isomorphism/graph_isomorphisms.h"
)
set(GHL_ALL_PURE_HEADERS ${GHL_TOP_HEADERS} ${GHL_BINDINGS_HEADERS} ${GHL_GRAPH_HEADERS} ${GHL_ISOMORPHISM_HEADERS})