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
 * @file apply_isomorphism.h
 *
 * @brief Implements functionality for applying graph isomorphisms in the GHL.
 *
 * This file defines the core mechanism for applying discovered isomorphisms to
 * transform graphs. It provides a templated function that handles the process
 * of applying structural and property modifications based on isomorphism
 * mappings, including:
 * - Applying vertex and edge transformations
 * - Managing external edge reconnection
 * - Maintaining property consistency
 * - Supporting both in-place and constructive updates
 *
 * The implementation works in conjunction with the Isomorphism class to enable
 * complete graph transformation workflows.
 */

#ifndef APPLY_ISOMORPHISM_H
#define APPLY_ISOMORPHISM_H

#include "graph_isomorphisms.h"

namespace ghl {

/**
 * @brief Applies a vector of isomorphism transformations to a graph.
 *
 * This function implements the process of applying a graph isomorphism, which
 * can modify both the structure and properties of the graph. It supports both
 * in-place updates and constructive transformations where a new subgraph
 * replaces part of the original graph.
 *
 * The function operates iteratively, implementing the passed vector of previously found
 * isomorphisms. This enables comprehensive graph transformations that may require multiple passes.
 *
 * @tparam VertexProperties The type of properties stored in vertices
 * @tparam EdgeProperties The type of properties stored in edges
 * @param graph The graph to transform
 * @param isomorphism The isomorphism object defining the transformation rules
 * @param isomorphisms The vector or view on the vector of previously found isomorphisms
 */
template <typename VertexProperties, typename EdgeProperties = boost::no_property>
void iterate_isomorphisms(ExtendedGraph<VertexProperties, EdgeProperties> &graph,
                          std::shared_ptr<ghl::Isomorphism<VertexProperties, EdgeProperties>> isomorphism,
                          auto isomorphisms) {
  using TemplatedVertex = Vertex<VertexProperties, EdgeProperties>;
  std::set<TemplatedVertex, std::greater<>> vertices_to_remove;
  for (auto isomorphism_ : isomorphisms) {
    // Check if in-place update is available
    auto inplace_update_function = isomorphism->inplace_update_function();
    if (inplace_update_function.has_value()) {
      // Apply in-place transformation if defined
      (*inplace_update_function)(graph.graph(), isomorphism_);
    } else {
      // Otherwise, perform constructive transformation
      auto desired_graph = isomorphism->apply_isomorphism_to_graph(graph.graph(), isomorphism_);

      // Construct new subgraph based on the transformation rules
      auto constructed_graph = isomorphism->construct_desired_graph(desired_graph, isomorphism_);

      // Replace the affected subgraph with the newly constructed one
      auto new_vertices_to_remove =
          graph.replace_subgraph(constructed_graph, isomorphism_, isomorphism->reconstruct_edges_function());
      vertices_to_remove.insert(new_vertices_to_remove.begin(), new_vertices_to_remove.end());
    }
  }
  for (auto vertex_to_remove : vertices_to_remove) {
    b::clear_vertex(vertex_to_remove, graph.graph());
    b::remove_vertex(vertex_to_remove, graph.graph());
  }
}

/**
 * @brief Finds and applies isomorphisms.
 *
 * This function implements finds isomorphisms and applies to the target graph.
 *
 * @tparam VertexProperties The type of properties stored in vertices
 * @tparam EdgeProperties The type of properties stored in edges
 * @param graph The graph to transform
 * @param isomorphism The isomorphism object defining the transformation rules
 * @param nonoverlapping If true, find isomorphisms one by one
 * @param use_vf3 If true, use the VF3 algorithm. If false, use VF2
 */
template <typename VertexProperties, typename EdgeProperties = boost::no_property>
void apply_isomorphism(ExtendedGraph<VertexProperties, EdgeProperties> &graph,
                       std::shared_ptr<ghl::Isomorphism<VertexProperties, EdgeProperties>> isomorphism,
                       bool nonoverlapping = true, bool use_vf3 = false) {

  using GraphType = Graph<VertexProperties, EdgeProperties>;
  using TemplatedVertex = Vertex<VertexProperties, EdgeProperties>;
  using TemplatedEdge = Edge<VertexProperties, EdgeProperties>;
  using IsoMap = std::map<TemplatedVertex, std::vector<TemplatedVertex>>;
  // Iterate until no more isomorphisms are found
  while (true) {
    isomorphism->reset_input_graph();
    auto &input_graph = isomorphism->input_graph_;

    // Discover all possible isomorphisms in the current graph state
    std::vector<IsoMap> isomorphisms =
        isomorphism->discover(input_graph, graph.graph(), !nonoverlapping, nonoverlapping, use_vf3);

    // If no more isomorphisms are found, transformation is complete
    if (isomorphisms.empty()) {
      break;
    }

    if (nonoverlapping) {
      // Apply all isomorphisms at once
      iterate_isomorphisms(graph, isomorphism, isomorphisms);
      break;
    } else {
      // Else, apply the first isomorphism
      iterate_isomorphisms(graph, isomorphism, std::ranges::subrange(isomorphisms.begin(), isomorphisms.begin() + 1));
    }
  }
  // Finalize vertex IDs
  graph.fix_vertex_obj_ids();
}

} // namespace ghl

#endif