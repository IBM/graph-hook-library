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
 * @tparam ExtendedGraphT Extended graph wrapper type that owns the Boost graph.
 * @param graph The graph to transform
 * @param isomorphism The isomorphism object defining the transformation rules
 * @param isomorphisms The vector or view on the vector of previously found isomorphisms
 */
template <typename ExtendedGraphT>
void iterate_isomorphisms(std::shared_ptr<ExtendedGraphT> graph,
                          std::shared_ptr<ghl::Isomorphism<typename ExtendedGraphT::VertexProperties,
                                                           typename ExtendedGraphT::DirectionProperty,
                                                           typename ExtendedGraphT::EdgeProperties>>
                              isomorphism,
                          auto isomorphisms) {
  using VertexDescriptor = ExtendedGraphT::VertexDescriptor;
  std::set<VertexDescriptor, std::greater<>> vertices_to_remove;
  for (auto isomorphism_ : isomorphisms) {
    // Check if in-place update is available
    auto inplace_update_function = isomorphism->inplace_update_function();
    if (inplace_update_function.has_value()) {
      // Apply in-place transformation if defined
      (*inplace_update_function)(graph->graph(), isomorphism->input_graph_, isomorphism_);
    } else {
      // Otherwise, perform constructive transformation
      auto desired_graph = isomorphism->apply_isomorphism_to_graph(graph->graph(), isomorphism_);

      // Construct new subgraph based on the transformation rules
      auto constructed_graph = isomorphism->construct_desired_graph(desired_graph, isomorphism_);

      // Replace the affected subgraph with the newly constructed one
      auto new_vertices_to_remove =
          graph->replace_subgraph(constructed_graph, isomorphism_, isomorphism->reconstruct_edges_function());
      vertices_to_remove.insert(new_vertices_to_remove.begin(), new_vertices_to_remove.end());
    }
  }
  for (auto vertex_to_remove : vertices_to_remove) {
    b::clear_vertex(vertex_to_remove, graph->graph());
    b::remove_vertex(vertex_to_remove, graph->graph());
  }
}

/**
 * @brief Discover and apply isomorphisms to a graph.
 *
 * Repeatedly discovers subgraph matches in the target graph and applies the
 * provided isomorphism rules until no further matches are found. When
 * @p nonoverlapping is true, all disjoint matches are collected and applied in
 * a single pass; when false, only the first match is applied each iteration,
 * allowing overlapping matches across iterations. All discovered IsoMaps are
 * returned.
 *
 * @tparam ExtendedGraphT The wrapped graph type to mutate.
 * @param graph The graph to transform.
 * @param isomorphism The isomorphism object defining the transformation rules.
 * @param nonoverlapping When true, collect and apply all non-overlapping matches at once.
 * @param use_vf3 If true, use the VF3 algorithm; otherwise use VF2.
 * @return A vector of all IsoMaps discovered during the run.
 */
template <typename ExtendedGraphT>
std::vector<std::map<typename ExtendedGraphT::VertexDescriptor, std::vector<typename ExtendedGraphT::VertexDescriptor>>>
apply_isomorphism(std::shared_ptr<ExtendedGraphT> graph,
                  std::shared_ptr<ghl::Isomorphism<typename ExtendedGraphT::VertexProperties,
                                                   typename ExtendedGraphT::DirectionProperty,
                                                   typename ExtendedGraphT::EdgeProperties>>
                      isomorphism,
                  bool nonoverlapping = true, bool use_vf3 = false) {
  using VertexDescriptor = typename ExtendedGraphT::VertexDescriptor;
  using IsoMap = std::map<VertexDescriptor, std::vector<VertexDescriptor>>;
  std::vector<IsoMap> matched_isos;
  // Iterate until no more isomorphisms are found
  while (true) {
    isomorphism->reset_input_graph();
    auto &input_graph = isomorphism->input_graph_;

    // Discover all possible isomorphisms in the current graph state
    std::vector<IsoMap> isomorphisms =
        isomorphism->discover(input_graph, graph->graph(), !nonoverlapping, nonoverlapping, use_vf3);

    // If no more isomorphisms are found, transformation is complete
    if (isomorphisms.empty()) {
      break;
    }

    if (nonoverlapping) {
      // Apply all isomorphisms at once
      iterate_isomorphisms(graph, isomorphism, isomorphisms);
      matched_isos.insert(matched_isos.end(), isomorphisms.begin(), isomorphisms.end());
      break;
    } else {
      // Else, apply the first isomorphism
      iterate_isomorphisms(graph, isomorphism, std::ranges::subrange(isomorphisms.begin(), isomorphisms.begin() + 1));
      matched_isos.insert(matched_isos.end(), isomorphisms.begin(), isomorphisms.begin() + 1);
    }
  }
  // Finalize vertex IDs
  graph->fix_vertex_obj_ids();
  return matched_isos;
}

} // namespace ghl

#endif
