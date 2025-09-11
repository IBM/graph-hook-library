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
 * @file graph_isomorphisms.h
 *
 * @brief Provides graph isomorphism detection and transformation capabilities.
 *
 * This file defines the core isomorphism framework for GHL, enabling detection
 * of structurally equivalent subgraphs and graph transformations. It builds
 * upon Boost's VF2 subgraph isomorphism algorithm while adding GHL-specific
 * functionality for property-based matching and graph transformations.
 *
 * The implementation supports:
 * - Custom vertex and edge matching criteria
 * - Isomorphism validation and specialization
 * - Graph transformation based on discovered isomorphisms
 * - In-place and constructive graph updates
 */

#ifndef GRAPH_ISOMORPHISMS_H
#define GRAPH_ISOMORPHISMS_H
#include "../graph/graph.h"
#include <unordered_set>
namespace ghl {

/**
 * @class Isomorphism
 * @brief Base class for implementing graph isomorphism operations.
 *
 * This class provides the framework for defining and discovering graph
 * isomorphisms, as well as applying transformations based on discovered
 * isomorphisms. It can be extended to implement specific isomorphism patterns
 * and transformations.
 *
 * @see examples/02_isomorphism.cpp for usage examples.
 *
 * @tparam VertexProperties The type of vertex properties in the graph.
 * @tparam EdgeProperties The type of edge properties in the graph (defaults to
 * boost::no_property).
 */
template <typename VertexProperties, typename EdgeProperties = boost::no_property> class Isomorphism {
public:
  using GraphType = Graph<VertexProperties, EdgeProperties>;
  using TemplatedVertex = Vertex<VertexProperties, EdgeProperties>;
  using TemplatedEdge = Edge<VertexProperties, EdgeProperties>;
  using IsoMap = std::map<TemplatedVertex, std::vector<TemplatedVertex>>;

  /** @brief Function type for specializing discovered isomorphisms */
  using SpecializeIsoFunction = std::function<IsoMap(const GraphType &graph, const IsoMap &isomorphism)>;

  /** @brief Function type for comparing vertices during isomorphism detection
   */
  using VertexCompFunction =
      std::function<bool(const GraphType &G1, const GraphType &G2, const TemplatedVertex v1, const TemplatedVertex v2)>;

  /** @brief Function type for comparing edges during isomorphism detection */
  using EdgeCompFunction =
      std::function<bool(const GraphType &G1, const GraphType &G2, const TemplatedEdge e1, const TemplatedEdge e2)>;

  /** @brief Function type for validating discovered isomorphisms */
  using IsIsomorphismValidFunction = std::function<bool(const GraphType &graph, const IsoMap &isomorphism)>;

  /** @brief Function type for constructing the desired graph after
   * transformation */
  using ConstructDesiredGraphFunction = std::function<GraphType(GraphType input_graph)>;

  using TemplatedEdgeReplacementStruct = EdgeReplacementStruct<VertexProperties, EdgeProperties>;

  /** @brief Optional function type for reconstructing edges after
   * transformation */
  using ReconstructEdgesFunction = std::optional<
      std::function<void(std::vector<TemplatedEdgeReplacementStruct>, std::vector<TemplatedEdgeReplacementStruct>,
                         std::vector<TemplatedVertex>, std::vector<TemplatedVertex>, GraphType &)>>;

  /** @brief Optional function type for in-place graph updates */
  using InPlaceUpdateFunction = std::optional<std::function<void(GraphType &, const IsoMap &)>>;

  // Original graph before any transformations
  GraphType original_input_graph_;
  GraphType input_graph_; //< Current working copy of the graph

  /**
   * @brief Constructs an Isomorphism instance with the given input graph.
   *
   * @param input_graph The graph to search for isomorphisms in.
   */
  explicit Isomorphism(GraphType input_graph) : original_input_graph_(input_graph), input_graph_(input_graph) {}

  virtual ~Isomorphism() = default;

  /**
   * @brief Resets the working graph to the original input graph.
   */
  void reset_input_graph() { input_graph_ = original_input_graph_; }

  /**
   * @brief Helper struct for vertex comparison during isomorphism detection.
   */
  struct vertex_comp_helper {
    const GraphType &G1_;
    const GraphType &G2_;
    VertexCompFunction vcf_;

    vertex_comp_helper(const GraphType &G1, const GraphType &G2, VertexCompFunction vcf)
        : G1_(G1), G2_(G2), vcf_(vcf) {}

    bool operator()(const TemplatedVertex v1, const TemplatedVertex v2) { return vcf_(G1_, G2_, v1, v2); }
  };

  /**
   * @brief Helper struct for edge comparison during isomorphism detection.
   */
  struct edge_comp_helper {
    const GraphType &G1_;
    const GraphType &G2_;
    EdgeCompFunction ecf_;

    edge_comp_helper(const GraphType &G1, const GraphType &G2, EdgeCompFunction ecf) : G1_(G1), G2_(G2), ecf_(ecf) {}

    bool operator()(const TemplatedEdge e1, const TemplatedEdge e2) const { return ecf_(G1_, G2_, e1, e2); }
  };

  /**
   * @brief Callback struct for collecting valid isomorphisms during search.
   */
  struct isomorphisms_callback {
    GraphType &sg_;
    const GraphType &g_;
    std::vector<IsoMap> &valid_isomorphisms_;
    bool discover_first_;
    IsIsomorphismValidFunction is_isomorphism_valid_;
    SpecializeIsoFunction specialize_isomorphism_;
    bool nonoverlapping;
    mutable std::unordered_set<TemplatedVertex> used_vertices_;

    isomorphisms_callback(GraphType &sg, const GraphType &g, std::vector<IsoMap> &iso, bool first,
                          IsIsomorphismValidFunction is_isomorphism_valid, SpecializeIsoFunction specialize_isomorphism,
                          bool nonoverlapping)
        : sg_(sg), g_(g), valid_isomorphisms_(iso), discover_first_(first), is_isomorphism_valid_(is_isomorphism_valid),
          specialize_isomorphism_(specialize_isomorphism), nonoverlapping(nonoverlapping) {}

    /**
     * @brief Processes each discovered isomorphism mapping.
     *
     * @return true to continue search, false to stop.
     */
    template <typename CorrespondenceMap1To2, typename CorrespondenceMap2To1>
    bool operator()(CorrespondenceMap1To2 cm1, CorrespondenceMap2To1) const {
      IsoMap current_mapping;
      std::vector<TemplatedVertex> current_vertices;
      if (nonoverlapping) {
        current_vertices.reserve(boost::num_vertices(sg_));
      }
      for (auto v : boost::make_iterator_range(boost::vertices(sg_))) {
        auto target_vertex = cm1[v];
        current_mapping[v].push_back(target_vertex);
        if (nonoverlapping) {
          if (used_vertices_.find(target_vertex) != used_vertices_.end())
            return true; // Skip this mapping if it overlaps with previous mappings
          current_vertices.push_back(target_vertex);
        }
      }
      if (auto isomorphism_ = specialize_isomorphism_(g_, current_mapping); is_isomorphism_valid_(g_, isomorphism_)) {
        valid_isomorphisms_.push_back(isomorphism_);
        if (nonoverlapping) {
          used_vertices_.insert(current_vertices.begin(), current_vertices.end());
        }
      }
      if (discover_first_ && valid_isomorphisms_.size() == 1)
        return false;
      else
        return true; // Continue the search.
    }
  };

  /**
   * @brief Virtual function defining vertex comparison criteria.
   * @return Function object for comparing vertices.
   */
  virtual VertexCompFunction vertex_comp_function() {
    return [](const GraphType &, const GraphType &, const TemplatedVertex, const TemplatedVertex) { return true; };
  }

  /**
   * @brief Virtual function defining edge comparison criteria.
   * @return Function object for comparing edges.
   */
  virtual EdgeCompFunction edge_comp_function() {
    return [](const GraphType &, const GraphType &, const TemplatedEdge, const TemplatedEdge) { return true; };
  }

  /**
   * @brief Virtual function for validating discovered isomorphisms.
   *
   * @param graph The graph containing the isomorphism.
   * @param isomorphism The discovered isomorphism mapping.
   * @return true if the isomorphism is valid, false otherwise.
   */
  virtual bool is_isomorphism_valid(const GraphType &graph, const IsoMap &isomorphism) { return true; }

  /**
   * @brief Virtual function for specializing discovered isomorphisms.
   *
   * This function can modify or extend the discovered isomorphism mapping
   * before validation.
   *
   * @param graph The graph containing the isomorphism.
   * @param isomorphism The discovered isomorphism mapping.
   * @return The specialized isomorphism mapping.
   */
  virtual IsoMap specialize_isomorphism(const GraphType &graph, const IsoMap &isomorphism) { return isomorphism; }

  /**
   * @brief Virtual function for constructing the transformed graph.
   *
   * @param input_graph The original graph to transform.
   * @param isomorphism The isomorphism mapping guiding the transformation.
   * @return The transformed graph.
   */
  virtual GraphType construct_desired_graph(const GraphType &input_graph, IsoMap &isomorphism) { return input_graph; }

  /**
   * @brief Virtual function providing edge reconstruction rules.
   * @return Optional function for reconstructing edges after transformation.
   */
  virtual ReconstructEdgesFunction reconstruct_edges_function() { return std::nullopt; }

  /**
   * @brief Virtual function providing in-place update rules.
   * @return Optional function for updating the graph in place.
   */
  virtual InPlaceUpdateFunction inplace_update_function() { return std::nullopt; }

  /**
   * @brief Discovers isomorphisms in the graph matching the defined criteria.
   *
   * This function uses Boost to find subgraph isomorphisms, then applies
   * specialization and validation as defined by the derived class.
   *
   * @param sg The subgraph pattern to search for.
   * @param g The graph to search in.
   * @param first If true, stops after finding the first valid isomorphism.
   * @param is_isomorphism_valid_ Optional custom validation function.
   * @param specialize_isomorphism_ Optional custom specialization function.
   * @param nonoverlapping Optional control for discovered isomorphisms.
   * @param use_vf3 Whether to use VF2 or VF3.
   * @return Vector of valid isomorphism mappings.
   */
  virtual std::vector<IsoMap> discover(GraphType &sg, const GraphType &g, bool first, bool nonoverlapping = false,
                                       bool use_vf3 = true, IsIsomorphismValidFunction is_isomorphism_valid_ = nullptr,
                                       SpecializeIsoFunction specialize_isomorphism_ = nullptr) {
    if (!is_isomorphism_valid_) {
      is_isomorphism_valid_ = [this](const GraphType &graph, const IsoMap &isomorphism) {
        return this->is_isomorphism_valid(graph, isomorphism);
      };
    }
    if (!specialize_isomorphism_) {
      specialize_isomorphism_ = [this](const GraphType &graph, const IsoMap &isomorphism) {
        return this->specialize_isomorphism(graph, isomorphism);
      };
    }
    std::vector<IsoMap> isomorphisms;
    isomorphisms_callback callback(sg, g, isomorphisms, first, is_isomorphism_valid_, specialize_isomorphism_,
                                   nonoverlapping);
    if (use_vf3)
      boost::vf3_subgraph_iso(sg, g, callback, vertex_comp_helper(sg, g, vertex_comp_function()),
                              edge_comp_helper(sg, g, edge_comp_function()));
    else
      boost::vf2_subgraph_iso(sg, g, callback, vertex_order_by_mult(sg),
                              boost::edges_equivalent(edge_comp_helper(sg, g, edge_comp_function()))
                                  .vertices_equivalent(vertex_comp_helper(sg, g, vertex_comp_function())));
    return isomorphisms;
  }

  /**
   * @brief Applies an isomorphism mapping to transform a graph.
   *
   * Creates a new graph based on the isomorphism mapping, preserving the
   * necessary vertices and edges while applying any transformations defined
   * by the derived class.
   *
   * @param g The graph to transform.
   * @param isomorphism The isomorphism mapping to apply.
   * @return The transformed graph.
   */
  virtual GraphType apply_isomorphism_to_graph(const GraphType &g, IsoMap isomorphism) {
    GraphType sgc;
    for (auto [sg_v, g_v_vec] : isomorphism) {
      if (g_v_vec.size() != 1) {
        throw std::runtime_error("In what situations in apply_isomorphism_to_graph does a "
                                 "subgraph vertex match to >1 graph vertex?");
      }
      b::add_vertex(g[g_v_vec[0]]->clone(), sgc);
    }
    for (auto [sg_v_src, g_v_vec_src] : isomorphism) {
      for (auto [sg_v_trg, g_v_vec_trg] : isomorphism) {
        if (auto [e, edge_exists] = boost::edge(g_v_vec_src[0], g_v_vec_trg[0], g); edge_exists) {
          b::add_edge(sg_v_src, sg_v_trg, g[e], sgc);
        }
      }
    }
    return sgc;
  }
};

} // namespace ghl
#endif