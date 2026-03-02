// Copyright (c) 2025 IBM
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/**
 * @file cities.h
 *
 * @brief City/road types and graph specialization for the hub insertion example.
 */

#ifndef ICSA_CITIES_H
#define ICSA_CITIES_H

#include <format>
#include <ghl/ghl.h>
#include <ghl/ghl_isomorphisms.h>
#include <memory>
#include <string>

// Vertex and edge property aliases
using VertexProperty = std::shared_ptr<ghl::BaseNode>;
using EdgeProperty = std::shared_ptr<ghl::PrimitiveEdge>;

class City : public ghl::BaseNode {
  std::string name_;
  int population_;
  bool has_post_office_;

public:
  explicit City(const std::string &name, int population = 100, bool has_post_office = true)
      : name_(name), population_(population), has_post_office_(has_post_office) {}

  std::shared_ptr<BaseNode> clone() final { return std::make_shared<City>(name_, population_, has_post_office_); }

  std::string name() const { return name_; }
  int population() const { return population_; }
  bool hasPostOffice() const { return has_post_office_; }

  std::map<std::string, std::string, std::less<>> graphAttributes() final {
    return {{"label", std::format("City: {} Population: {}", name_, population_)}};
  }
};

class HUB : public ghl::BaseNode {
public:
  std::shared_ptr<BaseNode> clone() final { return std::make_shared<HUB>(); }

  std::map<std::string, std::string, std::less<>> graphAttributes() final { return {{"label", "HUB"}}; }
};

class Road : public ghl::PrimitiveEdge {
  int num_lanes_;

public:
  explicit Road(int num_lanes) : num_lanes_(num_lanes) {}
  int numLanes() const { return num_lanes_; }
};

class MapGraph : public ghl::UndirectedExtendedGraph<VertexProperty, EdgeProperty> {
  using Base = ghl::UndirectedExtendedGraph<VertexProperty, EdgeProperty>;

public:
  using Base::Base;

  void vertex_property_writer(std::ostream &out, const VertexDescriptor &v) const override {
    if (auto city = std::dynamic_pointer_cast<City>(this->graph()[v])) {
      out << std::format("[label=\"City: {}\\nPopulation: {}\"]", city->name(), city->population());
    } else {
      out << "[label=\"HUB\", shape=\"diamond\"]";
    }
  }

  void edge_property_writer(std::ostream &out, const EdgeDescriptor &e) const override {
    if (auto road = std::dynamic_pointer_cast<Road>(this->graph()[e])) {
      auto lanes = road->numLanes();
      std::string color = "black";
      if (lanes == 2) {
        color = "orange";
      } else if (lanes == 5) {
        color = "red";
      }
      out << std::format("[color=\"{}\"]", color);
    } else {
      out << "[label=\"road\"]";
    }
  }
  void graph_properties_writer(std::ostream &out) const override { out << "layout=\"fdp\"\n"; }
};

using ImplementedIsomorphism = ghl::UndirectedIsomorphism<VertexProperty, EdgeProperty>;

class InsertHub : public ImplementedIsomorphism {

public:
  explicit InsertHub(const MapGraph::GraphType &iso_graph) : ImplementedIsomorphism(iso_graph) {}

  VertexCompFunction vertex_comp_function() final {
    return [](const GraphType &, const GraphType &target_graph, const VertexDescriptor, const VertexDescriptor target) {
      auto target_city = std::dynamic_pointer_cast<City>(target_graph[target]);
      return target_city && target_city->population() < 8;
    };
  }

  EdgeCompFunction edge_comp_function() final {
    return [](const GraphType &, const GraphType &, const EdgeDescriptor, const EdgeDescriptor) { return true; };
  }

  IsoMap specialize_isomorphism(const GraphType &graph, const IsoMap &isomorphism) final {
    IsoMap ret_iso = isomorphism;
    VertexDescriptor iso_vertex = isomorphism.size();

    for (int i = 0; i < static_cast<int>(isomorphism.size()); i++) {
      for (auto edge : b::make_iterator_range(b::in_edges(isomorphism.at(i)[0], graph))) {
        if (b::out_degree(b::source(edge, graph), graph) == 1) {
          ret_iso[iso_vertex].push_back(b::source(edge, graph));
          iso_vertex++;
        }
      }
    }
    return ret_iso;
  }

  bool is_isomorphism_valid(const GraphType &graph, const IsoMap &isomorphism) final {
    if (isomorphism.empty())
      return false;

    int total_population = 0;
    for (auto [iso_vertex, target_vertex_vec] : isomorphism) {
      auto target_vertex = target_vertex_vec[0];
      if (auto city = std::dynamic_pointer_cast<City>(graph[target_vertex]); city)
        total_population += city->population();
    }
    return total_population >= 20;
  }

  GraphType construct_desired_graph(const GraphType &input_graph, IsoMap &isomorphism) final {
    GraphType ret_graph = input_graph;

    auto hub = std::make_shared<HUB>();
    auto hub_vertex = b::add_vertex(hub, ret_graph);

    for (const auto &[iso_vertex, vertex_vec] : isomorphism) {
      auto city = std::dynamic_pointer_cast<City>(ret_graph[iso_vertex]);
      if (!city)
        continue;
      b::clear_vertex(iso_vertex, ret_graph);
      if (city->population() <= 5)
        continue;
      auto road = std::make_shared<Road>(5);
      b::add_edge(iso_vertex, hub_vertex, road, ret_graph);
    }

    for (const auto &[iso_vertex, vertex_vec] : isomorphism) {
      auto city = std::dynamic_pointer_cast<City>(input_graph[iso_vertex]);
      if (!city)
        continue;
      if (city->population() <= 5) {
        for (EdgeDescriptor edge : b::make_iterator_range(b::edges(input_graph))) {
          auto src = b::source(edge, input_graph);
          if (src == iso_vertex) {
            auto trg = b::target(edge, input_graph);
            auto road = std::dynamic_pointer_cast<Road>(input_graph[edge]);
            b::add_edge(src, trg, road, ret_graph);
            break;
          }
        }
      }
    }

    return ret_graph;
  }

  ReconstructEdgesFunction reconstruct_edges_function() final {
    return [](const std::vector<TemplatedEdgeReplacementStruct> &external_incoming_edges,
              const std::vector<TemplatedEdgeReplacementStruct> &external_outgoing_edges,
              const std::vector<VertexDescriptor> &, const std::vector<VertexDescriptor> &, GraphType &graph) {
      VertexDescriptor hub_vertex{};
      for (VertexDescriptor vertex : b::make_iterator_range(b::vertices(graph))) {
        if (std::dynamic_pointer_cast<HUB>(graph[vertex])) {
          hub_vertex = vertex;
          break;
        }
      }

      std::vector<std::string> connected_hub;
      for (auto &[src, _old_trg, new_trg, i_edge_properties] : external_incoming_edges) {
        auto city = std::dynamic_pointer_cast<City>(graph[src]);
        if (!city)
          continue;
        auto it = std::find(connected_hub.begin(), connected_hub.end(), city->name());
        if (city->population() > 10 && (connected_hub.empty() || it == connected_hub.end())) {
          b::clear_vertex(src, graph);
          auto road = std::make_shared<Road>(5);
          boost::add_edge(src, hub_vertex, road, graph);
          connected_hub.push_back(city->name());
        } else if (city->population() <= 10) {
          boost::add_edge(src, new_trg, i_edge_properties, graph);
        }
      }
    };
  }
};

#endif
