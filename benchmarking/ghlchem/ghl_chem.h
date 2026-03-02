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
 * @file ghl_chem.h
 *
 * @brief Defines lightweight atom/bond helpers and a chemistry-focused graph
 *        for the RDKit benchmark examples.
 */

#ifndef GHL_BENCHMARK_RDKIT_GHL_CHEM_H
#define GHL_BENCHMARK_RDKIT_GHL_CHEM_H

#include <format>
#include <ghl/ghl.h>
#include <ghl/ghl_isomorphisms.h>
#include <map>
#include <memory>
#include <string>

/**
 * @class Atom
 * @brief Vertex property that stores the minimal information we need from an RDKit atom.
 */
class Atom : public ghl::BaseNode {
  std::string symbol_;
  int atomic_number_;
  int formal_charge_;
  bool aromatic_;
  bool matched_;

public:
  explicit Atom(int id, std::string symbol, int atomic_number = 0, int formal_charge = 0, bool aromatic = false)
      : BaseNode(id), symbol_(std::move(symbol)), atomic_number_(atomic_number), formal_charge_(formal_charge),
        aromatic_(aromatic), matched_(false) {}

  std::shared_ptr<BaseNode> clone() final {
    return std::make_shared<Atom>(id(), symbol_, atomic_number_, formal_charge_, aromatic_);
  }

  const std::string &symbol() const { return symbol_; }
  void symbol(const std::string &symbol) { symbol_ = symbol; }
  int atomic_number() const { return atomic_number_; }
  void atomic_number(int atomic_number) { atomic_number_ = atomic_number; }
  int formal_charge() const { return formal_charge_; }
  void formal_charge(int formal_charge) { formal_charge_ = formal_charge; }
  bool aromatic() const { return aromatic_; }
  void aromatic(bool aromatic) { aromatic_ = aromatic; }
  bool matched() const { return matched_; }
  void matched(bool matched) { matched_ = matched; }

  std::map<std::string, std::string, std::less<>> graphAttributes() final {
    return {{"label", std::format("Atom: {}\\nZ: {}\\nCharge: {}", symbol_, atomic_number_, formal_charge_)}};
  }
};

/**
 * @class Bond
 * @brief Edge property that stores the bond order between two atoms.
 */
class Bond : public ghl::PrimitiveEdge {
  int order_;
  int matched_;

public:
  explicit Bond(int order) : order_(order), matched_(false) {}
  int order() const { return order_; }
  bool matched() const { return matched_; }
  void matched(bool matched) { matched_ = matched; }
};

using VertexProperty = std::shared_ptr<ghl::BaseNode>;
using EdgeProperty = std::shared_ptr<ghl::PrimitiveEdge>;

std::string get_bond_color(int order, std::string color) {
  if (order >= 2 && order <= 6) {
    std::string color_string;
    std::string base_string = color + ":invis:";
    color_string.reserve(base_string.size() * order);
    while (order--)
      color_string += base_string;
    return color_string;
  } else if (order >= 7 && order <= 11) {
    std::string color_string;
    std::string base_string = color + ":invis:";
    color_string.reserve(base_string.size() * order);
    while (order-- >= 1)
      color_string += base_string;
    color_string += "gray";
    return color_string;
  } else {
    return color;
  }
}
/**
 * @class ChemGraph
 * @brief Extended graph with DOT helpers tailored for atoms/bonds.
 */
class ChemGraph : public ghl::UndirectedExtendedGraph<VertexProperty, EdgeProperty> {
  using Base = ghl::UndirectedExtendedGraph<VertexProperty, EdgeProperty>;

public:
  void vertex_property_writer(std::ostream &out, const VertexDescriptor &v) const override {
    if (auto atom = std::dynamic_pointer_cast<Atom>(this->graph()[v])) {
      auto border_color = atom->matched() ? "blue" : "black";
      auto border_style = atom->matched() ? "bold" : "";
      out << std::format("[label=\"{}\", xlabel=\"{}\", shape=\"circle\", color=\"{}\", style=\"{}\"]", atom->symbol(),
                         atom->id(), border_color, border_style);
    } else {
      out << "[label=\"Atom\"]";
    }
  }

  void edge_property_writer(std::ostream &out, const EdgeDescriptor &e) const override {
    if (auto bond = std::dynamic_pointer_cast<Bond>(this->graph()[e])) {
      std::string border_color = bond->matched() ? "blue" : "black";
      border_color = get_bond_color(bond->order(), border_color);
      auto border_width_style = bond->matched() ? "bold" : "";
      auto border_line_style = bond->order() == 12 ? "dashed" : "";
      out << std::format("[color=\"{}\", style=\"{}, {}\"]", border_color, border_width_style, border_line_style);
    } else {
      out << "[label=\"bond\"]";
    }
  }
  void graph_properties_writer(std::ostream &out) const override { out << "layout=\"neato\"\n"; }
};

using ImplementedIsomorphism = ghl::UndirectedIsomorphism<VertexProperty, EdgeProperty>;

/**
 * @class SubStruct
 * @brief Minimal isomorphism used to locate matching substructures.
 */
class SubStruct : public ImplementedIsomorphism {
public:
  explicit SubStruct(const ChemGraph::GraphType &pattern) : ImplementedIsomorphism(pattern) {}

  VertexCompFunction vertex_comp_function() final {
    return [](const GraphType &iso_graph, const GraphType &target_graph, const VertexDescriptor iso_vertex,
              const VertexDescriptor target_vertex) {
      auto pattern_atom = std::dynamic_pointer_cast<Atom>(iso_graph[iso_vertex]);
      auto candidate_atom = std::dynamic_pointer_cast<Atom>(target_graph[target_vertex]);
      if (!pattern_atom || !candidate_atom)
        return false;
      return pattern_atom->symbol() == candidate_atom->symbol() &&
             pattern_atom->formal_charge() == candidate_atom->formal_charge() &&
             pattern_atom->aromatic() == candidate_atom->aromatic() && !pattern_atom->matched();
    };
  }

  EdgeCompFunction edge_comp_function() final {
    return [](const GraphType &iso_graph, const GraphType &target_graph, const EdgeDescriptor iso_edge,
              const EdgeDescriptor target_edge) {
      auto pattern_bond = std::dynamic_pointer_cast<Bond>(iso_graph[iso_edge]);
      auto candidate_bond = std::dynamic_pointer_cast<Bond>(target_graph[target_edge]);
      if (!pattern_bond || !candidate_bond)
        return false;
      return pattern_bond->order() == candidate_bond->order();
    };
  }
  InPlaceUpdateFunction inplace_update_function() {
    return [](GraphType &graph, const GraphType &iso_graph, const IsoMap &isomorphism) {
      for (auto [key, iso_vec] : isomorphism) {
        VertexDescriptor vertex = iso_vec[0];
        auto atom = std::dynamic_pointer_cast<Atom>(graph[vertex]);
        atom->matched(true);
        for (auto [key2, iso_vec2] : isomorphism) {
          if (auto [iso_e, exists] = b::edge(key, key2, iso_graph); exists) {
            VertexDescriptor vertex_2 = iso_vec2[0];
            auto [e, _] = b::edge(vertex, vertex_2, graph);
            auto bond = std::dynamic_pointer_cast<Bond>(graph[e]);
            bond->matched(true);
          }
        }
      }
    };
  }
};

#endif
