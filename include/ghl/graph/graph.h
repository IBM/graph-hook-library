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
 * @file graph.h
 *
 * @brief Header file defining the primary graph data structures and utilities
 * for the Generic Graph Library (GHL).
 *
 * This file contains core definitions and implementations for managing and
 * manipulating graphs. It utilizes the Boost Graph Library for foundational
 * graph structures and supports extended operations such as vertex duplication,
 * edge updates, and subgraph replacement.
 *
 * Dependencies:
 * - Boost Graph Library (>= 1.86.0)
 */

#ifndef GRAPH_H
#define GRAPH_H

#include "base_node.h"
#include "primitive_edge.h"
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/copy.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/subgraph.hpp>
#include <boost/graph/vf2_sub_graph_iso.hpp>
#include <boost/graph/vf3_sub_graph_iso.hpp>
#include <climits>
#include <filesystem>
#include <map>
#include <ranges>
#include <set>
#include <typeinfo>
#include <unordered_set>

namespace b = boost;

namespace ghl {

/**
 * @typedef ModifyVertexPairFunction
 * @brief Function type for modifying a pair of vertices.
 *
 * @tparam T The type of the vertices.
 */
template <typename T> using ModifyVertexPairFunction = std::function<void(T, T)>;

/**
 * @typedef CloneVertexFunction
 * @brief Function type for cloning a vertex.
 *
 * @tparam T The type of the vertex.
 */
template <typename T> using CloneVertexFunction = std::function<T(T)>;

/**
 * @struct generic_edge_descriptor
 * @brief Descriptor for defining an edge in the graph.
 *
 * @tparam T The type of the vertex identifier.
 * @tparam U The type of the edge properties (default: `boost::no_property`).
 */
template <typename T, typename U = b::no_property> struct generic_edge_descriptor {
  T in_;         //< Source vertex identifier.
  T out_;        //< Target vertex identifier.
  U properties_; //< Properties associated with the edge.
};

/**
 * @struct pair_hash
 * @brief Custom hash function for hashing pairs of objects.
 */
struct pair_hash {
  /**
   * @brief Compute the hash value for a pair.
   *
   * @tparam T1 The type of the first element in the pair.
   * @tparam T2 The type of the second element in the pair.
   * @param pair The pair to hash.
   * @return The computed hash value.
   */
  template <class T1, class T2> std::size_t operator()(const std::pair<T1, T2> &pair) const {
    auto h1 = std::hash<T1>{}(pair.first);
    auto h2 = std::hash<T2>{}(pair.second);
    return h1 ^ h2;
  }
};

// Define graph types.
/**
 * @typedef Graph
 * @brief A bidirectional graph structure with customizable properties.
 *
 * @tparam VertexProperties Properties associated with vertices.
 * This is commonly a class that will be instantiated at each vertex.
 * @tparam EdgeProperties Properties associated with edges (default:
 * `boost::no_property`). This is commonly a class that will be
 * instantiated at each edge.
 * @tparam GraphProperties Properties associated with the entire graph (default:
 * `boost::no_property`).
 */
template <typename VertexProperties, typename EdgeProperties = b::no_property,
          typename GraphProperties = b::no_property>
using Graph = b::adjacency_list<b::vecS, b::vecS, b::bidirectionalS, VertexProperties, EdgeProperties, GraphProperties>;

/**
 * @typedef UndirectedGraph
 * @brief An undirected graph structure with customizable properties.
 *
 * @tparam VertexProperties Properties associated with vertices.
 * This is commonly a class that will be instantiated at each vertex.
 * @tparam EdgeProperties Properties associated with edges (default:
 * `boost::no_property`). This is commonly a class that will be
 * instantiated at each edge.
 * @tparam GraphProperties Properties associated with the entire graph (default:
 * `boost::no_property`).
 */
template <typename VertexProperties, typename EdgeProperties = b::no_property,
          typename GraphProperties = b::no_property>
using UndirectedGraph =
    b::adjacency_list<b::vecS, b::vecS, b::undirectedS, VertexProperties, EdgeProperties, GraphProperties>;

// Define vertex types.
/**
 * @typedef Vertex
 * @brief Descriptor type for vertices in a bidirectional graph.
 *
 * @tparam VertexProperties Properties associated with vertices.
 * This is commonly a class that will be instantiated at each vertex.
 * @tparam EdgeProperties Properties associated with edges (default:
 * `boost::no_property`). This is commonly a class that will be
 * instantiated at each edge.
 */
template <typename VertexProperties, typename EdgeProperties = b::no_property>
using Vertex = typename b::graph_traits<Graph<VertexProperties, EdgeProperties>>::vertex_descriptor;

/**
 * @typedef UVertex
 * @brief Descriptor type for vertices in an undirected graph.
 *
 * @tparam VertexProperties Properties associated with vertices.
 * This is commonly a class that will be instantiated at each vertex.
 * @tparam EdgeProperties Properties associated with edges (default:
 * `boost::no_property`). This is commonly a class that will be
 * instantiated at each edge.
 */
template <typename VertexProperties, typename EdgeProperties = b::no_property>
using UVertex = typename b::graph_traits<UndirectedGraph<VertexProperties, EdgeProperties>>::vertex_descriptor;

// Define edge types.
/**
 * @typedef Edge
 * @brief Descriptor type for edges in a bidirectional graph.
 *
 * @tparam VertexProperties Properties associated with vertices.
 * This is commonly a class that will be instantiated at each vertex.
 * @tparam EdgeProperties Properties associated with edges (default:
 * `boost::no_property`). This is commonly a class that will be
 * instantiated at each edge.
 */
template <typename VertexProperties, typename EdgeProperties = b::no_property>
using Edge = typename b::graph_traits<Graph<VertexProperties, EdgeProperties>>::edge_descriptor;

/**
 * @typedef UEdge
 * @brief Descriptor type for edges in an undirected graph.
 *
 * @tparam VertexProperties Properties associated with vertices.
 * This is commonly a class that will be instantiated at each vertex.
 * @tparam EdgeProperties Properties associated with edges (default:
 * `boost::no_property`). This is commonly a class that will be
 * instantiated at each edge.
 */
template <typename VertexProperties, typename EdgeProperties = b::no_property>
using UEdge = typename b::graph_traits<UndirectedGraph<VertexProperties, EdgeProperties>>::edge_descriptor;

/**
 * @struct EdgeReplacementStruct
 * @brief Represents the replacement of an edge in a graph.
 *
 * @tparam VertexProperties Properties associated with vertices.
 * @tparam EdgeProperties Properties associated with edges (default:
 * `boost::no_property`).
 */
template <typename VertexProperties, typename EdgeProperties = b::no_property> struct EdgeReplacementStruct {
  Vertex<VertexProperties, EdgeProperties> src_trg;    //< The source or target vertex of the edge.
  Vertex<VertexProperties, EdgeProperties> old_vertex; //< The old vertex to be replaced.
  Vertex<VertexProperties, EdgeProperties> new_vertex; //< The new vertex replacing the old one.
  EdgeProperties edge_properties;                      //< Properties of the edge being replaced.
};

/**
 * @brief Constructs a graph from a set of vertices and edges.
 *
 * @tparam T The type of vertex properties.
 * @tparam E The type of edge properties (default: `boost::no_property`).
 * @param vertex_vector A vector of vertices to be added to the graph.
 * @param vertex_eds A vector of edge descriptors defining the edges between
 * vertices.
 * @return A constructed graph of type `Graph<T, E>`.
 */
template <typename T, typename E = b::no_property>
Graph<T, E> construct_graph(std::vector<T> vertex_vector, std::vector<generic_edge_descriptor<int, E>> vertex_eds) {
  Graph<T, E> g;
  std::vector<T> sorted_vertices;
  std::vector<Vertex<T, E>> vertices_;
  int64_t n_vertices = vertex_vector.size();
  int64_t n_edges = vertex_eds.size();
  sorted_vertices.resize(n_vertices);
  for (int64_t i = 0; i < n_vertices; i++) {
    sorted_vertices.at(vertex_vector.at(i)->id()) = vertex_vector.at(i);
  }
  for (int64_t i = 0; i < n_vertices; i++) {
    vertices_.push_back(b::add_vertex(sorted_vertices.at(i), g));
  }
  for (int64_t i = 0; i < n_edges; i++) {
    b::add_edge(vertices_.at(vertex_eds.at(i).in_), vertices_.at(vertex_eds.at(i).out_), vertex_eds.at(i).properties_,
                g);
  }
  return g;
}

/**
 * @brief Deconstructs a graph into its vertices and edge descriptors.
 *
 * @tparam T The type of vertex properties.
 * @tparam E The type of edge properties (default: `boost::no_property`).
 * @param g The graph to be deconstructed.
 * @return A pair containing a vector of vertices and a vector of edge
 * descriptors.
 */
template <typename T, typename E = b::no_property>
std::pair<std::vector<T>, std::vector<generic_edge_descriptor<int, E>>> deconstruct_graph(Graph<T, E> g) {
  std::vector<T> vertex_vector;
  vertex_vector.resize(b::num_vertices(g));
  std::vector<generic_edge_descriptor<int, E>> vertex_eds;
  for (Vertex<T, E> vertex : b::make_iterator_range(b::vertices(g))) {
    vertex_vector.at(g[vertex]->id()) = g[vertex];
  }
  for (Edge<T, E> edge : b::make_iterator_range(b::edges(g))) {
    vertex_eds.push_back(generic_edge_descriptor<int, E>(g[source(edge, g)]->id(), g[target(edge, g)]->id(), g[edge]));
  }
  return std::make_pair(vertex_vector, vertex_eds);
}

/**
 * @brief Converts a DOT file to a PDF using the Graphviz `dot` tool.
 *
 * @param dot_filename The path to the input DOT file.
 * @param pdf_filename The path to the output PDF file.
 */
inline void convert_dot_to_pdf(const std::string &dot_filename, const std::string &pdf_filename) {
  std::string command = "dot -Tpdf " + dot_filename + " -o " + pdf_filename;
  std::system(command.c_str());
}

/**
 * @brief Updates the source vertex of an edge in a graph.
 *
 * @tparam V The type of the vertex descriptor.
 * @tparam G The type of the graph.
 * @param source The current source vertex of the edge.
 * @param new_source The new source vertex for the edge.
 * @param target The target vertex of the edge.
 * @param g The graph where the edge resides.
 */
template <typename V, typename G> void update_edge_source(V source, V new_source, V target, G &g) {
  if (auto [e, exists] = b::edge(source, target, g); exists) {
    auto prop = std::move(g[e]);
    b::remove_edge(e, g);
    b::add_edge(new_source, target, prop, g);
  } else {
    throw std::runtime_error("Edge not found in update_edge_resource");
  }
}

/**
 * @brief Updates the target vertex of an edge in a graph.
 *
 * @tparam V The type of the vertex descriptor.
 * @tparam G The type of the graph.
 * @param source The source vertex of the edge.
 * @param target The current target vertex of the edge.
 * @param new_target The new target vertex for the edge.
 * @param g The graph where the edge resides.
 */
template <typename V, typename G> void update_edge_target(V source, V target, V new_target, G &g) {
  if (auto [e, exists] = b::edge(source, target, g); exists) {
    auto prop = std::move(g[e]);
    b::remove_edge(e, g);
    b::add_edge(source, new_target, prop, g);
  } else {
    throw std::runtime_error("Edge not found in update_edge_resource");
  }
}

/**
 * @brief A wrapper class around a Boost graph providing extended functionality.
 *
 * @tparam VertexProperties Properties associated with vertices.
 * @tparam EdgeProperties Properties associated with edges (default:
 * `boost::no_property`).
 */
template <typename VertexProperties, typename EdgeProperties = b::no_property> class ExtendedGraph {
protected:
  Graph<VertexProperties, EdgeProperties> graph_{}; //< Internal Boost graph instance.

public:
  using GraphType = Graph<VertexProperties, EdgeProperties>; //< Alias for the graph type.
  using TemplatedVertex = Vertex<VertexProperties,
                                 EdgeProperties>; //< Alias for vertex descriptor type.
  using TemplatedEdge = Edge<VertexProperties,
                             EdgeProperties>; //< Alias for edge descriptor type.
  using IsoMap = std::map<TemplatedVertex,
                          std::vector<TemplatedVertex>>; //< Alias for isomorphism map
                                                         //< type.
  using TemplatedEdgeReplacementStruct = EdgeReplacementStruct<VertexProperties,
                                                               EdgeProperties>; //< Alias for edge replacement
                                                                                //< structure.

  /**
   * @brief Default constructor.
   */
  ExtendedGraph() = default;

  /**
   * @brief Constructor initializing the graph with a given Boost graph
   * instance.
   *
   * @param graph The Boost graph instance to initialize with.
   */
  explicit ExtendedGraph(GraphType graph) : graph_(graph) {}

  /**
   * @brief Virtual destructor.
   */
  virtual ~ExtendedGraph() = default;

  /**
   * @brief Accessor for the internal graph.
   *
   * @return A reference to the internal graph instance.
   */
  GraphType &graph() { return graph_; }

  /**
   * @brief Const accessor for the internal graph.
   *
   * @return A reference to the internal graph instance.
   */
  const GraphType &graph() const { return graph_; }

  /**
   * @brief Adds a vertex to the graph.
   *
   * @param v The properties of the vertex to add (default constructed if not
   * provided).
   * @return The descriptor of the newly added vertex.
   */
  TemplatedVertex add_vertex(VertexProperties v = VertexProperties()) { return b::add_vertex(v, graph_); }

  /**
   * @brief Adds an edge between two vertices in the graph.
   *
   * @param src The source vertex of the edge.
   * @param trg The target vertex of the edge.
   * @param e The properties of the edge to add (default constructed if not
   * provided).
   * @return The descriptor of the newly added edge.
   */
  TemplatedEdge add_edge(TemplatedVertex src, TemplatedVertex trg, EdgeProperties e = EdgeProperties()) {
    auto [edge, added] = b::add_edge(src, trg, e, graph_);
    return edge;
  }

  /**
   * @brief Retrieves the source vertex of a given edge.
   *
   * @param e The edge descriptor.
   * @return The descriptor of the source vertex.
   */
  TemplatedVertex edge_source(TemplatedEdge e) const { return b::source(e, graph_); }

  /**
   * @brief Retrieves the target vertex of a given edge.
   *
   * @param e The edge descriptor.
   * @return The descriptor of the target vertex.
   */
  TemplatedVertex edge_target(TemplatedEdge e) const { return b::target(e, graph_); }

  /**
   * @brief Provides a range of all vertices in the graph.
   *
   * @return An iterator range of all vertices in the graph.
   */
  inline b::iterator_range<typename GraphType::vertex_iterator> vertex_range() const {
    return b::make_iterator_range(b::vertices(graph_));
  }

  /**
   * @brief Provides a range of all edges in the graph.
   *
   * @return An iterator range of all edges in the graph.
   */
  inline b::iterator_range<typename GraphType::edge_iterator> edge_range() const {
    return b::make_iterator_range(b::edges(graph_));
  }

  /**
   * @brief Accesses the properties of a vertex.
   *
   * @param v The vertex descriptor.
   * @return A reference to the properties of the specified vertex.
   */
  VertexProperties &operator[](const TemplatedVertex &v) { return graph_[v]; }

  /**
   * @brief Accesses the properties of a vertex (const).
   *
   * @param v The vertex descriptor.
   * @return A reference to the properties of the specified vertex.
   */
  const VertexProperties &operator[](const TemplatedVertex &v) const { return graph_[v]; }

  /**
   * @brief Accesses the properties of an edge.
   *
   * @param e The edge descriptor.
   * @return A reference to the properties of the specified edge.
   */
  EdgeProperties &operator[](const TemplatedEdge &e) { return graph_[e]; }

  /**
   * @brief Accesses the properties of an edge (const).
   *
   * @param e The edge descriptor.
   * @return A reference to the properties of the specified edge.
   */
  const EdgeProperties &operator[](const TemplatedEdge &e) const { return graph_[e]; }

  /**
   * @brief Checks if a vertex exists in the isomorphism map.
   *
   * @param vertex The vertex to check.
   * @param isomorphism The isomorphism map containing vertices.
   * @return True if the vertex exists in the isomorphism map, false otherwise.
   */
  bool is_vertex_in_isomorphism(const TemplatedVertex &vertex, const IsoMap &isomorphism) const {
    return std::ranges::any_of(isomorphism,
                               [&vertex](const std::pair<TemplatedVertex, std::vector<TemplatedVertex>> &iso) {
                                 return std::ranges::find(iso.second, vertex) != iso.second.end();
                               });
  }

  /**
   * @brief Performs a depth-first search (DFS) on the graph and prints events.
   *
   * This function uses a custom DFS visitor to print initialization, start,
   * discovery, and finish events for each vertex during the traversal.
   */
  void print_dfs_graph() const {
    struct custom_dfs_visitor : b::default_dfs_visitor {
      /**
       * @brief Logs when a vertex is initialized.
       * @param s The vertex being initialized.
       * @param g The graph being traversed.
       */
      void initialize_vertex(TemplatedVertex s, GraphType const &g) const {
        std::cout << "Initialize: " << g[s]->id() << std::endl;
      }

      /**
       * @brief Logs when a vertex is the start vertex.
       * @param s The vertex being started.
       * @param g The graph being traversed.
       */
      void start_vertex(TemplatedVertex s, GraphType const &g) const {
        std::cout << "Start:      " << g[s]->id() << std::endl;
      }

      /**
       * @brief Logs when a vertex is discovered.
       * @param s The vertex being discovered.
       * @param g The graph being traversed.
       */
      void discover_vertex(TemplatedVertex s, GraphType const &g) const {
        std::cout << "Discover:   " << g[s]->id() << std::endl;
      }

      /**
       * @brief Logs when a vertex is finished.
       * @param s The vertex being finished.
       * @param g The graph being traversed.
       */
      void finish_vertex(TemplatedVertex s, GraphType const &g) const {
        std::cout << "Finished:   " << g[s]->id() << std::endl;
      }
    };

    custom_dfs_visitor vis;
    b::depth_first_search(graph_, visitor(vis));
  }

  /**
   * @brief Writes the graph to a DOT file and optionally renders it as a PDF.
   * This is not const as instances of this class keep an internal counter of
   * the emitted graphs, and prefix it to the file name so that file names
   * reflect the ordering of calls to this method.
   *
   * @param filename The base filename (without extension) for the output files.
   * @param render_pdf If true, renders the DOT file to a PDF.
   */
  int curr_graph = 0;

  void write_graph(const std::string &filename, bool render_pdf = true) {
    std::filesystem::path p(filename);
    std::string base_path = p.parent_path().string();
    std::string file_name = p.filename().string();
    std::string indexed_filename = std::format("{}/{}_{}", base_path, curr_graph++, file_name);

    std::ofstream dotFile(indexed_filename + ".dot");
    auto vertexWriter = [this](std::ostream &out, const auto &v) { vertex_property_writer(out, v); };
    auto edgeWriter = [this](std::ostream &out, const auto &e) { edge_property_writer(out, e); };
    b::write_graphviz(dotFile, graph_, vertexWriter, edgeWriter);
    if (render_pdf) {
      convert_dot_to_pdf(indexed_filename + ".dot", indexed_filename + ".pdf");
    }
  };

  /**
   * @brief Duplicates a vertex in the graph, along with its edges.
   *
   * @param id_to_duplicate The ID of the vertex to duplicate.
   * @param cvf A function to clone the properties of the vertex.
   * @param mvpf A function to modify the pair of original and duplicated
   * vertices.
   * @return The descriptor of the newly duplicated vertex.
   */
  TemplatedVertex duplicate_vertex(int id_to_duplicate, CloneVertexFunction<VertexProperties> cvf,
                                   ModifyVertexPairFunction<VertexProperties> mvpf) {
    TemplatedVertex vertex_to_duplicate;
    bool found = false;
    // Search for the vertex with the given ID.
    for (TemplatedVertex v : b::make_iterator_range(b::vertices(graph_))) {
      if (graph_[v]->id() == id_to_duplicate) {
        vertex_to_duplicate = v;
        found = true;
        break;
      }
    }
    if (!found) {
      throw std::runtime_error("Vertex with the specified ID does not exist.");
    }
    VertexProperties duplicated_vertex = cvf(graph_[vertex_to_duplicate]);
    // Create a new vertex with the same properties.
    TemplatedVertex new_vertex = b::add_vertex(duplicated_vertex, graph_);
    // Duplicate the out-edges.
    for (TemplatedEdge edge : b::make_iterator_range(b::out_edges(vertex_to_duplicate, graph_))) {
      TemplatedVertex vtarget = b::target(edge, graph_);
      auto edge_pair = b::edge(vertex_to_duplicate, vtarget, graph_);
      assert(edge_pair.second);
      b::add_edge(new_vertex, vtarget, graph_[edge_pair.first], graph_);
    }
    // Duplicate the in-edges.
    for (TemplatedEdge edge : b::make_iterator_range(b::in_edges(vertex_to_duplicate, graph_))) {
      TemplatedVertex vsource = b::source(edge, graph_);
      auto edge_pair = b::edge(vsource, vertex_to_duplicate, graph_);
      assert(edge_pair.second);
      b::add_edge(vsource, new_vertex, graph_[edge_pair.first], graph_);
    }
    mvpf(graph_[vertex_to_duplicate], duplicated_vertex);
    return new_vertex;
  }

  /**
   * @brief Adds a vertex after a specified vertex and adjusts edges
   * accordingly.
   *
   * @param v The vertex after which the new vertex will be added.
   * @param new_vertex_properties The properties of the new vertex to add.
   */
  void add_vertex_after_vertex(TemplatedVertex v, const VertexProperties &new_vertex_properties) {
    // Add the new vertex with the provided properties.
    TemplatedVertex new_vertex = b::add_vertex(new_vertex_properties, graph_);
    // Transfer outgoing edges from 'v' to 'new_vertex'.
    std::vector<TemplatedVertex> out_vertices;
    for (TemplatedEdge edge : b::make_iterator_range(b::out_edges(v, graph_))) {
      TemplatedVertex target = b::target(edge, graph_);
      out_vertices.push_back(target);
    }
    // Remove old edges and add new edges.
    for (TemplatedVertex out_v : out_vertices) {
      b::remove_edge(v, out_v, graph_);
      b::add_edge(new_vertex, out_v, graph_);
    }
    // Add edge from 'v' to 'new_vertex'.
    b::add_edge(v, new_vertex, graph_);
  }

  /**
   * @brief Adds a vertex after vertices that satisfy a given predicate.
   *
   * @tparam Predicate A callable that takes vertex properties and returns true
   * if the vertex matches the criteria.
   * @param pred The predicate function to filter vertices.
   * @param new_vertex_properties The properties of the new vertex to add.
   */
  template <typename Predicate>
  void add_vertex_after_pred(Predicate pred, const VertexProperties &new_vertex_properties) {
    std::vector<TemplatedVertex> vertices_add_after;
    for (TemplatedVertex v : b::make_iterator_range(b::vertices(graph_))) {
      if (pred(graph_[v])) {
        vertices_add_after.push_back(v);
      }
    }
    for (TemplatedVertex v : vertices_add_after) {
      add_vertex_after_vertex(v, new_vertex_properties->clone());
    }
  }

  template <typename Predicate> void remove_vertex_pred(Predicate pred) {
    std::vector<TemplatedVertex> vertices_to_remove;
    for (TemplatedVertex v : b::make_iterator_range(b::vertices(graph_))) {
      if (pred(graph_[v])) {
        vertices_to_remove.push_back(v);
      }
    }
    for (TemplatedVertex v : vertices_to_remove) {
      std::vector<TemplatedVertex> in_vertices;
      std::vector<TemplatedVertex> out_vertices;
      // Collecting in-vertices and out-vertices.
      for (TemplatedEdge edge : b::make_iterator_range(b::in_edges(v, graph_))) {
        in_vertices.push_back(source(edge, graph_));
      }
      for (TemplatedEdge edge : b::make_iterator_range(b::out_edges(v, graph_))) {
        out_vertices.push_back(target(edge, graph_));
      }
      // Redirecting in-edges of v to each of its out-vertices.
      for (TemplatedVertex in_v : in_vertices) {
        for (TemplatedVertex out_v : out_vertices) {
          if (in_v != out_v) { // Avoid self-loops.
            b::add_edge(in_v, out_v, graph_);
          }
        }
      }
    }
    // Sort vertices in descending order to maintain valid descriptors.
    std::ranges::sort(vertices_to_remove, std::greater<TemplatedVertex>());
    for (TemplatedVertex v : vertices_to_remove) {
      clear_vertex(v, graph_);
      remove_vertex(v, graph_);
    }
  }

  void add_vertex_before_vertex(TemplatedVertex v, const VertexProperties &new_vertex_properties) {
    // Add the new vertex with the provided properties.
    TemplatedVertex new_vertex = b::add_vertex(new_vertex_properties, graph_);

    // Transfer incoming edges from 'v' to 'new_vertex'.
    std::vector<TemplatedVertex> in_vertices;
    for (TemplatedEdge edge : b::make_iterator_range(b::in_edges(v, graph_))) {
      TemplatedVertex source = b::source(edge, graph_);
      in_vertices.push_back(source);
    }

    // Remove old edges and add new edges.
    for (TemplatedVertex in_v : in_vertices) {
      b::remove_edge(in_v, v, graph_);
      b::add_edge(in_v, new_vertex, graph_);
    }
    b::add_edge(new_vertex, v, graph_);
  }

  template <typename Predicate>
  void add_vertex_before_pred(Predicate pred, const VertexProperties &new_vertex_properties) {
    std::vector<TemplatedVertex> vertices_add_before;
    for (TemplatedVertex v : b::make_iterator_range(b::vertices(graph_))) {
      if (pred(graph_[v])) {
        vertices_add_before.push_back(v);
      }
    }
    for (TemplatedVertex v : vertices_add_before) {
      add_vertex_before_vertex(v, new_vertex_properties->clone());
    }
  }

  /**
   * @brief Merges two vertices in the graph if they satisfy a given condition.
   *
   * This function attempts to merge two specified vertices (v1 and v2). The
   * vertices are merged by redirecting all in-edges and out-edges of v2 to v1
   * and then removing v2 from the graph. The merge only occurs if the provided
   * 'canMerge' predicate returns true for the two vertices.
   *
   * @param v1 The first vertex to be merged.
   * @param v2 The second vertex to be merged into the first.
   * @param can_merge A predicate that takes two vertices and returns true if
   * they can be merged.
   */
  void merge_vertices(TemplatedVertex v1, TemplatedVertex v2,
                      std::function<bool(TemplatedVertex, TemplatedVertex)> can_merge) {
    if (!can_merge(v1, v2)) {
      std::cout << "Cannot merge these vertices" << std::endl;
      return;
    }
    // Redirect in-edges and out-edges of v2 to v1, avoiding duplicates.
    std::unordered_set<std::pair<TemplatedVertex, TemplatedVertex>, pair_hash> unique_edges;
    // Redirect in-edges.
    for (TemplatedEdge edge : b::make_iterator_range(b::in_edges(v2, graph_))) {
      TemplatedVertex src = b::source(edge, graph_);
      if (src != v1 && unique_edges.emplace(src, v1).second) {
        b::add_edge(src, v1, graph_);
      }
    }
    // Redirect out-edges.
    for (TemplatedEdge edge : b::make_iterator_range(b::out_edges(v2, graph_))) {
      TemplatedVertex targ = b::target(edge, graph_);
      if (targ != v1 && unique_edges.emplace(v1, targ).second) {
        b::add_edge(v1, targ, graph_);
      }
    }
    // Remove the merged vertex.
    clear_vertex(v2, graph_);
    remove_vertex(v2, graph_);
  }

  std::set<TemplatedVertex, std::greater<>> replace_subgraph(
      const GraphType &ng, IsoMap &isomorphism,
      std::optional<
          std::function<void(std::vector<TemplatedEdgeReplacementStruct>, std::vector<TemplatedEdgeReplacementStruct>,
                             std::vector<TemplatedVertex>, std::vector<TemplatedVertex>, GraphType &)>>
          reconnect_edges = std::nullopt) {

    // Create unique list of vertices to be removed from isomorphism, sorted
    // largest->smallest.
    std::set<TemplatedVertex, std::greater<>> vertices_to_remove;
    std::ranges::for_each(isomorphism | std::views::values, [&vertices_to_remove](std::vector<TemplatedVertex> &v_vec) {
      auto s = std::set<TemplatedVertex>(v_vec.begin(), v_vec.end());
      vertices_to_remove.insert(s.begin(), s.end());
    });

    std::map<TemplatedVertex, TemplatedVertex> ng_vertices_map;
    // More than one old node may be associated with a new node.
    IsoMap new_vertices_map;
    std::vector<TemplatedVertex> roots;
    std::vector<TemplatedVertex> leaves;
    // Add vertices of ng to g and create a mapping.
    BGL_FORALL_VERTICES_T(v, ng, GraphType) {
      TemplatedVertex new_v = b::add_vertex(graph_);
      graph_[new_v] = ng[v]; // Copy vertex properties.
      if (in_degree(v, ng) == 0) {
        roots.push_back(new_v);
      }
      if (out_degree(v, ng) == 0) {
        leaves.push_back(new_v);
      }
      if (isomorphism.contains(v)) {
        for (TemplatedVertex old_v : isomorphism[v])
          new_vertices_map[old_v].push_back(new_v);
      }
      ng_vertices_map[v] = new_v;
    }
    // Add edges of ng to g using the new vertices map.
    BGL_FORALL_EDGES_T(e, ng, GraphType) {
      auto src = b::source(e, ng);
      auto targ = b::target(e, ng);
      b::add_edge(ng_vertices_map[src], ng_vertices_map[targ], ng[e], graph_);
    }
    // Build lists of external facing edges.
    std::vector<TemplatedEdgeReplacementStruct> external_incoming_edges;
    std::vector<TemplatedEdgeReplacementStruct> external_outgoing_edges;
    // Function for collecting external facing incoming and outgoing edges.
    auto external_edge_lambda = [*this, &vertices_to_remove](bool in, TemplatedVertex original_vertex,
                                                             TemplatedVertex replacement_vertex, auto edges,
                                                             std::vector<TemplatedEdgeReplacementStruct> &edge_list) {
      for (TemplatedEdge edge : b::make_iterator_range(edges)) { // Iterate over edges connecting
                                                                 // to original_vertex.
        // Get the src or target of the edge.
        TemplatedVertex src_trg = in ? b::source(edge, graph_) : b::target(edge, graph_);
        if (std::ranges::find(vertices_to_remove, src_trg) != vertices_to_remove.end()) // Don't include internal edges.
          continue;
        // Log the source or target edge with its original/new counterpart
        // vertex, and the EdgeType.
        edge_list.emplace_back(src_trg, original_vertex, replacement_vertex, graph_[edge]);
      }
    };
    // Iterate over all vertices to remove looking for external facing edges.
    for (const auto &original_vertex : vertices_to_remove) {
      // Get the vertices in the new subgraph that the original target vertex
      // corresponds to if they exist.
      std::vector<TemplatedVertex> replacement_vertices = new_vertices_map.contains(original_vertex)
                                                              ? new_vertices_map[original_vertex]
                                                              : std::vector<TemplatedVertex>({ULONG_MAX});
      for (TemplatedVertex replacement_vertex : replacement_vertices) { // Iterate over new vertices.
        external_edge_lambda(true, original_vertex, replacement_vertex, b::in_edges(original_vertex, graph_),
                             external_incoming_edges);
        external_edge_lambda(false, original_vertex, replacement_vertex, b::out_edges(original_vertex, graph_),
                             external_outgoing_edges);
      }
    }
    if (reconnect_edges.has_value()) {
      (*reconnect_edges)(external_incoming_edges, external_outgoing_edges, roots, leaves, graph_);
    } else {
      // Fallback behavior - connect all external incoming edges to root
      // vertices.
      //                     connect all external outgoing edges to leaf
      //                     vertices.
      for (const auto &root : roots) {
        for (auto &[src, _old_trg, _new_trg, edge_properties] : external_incoming_edges) {
          b::add_edge(src, root, edge_properties, graph_);
        }
      }
      for (const auto &leaf : leaves) {
        for (auto &[trg, _old_src, _new_src, edge_properties] : external_outgoing_edges) {
          b::add_edge(leaf, trg, edge_properties, graph_);
        }
      }
    }
    // for (auto vertex_to_remove : vertices_to_remove) {
    //   b::clear_vertex(vertex_to_remove, graph_);
    //   b::remove_vertex(vertex_to_remove, graph_);
    // }
    return vertices_to_remove;
  }

  /**
   * Reassign the IDs of nodes to match their indices in the graph.
   * This might be necessary after mutations in the graph. Applying
   * isomorphisms calls this transparently already.
   */
  void fix_vertex_obj_ids()
    requires NodeLike<VertexProperties>
  {
    // BFS Visitor to adjust the IDs of each vertex class (NOT the associated
    // vertices).
    struct custom_bfs_visitor : public b::default_bfs_visitor {
      GraphType &gref_;
      int &id_counter_;

      custom_bfs_visitor(GraphType &graph, int &id_counter) : gref_(graph), id_counter_(id_counter) {}

      void discover_vertex(TemplatedVertex v, const GraphType &) {
        auto &cls = gref_[v];
        if (cls->id() == -1) {
          cls->id(id_counter_);
          id_counter_++;
        }
      }

      void finish_vertex(TemplatedVertex v, const GraphType &) {}
    };

    // First, discover all root vertices.
    std::vector<TemplatedVertex> root_vertices;
    for (TemplatedVertex v : b::make_iterator_range(b::vertices(graph_))) {
      graph_[v]->id(-1);
      if (b::in_degree(v, graph_) == 0) {
        root_vertices.push_back(v);
      }
    }
    // For all root vertices, perform a breadth first search.
    int id_counter = 0;
    for (auto root_vertex : root_vertices) {
      custom_bfs_visitor vis(graph_, id_counter);
      b::breadth_first_search(graph_, root_vertex, visitor(vis));
    }
  }

  /**
   * @brief Extension point for [write_graph]. Write the attributes of the
   * vertex in dot format to the output stream.
   */
  virtual void vertex_property_writer(std::ostream &out, const TemplatedVertex &v) const {
    out << "[shape=rectangle, label=\"" << v << ":" << this->graph_[v]->id() << "\"]";
  }

  /**
   * @brief Extension point for [write_graph]. Write the attributes of the edge
   * in dot format to the output stream.
   */
  virtual void edge_property_writer(std::ostream &out, const TemplatedEdge &) const { out << "[label=\"\"]"; }

  void set_base_graph(const GraphType &g) { graph_ = g; }
};

} // namespace ghl
#endif
