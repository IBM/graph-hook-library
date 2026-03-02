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
 * @file vf3_vs_vf2.cpp
 *
 * @brief Benchmark driver comparing Boost VF2 and VF3 subgraph isomorphism.
 *
 * The program reads two graphs from adjacency-list text files, runs both VF2
 * and VF3 to collect all isomorphism mappings, and writes performance and
 * mismatch results to CSV outputs. It is intended for profiling and regression
 * checking of the underlying Boost algorithms used by GHL.
 */
#include <algorithm>
#include <atomic>
#include <boost/graph/vf2_sub_graph_iso.hpp>
#include <boost/graph/vf3_sub_graph_iso.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numeric>
#include <stop_token>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// The per-iteration guard enforces a hard cap via MAX_TOTAL_SECONDS.
constexpr double MAX_TOTAL_SECONDS = 300.0;
// Time out the algorithm after 300 seconds.
constexpr std::chrono::seconds MAX_ISO_DURATION_VF3(300);
constexpr std::chrono::seconds MAX_ISO_DURATION_VF2(300);
constexpr const char *PERF_HEADERS =
    "vf2_mean,vf3_mean,iterations,vf2_matches,vf3_matches,target_path,pattern_path,target_vertices,target_edges,"
    "pattern_vertices,pattern_edges";
constexpr const char *MISMATCH_HEADERS =
    "vf2_matches,vf3_matches,iterations,target_path,pattern_path,target_vertices,target_edges,pattern_vertices,"
    "pattern_edges,vf3_extra,vf3_missing";
constexpr const char *ABORTED_HEADERS = PERF_HEADERS;

/**
 * @brief Simple CSV writer that appends rows to a file.
 *
 * Handles writing headers once and exposes helpers for writing typed rows in a
 * benchmarking context.
 */
class CSVWriter {
private:
  std::ofstream file;

public:
  CSVWriter(const std::string &filename, const std::string &headers) {
    bool exists = std::filesystem::exists(filename);
    file.open(filename, std::ios::app);
    if (!file.is_open()) {
      throw std::runtime_error(std::format("Could not open file: {}", filename));
    }
    if (!exists) {
      file << headers << "\n";
    }
  }

  ~CSVWriter() {
    if (file.is_open()) {
      file.close();
    }
  }

  template <typename T> void writeRow(const std::vector<T> &data) {
    for (size_t i = 0; i < data.size(); ++i) {
      if (i > 0)
        file << ",";
      file << data[i];
    }
    file << "\n";
    file.flush();
  }
};

/**
 * @brief Lightweight vertex properties used for the benchmark graphs.
 */
struct VertexProps {};

/**
 * @brief Lightweight edge properties used for the benchmark graphs.
 */
struct EdgeProps {};

/**
 * @brief VF2 callback that collects mappings while honoring cooperative cancellation.
 *
 * @tparam Graph1 Pattern graph type.
 * @tparam Graph2 Target graph type.
 */
template <typename Graph1, typename Graph2> struct collect_mappings_callback {
  using vertex_type1 = boost::graph_traits<Graph1>::vertex_descriptor;
  using vertex_type2 = boost::graph_traits<Graph2>::vertex_descriptor;

public:
  explicit collect_mappings_callback(const Graph1 &g1, const Graph2 &g2,
                                     std::vector<std::unordered_map<vertex_type1, vertex_type2>> &v,
                                     std::stop_token stop_token)
      : graph1_(g1), graph2_(g2), res(v), stop_token_(stop_token) {}

  template <typename CorrespondenceMap1To2, typename CorrespondenceMap2To1>
  bool operator()(CorrespondenceMap1To2 f, CorrespondenceMap2To1) {
    if (stop_token_.stop_requested()) {
      return false;
    }
    std::unordered_map<vertex_type1, vertex_type2> new_mapping;
    BGL_FORALL_VERTICES_T(v, graph1_, Graph1) {
      new_mapping.insert({boost::get(boost::vertex_index_t(), graph1_, v),
                          boost::get(boost::vertex_index_t(), graph2_, boost::get(f, v))});
    }
    res.push_back(new_mapping);
    return true;
  }
  std::stop_token stop_token_;

private:
  const Graph1 &graph1_;
  const Graph2 &graph2_;
  std::vector<std::unordered_map<vertex_type1, vertex_type2>> &res;
};

/**
 * @brief VF3 callback that collects mappings while honoring cooperative cancellation.
 *
 * @tparam Graph1 Pattern graph type.
 * @tparam Graph2 Target graph type.
 */
template <typename Graph1, typename Graph2> struct vf3_collect_mappings_callback {

  using vertex_type1 = boost::graph_traits<Graph1>::vertex_descriptor;
  using vertex_type2 = boost::graph_traits<Graph2>::vertex_descriptor;

public:
  explicit vf3_collect_mappings_callback(const Graph1 &g1, const Graph2 &g2,
                                         std::vector<std::unordered_map<vertex_type1, vertex_type2>> &v,
                                         std::stop_token stop_token)
      : graph1_(g1), graph2_(g2), res(v), stop_token_(stop_token) {}

  template <typename CorrespondenceMap1To2, typename CorrespondenceMap2To1>
  bool operator()(CorrespondenceMap1To2 f, CorrespondenceMap2To1) {
    if (stop_token_.stop_requested()) {
      return false;
    }
    std::unordered_map<vertex_type1, vertex_type2> new_mapping;
    for (const auto &[v_small, v_large] : f) {
      new_mapping[v_small] = v_large;
    }
    res.push_back(new_mapping);
    return true;
  }
  std::stop_token stop_token_;

private:
  const Graph1 &graph1_;
  const Graph2 &graph2_;
  std::vector<std::unordered_map<vertex_type1, vertex_type2>> &res;
};

using Graph = boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, VertexProps, EdgeProps>;
using vertex_type = boost::graph_traits<Graph>::vertex_descriptor;
using Duration = std::chrono::duration<double>;

/**
 * @brief Produce a canonical string representation of a single mapping.
 *
 * @param mapping Mapping from pattern vertices to target vertices.
 * @param pattern Pattern graph.
 * @param target Target graph.
 * @return Comma-separated string of "pattern:target" pairs sorted by pattern id.
 */
std::string canonicalize_mapping(const std::unordered_map<vertex_type, vertex_type> &mapping, const Graph &pattern,
                                 const Graph &target) {
  std::vector<std::pair<int, int>> entries;
  entries.reserve(mapping.size());
  for (const auto &[small_vertex, large_vertex] : mapping) {
    int pattern_idx = boost::get(boost::vertex_index, pattern, small_vertex);
    int target_idx = boost::get(boost::vertex_index, target, large_vertex);
    entries.emplace_back(pattern_idx, target_idx);
  }
  std::sort(entries.begin(), entries.end());
  std::string repr;
  repr.reserve(entries.size() * 10);
  for (const auto &[pattern_idx, target_idx] : entries) {
    repr.append(std::to_string(pattern_idx));
    repr.push_back(':');
    repr.append(std::to_string(target_idx));
    repr.push_back(',');
  }
  return repr;
}

std::vector<std::string>
canonicalize_mappings(const std::vector<std::unordered_map<vertex_type, vertex_type>> &mappings, const Graph &pattern,
                      const Graph &target) {
  std::vector<std::string> result;
  result.reserve(mappings.size());
  for (const auto &mapping : mappings) {
    result.emplace_back(canonicalize_mapping(mapping, pattern, target));
  }
  return result;
}

/**
 * @brief Compute the mean of a sample set or -1.0 if empty.
 *
 * @param samples Collection of sample durations.
 * @return Sample mean or -1.0 when no samples are provided.
 */
double mean_or_negative(const std::vector<double> &samples) {
  if (samples.empty())
    return -1.0;
  return std::accumulate(samples.begin(), samples.end(), 0.0) / static_cast<double>(samples.size());
};

/**
 * @brief Start a watchdog thread that requests cancellation after a duration.
 *
 * @param source Stop source shared with the algorithm under test.
 * @param timed_out Atomic flag set when the watchdog triggers.
 * @param duration Maximum allowed duration for the guarded section.
 * @return std::jthread managing the watchdog lifetime.
 */
std::jthread start_timeout_watchdog(std::stop_source &source, std::atomic<bool> &timed_out,
                                    std::chrono::seconds duration) {
  // Capture duration by value so the watchdog thread doesn't access a dangling reference.
  return std::jthread([&source, &timed_out, duration](std::stop_token st) {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (!st.stop_requested()) {
      if (std::chrono::steady_clock::now() >= deadline) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (st.stop_requested()) {
      return;
    }
    timed_out.store(true);
    source.request_stop();
  });
}

/**
 * @brief Parse a graph from an adjacency-list text file.
 *
 * Each line after the first lists the number of neighbors followed by neighbor
 * indices for that vertex. The first line stores the vertex count.
 *
 * @param filename Path to the graph file.
 * @return Populated Boost graph instance.
 * @throws std::runtime_error if the file is missing or malformed.
 */
Graph read_graph_from_file(const std::string &filename) {
  std::ifstream infile(filename);
  if (!infile) {
    throw std::runtime_error(std::format("Could not open file: {}", filename));
  }
  size_t num_vertices;
  infile >> num_vertices;
  Graph g(num_vertices);
  std::string line;
  std::getline(infile, line);
  for (size_t i = 0; i < num_vertices; ++i) {
    if (!std::getline(infile, line)) {
      throw std::runtime_error(std::format("Unexpected end of file while reading vertex {}", std::to_string(i)));
    }
    std::istringstream iss(line);
    int edge_count, neighbor;
    iss >> edge_count;
    for (int j = 0; j < edge_count; ++j) {
      if (!(iss >> neighbor)) {
        throw std::runtime_error(std::format("Malformed line at vertex {}", std::to_string(i)));
      }
      boost::add_edge(i, neighbor, g);
    }
  }
  return g;
}

/**
 * @brief Entry point for the VF2/VF3 comparison benchmark.
 *
 * Expects five arguments: target graph, pattern graph, performance CSV path,
 * mismatch CSV path, and aborted CSV path.
 *
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return 0 on success; non-zero on failure.
 */
int main(int argc, char *argv[]) {
  if (argc != 6) {
    std::cerr << "Expected exactly 5 path arguments\n";
    return 1;
  }

  namespace fs = std::filesystem;

  std::string target_path_str;
  std::string pattern_path_str;
  std::string perf_output_path_str;
  std::string mismatch_output_path_str;
  std::string aborted_output_path_str;

  try {
    fs::path target_path = fs::path(argv[1]);
    fs::path pattern_path = fs::path(argv[2]);
    fs::path perf_output_path = fs::path(argv[3]);
    fs::path mismatch_output_path = fs::path(argv[4]);
    fs::path aborted_output_path = fs::path(argv[5]);

    if (!fs::exists(target_path)) {
      std::cerr << "Provided target_path " << target_path << " does not exist!. Quitting." << std::endl;
      return 1;
    }

    if (!fs::exists(pattern_path)) {
      std::cout << "Provided pattern_path " << pattern_path << " does not exist!. Quitting." << std::endl;
      return 1;
    }
    if (!fs::is_regular_file(target_path)) {
      std::cerr << "Error: Target path is not a regular file: " << target_path << std::endl;
      return 1;
    }

    if (!fs::is_regular_file(pattern_path)) {
      std::cerr << "Error: Pattern path is not a regular file: " << pattern_path << std::endl;
      return 1;
    }

    target_path_str = target_path.string();
    pattern_path_str = pattern_path.string();
    perf_output_path_str = perf_output_path.string();
    mismatch_output_path_str = mismatch_output_path.string();
    aborted_output_path_str = aborted_output_path.string();

  } catch (const fs::filesystem_error &ex) {
    std::cerr << "Filesystem error: " << ex.what() << std::endl;
    return 1;
  } catch (const std::exception &ex) {
    std::cerr << "Error: " << ex.what() << std::endl;
    return 1;
  }

  auto already_in_csv = [&](const std::string &csv_path) -> bool {
    if (!std::filesystem::exists(csv_path))
      return false;
    std::ifstream fin(csv_path);
    if (!fin.is_open())
      return false;
    std::string line;
    while (std::getline(fin, line)) {
      if (line.find(target_path_str) != std::string::npos && line.find(pattern_path_str) != std::string::npos) {
        return true;
      }
    }
    return false;
  };

  if (already_in_csv(perf_output_path_str) || already_in_csv(mismatch_output_path_str)) {
    std::cout << "Target/Pattern combination already processed. Exiting!" << std::endl;
    return 0;
  }
  Graph pattern = read_graph_from_file(pattern_path_str);
  Graph target = read_graph_from_file(target_path_str);
  std::vector<double> vf2_elapsed_samples;
  std::vector<double> vf3_elapsed_samples;
  bool mappings_checked = false;
  bool aborted_iteration = false;
  int recorded_vf2_size = 0;
  int recorded_vf3_size = 0;
  int recorded_extra = 0;
  int recorded_missing = 0;
  auto benchmark_start = std::chrono::high_resolution_clock::now();
  std::size_t iterations_run = 0;
  for (int i = 0; i < 10; ++i) {
    if (iterations_run > 0) {
      double elapsed_before_iter =
          std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - benchmark_start).count();
      if (elapsed_before_iter >= MAX_TOTAL_SECONDS) {
        break;
      }
    }
    bool iteration_aborted = false;
    std::vector<std::unordered_map<vertex_type, vertex_type>> vf3_res{};
    std::stop_source vf3_stop_source;
    std::atomic<bool> vf3_timed_out(false);
    vf3_collect_mappings_callback<Graph, Graph> vf3_callback(pattern, target, vf3_res, vf3_stop_source.get_token());
    auto vf3_timeout_guard = start_timeout_watchdog(vf3_stop_source, vf3_timed_out, MAX_ISO_DURATION_VF3);
    auto vf3_start = std::chrono::high_resolution_clock::now();
    boost::vf3_subgraph_iso(pattern, target, vf3_callback);
    auto vf3_end = std::chrono::high_resolution_clock::now();
    vf3_timeout_guard.request_stop();
    if (vf3_timed_out.load()) {
      std::cout << "VF3 (timed out) " << std::endl;
      iteration_aborted = true;
      aborted_iteration = true;
      recorded_vf3_size = -1;
    } else {
      recorded_vf3_size = vf3_res.size();
      Duration vf3_duration = vf3_end - vf3_start;
      vf3_elapsed_samples.push_back(vf3_duration.count());
    }

    std::vector<std::unordered_map<vertex_type, vertex_type>> vf2_res{};
    std::stop_source vf2_stop_source;
    std::atomic<bool> vf2_timed_out(false);
    auto callback = collect_mappings_callback<Graph, Graph>(pattern, target, vf2_res, vf2_stop_source.get_token());
    auto vf2_timeout_guard = start_timeout_watchdog(vf2_stop_source, vf2_timed_out, MAX_ISO_DURATION_VF2);
    auto vf2_start = std::chrono::high_resolution_clock::now();
    bool vf2_found = boost::vf2_subgraph_iso(pattern, target, callback);
    auto vf2_end = std::chrono::high_resolution_clock::now();
    vf2_timeout_guard.request_stop();
    if (vf2_timed_out.load()) {
      std::cout << "VF2 (timed out) " << std::endl;
      iteration_aborted = true;
      aborted_iteration = true;
      recorded_vf2_size = -1;
      break;
    } else {
      if (vf2_found) {
        recorded_vf2_size = vf2_res.size();
      }
      Duration vf2_duration = vf2_end - vf2_start;
      vf2_elapsed_samples.push_back(vf2_duration.count());
    }
    if (iteration_aborted)
      break;
    if (!mappings_checked) {
      auto canonical_vf2_mappings = canonicalize_mappings(vf2_res, pattern, target);
      auto canonical_vf3_mappings = canonicalize_mappings(vf3_res, pattern, target);

      // Mappings that are in VF3 but not in VF2 -> Extra Matches
      std::unordered_set<std::string> vf2_mappings_set(canonical_vf2_mappings.begin(), canonical_vf2_mappings.end());
      for (const auto &mapping : canonical_vf3_mappings) {
        if (vf2_mappings_set.find(mapping) == vf2_mappings_set.end()) {
          recorded_extra++;
        }
      }
      // Mappings that are in VF2 but not in VF3 -> Missing mappings
      std::unordered_set<std::string> vf3_mappings_set(canonical_vf3_mappings.begin(), canonical_vf3_mappings.end());
      for (const auto &mapping : canonical_vf2_mappings) {
        if (vf3_mappings_set.find(mapping) == vf3_mappings_set.end()) {
          recorded_missing++;
        }
      }
      mappings_checked = true;
    }
    iterations_run++;
    double elapsed_total =
        std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - benchmark_start).count();
    if (elapsed_total >= MAX_TOTAL_SECONDS) {
      break;
    }
  }
  std::cout << std::endl;

  double vf2_mean = mean_or_negative(vf2_elapsed_samples);
  double vf3_mean = mean_or_negative(vf3_elapsed_samples);

  CSVWriter matching_csv(perf_output_path_str, PERF_HEADERS);
  CSVWriter mismatching_csv(mismatch_output_path_str, MISMATCH_HEADERS);
  CSVWriter aborted_csv(aborted_output_path_str, ABORTED_HEADERS);

  if (aborted_iteration) {
    aborted_csv.writeRow(std::vector<std::string>{
        std::to_string(vf2_mean), std::to_string(vf3_mean), std::to_string(iterations_run),
        std::to_string(recorded_vf2_size), std::to_string(recorded_vf3_size), target_path_str, pattern_path_str,
        std::to_string(boost::num_vertices(target)), std::to_string(boost::num_edges(target)),
        std::to_string(boost::num_vertices(pattern)), std::to_string(boost::num_edges(pattern))});
  } else if (!recorded_extra && !recorded_missing) {
    matching_csv.writeRow(std::vector<std::string>{
        std::to_string(vf2_mean), std::to_string(vf3_mean), std::to_string(iterations_run),
        std::to_string(recorded_vf2_size), std::to_string(recorded_vf3_size), target_path_str, pattern_path_str,
        std::to_string(boost::num_vertices(target)), std::to_string(boost::num_edges(target)),
        std::to_string(boost::num_vertices(pattern)), std::to_string(boost::num_edges(pattern))});
  } else {
    mismatching_csv.writeRow(std::vector<std::string>{
        std::to_string(recorded_vf2_size), std::to_string(recorded_vf3_size), std::to_string(iterations_run),
        target_path_str, pattern_path_str, std::to_string(boost::num_vertices(target)),
        std::to_string(boost::num_edges(target)), std::to_string(boost::num_vertices(pattern)),
        std::to_string(boost::num_edges(pattern)), std::to_string(recorded_extra), std::to_string(recorded_missing)});
  }
  return 0;
}
