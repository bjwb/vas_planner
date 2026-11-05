#include "vas_planner/voronoi_graph.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include <nlohmann/json.hpp>

namespace vas_planner
{
namespace
{
using Json = nlohmann::json;

double finiteNumber(const Json & value, const std::string & name)
{
  if (!value.is_number()) {
    throw std::runtime_error(name + " must be a finite number");
  }
  const double number = value.get<double>();
  if (!std::isfinite(number)) {
    throw std::runtime_error(name + " must be a finite number");
  }
  return number;
}

int64_t nodeId(const Json & value, const std::string & name)
{
  if (!value.is_number_integer() ||
    (value.is_number_unsigned() &&
    value.get<uint64_t>() > static_cast<uint64_t>(std::numeric_limits<int64_t>::max())))
  {
    throw std::runtime_error(name + " must be a nonnegative 64-bit integer");
  }
  const int64_t id = value.get<int64_t>();
  if (id < 0) {
    throw std::runtime_error(name + " must be a nonnegative 64-bit integer");
  }
  return id;
}

double distance(const Point2D & a, const Point2D & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}
}  // namespace

void VoronoiGraph::load(const std::string & filename)
{
  try {
    std::ifstream input(filename);
    if (!input) {
      throw std::runtime_error("cannot open file");
    }
    const Json document = Json::parse(input);
    if (!document.is_object() || !document.at("format_version").is_number_integer() ||
      document.at("format_version") != 1)
    {
      throw std::runtime_error("expected format_version 1");
    }
    if (!document.at("directed").is_boolean() || document.at("directed").get<bool>()) {
      throw std::runtime_error("expected an undirected graph (directed: false)");
    }
    if (document.at("map").at("units") != "m") {
      throw std::runtime_error(
              "map.units must be 'm'; regenerate the graph with map.yaml or "
              "--resolution and --origin so positions match the ROS map frame");
    }
    const auto & nodes = document.at("nodes");
    const auto & edges = document.at("edges");
    if (!nodes.is_array() || nodes.empty() || !edges.is_array()) {
      throw std::runtime_error("nodes must be a nonempty array and edges must be an array");
    }

    // Build independently: a failed reload leaves the previous graph intact.
    VoronoiGraph loaded;
    if (document.contains("parameters")) {
      const auto & parameters = document.at("parameters");
      if (!parameters.is_object()) {
        throw std::runtime_error("parameters must be an object");
      }
      loaded.robot_radius_ = finiteNumber(parameters.value("robot_radius", Json(0.0)),
        "parameters.robot_radius");
      loaded.safety_margin_ = finiteNumber(parameters.value("safety_margin", Json(0.0)),
        "parameters.safety_margin");
      if (loaded.robot_radius_ < 0.0 || loaded.safety_margin_ < 0.0) {
        throw std::runtime_error("robot_radius and safety_margin must be nonnegative");
      }
    }
    std::unordered_map<int64_t, size_t> id_to_index;
    loaded.nodes_.reserve(nodes.size());
    loaded.adjacency_.resize(nodes.size());
    for (const auto & node : nodes) {
      const int64_t id = nodeId(node.at("id"), "node.id");
      const auto & position = node.at("position");
      if (!position.is_array() || position.size() != 2) {
        throw std::runtime_error("node.position must contain [x, y]");
      }
      const Point2D point{
        finiteNumber(position.at(0), "node.position.x"),
        finiteNumber(position.at(1), "node.position.y")};
      if (!id_to_index.emplace(id, loaded.nodes_.size()).second) {
        throw std::runtime_error("duplicate node ID " + std::to_string(id));
      }
      loaded.nodes_.push_back({id, point});
    }
    loaded.edges_.reserve(edges.size());
    for (const auto & edge : edges) {
      const auto source = id_to_index.find(nodeId(edge.at("source"), "edge.source"));
      const auto target = id_to_index.find(nodeId(edge.at("target"), "edge.target"));
      if (source == id_to_index.end() || target == id_to_index.end()) {
        throw std::runtime_error("edge references a nonexistent node ID");
      }
      const double length = finiteNumber(edge.at("length"), "edge.length");
      const double clearance = finiteNumber(edge.at("min_clearance"), "edge.min_clearance");
      if (length <= 0.0 || clearance < 0.0) {
        throw std::runtime_error("edge.length must be positive and min_clearance nonnegative");
      }
      const size_t edge_index = loaded.edges_.size();
      loaded.edges_.push_back({source->second, target->second, length, clearance});
      loaded.adjacency_[source->second].push_back({target->second, edge_index});
      loaded.adjacency_[target->second].push_back({source->second, edge_index});
      const double geometric_length = distance(
        loaded.nodes_[source->second].position, loaded.nodes_[target->second].position);
      if (geometric_length > 0.0) {
        loaded.heuristic_scale_ = std::min(
          loaded.heuristic_scale_, length / geometric_length);
      }
    }
    *this = std::move(loaded);
  } catch (const std::exception & error) {
    throw std::runtime_error("Cannot load Voronoi graph '" + filename + "': " + error.what());
  }
}

size_t VoronoiGraph::nearestNode(double x, double y) const
{
  if (nodes_.empty()) {
    throw std::runtime_error("Voronoi graph has no nodes");
  }
  if (!std::isfinite(x) || !std::isfinite(y)) {
    throw std::invalid_argument("nearestNode requires finite map coordinates");
  }
  const Point2D query{x, y};
  size_t nearest = 0;
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < nodes_.size(); ++i) {
    const double candidate = distance(query, nodes_[i].position);
    if (candidate < best) {
      best = candidate;
      nearest = i;
    }
  }
  return nearest;
}

std::vector<size_t> VoronoiGraph::findPath(
  size_t start, size_t goal, double minimum_clearance,
  const std::function<bool(size_t)> & edge_valid) const
{
  if (start >= nodes_.size() || goal >= nodes_.size()) {
    throw std::out_of_range("A* start or goal node index is out of range");
  }
  if (!std::isfinite(minimum_clearance) || minimum_clearance < 0.0) {
    throw std::invalid_argument("minimum_clearance must be finite and nonnegative");
  }
  if (start == goal) {
    return {start};
  }
  struct Candidate
  {
    double f;
    double g;
    size_t node;
    bool operator>(const Candidate & other) const
    {
      if (f != other.f) {return f > other.f;}
      if (g != other.g) {return g > other.g;}
      return node > other.node;
    }
  };
  const size_t no_parent = nodes_.size();
  std::vector<size_t> parent(nodes_.size(), no_parent);
  std::vector<double> cost(nodes_.size(), std::numeric_limits<double>::infinity());
  std::priority_queue<Candidate, std::vector<Candidate>, std::greater<Candidate>> open;
  const auto heuristic = [this, goal](size_t node) {
      // Zero explicitly avoids 0 * infinity for extreme but finite coordinates.
      return heuristic_scale_ == 0.0 ? 0.0 : heuristic_scale_ *
             distance(nodes_[node].position, nodes_[goal].position);
    };
  cost[start] = 0.0;
  open.push({heuristic(start), 0.0, start});
  while (!open.empty()) {
    const Candidate current = open.top();
    open.pop();
    if (current.g > cost[current.node]) {
      continue;
    }
    if (current.node == goal) {
      std::vector<size_t> path;
      for (size_t node = goal; node != no_parent; node = parent[node]) {
        path.push_back(node);
      }
      std::reverse(path.begin(), path.end());
      return path;
    }
    for (const auto & neighbor : adjacency_[current.node]) {
      const auto & edge = edges_[neighbor.edge];
      if (edge.min_clearance <= minimum_clearance ||
        (edge_valid && !edge_valid(neighbor.edge)))
      {
        continue;
      }
      const double tentative = current.g + edge.length;
      if (tentative < cost[neighbor.node]) {
        cost[neighbor.node] = tentative;
        parent[neighbor.node] = current.node;
        open.push({tentative + heuristic(neighbor.node), tentative, neighbor.node});
      }
    }
  }
  return {};
}

}  // namespace vas_planner
