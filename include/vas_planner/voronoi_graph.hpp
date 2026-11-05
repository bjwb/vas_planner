#ifndef VAS_PLANNER__VORONOI_GRAPH_HPP_
#define VAS_PLANNER__VORONOI_GRAPH_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace vas_planner
{

struct Point2D
{
  double x = 0.0;
  double y = 0.0;
};

struct GraphNode
{
  int64_t id;
  Point2D position;
};

struct GraphEdge
{
  size_t source;
  size_t target;
  double length;
  double min_clearance;
};

// Nodes, edges and returned paths use vector indices. GraphNode::id preserves
// the original JSON ID, which does not have to be contiguous or sorted.
class VoronoiGraph
{
public:
  void load(const std::string & filename);

  const std::vector<GraphNode> & nodes() const {return nodes_;}
  const std::vector<GraphEdge> & edges() const {return edges_;}
  double robotRadius() const {return robot_radius_;}
  double safetyMargin() const {return safety_margin_;}

  size_t nearestNode(double x, double y) const;

  // edge_valid receives an index into edges(). An edge must pass both the
  // strict clearance threshold and this optional current-costmap predicate.
  std::vector<size_t> findPath(
    size_t start, size_t goal, double minimum_clearance = 0.0,
    const std::function<bool(size_t)> & edge_valid = {}) const;

private:
  struct Neighbor
  {
    size_t node;
    size_t edge;
  };

  std::vector<GraphNode> nodes_;
  std::vector<GraphEdge> edges_;
  std::vector<std::vector<Neighbor>> adjacency_;
  double robot_radius_ = 0.0;
  double safety_margin_ = 0.0;
  double heuristic_scale_ = 1.0;
};

}  // namespace vas_planner

#endif  // VAS_PLANNER__VORONOI_GRAPH_HPP_
