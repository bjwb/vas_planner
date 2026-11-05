// Copyright 2026. Licensed under the BSD-3-Clause license.
#ifndef VAS_PLANNER__VORONOI_PLANNING_HPP_
#define VAS_PLANNER__VORONOI_PLANNING_HPP_

#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "vas_planner/voronoi_graph.hpp"

namespace vas_planner
{

// Exact capsule vs. occupied cell-square check. Caller must lock the costmap.
// The capsule is the swept circle with radius robot_radius + safety_margin.
// Unknown cells and the map exterior are blocked. Inflation costs are not
// inflated a second time: only lethal/unknown cell squares are obstacles.
bool segmentIsFree(
  const nav2_costmap_2d::Costmap2D & costmap,
  const Point2D & a, const Point2D & b, double radius);

struct GraphPlan
{
  std::size_t start_node;
  std::size_t goal_node;
  std::vector<std::size_t> node_indices;
  std::vector<Point2D> waypoints;
};

// Strict Euclidean-nearest node selection, with no grid search or automatic
// fallback to another attachment node. Throws on collision or no graph path.
// max_connection_distance == 0 disables the distance limit.
GraphPlan planOnGraph(
  const VoronoiGraph & graph, const nav2_costmap_2d::Costmap2D & costmap,
  const Point2D & start, const Point2D & goal, double clearance,
  double max_connection_distance = 0.0);

// Preserve every bend and both endpoints; add points at most spacing apart.
std::vector<Point2D> interpolatePolyline(
  const std::vector<Point2D> & waypoints, double spacing);

}  // namespace vas_planner
#endif  // VAS_PLANNER__VORONOI_PLANNING_HPP_
