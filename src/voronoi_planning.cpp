// Copyright 2026. Licensed under the BSD-3-Clause license.
#include "vas_planner/voronoi_planning.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

#include "nav2_costmap_2d/cost_values.hpp"

namespace vas_planner
{
namespace
{
double distance(const Point2D & a, const Point2D & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double pointSegmentDistanceSquared(const Point2D & p, const Point2D & a, const Point2D & b)
{
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double length_squared = dx * dx + dy * dy;
  const double t = length_squared > 0.0 ?
    std::max(0.0, std::min(1.0, ((p.x - a.x) * dx + (p.y - a.y) * dy) /
    length_squared)) : 0.0;
  const double ex = p.x - (a.x + t * dx);
  const double ey = p.y - (a.y + t * dy);
  return ex * ex + ey * ey;
}

double pointBoxDistanceSquared(
  const Point2D & p, double xmin, double ymin, double xmax, double ymax)
{
  const double dx = std::max({xmin - p.x, 0.0, p.x - xmax});
  const double dy = std::max({ymin - p.y, 0.0, p.y - ymax});
  return dx * dx + dy * dy;
}

double segmentBoxDistanceSquared(
  const Point2D & a, const Point2D & b,
  double xmin, double ymin, double xmax, double ymax)
{
  // Slab intersection detects crossings even when both endpoints are outside.
  double lo = 0.0;
  double hi = 1.0;
  const auto clip = [&lo, &hi](double origin, double delta, double low, double high) {
      if (delta == 0.0) {
        return origin >= low && origin <= high;
      }
      double first = (low - origin) / delta;
      double last = (high - origin) / delta;
      if (first > last) {
        std::swap(first, last);
      }
      lo = std::max(lo, first);
      hi = std::min(hi, last);
      return lo <= hi;
    };
  if (clip(a.x, b.x - a.x, xmin, xmax) && clip(a.y, b.y - a.y, ymin, ymax)) {
    return 0.0;
  }
  double result = std::min(
    pointBoxDistanceSquared(a, xmin, ymin, xmax, ymax),
    pointBoxDistanceSquared(b, xmin, ymin, xmax, ymax));
  for (const Point2D & corner : {
      Point2D{xmin, ymin}, Point2D{xmin, ymax}, Point2D{xmax, ymin}, Point2D{xmax, ymax}})
  {
    result = std::min(result, pointSegmentDistanceSquared(corner, a, b));
  }
  return result;
}
}  // namespace

bool segmentIsFree(
  const nav2_costmap_2d::Costmap2D & costmap,
  const Point2D & a, const Point2D & b, double radius)
{
  if (!std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(b.x) ||
    !std::isfinite(b.y) || !std::isfinite(radius) || radius < 0.0)
  {
    return false;
  }
  const double resolution = costmap.getResolution();
  const unsigned int width = costmap.getSizeInCellsX();
  const unsigned int height = costmap.getSizeInCellsY();
  if (width == 0 || height == 0 || !std::isfinite(resolution) || resolution <= 0.0) {
    return false;
  }
  const double ox = costmap.getOriginX();
  const double oy = costmap.getOriginY();
  const double xmin = std::min(a.x, b.x) - radius;
  const double ymin = std::min(a.y, b.y) - radius;
  const double xmax = std::max(a.x, b.x) + radius;
  const double ymax = std::max(a.y, b.y) + radius;
  if (xmin <= ox || ymin <= oy || xmax >= ox + width * resolution ||
    ymax >= oy + height * resolution)
  {
    return false;
  }

  // Include neighboring cells at exact grid boundaries: touching is collision.
  const int x0 = std::max(0, static_cast<int>(std::floor((xmin - ox) / resolution)) - 1);
  const int y0 = std::max(0, static_cast<int>(std::floor((ymin - oy) / resolution)) - 1);
  const int x1 = std::min( 
    static_cast<int>(width) - 1, static_cast<int>(std::floor((xmax - ox) / resolution)) + 1);
  const int y1 = std::min(
    static_cast<int>(height) - 1, static_cast<int>(std::floor((ymax - oy) / resolution)) + 1);
  const double limit = radius * radius + 1e-12;
  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      if (costmap.getCost(x, y) < nav2_costmap_2d::LETHAL_OBSTACLE) {
      // if (costmap.getCost(x, y) >= 0) {
        continue;
      }
      const double cell_x = ox + x * resolution;
      const double cell_y = oy + y * resolution;
      if (segmentBoxDistanceSquared(
          a, b, cell_x, cell_y, cell_x + resolution, cell_y + resolution) <= limit)
      {
        return false;
      }
    }
  }
  return true;
}

GraphPlan planOnGraph(
  const VoronoiGraph & graph, const nav2_costmap_2d::Costmap2D & costmap,
  const Point2D & start, const Point2D & goal, double clearance,
  double max_connection_distance)
{
  if (!std::isfinite(clearance) || clearance < 0.0 ||
    !std::isfinite(max_connection_distance) || max_connection_distance < 0.0)
  {
    throw std::runtime_error("Clearance and connection distance must be finite and nonnegative");
  }
  if (!segmentIsFree(costmap, start, start, clearance)) {
    throw std::runtime_error("Start footprint is occupied, unknown, or outside the costmap");
  }
  if (!segmentIsFree(costmap, goal, goal, clearance)) {
    throw std::runtime_error("Goal footprint is occupied, unknown, or outside the costmap");
  }
  GraphPlan result;
  result.start_node = graph.nearestNode(start.x, start.y);
  result.goal_node = graph.nearestNode(goal.x, goal.y);
  if (distance(start, goal) == 0.0) {
    result.waypoints.push_back(goal);
    return result;
  }
  const auto & first = graph.nodes().at(result.start_node);
  const auto & last = graph.nodes().at(result.goal_node);
  if (max_connection_distance > 0.0 &&
    (distance(start, first.position) > max_connection_distance ||
    distance(last.position, goal) > max_connection_distance))
  {
    throw std::runtime_error("Nearest node exceeds max_connection_distance");
  }
  if (!segmentIsFree(costmap, start, first.position, clearance)) {
    throw std::runtime_error(
            "Start-to-nearest-node connector collides (node " + std::to_string(first.id) + ")");
  }
  if (!segmentIsFree(costmap, last.position, goal, clearance)) {
    throw std::runtime_error(
            "Nearest-node-to-goal connector collides (node " + std::to_string(last.id) + ")");
  }
  std::vector<int8_t> edge_validity(graph.edges().size(), -1);
  result.node_indices = graph.findPath(
    result.start_node, result.goal_node, clearance,
    [&graph, &costmap, &edge_validity, clearance](std::size_t edge_id) {
      auto & valid = edge_validity.at(edge_id);
      if (valid < 0) {
        const auto & edge = graph.edges().at(edge_id);
        valid = segmentIsFree(
          costmap, graph.nodes().at(edge.source).position,
          graph.nodes().at(edge.target).position, clearance) ? 1 : 0;
      }
      return valid == 1;
    });
  if (result.node_indices.empty()) {
    throw std::runtime_error(
            "No valid graph path between nearest nodes " + std::to_string(first.id) + " and " +
            std::to_string(last.id) + " (disconnected, insufficient clearance, or blocked)");
  }
  result.waypoints.push_back(start);
  for (const auto index : result.node_indices) {
    result.waypoints.push_back(graph.nodes().at(index).position);
  }
  result.waypoints.push_back(goal);
  return result;
}

std::vector<Point2D> interpolatePolyline(
  const std::vector<Point2D> & waypoints, double spacing)
{
  if (!std::isfinite(spacing) || spacing <= 0.0) {
    throw std::runtime_error("interpolation_resolution must be finite and positive");
  }
  if (waypoints.empty()) {
    return {};
  }
  std::vector<Point2D> result{waypoints.front()};
  for (std::size_t i = 1; i < waypoints.size(); ++i) {
    const auto & a = waypoints[i - 1];
    const auto & b = waypoints[i];
    const double length = distance(a, b);
    if (length == 0.0) {
      continue;
    }
    const double count = std::ceil(length / spacing);
    if (!std::isfinite(count) || count + result.size() > 1000000) {
      throw std::runtime_error("Path interpolation exceeds one million poses; increase spacing");
    }
    const auto steps = static_cast<std::size_t>(count);
    for (std::size_t step = 1; step <= steps; ++step) {
      const double ratio = static_cast<double>(step) / steps;
      result.push_back(step == steps ? b :
        Point2D{a.x + ratio * (b.x - a.x), a.y + ratio * (b.y - a.y)});
    }
  }
  return result;
}
}  // namespace vas_planner
