// Copyright 2026. Licensed under the BSD-3-Clause license.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <unistd.h>

#include "nlohmann/json.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "vas_planner/voronoi_planning.hpp"

namespace nsp = vas_planner;
using nsp::Point2D;

namespace
{
nav2_costmap_2d::Costmap2D freeMap()
{
  return nav2_costmap_2d::Costmap2D(100, 100, 0.1, 0.0, 0.0, 0);
}

nsp::VoronoiGraph squareGraph()
{
  nlohmann::json data = {
    {"format_version", 1}, {"directed", false}, {"map", {{"units", "m"}}},
    {"nodes", {
        {{"id", 10}, {"position", {1.0, 1.0}}},
        {{"id", 20}, {"position", {4.0, 1.0}}},
        {{"id", 30}, {"position", {4.0, 4.0}}},
        {{"id", 40}, {"position", {1.0, 4.0}}}}},
    {"edges", {
        {{"source", 10}, {"target", 20}, {"length", 3.0}, {"min_clearance", 2.0}},
        {{"source", 20}, {"target", 30}, {"length", 3.0}, {"min_clearance", 2.0}},
        {{"source", 30}, {"target", 40}, {"length", 3.0}, {"min_clearance", 2.0}},
        {{"source", 40}, {"target", 10}, {"length", 3.0}, {"min_clearance", 2.0}}}}};
  char filename[] = "/tmp/voronoi_plan_test_XXXXXX";
  const int fd = mkstemp(filename);
  if (fd < 0) {
    throw std::runtime_error("mkstemp failed");
  }
  close(fd);
  std::ofstream(filename) << data;
  nsp::VoronoiGraph graph;
  try {
    graph.load(filename);
  } catch (...) {
    std::remove(filename);
    throw;
  }
  std::remove(filename);
  return graph;
}
}  // namespace

TEST(CapsuleCollision, LongEdgeCannotSkipSingleCellObstacle)
{
  auto map = freeMap();
  map.setCost(50, 50, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_FALSE(nsp::segmentIsFree(map, {1.0, 5.05}, {9.0, 5.05}, 0.0));
  EXPECT_FALSE(nsp::segmentIsFree(map, {9.0, 5.05}, {1.0, 5.05}, 0.0));
}

TEST(CapsuleCollision, DetectsObstacleInsideFootprintAndAlongSegmentSide)
{
  auto map = freeMap();
  map.setCost(50, 50, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_FALSE(nsp::segmentIsFree(map, {5.3, 5.3}, {5.3, 5.3}, 0.6));
  EXPECT_FALSE(nsp::segmentIsFree(map, {1.0, 4.7}, {9.0, 4.7}, 0.35));
  EXPECT_TRUE(nsp::segmentIsFree(map, {1.0, 4.7}, {9.0, 4.7}, 0.25));
}

TEST(CapsuleCollision, TangentAndDiagonalCornerContactsAreBlocked)
{
  auto map = freeMap();
  map.setCost(50, 50, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_FALSE(nsp::segmentIsFree(map, {4.0, 4.0}, {4.0, 5.0}, 1.0));
  EXPECT_TRUE(nsp::segmentIsFree(map, {4.0, 4.0}, {4.0, 5.0}, 0.99));
  map.setCost(50, 50, 0);
  map.setCost(50, 49, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_FALSE(nsp::segmentIsFree(map, {1.0, 1.0}, {9.0, 9.0}, 0.0));
}

TEST(CapsuleCollision, UnknownExteriorAndInvalidInputsAreBlocked)
{
  auto map = freeMap();
  map.setCost(50, 50, nav2_costmap_2d::NO_INFORMATION);
  EXPECT_FALSE(nsp::segmentIsFree(map, {1.0, 5.05}, {9.0, 5.05}, 0.0));
  EXPECT_FALSE(nsp::segmentIsFree(map, {0.1, 1.0}, {2.0, 1.0}, 0.2));
  EXPECT_FALSE(nsp::segmentIsFree(map, {9.9, 1.0}, {8.0, 1.0}, 0.2));
  EXPECT_FALSE(nsp::segmentIsFree(map, {1.0, 1.0}, {2.0, 1.0}, -1.0));
  EXPECT_FALSE(nsp::segmentIsFree(
      map, {std::numeric_limits<double>::quiet_NaN(), 1.0}, {2.0, 1.0}, 0.0));
}

TEST(CapsuleCollision, DoesNotApplyInflationCostsTwice)
{
  auto map = freeMap();
  map.setCost(50, 50, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  EXPECT_TRUE(nsp::segmentIsFree(map, {1.0, 5.05}, {9.0, 5.05}, 0.2));
}

TEST(NearestNodePlanning, UsesGraphBetweenNearestNodesAndIncludesStartGoal)
{
  auto map = freeMap();
  const auto graph = squareGraph();
  const auto plan = nsp::planOnGraph(graph, map, {0.8, 1.0}, {4.2, 1.0}, 0.1);
  EXPECT_EQ(plan.start_node, 0u);
  EXPECT_EQ(plan.goal_node, 1u);
  EXPECT_EQ(plan.node_indices, (std::vector<std::size_t>{0, 1}));
  EXPECT_DOUBLE_EQ(plan.waypoints.front().x, 0.8);
  EXPECT_DOUBLE_EQ(plan.waypoints.back().x, 4.2);
}

TEST(NearestNodePlanning, CurrentCostmapBlocksShortestEdgeAndRoutesAround)
{
  auto map = freeMap();
  map.setCost(25, 10, nav2_costmap_2d::LETHAL_OBSTACLE);
  const auto graph = squareGraph();
  const auto plan = nsp::planOnGraph(graph, map, {0.8, 1.0}, {4.2, 1.0}, 0.1);
  EXPECT_EQ(plan.node_indices, (std::vector<std::size_t>{0, 3, 2, 1}));
  for (std::size_t i = 1; i < plan.waypoints.size(); ++i) {
    EXPECT_TRUE(nsp::segmentIsFree(map, plan.waypoints[i - 1], plan.waypoints[i], 0.1));
  }
  map.setCost(25, 40, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_THROW(nsp::planOnGraph(graph, map, {0.8, 1.0}, {4.2, 1.0}, 0.1), std::runtime_error);
}

TEST(NearestNodePlanning, RejectsBlockedConnectorsRatherThanJumpingWalls)
{
  auto map = freeMap();
  const auto graph = squareGraph();
  map.setCost(8, 10, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_THROW(nsp::planOnGraph(graph, map, {0.6, 1.0}, {4.2, 1.0}, 0.0), std::runtime_error);
  map.setCost(8, 10, 0);
  map.setCost(42, 10, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_THROW(nsp::planOnGraph(graph, map, {0.8, 1.0}, {4.5, 1.0}, 0.0), std::runtime_error);
}

TEST(NearestNodePlanning, SameNodeAndIdenticalPosesDoNotDivideByZero)
{
  auto map = freeMap();
  const auto graph = squareGraph();
  const auto plan = nsp::planOnGraph(graph, map, {0.8, 1.0}, {1.2, 1.0}, 0.0);
  EXPECT_EQ(plan.node_indices, (std::vector<std::size_t>{0}));
  const auto same = nsp::planOnGraph(graph, map, {0.8, 1.0}, {0.8, 1.0}, 0.0);
  const auto points = nsp::interpolatePolyline(same.waypoints, 0.05);
  ASSERT_EQ(points.size(), 1u);
  EXPECT_DOUBLE_EQ(points.front().x, 0.8);
}

TEST(NearestNodePlanning, ChecksStartFootprintAndConnectionLimit)
{
  auto map = freeMap();
  const auto graph = squareGraph();
  EXPECT_THROW(nsp::planOnGraph(graph, map, {0.8, 1.0}, {4.2, 1.0}, 0.1, 0.1), std::runtime_error);
  map.setCost(8, 10, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_THROW(nsp::planOnGraph(graph, map, {0.8, 1.0}, {4.2, 1.0}, 0.1), std::runtime_error);
}

TEST(PathInterpolation, PreservesBendsEndpointsAndMaximumSpacing)
{
  const std::vector<Point2D> input{{1.0, 1.0}, {1.0, 1.0}, {1.02, 1.0}, {1.02, 1.12}};
  const auto points = nsp::interpolatePolyline(input, 0.05);
  ASSERT_GT(points.size(), 2u);
  EXPECT_DOUBLE_EQ(points.front().x, 1.0);
  EXPECT_DOUBLE_EQ(points.back().y, 1.12);
  EXPECT_DOUBLE_EQ(points[1].x, 1.02);
  EXPECT_DOUBLE_EQ(points[1].y, 1.0);
  for (std::size_t i = 1; i < points.size(); ++i) {
    EXPECT_LE(std::hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y), 0.05 + 1e-12);
  }
  EXPECT_THROW(nsp::interpolatePolyline(input, 0.0), std::runtime_error);
  EXPECT_THROW(nsp::interpolatePolyline(input, 1e-20), std::runtime_error);
}
