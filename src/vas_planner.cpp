/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 Shivang Patel
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Shivang Patel
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_nav2planner_plugin.html
 *********************************************************************/

#include "vas_planner/vas_planner.hpp"
#include "vas_planner/voronoi_planning.hpp"

#include <algorithm>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <utility>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace vas_planner
{

void VasPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  if (!node_ || !costmap_ros) {
    throw std::runtime_error("vas_planner requires a live parent node and costmap");
  }
  name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();
  const auto parameter = [this](const std::string & key, const rclcpp::ParameterValue & value) {
      nav2_util::declare_parameter_if_not_declared(node_, name_ + "." + key, value);
    };
  parameter("graph_file", rclcpp::ParameterValue(std::string("")));
  parameter("graph_frame", rclcpp::ParameterValue(std::string("map")));
  parameter("interpolation_resolution", rclcpp::ParameterValue(0.05));
  parameter("robot_radius", rclcpp::ParameterValue(-1.0));
  parameter("safety_margin", rclcpp::ParameterValue(-1.0));
  parameter("max_connection_distance", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_ + ".graph_file", graph_file_);
  node_->get_parameter(name_ + ".graph_frame", graph_frame_);
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
  node_->get_parameter(name_ + ".robot_radius", robot_radius_);
  node_->get_parameter(name_ + ".safety_margin", safety_margin_);
  node_->get_parameter(name_ + ".max_connection_distance", max_connection_distance_);
  if (graph_file_.empty()) {
    throw std::runtime_error(name_ + ".graph_file is required; use an absolute path to a Voronoi JSON file");
  }
  if (graph_frame_ != global_frame_) {
    throw std::runtime_error("graph_frame must match the global costmap frame: " + global_frame_);
  }
  const auto valid_auto_distance = [](double value) {
      return std::isfinite(value) && (value == -1.0 || value >= 0.0);
    };
  if (!std::isfinite(interpolation_resolution_) || interpolation_resolution_ <= 0.0 ||
    !valid_auto_distance(robot_radius_) || !valid_auto_distance(safety_margin_) ||
    !std::isfinite(max_connection_distance_) || max_connection_distance_ < 0.0)
  {
    throw std::runtime_error("Invalid vas_planner distance parameters; see config/vas_planner.yaml");
  }
  graph_.load(graph_file_);
  RCLCPP_INFO(
    node_->get_logger(), "Loaded Voronoi JSON: %zu nodes, %zu undirected edges from %s",
    graph_.nodes().size(), graph_.edges().size(), graph_file_.c_str());
}

void VasPlanner::cleanup()
{
  graph_ = VoronoiGraph();
  costmap_ = nullptr;
  costmap_ros_.reset();
  tf_.reset();
  RCLCPP_INFO(node_->get_logger(), "Cleaned up vas_planner %s", name_.c_str());
}

void VasPlanner::activate()
{
  RCLCPP_INFO(node_->get_logger(), "Activated vas_planner %s", name_.c_str());
}

void VasPlanner::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "Deactivated vas_planner %s", name_.c_str());
}

nav_msgs::msg::Path VasPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  if (!node_ || !costmap_) {
    return path;
  }
  path.header.stamp = node_->now();
  try {
    if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_) {
      throw std::runtime_error("Start and goal must be in the global costmap frame: " + global_frame_);
    }
    const auto & q = goal.pose.orientation;
    const double qnorm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
    if (!std::isfinite(qnorm) || qnorm < 1e-12) {
      throw std::runtime_error("Goal orientation must be a finite, nonzero quaternion");
    }
    GraphPlan plan;
    double clearance = 0.0;
    {
      // Use one consistent costmap for the connectors and the entire A* search.
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());
      const double footprint_radius = costmap_ros_->getLayeredCostmap()->getCircumscribedRadius();
      const double radius = std::max(
        robot_radius_ < 0.0 ? graph_.robotRadius() : robot_radius_, footprint_radius);
      const double margin = safety_margin_ < 0.0 ? graph_.safetyMargin() : safety_margin_;
      clearance = radius + margin;
      plan = planOnGraph(
        graph_, *costmap_, Point2D{start.pose.position.x, start.pose.position.y},
        Point2D{goal.pose.position.x, goal.pose.position.y}, clearance, max_connection_distance_);
    }
    const auto points = interpolatePolyline(plan.waypoints, interpolation_resolution_);
    path.poses.reserve(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = points[i].x;
      pose.pose.position.y = points[i].y;
      if (i + 1 < points.size()) {
        const double yaw = std::atan2(points[i + 1].y - points[i].y, points[i + 1].x - points[i].x);
        pose.pose.orientation.z = std::sin(yaw * 0.5);
        pose.pose.orientation.w = std::cos(yaw * 0.5);
      } else {
        pose.pose.orientation.x = q.x / qnorm;
        pose.pose.orientation.y = q.y / qnorm;
        pose.pose.orientation.z = q.z / qnorm;
        pose.pose.orientation.w = q.w / qnorm;
      }
      path.poses.push_back(pose);
    }
    RCLCPP_INFO(
      node_->get_logger(),
      "vas_planner: node %lld -> node %lld, %zu graph nodes, %zu poses, clearance %.3f m",
      static_cast<long long>(graph_.nodes().at(plan.start_node).id),
      static_cast<long long>(graph_.nodes().at(plan.goal_node).id),
      plan.node_indices.size(), path.poses.size(), clearance);
  } catch (const std::exception & error) {
    RCLCPP_WARN(node_->get_logger(), "vas_planner planning failed: %s", error.what());
    path.poses.clear();
  }
  return path;
}

}  // namespace vas_planner
PLUGINLIB_EXPORT_CLASS(vas_planner::VasPlanner, nav2_core::GlobalPlanner)
