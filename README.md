# vas_planner

`vas_planner` is a C++ global planner for **ROS 2 Humble / Nav2**. It loads the roadmap JSON produced by `pgm_voronoi.py` and searches its nodes and edges with A*.

```text
robot pose -> nearest graph node -> A* graph route -> nearest goal node -> goal pose
```

The two attachment nodes are chosen independently by Euclidean distance in the map frame. A* minimizes the sum of JSON edge `length` values between those fixed nodes. This is the shortest valid route on the supplied graph, not necessarily the shortest route through all free space. The costmap is used for collision checking; it is not searched with grid A*.

## Build

From a terminal with ROS 2 Humble and your Nav2 installation sourced:

```bash
source /opt/ros/humble/setup.bash
# Source this overlay if Nav2 was built here on your machine:
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws
colcon build --packages-select vas_planner --symlink-install
source install/setup.bash
```

## Dependencies

Requires ROS 2 Humble / Nav2. The dependencies used by [package.xml](package.xml) and [CMakeLists.txt](CMakeLists.txt) are:

| Group | Dependencies |
| --- | --- |
| Build tools | C++14 compiler, CMake 3.5 or newer, `colcon`, `ament_cmake` |
| ROS 2 C++ libraries | `rclcpp`, `rclcpp_action`, `rclcpp_lifecycle` |
| Nav2 | `nav2_core`, `nav2_costmap_2d`, `nav2_util`, `nav2_msgs` |
| ROS messages | `std_msgs`, `visualization_msgs`, `nav_msgs`, `geometry_msgs`, `builtin_interfaces` |
| Transforms and plugins | `tf2_ros`, `pluginlib` |
| JSON | `nlohmann-json3-dev` (provides the CMake package `nlohmann_json`) |
| Tests only | `ament_cmake_gtest` |

After sourcing ROS 2, install the declared package dependencies from your workspace root before building:

```bash
cd ~/ros2_ws
rosdep install --from-paths "$(colcon list --packages-select vas_planner --paths-only)" \
  --ignore-src -r -y
```

## Nav2 configuration

Merge [config/vas_planner.yaml](config/vas_planner.yaml) into the **existing Nav2 parameter file used by your launch**:

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "vas_planner/VasPlanner"
      graph_file: "/absolute/path/to/voronoi_graph.json" # Required: pre-generated Voronoi graph JSON
      graph_frame: "map"
      interpolation_resolution: 0.05
      robot_radius: -1.0
      safety_margin: -1.0
      max_connection_distance: 0.0
```

A pre-generated Voronoi graph JSON is **required**. Replace the example `graph_file` value with the absolute path to your existing JSON file. The planner fails to configure if the file is missing or invalid; it does not generate the graph automatically.

This YAML is a planner configuration fragment, not a complete Nav2 bringup configuration. Keep the robot's map server, localization, costmaps, controller, behavior tree, and other settings. Restart or clean up/reconfigure `planner_server` after changing the JSON or these parameters. The graph is loaded once during configuration; parameter changes are not applied dynamically.

`GridBased` is the planner instance ID, retained because standard Nav2 behavior trees commonly request that ID. The implementation behind it is graph A*. If you change the ID, update the behavior tree or `ComputePathToPose` request as well.

The plugin ID is `vas_planner/VasPlanner`, backed by the C++ class `vas_planner::VasPlanner`.

The JSON coordinates and the global costmap must describe the same map with the same metric origin. `graph_frame` must match the costmap's global frame. Merely setting the frame name does not transform or align graph coordinates. Generate the graph from the same map YAML used by Nav2 so the resolution and origin match.

## Parameters

| Parameter | Default | Meaning |
| --- | --- | --- |
| `graph_file` | Required | Path to the Voronoi JSON; use an absolute path. |
| `graph_frame` | `map` | Coordinate frame of JSON node positions. Must equal the global costmap frame. |
| `interpolation_resolution` | `0.05` | Maximum spacing between output poses, in meters. Must be positive. |
| `robot_radius` | `-1.0` | Use `parameters.robot_radius` from JSON when `-1`; otherwise use the specified nonnegative radius in meters. The current costmap footprint's circumscribed radius is always a lower bound. |
| `safety_margin` | `-1.0` | Use `parameters.safety_margin` from JSON when `-1`; otherwise use the specified nonnegative margin in meters. |
| `max_connection_distance` | `0.0` | Maximum distance for each start/goal connector, in meters. `0` disables this distance limit. Collision checks still apply. |

If the JSON has no radius/margin metadata, their inherited values are zero. The effective collision-checking clearance is:

```text
max(selected robot radius, current costmap footprint circumscribed radius)
+ selected safety margin
```

For example, JSON values `robot_radius: 0.2` and `safety_margin: 0.05` give a 0.25 m clearance when the current costmap footprint's circumscribed radius is at most 0.2 m. A larger footprint increases clearance automatically. The costmap footprint can already include `footprint_padding`.

## Planning behavior

1. Nav2 supplies the start and goal in the global costmap frame. For normal navigation, the start is the localized robot pose.
2. Check both endpoint footprints and find the nearest node to each endpoint.
3. Check the straight segments from start to its nearest node and from the goal's nearest node to goal.
4. Run A* over the undirected JSON graph. Reject edges with insufficient stored `min_clearance` or a collision in the current global costmap. A blocked edge can therefore cause A* to choose another graph route.
5. Join start, graph nodes, and goal; interpolate without cutting corners. Return `nav_msgs/msg/Path` with tangent orientations and the requested final goal orientation.

Collision checking covers the **entire segment swept by a circle**, including cell interiors and boundaries. Lethal cells (`254`), unknown cells (`255`), and space outside the current costmap are blocked. Inflation costs below `254` do not add a cost penalty or a second obstacle expansion; the explicit footprint radius and margin provide clearance. The planner locks the costmap during the search so each request uses a consistent snapshot.

If a nearest-node connector crosses a wall, the selected nodes are disconnected, or no route passes clearance/collision checks, the planner returns an empty path and logs the reason. It does not attach to a different node or fall back to grid search. When start and goal share a nearest node, the path goes through that node. Identical start/goal positions return a single goal pose after checking the footprint.

For polygonal robots, the circumscribed circle is conservative and may reject narrow passages that would fit at a particular orientation. This planner produces a geometric path and does not model a minimum turning radius. The local controller remains responsible for following the path and reacting to changes after planning.

## Visualize or request a path

Run your usual Nav2 bringup with the merged parameter file, localize the robot, and set a goal in RViz. Add a `Path` display on the planner server's `plan` topic, commonly `/plan` (namespace dependent).

To request a path without executing navigation, call `ComputePathToPose` after Nav2 and localization are active. Substitute a valid goal in your map:

```bash
ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 2.0}, orientation: {w: 1.0}}}, planner_id: GridBased, use_start: false}"
```

`use_start: false` asks Nav2 to use the current robot pose. This action computes a path; sending a navigation goal in RViz also asks the robot to follow it.

## JSON contract

Expected fields are `format_version: 1`, `directed: false`, `map.units: "m"`, `nodes`, and `edges`. Nodes contain a unique integer `id` and a metric `position: [x, y]`; IDs need not be contiguous. Edges contain `source`, `target`, positive `length`, and nonnegative `min_clearance`. Endpoints reference node IDs. Both directions are traversable.

Pixel-only graphs are rejected. Generate metric coordinates from the map YAML or supply the correct resolution and origin to the graph generator. The planner consumes the JSON and Nav2's current costmap; it does not read the PGM or map YAML itself.

## Tests

```bash
cd ~/ros2_ws
colcon test --packages-select vas_planner --event-handlers console_direct+
colcon test-result --verbose
```

Tests cover JSON validation, sparse IDs, shortest-path selection, bidirectional and disconnected graphs, edge filtering, nearest-node connectors, dynamic edge blockage, footprint clearance, unknown/outside cells, and interpolation. Tests use synthetic costmaps and do not move a robot.
=======
# vas_planner
>>>>>>> origin/main
