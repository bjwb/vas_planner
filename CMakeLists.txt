cmake_minimum_required(VERSION 3.5)
project(vas_planner)

# Default to C99
set(CMAKE_C_STANDARD 99)


# Default to C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(
  include
)

set(library_name ${PROJECT_NAME}_plugin)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  visualization_msgs
  nav2_util
  nav2_msgs
  nav_msgs
  geometry_msgs
  builtin_interfaces
  tf2_ros
  nav2_costmap_2d
  nav2_core
  pluginlib
)

add_library(${library_name} SHARED
  src/vas_planner.cpp
  src/voronoi_graph.cpp
  src/voronoi_planning.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)
target_link_libraries(${library_name} nlohmann_json::nlohmann_json)
target_compile_options(${library_name} PRIVATE -Wall -Wextra -Wpedantic)

target_compile_definitions(${library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")


pluginlib_export_plugin_description_file(nav2_core global_planner_plugin.xml)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES global_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_voronoi_graph test/test_voronoi_graph.cpp)
  target_link_libraries(test_voronoi_graph ${library_name} nlohmann_json::nlohmann_json)
  ament_add_gtest(test_voronoi_planning test/test_voronoi_planning.cpp)
  ament_target_dependencies(test_voronoi_planning nav2_costmap_2d)
  target_link_libraries(test_voronoi_planning ${library_name} nlohmann_json::nlohmann_json)
endif()


ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_export_dependencies(nlohmann_json)
ament_package()
