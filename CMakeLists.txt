cmake_minimum_required(VERSION 3.5)
project(nav2_gridmap_costmap_plugin)

set(lib_name ${PROJECT_NAME}_core)

# === Environment ===

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === Dependencies ===

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)

find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)

set(dep_pkgs
    rclcpp
    nav2_costmap_2d
    tf2 
    geometry_msgs 
    visualization_msgs
    nav_msgs
    nav2_util
    grid_map_msgs
    grid_map_ros
    std_msgs
  )


foreach(pkg IN LISTS dep_pkgs)
  find_package(${pkg} REQUIRED)
endforeach()

# === Build ===

add_library(${lib_name} SHARED
            src/gridmap_layer.cpp
            src/gridmap_handler.cpp)
include_directories(include)

# === Installation ===

install(TARGETS ${lib_name}
        DESTINATION lib)

# === Ament work ===

pluginlib_export_plugin_description_file(nav2_costmap_2d gridmap_layer.xml)
ament_target_dependencies(${lib_name} ${dep_pkgs})
ament_package()
