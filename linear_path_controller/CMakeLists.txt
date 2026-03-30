cmake_minimum_required(VERSION 3.8)
project(linear_path_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ===== 依存関係 =====
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)

# ===== ライブラリ =====
add_library(linear_path_controller SHARED
  src/linear_path_controller.cpp
)


# target_include_directories(linear_path_controller PUBLIC
#   include
# )

ament_target_dependencies(linear_path_controller
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
  nav2_core
  nav2_costmap_2d
  tf2
  tf2_ros
  pluginlib
)

# ===== プラグイン登録 =====
pluginlib_export_plugin_description_file(nav2_core plugin.xml)

# ===== インストール =====
install(TARGETS linear_path_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


