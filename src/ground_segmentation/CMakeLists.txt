cmake_minimum_required(VERSION 3.8)
project(ground_segmentation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pc_msgs REQUIRED)
find_package(Open3D REQUIRED)

# 确保能找到源码编译并安装在 /usr/local 下的 Open3D
list(APPEND CMAKE_PREFIX_PATH "/usr/local")
add_executable(ransac_node src/ransac_node.cpp)
target_include_directories(ransac_node PUBLIC ${Open3D_INCLUDE_DIRS})
target_link_libraries(ransac_node ${Open3D_LIBRARIES})
ament_target_dependencies(ransac_node rclcpp pc_msgs)

install(TARGETS
        ransac_node
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
        launch
        config
        DESTINATION share/${PROJECT_NAME}
)

ament_target_dependencies(
  ransac_node
  "rclcpp"
  "pc_msgs"
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
