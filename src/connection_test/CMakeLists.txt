cmake_minimum_required(VERSION 3.10)
project(connection_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17 -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pc_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(mesh_publisher src/mesh_publisher.cpp)
ament_target_dependencies(mesh_publisher rclcpp pc_msgs std_msgs)

add_executable(mesh_subscriber src/mesh_subscriber.cpp)
ament_target_dependencies(mesh_subscriber rclcpp pc_msgs std_msgs)

install(TARGETS
        mesh_publisher
        mesh_subscriber
        DESTINATION lib/${PROJECT_NAME}
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
