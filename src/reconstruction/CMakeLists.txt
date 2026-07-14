cmake_minimum_required(VERSION 3.10)
project(reconstruction)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17 -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(pc_msgs REQUIRED)
find_package(Open3D REQUIRED)
# 查找 Eigen3
find_package(Eigen3 REQUIRED)

# 确保能找到安装在 /usr/local 下的 Open3D
list(APPEND CMAKE_PREFIX_PATH "/usr/local")


add_executable(reconstruction_node src/reconstruction_node.cpp)
ament_target_dependencies(
        reconstruction_node
        "rclcpp"
        "message_filters"
        "pc_msgs"
)
target_include_directories(reconstruction_node PUBLIC
        ${Open3D_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(reconstruction_node ${Open3D_LIBRARIES})
target_compile_features(reconstruction_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
install(TARGETS
        reconstruction_node
        DESTINATION lib/${PROJECT_NAME}
)


add_executable(reconstruction_g_poisson_ng_qh_node src/reconstruction_g_poisson_ng_qh_node.cpp)
ament_target_dependencies(
        reconstruction_g_poisson_ng_qh_node
        "rclcpp"
        "pc_msgs"
        "message_filters"
)
target_include_directories(reconstruction_g_poisson_ng_qh_node PUBLIC
        ${Open3D_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(reconstruction_g_poisson_ng_qh_node ${Open3D_LIBRARIES})
install(TARGETS
        reconstruction_g_poisson_ng_qh_node
        DESTINATION lib/${PROJECT_NAME}
)

add_executable(reconstruction_g_dln_ng_qh_node src/reconstruction_g_dln_ng_qh_node.cpp)
ament_target_dependencies(
        reconstruction_g_dln_ng_qh_node
        "rclcpp"
        "message_filters"
        "pc_msgs"
)
target_include_directories(reconstruction_g_dln_ng_qh_node PUBLIC
        ${Open3D_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(reconstruction_g_dln_ng_qh_node ${Open3D_LIBRARIES})
target_compile_features(reconstruction_g_dln_ng_qh_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
install(TARGETS
        reconstruction_g_dln_ng_qh_node
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
        launch
        config
        DESTINATION share/${PROJECT_NAME}
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
