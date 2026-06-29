cmake_minimum_required(VERSION 4.3)
project(io_handlers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pc_msgs REQUIRED)

# 寻找 Open3D C++ 库
list(APPEND CMAKE_PREFIX_PATH "/usr/local")
find_package(Open3D REQUIRED)

# 添加可执行文件
add_executable(sim_lidar_data_flow_node src/sim_lidar_data_flow_node.cpp)

# 包含 Open3D 的目录并链接库
target_include_directories(sim_lidar_data_flow_node PUBLIC ${Open3D_INCLUDE_DIRS})
target_link_libraries(sim_lidar_data_flow_node ${Open3D_LIBRARIES})
target_compile_features(sim_lidar_data_flow_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  sim_lidar_data_flow_node
  "rclcpp"
  "pc_msgs"
)

# 安装可执行文件
install(TARGETS
        sim_lidar_data_flow_node
        DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 和 config 文件夹，以保证 C++ 节点能按原工程规范读取
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
