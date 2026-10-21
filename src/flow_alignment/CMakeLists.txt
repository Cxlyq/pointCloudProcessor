cmake_minimum_required(VERSION 3.10)
project(flow_alignment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17 -O3)
endif()

# ---- 依赖（与 reconstruction 包一致的链接方式）----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pc_msgs REQUIRED)
find_package(Open3D REQUIRED)
find_package(Eigen3 REQUIRED)

# 确保能找到安装在 /usr/local 下的 Open3D
list(APPEND CMAKE_PREFIX_PATH "/usr/local")

# ---- 节点可执行 ----
add_executable(flow_alignment_node src/flow_alignment_node.cpp)
ament_target_dependencies(
        flow_alignment_node
        "rclcpp"
        "std_msgs"
        "pc_msgs"
)
target_include_directories(flow_alignment_node PUBLIC
        ${Open3D_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(flow_alignment_node ${Open3D_LIBRARIES})
target_compile_features(flow_alignment_node PUBLIC c_std_99 cxx_std_17)

install(TARGETS
        flow_alignment_node
        DESTINATION lib/${PROJECT_NAME}
)

# ---- 运行时文件：参数 yaml + launch ----
install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
