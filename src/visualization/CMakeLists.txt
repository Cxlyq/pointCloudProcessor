cmake_minimum_required(VERSION 3.10)
project(visualization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17 -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pc_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Threads REQUIRED) # 线程库
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc video)
list(APPEND CMAKE_PREFIX_PATH "/usr/local")
find_package(Open3D REQUIRED)


add_executable(visualization_node
        src/visualization_node.cpp
        src/dis_frame_interpolator.cpp
)

target_include_directories(visualization_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(visualization_node PUBLIC
        ${Open3D_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(visualization_node
        ${Open3D_LIBRARIES}
        ${OpenCV_LIBS}
        Threads::Threads
)
target_compile_features(visualization_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  visualization_node
  "rclcpp"
  "pc_msgs"
  "sensor_msgs"
)

add_executable(interpolation_display_node
        src/interpolation_display_node.cpp
)
target_include_directories(interpolation_display_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(interpolation_display_node
        ${OpenCV_LIBS}
)
target_compile_features(
  interpolation_display_node PUBLIC cxx_std_17)
ament_target_dependencies(
  interpolation_display_node
  "rclcpp"
  "sensor_msgs"
)

install(TARGETS
  visualization_node
  interpolation_display_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
        launch
        config
        DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(dis_frame_interpolator_test
    test/dis_frame_interpolator_test.cpp
    src/dis_frame_interpolator.cpp
  )
  target_include_directories(dis_frame_interpolator_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(dis_frame_interpolator_test
    ${OpenCV_LIBS}
    Threads::Threads
  )
  target_compile_features(
    dis_frame_interpolator_test PUBLIC cxx_std_17)

  ament_add_gtest(ros_image_conversion_test
    test/ros_image_conversion_test.cpp
  )
  target_include_directories(ros_image_conversion_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(ros_image_conversion_test
    ${OpenCV_LIBS}
  )
  target_compile_features(
    ros_image_conversion_test PUBLIC cxx_std_17)
  ament_target_dependencies(
    ros_image_conversion_test
    "sensor_msgs"
  )

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
