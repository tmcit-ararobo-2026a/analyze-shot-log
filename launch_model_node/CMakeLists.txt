cmake_minimum_required(VERSION 3.10)
project(launch_model_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(launch_model_node src/launch_model_node.cpp)
target_include_directories(launch_model_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(launch_model_node PUBLIC cxx_std_17)
ament_target_dependencies(launch_model_node rclcpp std_msgs)

install(TARGETS launch_model_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_launch_solver test/test_launch_solver.cpp)
  target_include_directories(test_launch_solver PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_compile_features(test_launch_solver PUBLIC cxx_std_17)
endif()

ament_package()
