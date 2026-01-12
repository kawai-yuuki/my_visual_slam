cmake_minimum_required(VERSION 3.8)
project(my_visual_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# 実行ファイルの生成
add_executable(slam_main src/slam_main.cpp)

# ライブラリのリンク
ament_target_dependencies(slam_main
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
)

target_link_libraries(slam_main ${OpenCV_LIBS})

install(TARGETS slam_main
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
