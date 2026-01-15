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
add_executable(kitti_player src/kitti_player.cpp)

# ライブラリのリンク
ament_target_dependencies(kitti_player
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
)
target_link_libraries(kitti_player ${OpenCV_LIBS})

add_executable(euroc_player src/euroc_player.cpp)
ament_target_dependencies(euroc_player
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
)
target_link_libraries(euroc_player ${OpenCV_LIBS})

install(TARGETS kitti_player euroc_player
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
