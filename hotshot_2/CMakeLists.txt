cmake_minimum_required(VERSION 3.0.2)
project(hotshot_2)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gb_visual_detection_3d_msgs
  message_generation
  roscpp
  std_msgs
)

generate_messages(DEPENDENCIES std_msgs)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(hotshot_distance_node src/hotshot_distance_node.cpp)
target_link_libraries(hotshot_distance_node ${catkin_LIBRARIES})
