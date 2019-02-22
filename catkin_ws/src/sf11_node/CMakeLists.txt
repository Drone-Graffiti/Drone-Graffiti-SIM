cmake_minimum_required(VERSION 2.8.3)
project(sf11_node)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES s11_node
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME} src/sf11_node.cpp )
add_executable(${PROJECT_NAME}_i2c src/sf11_i2c_node.cpp )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_i2c ${catkin_LIBRARIES})


