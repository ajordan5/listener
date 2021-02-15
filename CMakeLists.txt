cmake_minimum_required(VERSION 3.4...3.18)
project(listener LANGUAGES CXX)

find_package(gazebo REQUIRED) 
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}") 

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})  

add_subdirectory(extern/pybind11)

add_library(listener
    src/listener.cpp
    src/listener.h
    )  
target_link_libraries(listener PRIVATE pybind11::embed ${GAZEBO_LIBRARIES} pthread) 

add_executable(listener_node src/listener_node.cpp)
target_link_libraries(listener_node PRIVATE pybind11::embed ${GAZEBO_LIBRARIES} listener)
