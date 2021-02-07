cmake_minimum_required(VERSION 3.4...3.18)
project(listener LANGUAGES CXX)

find_package(gazebo REQUIRED)  

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}") 

# include appropriate directories
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})  

add_subdirectory(extern/pybind11)
pybind11_add_module(bind_listener bind_listener.cpp)
# pybind11_add_module(ilist independent_listener.cpp)
# target_link_libraries(ilist PUBLIC ${GAZEBO_LIBRARIES} pthread) 

add_executable(independent_listener independent_listener.cpp)  
target_link_libraries(independent_listener PRIVATE pybind11::embed ${GAZEBO_LIBRARIES} pthread) 