cmake_minimum_required(VERSION 3.5)
project(BCD_Planner)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)
include_directories(OpenCV_INCLUDE_DIRS)

#add_executable(BCD_Planner main.cpp a-star.h)
add_executable(BCD_Planner main.cpp)
target_link_libraries(BCD_Planner ${OpenCV_LIBS})