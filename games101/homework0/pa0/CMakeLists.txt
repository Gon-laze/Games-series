cmake_minimum_required (VERSION 2.8.11)
project (Transformation)

# Enable C++1
# added by Gon laze
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-std=c++17)

find_package(Eigen3 REQUIRED)
include_directories(EIGEN3_INCLUDE_DIR)

add_executable (Transformation main.cpp)


