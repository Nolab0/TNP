cmake_minimum_required(VERSION 3.5)
project(tp3)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -O3")

include_directories(eigen-3.4.0 src)
add_executable(ransac
        src/ransac.cpp
        src/obj.cpp
        src/kdtree.cpp)
