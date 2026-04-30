cmake_minimum_required(VERSION 3.20)
project(drone_emulation CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(drone_emulation solution.cpp)
target_compile_options(drone_emulation PRIVATE -Wall -Wextra -pedantic)
