cmake_minimum_required(VERSION 3.20)
project(ballistic_task CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(ballistic_task solution.cpp)
target_compile_options(ballistic_task PRIVATE -Wall -Wextra -pedantic)
