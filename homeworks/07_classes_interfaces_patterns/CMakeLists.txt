cmake_minimum_required(VERSION 3.20)
project(memory_basics CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(memory_basics solution.cpp)
target_compile_options(memory_basics PRIVATE -Wall -Wextra -pedantic)
