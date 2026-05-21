cmake_minimum_required(VERSION 3.20)
project(classes_interfaces_patterns CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(classes_interfaces_patterns solution.cpp)
target_compile_options(classes_interfaces_patterns PRIVATE -Wall -Wextra -pedantic)
