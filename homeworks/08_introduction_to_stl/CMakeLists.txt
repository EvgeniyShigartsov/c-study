cmake_minimum_required(VERSION 3.20)
project(introduction_to_stl CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(
    introduction_to_stl
    src/main.cpp
    src/config/ComponentFactory.cpp
    src/config/FileConfigLoader.cpp
    src/providers/JsonTargetProvider.cpp
    src/solvers/AnalyticalSolver.cpp
    src/utils/MathUtils.cpp
    src/MissionProcessor.cpp)

target_include_directories(introduction_to_stl PRIVATE include)
target_compile_options(introduction_to_stl PRIVATE -Wall -Wextra -pedantic)
