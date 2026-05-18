cmake_minimum_required(VERSION 3.20)
project(section2 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Домашні роботи. По мірі появи додаємо кожну окремим add_subdirectory.
add_subdirectory(homeworks/01_ballistic_task)
add_subdirectory(homeworks/02_drone_emulation)
add_subdirectory(homeworks/03_memory_basics)
add_subdirectory(homeworks/04_ugv_odometry)
add_subdirectory(homeworks/05_diagnostic_telemetry_parser)
add_subdirectory(homeworks/06_tests_code_quality)

enable_testing()