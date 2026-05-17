cmake_minimum_required(VERSION 3.20)
project(tests_code_quality CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(ballisticts src/ballisticts.cpp)
target_include_directories(ballisticts PUBLIC include)

add_executable(tests_code_quality src/main.cpp)
target_link_libraries(tests_code_quality PRIVATE ballisticts)

target_compile_options(ballisticts PRIVATE -Wall -Wextra -pedantic)
target_compile_options(tests_code_quality PRIVATE -Wall -Wextra -pedantic)
