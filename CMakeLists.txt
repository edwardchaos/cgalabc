cmake_minimum_required(VERSION 3.17)
project(cgalabc C CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

enable_testing()

# Build my library as shared, does not install it to system.
add_library(cgalabc SHARED cgalabc.cpp cgalabc.hpp)

add_executable(main main.cpp)
target_link_libraries(main cgalabc)