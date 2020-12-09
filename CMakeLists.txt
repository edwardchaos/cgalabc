cmake_minimum_required(VERSION 3.17)
project(cgalabc)

set(CMAKE_CXX_STANDARD 14)

# Build my library as shared, does not install it to system.
add_library(cgalabc SHARED cgalabc.cpp cgalabc.hpp)

add_executable(main main.cpp)
target_link_libraries(main cgalabc)