cmake_minimum_required(VERSION 3.17)
project(cgalabc C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

enable_testing()

include_directories(src)
include_directories(lib/olcPixelGameEngine)

add_subdirectory(src)
add_subdirectory(test)
add_subdirectory(lib/googletest)
