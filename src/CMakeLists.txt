# Cmake project name is defined in parent cmakelists "project(cgalabc)"
set (BINARY ${CMAKE_PROJECT_NAME})

set (SOURCES
        type.cpp
        Point2d.cpp
        Line2d.cpp
        Algo2d.cpp
        Polygon.cpp
)

set (HEADERS
        type.h
        Point2d.h
        Line2d.h
        Algo2d.h
        Polygon.h
)

# Create library object so it can be linked with in tests
add_library(${BINARY}_lib STATIC ${SOURCES} ${HEADERS})

# for live code, as opposed to unit testing
add_executable(${BINARY}_main main.cpp)
target_link_libraries(${BINARY}_main ${BINARY}_lib)
