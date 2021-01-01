# Cmake project name is defined in parent cmakelists "project(cgalabc)"
set (BINARY ${CMAKE_PROJECT_NAME})

set (SOURCES
        olcPixelGameEngine.cpp
        type.cpp
        Point2d.cpp
        Line2d.cpp
        Algo2d.cpp
        Polygon.cpp
        BezierCurve.cpp
        CatmullRomCurve.cpp
        Camera.cpp
        Mesh.cpp
        Algo3d.cpp
        Utility.cpp
        Pose.cpp
        Material.cpp
        Light.cpp
        Shader.cpp
)

set (HEADERS
        include/type.h
        include/Point2d.h
        include/Line2d.h
        include/Algo2d.h
        include/Polygon.h
        include/Curve2d.h
        include/BezierCurve.h
        include/CatmullRomCurve.h
        include/Camera.h
        include/Mesh.h
        include/Algo3d.h
        include/Utility.h
        include/Pose.h
        include/Material.h
        include/Light.h
        include/Shader.h
)

# Create library object so it can be linked with in tests
add_library(${BINARY}_lib STATIC ${SOURCES} ${HEADERS})
link_libraries(${BINARY}_lib png X11 GL pthread)

# for live code, as opposed to unit testing
add_executable(${BINARY}_shadowcast ShadowCast.cpp)
add_executable(${BINARY}_playground Playground.cpp)
