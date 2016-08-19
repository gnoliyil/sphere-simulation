cmake_minimum_required(VERSION 3.5)
project(sphere_simulation)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Ofast")
set(CMAKE_CXX_COMPILER "/opt/intel/bin/icc")
add_definitions("-DdDOUBLE")

set(SOURCE_FILES
    axisorientationsensor.cpp
    sphere.cpp
    conceptordep.cpp
    dep.cpp
    main.cpp)

set(LPZROBOTS_DIR /home/gnoliyil/tools/lpzrobots)

include_directories("/usr/local/include")
include_directories("${LPZROBOTS_DIR}/ode_robots/include")

add_executable(start ${SOURCE_FILES})

link_directories("${LPZROBOTS_DIR}/selforg")
link_directories("${LPZROBOTS_DIR}/ode_robots")
link_directories("/usr/local/lib")

add_library(ode_robots STATIC IMPORTED)
set_target_properties(ode_robots PROPERTIES IMPORTED_LOCATION
        ${LPZROBOTS_DIR}/ode_robots/libode_robots.a)
add_library(selforg STATIC IMPORTED)
set_target_properties(selforg PROPERTIES IMPORTED_LOCATION
        ${LPZROBOTS_DIR}/selforg/libselforg.a)

target_link_libraries(start ode_robots m osgShadow osgText osgUtil osgViewer osgGA osgDB
        OpenThreads osg GL GLU glut pthread ode_dbl readline ncurses selforg gsl gslcblas)

