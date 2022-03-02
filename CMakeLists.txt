cmake_minimum_required(VERSION 3.21)
project(traffic_assignment)

set(CMAKE_CXX_STANDARD 23)

set(BOOST_ROOT "/usr/local/include")

FIND_PACKAGE(Boost 1.73.0 COMPONENTS system filesystem thread REQUIRED)
IF (Boost_FOUND)
    INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})
    ADD_DEFINITIONS("-DHAS_BOOST")
ENDIF ()

add_executable(traffic_assignment
        main.cpp
        baseline/KSPAlloc.h baseline/KSPAlloc.cpp
        KSP/completeness.cpp KSP/esx.cpp KSP/kspwlo.h KSP/multipass.cpp KSP/onepass.cpp KSP/onepass_plus.cpp KSP/skyline.cpp KSP/svp_plus.cpp
        model/graph.h model/graph.cpp
        tools/tools.h tools/dijkstra.cpp tools/astar.cpp
        traffic/Routing.h traffic/Routing.cpp traffic/Traffic.cpp traffic/Traffic.cpp
        )
