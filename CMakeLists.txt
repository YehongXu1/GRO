cmake_minimum_required(VERSION 3.21)
project(traffic_assignment)

set(CMAKE_CXX_STANDARD 23)

set(BOOST_ROOT "/usr/local/include")

FIND_PACKAGE(Boost)
IF (Boost_FOUND)
    INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})
    ADD_DEFINITIONS( "-DHAS_BOOST" )
ENDIF()

add_executable(traffic_assignment
        main.cpp
        model/graph.h model/graph.cpp
        tools/tools.h tools/dijkstra.cpp tools/astar.cpp
        KSP/completeness.cpp KSP/esx.cpp KSP/kspwlo.h KSP/multipass.cpp KSP/onepass.cpp KSP/onepass_plus.cpp KSP/skyline.cpp KSP/svp_plus.cpp
        rtree/ConfigTool.cpp rtree/ConfigTool.h rtree/GenericTool.h rtree/GenericTool.cpp rtree/mt19937ar.cpp rtree/mt19937ar.h rtree/PageFile.cpp rtree/PageFile.h rtree/RStarTree.h rtree/TimerTool.cpp rtree/TimerTool.h rtree/test.cpp traffic/Traffic.cpp traffic/Traffic.h)
