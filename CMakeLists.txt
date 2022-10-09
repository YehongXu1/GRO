cmake_minimum_required(VERSION 3.21)
project(traffic_assignment)

set(CMAKE_CXX_STANDARD 23)

set(BOOST_ROOT "/usr/local/include")

find_package(Boost COMPONENTS thread system REQUIRED)
IF (Boost_FOUND)
    INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})
    ADD_DEFINITIONS("-DHAS_BOOST")
ENDIF ()

add_executable(traffic_assignment
        main.cpp
        baseline/KSPAlloc.h baseline/KSPAlloc.cpp
        KSP/completeness.cpp KSP/esx.cpp KSP/kspwlo.h KSP/multipass.cpp KSP/onepass.cpp KSP/onepass_plus.cpp KSP/skyline.cpp KSP/svp_plus.cpp
        model/Graph.h model/Graph.cpp model/Semaphore.h
        tools/tools.h tools/dijkstra.cpp tools/astar.cpp
        traffic/Traffic.h traffic/Framework.cpp traffic/TrafficMaintain.cpp traffic/TrafficMaintain.cpp traffic/PathSelection.cpp traffic/MainAlg.cpp model/DisGraph.cpp baseline/IJCAI.cpp baseline/IJCAI.h baseline/Baseline.cpp baseline/Baseline.h)

target_link_libraries(
        ${PROJECT_NAME}
        Boost::thread
)
