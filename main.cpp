#include <iostream>

#include "traffic/Traffic.h"
#include "KSP/kspwlo.h"
#include <random>

using namespace std;

int main()
{
    string map = "/Users/xyh/Desktop/traffic-assignment/data/BJ_minTravelTime.txt";
    string coords = "/Users/xyh/Desktop/traffic-assignment/data/BJ_NodeIDLonLat.txt";
    string odpath = "/Users/xyh/Desktop/traffic-assignment/data/BJ_ODs.txt";
    string path = "/Users/xyh/Desktop/traffic-assignment/output/exp1/";

    RoadNetwork rN(map.c_str(), coords.c_str());
    Coord sCenter(116.207, 40.091);
    Coord tCenter(116.518, 39.8754);
    Traffic traffic(rN, sCenter, tCenter, 500, 500, 10);
    /* initialization */
    traffic.writeSetting(path + "setting.txt", "BJ");

    Simulation simulation(traffic, 288, 300);
    cout << simulation.rerouteAllPaths() << endl;

//    simulation.reroutePartialBlockETs();
//    simulation.writeTrajectories(path + "sp_trajecs.csv");
//    simulation.writeEdgeFlowDist(path + "sps_edgeflow.txt");

//    simulation.reroutePartialBlockETs();
//    simulation.writeTrajectories(path + "rerouteByBlocking.csv");
//    simulation.writeEdgeFlowDist(path + "rerouteByBlocking_edgeflow.txt");
    return 0;
}
