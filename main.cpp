#include <iostream>

#include "traffic/Traffic.h"
#include "KSP/kspwlo.h"

using namespace std;

int main()
{
    string map = "/Users/xyh/Desktop/traffic-assignment/data/BJ_RoadIDWeight.txt";
    string coords = "/Users/xyh/Desktop/traffic-assignment/data/BJ_NodeIDLonLat.txt";
    string path = "/Users/xyh/Desktop/traffic-assignment/output/exp1/";

    RoadNetwork rN(map.c_str(), coords.c_str());

    /* initialization */
//    int radius = 500;
//    Coord sCenter = make_pair(40.803655, -73.919705);
//    Coord tCenter = make_pair(40.950238, -73.884295);
//    Traffic traffic(rN, sCenter, tCenter, radius, radius, 5);
//    traffic.writeSetting(path + "setting.txt");

//    Simulation simulation(traffic);
//    simulation.basicSimulation();
//
//    cout << simulation.getCost() << endl;
//
//    simulation.reroutePartialReqsByBlocking();
//    cout << simulation.getCost() << endl;

//    simulation.writeTrajectories(path + "sp_trajecs.csv");
//    simulation.writeEdgeFlowDist(path + "sps_edgeflow.txt");

//    simulation.reroutePartialReqsByBlocking();
//    simulation.writeTrajectories(path + "rerouteByBlocking.csv");
//    simulation.writeEdgeFlowDist(path + "rerouteByBlocking_edgeflow.txt");
    return 0;
}
