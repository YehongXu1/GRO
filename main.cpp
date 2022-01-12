#include <iostream>

#include "traffic/Traffic.h"
#include "KSP/kspwlo.h"

using namespace std;

int main()
{
    string map = "/Users/xyh/Desktop/traffic-assignment/data/BJ_RoadIDWeight.txt";
    string coords = "/Users/xyh/Desktop/traffic-assignment/data/BJ_NodeIDLonLat.txt";
    string odpath = "/Users/xyh/Desktop/traffic-assignment/data/BJ_ODs.txt";
    string path = "/Users/xyh/Desktop/traffic-assignment/output/exp1/";

    RoadNetwork rN(map.c_str(), coords.c_str());
    Traffic traffic(rN, odpath, 5);
    cout << 1;
//    /* initialization */
    traffic.writeSetting(path + "setting.txt");
//
    Simulation simulation(traffic, 1000, 100);
    simulation.basicSimulation();

    cout << simulation.getCost() << endl;
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
