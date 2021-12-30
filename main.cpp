#include <iostream>

#include "traffic/Traffic.h"
#include "KSP/kspwlo.h"

using namespace std;

int main()
{
    string map = "/Users/xyh/Desktop/traffic-assignment/data/NY.txt";
    string coords = "/Users/xyh/Desktop/traffic-assignment/data/NY_coord.txt";
    string path = "/Users/xyh/Desktop/traffic-assignment/output/exp2";

    RoadNetwork rN(map.c_str(), coords.c_str());

    /* initialization */
    int radius = 200;
    Coord sCenter = make_pair(40.803655, -73.919705);
    Coord tCenter = make_pair(40.950238, -73.884295);
    vector<unordered_map<NodeId, pair<vector<Interval *>, int>>> edgeFlowDist(rN.numNodes + 1);
    vector<Label *> trajs;

    Traffic traffic(rN, sCenter, tCenter, radius, radius);

    Simulation simulation(traffic, trajs, edgeFlowDist);
//    vector<Path> paths = esx(&rN, 135346, 248574, 5, 0.5);
//    string path2 = "/Users/xyh/Desktop/traffic-assignment/KSP/test/";
//    for (int i = 0; i < paths.size(); i++)
//    {
//        ofstream ofstream1(path2 + to_string(i) + ".csv");
//        for (unsigned int node: paths[i].nodes)
//        {
//            ofstream1 << rN.coords[node].second << "," << rN.coords[node].first << endl;
//        }
//        ofstream1.close();
//    }
    
    simulation.basicSimulation();
//`
//    simulation.totalCost();
//    simulation.totalCostT();
//    simulation.writeResults(path);
//    simulation.clearTraffic();
    return 0;
}
