//
// Created by yehong.xu on 28/12/21.
//

#ifndef TRAFFIC_ASSIGNMENT_TRAFFIC_H
#define TRAFFIC_ASSIGNMENT_TRAFFIC_H

#include "../tools/tools.h"

typedef pair<int, int> Interval;

class IntComparator
{
    bool reverse;
public:
    explicit IntComparator(const bool &revparam = false)
    {
        reverse = revparam;
    }

    bool operator()(const Interval &int1, const Interval &int2) const
    {
        return (int1.first > int2.first);
    }
};


typedef priority_queue<Interval, std::vector<Interval>, IntComparator> PriorityQueueIn;

class Traffic
{
public:
    vector<NodeId> sources, targets;
    int r1{}, r2{};
    Coord &sCenter, &tCenter;
    RoadNetwork &rN;

    Traffic(RoadNetwork &rN, Coord &sCenter, Coord &tCenter, int r1, int r2) :
            rN(rN), sCenter(sCenter), tCenter(tCenter)
    {
        this->r1 = r1;
        this->r2 = r2;
        vector<NodeId> ss = getNodesInR(sCenter, r1);
        for (const auto &item: ss)
        {
            this->sources.push_back(item);
        }

        vector<NodeId> ts = getNodesInR(tCenter, r2);
        for (const auto &item: ts)
        {
            this->targets.push_back(item);
        }
    }



private:
    vector<NodeId> getNodesInR(Coord &center, int r);
};


class Simulation
{
private:
    Traffic &traffic;
    vector<Label *> &trajs;
    vector<unordered_map<NodeId, pair<vector<Interval *>, int>>> &edgeFlowDist;
    vector<Edge> traversedEdges;
    int costsT = 0, costs = 0;

public:
    Simulation(Traffic &traffic, vector<Label *> &trajs,
               vector<unordered_map<NodeId, pair<vector<Interval *>, int>>> &edgeFlowDist) :
            traffic(traffic), trajs(trajs), edgeFlowDist(edgeFlowDist)
    {
    }

    void getEdgeFlowInRN();

    int getCollisionT(Label *label);

    int trajCostT(Label *sourceLabel);

    void totalCostT();

    void clearTraffic();

    int trajCostCascade(Label *sourceLabel);

    int totalCostCascade();

    void writeResults(string &path);

    int getCollision(Label *label);

    int trajCost(Label *sourceLabel);

    void totalCost();

    void basicSimulation();

    void heuBasedSimulation();

    void updateHeuWeight();

};


#endif //TRAFFIC_ASSIGNMENT_TRAFFIC_H
