//
// Created by yehong.xu on 26/12/21.
//

#ifndef TRAFFIC_ASSIGNMENT_GRAPH_H
#define TRAFFIC_ASSIGNMENT_GRAPH_H


//
// Created by yehong.xu on 11/12/21.
//

#include <iostream>
#include <fstream>
#include <queue>
#include <unordered_set>
#include <unordered_map>

#include <boost/functional/hash.hpp>

using namespace std;

typedef unsigned int NodeId;

typedef pair<NodeId, NodeId> Edge;

struct hash_edge
{
    size_t operator()(const Edge &p) const
    {
        auto hash1 = hash<NodeId>{}(p.first);
        auto hash2 = hash<NodeId>{}(p.second);
        return hash1 ^ hash2;
    }
};


typedef pair<float, float> Coord; // lon, lat

typedef unordered_map<NodeId, int> EdgeList; // node_id, edge_weight
typedef int RequestId;
typedef int TimeIntIdx;

struct EdgeFlowInfo
{
    vector<unordered_set<RequestId>> tempReqs;
    int liveFlow = 0, weight = -1; // total liveFlow
    vector<int> tempFlow;
    vector<long long int> tempWeight;
    vector<bool> modified;

    EdgeFlowInfo()
    = default;
};

class RoadNetwork
{
public:
    unsigned int numNodes{};
    unsigned int numEdges{};
    vector<EdgeList> adjListOut;
    vector<EdgeList> adjListInc;
    vector<Coord> coords;
    vector<EdgeList> adjListInHeu;
    vector<int> capacity;
    vector<EdgeList> edgeMap;

    explicit RoadNetwork(const char *filename);

    explicit RoadNetwork(const char *filename, const char *filename1);

    ~RoadNetwork()
    {
        this->adjListOut.clear();
        this->adjListInc.clear();
    }
    int getEdgeWeight(NodeId lnode, NodeId rnode);

    void adjustWeight(NodeId lnode, NodeId rnode, int increment);

    static float distance(Coord p1, Coord p2);

    static float toRadians(float degree);

};

// This is to ensure that edges are considered in a bidirectional fashion for the computation of the overlap.
bool operator==(const Edge &le, const Edge &re);

class Path
{
public:
    vector<NodeId> nodes;
    int length;

    Path()
    {
        length = -1;
    }

    bool containsEdge(Edge &e);

    double overlap_ratio(RoadNetwork *rN, Path &path2);

};


bool operator==(const Path &lp, const Path &rp);

#endif //TRAFFIC_ASSIGNMENT_GRAPH_H
