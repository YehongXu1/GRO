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

struct hash_edge {
    size_t operator()(const Edge& p) const
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
    int totalFlow = 0, weight = -1; // total totalFlow
    vector<long int> tempFlow;
    vector<long long int> tempWeight;

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

    explicit RoadNetwork(const char *filename);

    explicit RoadNetwork(const char *filename, const char *filename1);

    int getEdgeWeight(NodeId lnode, NodeId rnode);

    void adjustWeight(NodeId lnode, NodeId rnode, int increment);

    RoadNetwork() = default;

    static float distance(Coord p1, Coord p2)
    {
        float lon1 = p1.first;
        float lat1 = p1.second;
        float lon2 = p2.first;
        float lat2 = p2.second;
        // Convert the latitudes
        // and longitudes
        // from degree to radians.
        lat1 = toRadians(lat1);
        lon1 = toRadians(lon1);
        lat2 = toRadians(lat2);
        lon2 = toRadians(lon2);

        // Haversine Formula
        float dlon = lon2 - lon1;
        float dlat = lat2 - lat1;

        float ans = pow(sin(dlat / 2), 2) +
                    cos(lat1) * cos(lat2) *
                    pow(sin(dlon / 2), 2);

        ans = 2 * asin(sqrt(ans));

        // Radius of Earth in
        // Kilometers, R = 6371
        // Use R = 3956 for miles
        float R = 6371000;

        // Calculate the result
        ans = ans * R;

        return ans;

    }

    static float toRadians(const float degree)
    {
        // cmath library in C++
        // defines the constant
        // M_PI as the value of
        // pi accurate to 1e-30
        float one_deg = (M_PI) / 180;
        return (one_deg * degree);
    }

    ~RoadNetwork();

};

// This is to ensure that edges are considered in a bidirectional fashion for the computation of the overlap.
bool operator==(const Edge &le, const Edge &re);

class Path
{
public:
    vector<NodeId> nodes;
    unordered_set<Edge, boost::hash<Edge>> edges;
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
