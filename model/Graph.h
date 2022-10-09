//
// Created by yehong.xu on 26/12/21.
//

#ifndef TRAFFIC_ASSIGNMENT_GRAPH_H
#define TRAFFIC_ASSIGNMENT_GRAPH_H

//
// Created by yehong.xu on 11/12/21.
//
#include "Semaphore.h"
#include <iostream>
#include <fstream>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <boost/thread/thread.hpp>
#include <boost/functional/hash.hpp>
#include <boost/range/algorithm_ext.hpp>
#include <boost/range/irange.hpp>


using namespace std;

typedef unsigned int NodeId;

typedef pair<NodeId, NodeId> Edge;

struct hash_edge {
    size_t operator()(const Edge &p) const {
        auto hash1 = hash<NodeId>{}(p.first);
        auto hash2 = hash<NodeId>{}(p.second);
        return hash1 ^ hash2;
    }
};

// This is to ensure that edges are considered in a bidirectional fashion for the computation of the overlap.
bool operator==(const Edge &le, const Edge &re);


typedef pair<float, float> Coord; // lat, lon

typedef unordered_map<NodeId, int> EdgeList; // node_id, edge_weight or edge id
typedef int RequestId;
typedef int TimeIntIdx;

struct Request {
    RequestId id;
    NodeId o, d;
    unsigned int waitingTime = 0;
    int departT = 0;
    long int cost = 0;
    long int temDijTime = 0;
    unordered_set<int> congs;

    Request(RequestId i, NodeId o, NodeId d, int departT) {
        this->id = i;
        this->o = o;
        this->d = d;
        this->departT = departT;
    }
};


class RoadNetwork {
public:
    unsigned int numNodes{};
    unsigned int numEdges{};
    vector<EdgeList> adjListOut;
    vector<EdgeList> adjListInc;
    vector<Coord> coords;
    vector<unordered_map<NodeId, int>> capacity;
    float maxLon = -10000, maxLat = -100000, minLon = 100000, minLat = 100000;

    void readGraph(const string &filename, const string &filename1, int type);

    void readGraph1(const string &filename, const string &filename1);

    void readGraph2(const string &filename, const string &filename2);

    ~RoadNetwork() {
        this->adjListOut.clear();
        this->adjListInc.clear();
    }

    int getEdgeWeight(NodeId lnode, NodeId rnode);

    static float distance(float lat1, float lon1, float lat2, float lon2); // Return in Meters

    static float toRadians(float degree);

    static Coord interPoint(float frac, float lat1, float lon1, float lat2, float lon2);

    float bearing(float lat1, float lon1, float lat2, float lon2);
};

struct Grid {
    vector<NodeId> vertices;
    int latIdx = -1, longIdx = -1;
    float eFlow = 0;

    Grid(int latIdx, int longIdx) {
        this->longIdx = longIdx;
        this->latIdx = latIdx;
    }
};

class DisGraph {
public:
    float sigma, tau;

    RoadNetwork &rN;

    vector<vector<Grid>> grids;

    unordered_set<pair<int, int>, hash_edge> relatedG;
    vector<vector<RequestId>> batches;

    vector<float> longitutes, latitudes;
    vector<pair<int, int>> vToGrid;

    DisGraph(RoadNetwork &rN, float sigma, float tau, int threadNum);

    static int binarySearch(float x, vector<float> &arr);

    static void rangeBinarySearch(int begin, int end, vector<float> &longitutes,
                                  vector<float> &latitudes, vector<Coord> &coord, vector<pair<int, int>> &vToGrid);

    bool evaluateQ(Request &req);

    void findNearbyGrids(Request &req);

    float occurProb(int i);

    void countIntoG(int i, int lat, int lon, bool &overLoad);

    void newABatch();
};


class Path {
public:
    vector<NodeId> nodes;
    int length;

    Path() {
        length = -1;
    }

    bool containsEdge(Edge &e);

    double overlap_ratio(RoadNetwork *rN, Path &path2);
};

bool operator==(const Path &lp, const Path &rp);


template<class DT = std::chrono::microseconds,
        class ClockT = std::chrono::steady_clock>
class Timer {
    using timep_t = typename ClockT::time_point;
    timep_t _start = ClockT::now(), _end = {};

public:
    void tick() {
        _end = timep_t{};
        _start = ClockT::now();
    }

    void tock() { _end = ClockT::now(); }

    template<class T = DT>
    auto duration() const {
        return std::chrono::duration_cast<T>(_end - _start);
    }
};

#endif // TRAFFIC_ASSIGNMENT_GRAPH_H
