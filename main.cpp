#include <iostream>

#include "traffic/Traffic.h"
#include "traffic/routingMain.cpp"
#include "KSP/kspwlo.h"
#include "baseline/KSPAlloc.h"
#include <random>
#include <chrono>
#include <experimental/filesystem>

using namespace std;


int getRandInt(int range)
{
    random_device rd; // obtain a random number from hardware
    mt19937 gen(rd()); // seed the generator
    uniform_int_distribution<> distr(0, range); // define the range
    return distr(gen);
}

vector<NodeId> getNodesInR(Coord &center, int r, RoadNetwork &rN)
{
    vector<NodeId> v;
    for (NodeId i = 0; i <= rN.numNodes; i++)
    {
        if (RoadNetwork::distance(center, rN.coords[i]) <= r)
        {
            v.emplace_back(i);
        }
    }

    return v;
}

vector<Request> createRequests(
        RoadNetwork &rN, Coord sCenter, Coord tCenter, int r1, int r2)
{
    vector<Request> requestODs;
    vector<NodeId> sources = getNodesInR(sCenter, r1, rN);
    vector<NodeId> targets = getNodesInR(tCenter, r2, rN);

    int i = 0;
    for (unsigned int &source: sources)
    {
        for (unsigned int &target: targets)
        {
            requestODs.emplace_back(Request(i, source, target, 0));
            i++;
        }
    }
    return requestODs;
}

vector<Request> createRequests(
        RoadNetwork &rN, NodeId v1, NodeId v2, int r1, int r2)
{
    vector<Request> requestODs;
    vector<NodeId> sources = getNodesInR(rN.coords[v1], r1, rN);
    vector<NodeId> targets = getNodesInR(rN.coords[v2], r2, rN);
    int i;
    for (unsigned int &source: sources)
    {
        for (unsigned int &target: targets)
        {
            requestODs.emplace_back(Request(i, source, target, 0));
            i++;
        }
    }

    return requestODs;
}

vector<Request> createRequests(RoadNetwork &rN, const basic_string<char> &input)
{
    ifstream infile(input);
    NodeId v1, v2;
    vector<Request> requestODs;
    int i = 0;
    while (infile >> v1 >> v2)
    {
        assert(v1 < rN.numNodes && v2 < rN.numNodes);
        if (v1 == v2)
            continue;
        requestODs.emplace_back(Request(i, v1, v2, 0));
        i++;
    }

    return requestODs;
}

vector<Path> writeEsxPaths(TrafficMaintain &traffic, int k)
{
    chrono::steady_clock::time_point begin = chrono::steady_clock::now();
    vector<Path> paths;
    for (const auto &od: traffic.requestODs)
    {
        vector<Path> vP = esx(&traffic.rN, od.o, od.d, k, 0.75);
    }
    chrono::steady_clock::time_point end = chrono::steady_clock::now();
    cout << "Esx consumes " << chrono::duration_cast<chrono::seconds>(end - begin).count()
         << "[s] for " << traffic.reqNo << " ods" << endl;

    begin = chrono::steady_clock::now();
    for (const auto &od: traffic.requestODs)
    {
        esx_complete(&traffic.rN, od.o, od.d, k, 0.75);
    }
    end = chrono::steady_clock::now();
    cout << "Esx_c consumes " << chrono::duration_cast<chrono::seconds>(end - begin).count()
         << "[s] for " << traffic.reqNo << " ods" << endl;

    begin = chrono::steady_clock::now();
    for (const auto &od: traffic.requestODs)
    {
        svp_plus(&traffic.rN, od.o, od.d, k, 0.75);
    }
    end = chrono::steady_clock::now();
    cout << "svp_plus consumes " << chrono::duration_cast<chrono::seconds>(end - begin).count()
         << "[s] for " << traffic.reqNo << " ods" << endl;

    begin = chrono::steady_clock::now();
    for (const auto &od: traffic.requestODs)
    {
        svp_plus_complete(&traffic.rN, od.o, od.d, k, 0.75);
    }
    end = chrono::steady_clock::now();
    cout << "svp_plus_c consumes " << chrono::duration_cast<chrono::seconds>(end - begin).count()
         << "[s] for " << traffic.reqNo << " ods" << endl;

    return paths;
}

void runBaseLine(RoadNetwork &rN, vector<Request> &requestODs, int k, double theta,
                 int timeIntNum, int timeResolution, int penalR, int threadNum, double threshold)
{
    TrafficMaintain traffic(rN, requestODs, timeIntNum, timeResolution, penalR, threadNum, threshold);

    chrono::steady_clock::time_point begin = chrono::steady_clock::now();
    KSPAlloc kspAlloc(traffic, k, theta);
    kspAlloc.assignPath();
    chrono::steady_clock::time_point end = chrono::steady_clock::now();
    cout << "random allocation cost: " << kspAlloc.getCost() << endl;
    cout << "baseline consumes [s]: " << chrono::duration_cast<chrono::seconds>(end - begin).count() << endl;
}

void runBaseLine(RoadNetwork &rN, Coord &sc, Coord &tc, int k, int r, double theta,
                 int timeIntNum, int timeResolution, int penalR, int threadNum, double threshold)
{
    vector<Request> requestODs = createRequests(rN, sc, tc, r, r);
    cout << "\nreqNo: " << requestODs.size() << endl;
    runBaseLine(rN, requestODs, k, theta, timeIntNum, timeResolution, penalR, threadNum, threshold);
}

void runBaseLine(RoadNetwork &rN, pair<NodeId, NodeId> seedOd, int k, int r, double theta,
                 int timeIntNum, int timeResolution, int penalR, int threadNum, int threshold)
{
    vector<Request> requestODs = createRequests(rN, seedOd.first, seedOd.second, r, r);
    if (requestODs.size() > 4000)
        return;
    cout << "\nreqNo: " << requestODs.size() << endl;
    runBaseLine(rN, requestODs, k, theta, timeIntNum, timeResolution, penalR, threadNum, threshold);
}

void runMyAlg(RoadNetwork &rN, vector<Request> &requestODs,
              int timeIntNum, int timeResolution, int penalR, double frac, bool fix, int threadNum, double threshold)
{
    TrafficMaintain traffic(rN, requestODs, timeIntNum, timeResolution, penalR, threadNum, threshold);

    chrono::steady_clock::time_point begin = chrono::steady_clock::now();
    routingMain(traffic, frac, fix);
    chrono::steady_clock::time_point end = chrono::steady_clock::now();
    cout << "time difference[s] = " << chrono::duration_cast<chrono::seconds>(end - begin).count() << endl;
    vector<RequestId> reqs;
    boost::push_back(reqs, boost::irange(0, traffic.reqNo));
    traffic.deleteLabels(reqs);
}

void runMyAlg(RoadNetwork &rN, pair<NodeId, NodeId> &od, int timeIntNum, int timeResolution,
              int penalR, int r, double frac, bool fix, int threadNum, double threshold)
{
    vector<Request> requestODs = createRequests(rN, od.first, od.second, r, r);
    if (requestODs.size() > 4000)
        return;
    cout << "\nreqNo: " << requestODs.size() << endl;
    runMyAlg(rN, requestODs, timeIntNum, timeResolution, penalR, frac, fix, threadNum, threshold);
}

void runMyAlg(RoadNetwork &rN, Coord &o, Coord &d, int timeIntNum, int timeResolution,
              int penalR, int r, double frac, bool fix, int threadNum, double threshold)
{
    vector<Request> requestODs = createRequests(rN, o, d, r, r);
    cout << "\nreqNo: " << requestODs.size() << endl;
    runMyAlg(rN, requestODs, timeIntNum, timeResolution, penalR, frac, fix, threadNum, threshold);
}

vector<pair<NodeId, NodeId>> readSeedOds(const basic_string<char> &seedOdPath)
{
    ifstream infile(seedOdPath);
    int lnode, rnode;
    vector<pair<NodeId, NodeId>> seedOds;
    while (infile >> lnode >> rnode)
    {
        seedOds.emplace_back(make_pair(lnode, rnode));
    }

    return seedOds;
}

int main()
{
    string basepath1 = "/media/TraminerData/yehong_ziyi/global_routing/input/";
    string basepath2 = "/media/bigdata/s4451682/Yehong/";
    string basepath3 = "/Users/xyh/Desktop/traffic-assignment/data/";

    string basepath = basepath3;
    string map = basepath + "BJ_map.txt";
    string coords = basepath + "BJ_NodeIDLonLat.txt";

    string queryset = "seedod50";
    string inputSeedOd = basepath + queryset + ".txt";
    int r = 500, penalR = 5, timeResolution = 600, timeIntNum = 2000, threadNum = 50;
    double frac = 0.3, threshold = 0.67;
    bool fix = false;
    RoadNetwork rN(map.c_str(), coords.c_str());
    cout << "query: " << queryset << " r: " << r << " threshold: " << threshold << " frac: " << "threadNum: " << threadNum <<
         frac << " fix:" << fix << " timeIntNum: " << timeIntNum << " timeResolution: " << timeResolution << endl;

    vector<pair<NodeId, NodeId>> seedODs = readSeedOds(inputSeedOd);

    for (auto &seedOD: seedODs)
    {
        runMyAlg(rN, seedOD, timeIntNum, timeResolution, penalR, r, frac, fix, threadNum, threshold);
//        runBaseLine(rN, seedOD, 10, r, 0.75, timeIntNum, timeResolution, penalR, threadNum);
    }

//    Coord sc(116.207, 40.091), tc(116.518, 39.8754);
//    runMyAlg(rN, sc, tc, timeIntNum, timeResolution, penalR, r, frac, fix, threadNum, threshold);

//    runBaseLine(rN, sc, tc, 10, r, 0.75, timeIntNum, timeResolution, penalR, threadNum, threshold);
    cout << "done" << endl;
    return 0;
}
