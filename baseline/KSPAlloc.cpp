//
// Created by yehong.xu on 31/1/2022.
//

#include "KSPAlloc.h"
#include "../KSP/kspwlo.h"

KSPAlloc::KSPAlloc(Traffic &traffic, int k, double theta) : traffic(traffic)
{
    this->k = k;
    this->theta = theta;
    vector<Path> v;
    candidates.assign(traffic.reqNo, v);
    getCandiPaths();
}

void KSPAlloc::getCandiPaths()
{
    boost::thread_group tGroup;
    int interval = ceil(1.0 * traffic.reqNo / traffic.threadNum);
    for (int i = 0; i < traffic.threadNum; ++i)
    {
        int begin = i * interval, end = (i + 1) * interval;
        if (end >= traffic.reqNo)
            end = traffic.reqNo;
        tGroup.create_thread(boost::bind(&KSPAlloc::getCandiPaths, this, begin, end));
    }
    tGroup.join_all();
}

void KSPAlloc::getCandiPaths(int begin, int end)
{
    for (RequestId req = begin; req < end; req++)
    {
        candidates[req] = svp_plus_complete(
                &traffic.rN, traffic.requestODs[req].o, traffic.requestODs[req].d, k, theta).first;
    }
}

int KSPAlloc::selectAPath(RequestId req)
{
    random_device rd; // obtain a random number from hardware
    mt19937 gen(rd()); // seed the generator
    uniform_int_distribution<> distr(0, candidates[req].size() - 1); // define the range

    return distr(gen); // generate numbers
}

void KSPAlloc::assignPath()
{
    unordered_map<Edge, unordered_set<TimeIntIdx>, hash_edge> traversedEdges;
    for (RequestId id = 0; id < traffic.reqNo; id++)
    {
        int theP = selectAPath(id);
        traffic.trajectories[id] = pathToLabel(candidates[id][theP]);
    }
    cost = traffic.simulateTraffic();

    for (RequestId id = 0; id < traffic.reqNo; id++)
    {
        Label *label = traffic.trajectories[id];
        while (label != nullptr)
        {
            Label *nextL = label->nextL;
            delete label;
            label = nextL;
        }
    }
}

Label *KSPAlloc::pathToLabel(Path &path)
{
    int length = 0;
    auto *prevL = new Label(path.nodes[0], length);
    Label *firstL = prevL;
    for (int i = 1; i < path.nodes.size(); i++)
    {
        NodeId curId = path.nodes[i];
        auto *curL = new Label(curId, 0, prevL);
        prevL->nextL = curL;
        prevL = curL;
    }
    return firstL;
}

long long int KSPAlloc::getCost()
{
    return cost;
}
