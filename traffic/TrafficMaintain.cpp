//
// Created by yehong.xu on 28/12/21.
//

#include "Traffic.h"
#include <fstream>

TrafficMaintain::TrafficMaintain(
        RoadNetwork &rN, vector<Request> &requestODs, int timeIntNum, int timeReslo,
        int capThres, int threadNum, double threshold) : rN(rN), requestODs(requestODs)
{
    this->timeIntNum = timeIntNum;
    this->timeReslo = timeReslo;
    this->threadNum = threadNum;
    this->threshold = threshold;

    this->reqNo = requestODs.size();
    this->capThres = capThres;

    trafficStat.resize(rN.numNodes + 1);
    affectedHistBins.resize(rN.numNodes + 1);
    boost::thread_group tGroup;
    int interval = ceil(1.0 * (1 + rN.numNodes) / threadNum);
    for (int i = 0; i < this->threadNum; ++i)
    {
        NodeId begin = i * interval, end = (i + 1) * interval;
        if (end > rN.numNodes + 1)
            end = rN.numNodes + 1;
        tGroup.create_thread([this, begin, end] { initialize(begin, end); });
    }
    tGroup.join_all();

    vector<Label> v;
    footprints.assign(reqNo, v);
}

TrafficMaintain::TrafficMaintain(
        RoadNetwork &rN, vector<Request> &requestODs, int threadNum, int capThres) : rN(rN), requestODs(requestODs)
{
    this->threadNum = threadNum;
    this->capThres = capThres;
    this->reqNo = requestODs.size();

    trafficStat.resize(rN.numNodes + 1);
    affectedHistBins.resize(rN.numNodes + 1);

    boost::thread_group tGroup;
    int interval = ceil(1.0 * (1 + rN.numNodes) / threadNum);
    for (int i = 0; i < this->threadNum; ++i)
    {
        NodeId begin = i * interval, end = (i + 1) * interval;
        if (end > rN.numNodes + 1)
            end = rN.numNodes + 1;
        tGroup.create_thread([this, begin, end] { initialize(begin, end); });
    }
    tGroup.join_all();

    vector<Label> v;
    footprints.assign(reqNo, v);
}

void TrafficMaintain::initialize(NodeId begin, NodeId end)
{
    for (NodeId vertex = begin; vertex < end; vertex++)
    {
        for (const auto &edgeList: rN.adjListOut[vertex])
        {
            trafficStat[vertex][edgeList.first].tempFlow.assign(timeIntNum, 0);
            trafficStat[vertex][edgeList.first].tempWeight.assign(timeIntNum, edgeList.second);
            trafficStat[vertex][edgeList.first].baseCost = edgeList.second;
        }
    }
}

void TrafficMaintain::revertTrafficCondition()
{
    // erase traffic condition (histogram) which includes traffic flow and travel time of each edge
    boost::thread_group tGroup;
    int interval = ceil(1.0 * traversedE.size() / threadNum);
    for (int i = 0; i < this->threadNum; ++i)
    {
        int begin = i * interval, end = (i + 1) * interval;
        if (end > traversedE.size())
            end = traversedE.size();
        tGroup.create_thread([this, begin, end]
                             {
                                 revertTrafficCondition(begin, end);
                             });
    }
    tGroup.join_all();
    traversedE.clear();
}

void TrafficMaintain::revertTrafficCondition(int begin, int end)
{
    for (int j = begin; j < end; j++)
    {
        Edge e = traversedE[j];
        NodeId v1 = e.first, v2 = e.second;
        trafficStat[v1][v2].liveFlow = 0;

        for (const auto &timeInt: this->affectedHistBins[v1][v2])
        {
            trafficStat[v1][v2].tempFlow[timeInt] = 0;
            trafficStat[v1][v2].tempWeight[timeInt] = trafficStat[v1][v2].baseCost;
        }
        affectedHistBins[v1][v2].clear();
    }
}


void TrafficMaintain::calculateTimeDepEdgeWeight()
{

    boost::thread_group tGroup;
    int interval = ceil(1.0 * traversedE.size() / threadNum);
    for (int i = 0; i < this->threadNum; ++i)
    {
        int begin = i * interval, end = (i + 1) * interval;
        if (end > traversedE.size())
            end = traversedE.size();
        tGroup.create_thread([this, begin, end]
                             {
                                 calculateTimeDepEdgeWeight( begin, end);
                             });
    }
    tGroup.join_all();
}

void TrafficMaintain::calculateTimeDepEdgeWeight(int begin, int end)
{
    for (int j = begin; j < end; j++)
    {
        Edge e = traversedE[j];
        NodeId v1 = e.first, v2 = e.second;
        for (const auto &timeInt: affectedHistBins[v1][v2])
        {
            int flow = trafficStat[v1][v2].tempFlow[timeInt];
            assert (flow > 0);
            long int cost = trafficStat[v1][v2].tempWeight[timeInt];
            trafficStat[v1][v2].tempWeight[timeInt] = (cost - trafficStat[v1][v2].baseCost) / flow;
        }
    }
}

void TrafficMaintain::clearCongs()
{
    if (congMap.empty())
        return;

    boost::thread_group tGroup;
    int interval = ceil(1.0 * reqNo / threadNum);
    for (int i = 0; i < threadNum; ++i)
    {
        int begin = i * interval, end = (i + 1) * interval;
        if (end >= reqNo)
            end = reqNo;
        tGroup.create_thread([this, begin, end] { rangeClearReqCngs(begin, end); });
    }
    tGroup.join_all();

    for (auto &edge: traversedE)
    {
        edgeCongList[edge].clear();
    }

    congMap.clear();
}

void TrafficMaintain::rangeClearReqCngs(int begin, int end)
{
    for (RequestId idx = begin; idx < end; idx++)
    {
        unordered_set<int> v;
        requestODs[idx].congs.swap(v);
    }
}

TimeIntIdx TrafficMaintain::getTimeInt(long int time) const
{
    return floor(time / timeReslo);
}


bool TrafficMaintain::detectCycles()
{
    unordered_map<NodeId, unordered_set<NodeId>> adjList(rN.numNodes + 1);
    unordered_set<NodeId> nset;
    for (RequestId req = 0; req < reqNo; req++)
    {
        if (footprints[req].empty())
            continue;

        vector<Label> const &path = footprints[req];
        nset.insert(path[0].node_id);
        for (int j = 1; j < path.size(); j++)
        {
            adjList[path[j - 1].node_id].insert(path[j].node_id);
            nset.insert(path[j - 1].node_id);
        }
    }

    unordered_set<NodeId> origins;
    for (const auto &req: requestODs)
    {
        origins.insert(req.o);
    }

    unordered_map<NodeId, bool> visited;
    for (const auto &v: nset)
        visited[v] = false;

    for (const auto &o: origins)
    {
        // dfs detect back edge from every o
        unordered_set<NodeId> recS;
        visited[o] = true;

        bool breakOut = false;
        detectCycleRec(o, recS, breakOut, visited, adjList);
        if (breakOut)
        {
            cout << "detect a cycle" << endl;
            return true;
        }
    }
    cout << "finish detecting cycle" << endl;
    return false;
}

void TrafficMaintain::detectCycleRec(NodeId o, unordered_set<NodeId> recS, bool &breakOut,
                                     unordered_map<NodeId, bool> &visited,
                                     unordered_map<NodeId, unordered_set<NodeId>> &adjList)
{
    if (breakOut)
        return;

    recS.insert(o);
    for (const auto &nei: adjList[o])
    {
        if (visited[nei])
        {
            if (recS.find(nei) != recS.end())
            {
                breakOut = true;
                break;
            }
            continue;
        }
        visited[nei] = true;

        detectCycleRec(nei, recS, breakOut, visited, adjList);
    }
}
