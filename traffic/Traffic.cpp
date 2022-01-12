//
// Created by yehong.xu on 28/12/21.
//

#include "Traffic.h"
#include <fstream>


void Traffic::writeSetting(const basic_string<char> &path) const
{
    ofstream ofstream(path);
    ofstream << "no. requests: " << requestIdMap.size() << endl;
    ofstream.close();
}


vector<NodeId> Traffic::getNodesInR(Coord &center, int r)
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


int Simulation::onePass()
{
    /**
     * Based on base cost // first iteration
     */
    clearTraffic();

    set<Edge> newOverflowEdges;
    for (RequestId req = 0; req < traffic.requestIdMap.size(); req++)
    {
        auto *sourceLabel = new Label();
        dijkstra_label_heu(&traffic.rN, traffic.requestIdMap[req].first, traffic.requestIdMap[req].second, sourceLabel);

        trajectories[req] = sourceLabel;
        assert(sourceLabel != nullptr);

        set<Edge> s = addTrajectoryInRN(req);
        newOverflowEdges.insert(s.begin(), s.end());
    }

    overflowEdgeCnt = newOverflowEdges.size();
    return getCost();
}


int Simulation::basicSimulation()
{
    // do not voluntary to adjust edge weight, just update edge weight based on previous traffic
    int cost = 0;
    while (true)
    {
        for (RequestId req = 0; req < traffic.requestIdMap.size(); req++)
        {
            auto *sourceLabel = new Label();
            dijkstra_label_heu(&traffic.rN, traffic.requestIdMap[req].first, traffic.requestIdMap[req].second, sourceLabel);

            trajectories[req] = sourceLabel;
            assert(sourceLabel != nullptr);
        }

        updateTrafficStat();
        updateEdgeCost();
        int curCost = getCost();
        if (cost > curCost)
            break;

        cost = curCost;
    }

    return cost;
}

int Simulation::reroutePartialReqsByBlocking()
{
    clearTraffic();

    vector<int> reqOverflowEdge(traffic.rN.numNodes + 1);
    benchmark::heap<2, int, RequestId> requestHeap(traffic.rN.numNodes + 1);

    vector<RequestId> request;
    for (RequestId i = 0; i < traffic.requestIdMap.size(); i++)
        request.emplace_back(i);

    while (true)
    {
        set<Edge> newOverflowEdges = heuBasedSimulation(request);
        overflowEdgeCnt = newOverflowEdges.size();

        blockOverflowEdges(newOverflowEdges);

        if (overflowEdgeCnt == 0)
            break;

        for (const auto &changeEdge: newOverflowEdges)
        {
            for (const auto &relatedReq: trafficStat[changeEdge.first][changeEdge.second].requests)
            {
                reqOverflowEdge[relatedReq] += 1;
                requestHeap.update(relatedReq, reqOverflowEdge[relatedReq]);
            }
        }

        request = reqsByCollCnt(reqOverflowEdge, requestHeap);
        assert(overflowEdgeCnt == 0);
        updateEdgeCost();
    }

    return getCost();
}

vector<RequestId>
Simulation::reqsByCollCnt(vector<int> &reqOverflowEdge, benchmark::heap<2, int, RequestId> &requestHeap)
{
    vector<RequestId> reqs;
    RequestId request;
    int overflowCnt;

    while (overflowEdgeCnt > 0 && !requestHeap.empty())
    {
        requestHeap.extract_max(request, overflowCnt);
        reqOverflowEdge[request] = 0;

        // delete the selected path from traffic
        set<Edge> newUnderflowEdges = delTrajectoryInRN(request); // unable to update temporal flow

        // update number of collisions of each request
        for (const auto &changeEdge: newUnderflowEdges)
        {
            for (const auto &relatedReq: trafficStat[changeEdge.first][changeEdge.second].requests)
            {
                reqOverflowEdge[relatedReq] -= 1;
                requestHeap.update(relatedReq, reqOverflowEdge[relatedReq]);
            }
        }

        overflowEdgeCnt -= newUnderflowEdges.size();
        reqs.emplace_back(request);
    }
    return reqs;
}

void Simulation::blockOverflowEdges(set<Edge> &newOverflowEdges)
{
    for (const auto &edge: newOverflowEdges)
    {
        if (trafficStat[edge.first][edge.second].totalFlow > traffic.capacity)
        {
            traffic.rN.adjListInHeu[edge.second].erase(edge.first);
        }
    }
}

set<Edge> Simulation::heuBasedSimulation(vector<RequestId> &requests)
{
    set<Edge> newOverflowEdges;
    for (const auto &req: requests)
    {
        auto *sourceLabel = new Label();
        dijkstra_label_heu(&traffic.rN, traffic.requestIdMap[req].first, traffic.requestIdMap[req].second, sourceLabel);

        assert(sourceLabel != nullptr); // otherwise, there is no path from source to target

        trajectories[req] = sourceLabel;
        set<Edge> s = addTrajectoryInRN(req);
        newOverflowEdges.insert(s.begin(), s.end());
    }

    return newOverflowEdges;
}

set<Edge> Simulation::addTrajectoryInRN(RequestId request)
{
    set<Edge> newOverflowEdges;
    auto *curLabel = trajectories[request];
    int totalTime = curLabel->length;

    while (curLabel->previous != nullptr)
    {
        traversedEdges.insert(make_pair(curLabel->node_id, curLabel->previous->node_id));

        trafficStat[curLabel->node_id][curLabel->previous->node_id].requests.insert(request);
        trafficStat[curLabel->node_id][curLabel->previous->node_id].totalFlow += 1;

        int timeInt = (totalTime - curLabel->length) / timeResl;
        assert(timeInt < timeInts);
        trafficStat[curLabel->node_id][curLabel->previous->node_id].tempFlow[timeInt] += 1;
        if (trafficStat[curLabel->node_id][curLabel->previous->node_id].tempFlow[timeInt]
            > trafficStat[curLabel->node_id][curLabel->previous->node_id].maxTempFlow)
        {
            trafficStat[curLabel->node_id][curLabel->previous->node_id].maxTempFlow =
                    trafficStat[curLabel->node_id][curLabel->previous->node_id].tempFlow[timeInt];
        }

        if (trafficStat[curLabel->node_id][curLabel->previous->node_id].totalFlow == traffic.capacity + 1)
        {
            newOverflowEdges.insert(make_pair(curLabel->node_id, curLabel->previous->node_id));
        }

        curLabel = curLabel->previous;
    }
    return newOverflowEdges;
}

set<Edge> Simulation::delTrajectoryInRN(RequestId request)
{
    set<Edge> newUnderflowEdges;

    auto *curLabel = trajectories[request];
    int totalTime = curLabel->length;

    while (curLabel->previous != nullptr)
    {
        trafficStat[curLabel->node_id][curLabel->previous->node_id].requests.erase(request);

        assert(trafficStat[curLabel->node_id][curLabel->previous->node_id].totalFlow > 0);
        trafficStat[curLabel->node_id][curLabel->previous->node_id].totalFlow -= 1;

        if (trafficStat[curLabel->node_id][curLabel->previous->node_id].totalFlow == 0)
        {
            traversedEdges.erase(make_pair(curLabel->node_id, curLabel->previous->node_id));
        }

        int timeInt = (totalTime - curLabel->length) / timeResl;
        assert(timeInt < timeInts); // 1000 is the number of time intervals
        assert(trafficStat[curLabel->node_id][curLabel->previous->node_id].tempFlow[timeInt] > 0);

        if (trafficStat[curLabel->node_id][curLabel->previous->node_id].tempFlow[timeInt] ==
            trafficStat[curLabel->node_id][curLabel->previous->node_id].maxTempFlow)
        {
            trafficStat[curLabel->node_id][curLabel->previous->node_id].maxTempFlow -= 1;
        }

        trafficStat[curLabel->node_id][curLabel->previous->node_id].tempFlow[timeInt] -= 1;

        if (trafficStat[curLabel->node_id][curLabel->previous->node_id].totalFlow == traffic.capacity)
        {
            newUnderflowEdges.insert(make_pair(curLabel->node_id, curLabel->previous->node_id));
        }

        curLabel = curLabel->previous;
    }

    return newUnderflowEdges;
}

void Simulation::clearTraffic()
{
    overflowEdgeCnt = 0;

    for (int req = 0; req < traffic.requestIdMap.size(); req++)
    {
        trajectories[req] = nullptr;
    }

    vector<int> v(timeInts, 0);
    for (const auto edge: traversedEdges)
    {
        set<RequestId> s;
        trafficStat[edge.first][edge.second].requests.swap(s);
        trafficStat[edge.first][edge.second].totalFlow = 0;
        copy(v.begin(), v.end(), trafficStat[edge.first][edge.second].tempFlow.begin());
        trafficStat[edge.first][edge.second].maxTempFlow = INT_MIN;
    }
}

void Simulation::updateTrafficStat()
{
    clearTraffic();

    for (const auto label: trajectories)
    {
        NodeId nextId = label->node_id;
        int totalTime = label->length;
        while (label->previous != nullptr)
        {
            int timeInt = (totalTime - label->length) / timeResl;

            assert(timeInt < timeInts); // 1000 is the number of time intervals
            trafficStat[label->node_id][label->previous->node_id].tempFlow[timeInt] += 1;

            if (trafficStat[label->node_id][label->previous->node_id].tempFlow[timeInt]
                > trafficStat[label->node_id][label->previous->node_id].maxTempFlow)
            {
                trafficStat[label->node_id][label->previous->node_id].maxTempFlow =
                        trafficStat[label->node_id][label->previous->node_id].tempFlow[timeInt];
            }

            traversedEdges.insert(make_pair(label->node_id, label->previous->node_id));
        }

    }
}

void Simulation::updateEdgeCost()
{
    // update heuweight based on max temp edgeflow
    for (const auto &edge: traversedEdges)
    {
        int base = traffic.rN.adjListOut[edge.first][edge.second];
        traffic.rN.adjListInHeu[edge.second][edge.first] =
                trajCostFun1(const_cast<Edge &>(edge), trafficStat[edge.first][edge.second].maxTempFlow);
    }
}

int Simulation::trajCostFun1(Edge &edge, int edgeFlow)
{
    int base = traffic.rN.adjListOut[edge.first][edge.second];
    return base * (1 + 0.15 * pow(edgeFlow / traffic.capacity, 4));
}


int Simulation::getCost()
{
    int cost = 0;
    for (const auto &label: trajectories)
    {
        cost += label->length;
    }

    return cost;
}

void Simulation::writeTrajectories(const basic_string<char> &path)
{
    ofstream ofstream(path);

    if (trajectories.empty())
    {
        ofstream << "no result" << endl;
        return;
    }

    for (const auto &sourceLabel: trajectories)
    {
        Label *label = sourceLabel;
        while (label != nullptr)
        {
            ofstream << traffic.rN.coords[label->node_id].second << "," << traffic.rN.coords[label->node_id].first
                     << endl;
            label = label->previous;
        }
    }
    ofstream.close();
}

void Simulation::writeEdgeFlowDist(const basic_string<char> &path)
{
    ofstream ofstream(path);
    for (NodeId lnode = 0; lnode <= traffic.rN.numNodes; lnode++)
    {
        for (const auto &rnodeW: traffic.rN.adjListOut[lnode])
        {
            ofstream << trafficStat[lnode][rnodeW.first].totalFlow << endl;
        }
    }

    ofstream.close();
}

void Simulation::writeCollision(const basic_string<char> &path, vector<Edge> &overflowEdges)
{
    ofstream ofstream(path);
    float totalColCnt = 0;
    for (const auto edge: overflowEdges)
    {
        totalColCnt += trafficStat[edge.first][edge.second].totalFlow;
        ofstream << trafficStat[edge.first][edge.second].totalFlow << endl;
    }

    ofstream << totalColCnt / overflowEdges.size() << endl;
}

