//
// Created by yehong.xu on 28/12/21.
//

#include "Traffic.h"
#include <fstream>


void Traffic::writeSetting(const basic_string<char> &path) const
{
    ofstream ofstream(path);
    ofstream << "r1 " << r1 << endl;
    ofstream << "r2 " << r2 << endl;
    ofstream << "sCenter " << sCenter.second << "," << sCenter.first << endl;
    ofstream << "sCenter " << tCenter.second << "," << tCenter.first << endl;
    ofstream.close();
}


void Traffic::getSourcesInR(Coord &center, int r)
{
    for (NodeId i = 0; i <= rN.numNodes; i++)
    {
        if (RoadNetwork::distance(center, rN.coords[i]) <= r)
        {
            sources.push_back(i);
        }
    }
}


void Traffic::getTargetsInR(Coord &center, int r)
{
    for (NodeId i = 0; i <= rN.numNodes; i++)
    {
        if (RoadNetwork::distance(center, rN.coords[i]) <= r)
        {
            targets.push_back(i);
        }
    }
}

void Simulation::basicSimulation()
{
    /**
     * Based on base cost
     */
    clearTraffic();

    set<Edge> newOverflowEdges;
    for (RequestId req = 0; req < traffic.requestIdMap.size(); req++)
    {
        auto *sourceLabel = new Label();
        dijkstra_label(&traffic.rN, traffic.requestIdMap[req].first, traffic.requestIdMap[req].second, sourceLabel);

        trajectories[req] = sourceLabel;
        assert(sourceLabel != nullptr);

        set<Edge> s = addTrajectoryInRN(req);
        newOverflowEdges.insert(s.begin(), s.end());
    }

    for (const auto &edge: newOverflowEdges)
    {
        for (const auto &request: edgeFlowDist[edge.first][edge.second].requests)
        {
            reqOverflowEdge[request] += 1;
            requestHeap.update(request, reqOverflowEdge[request]);
        }
    }
    overflowEdgeCnt = newOverflowEdges.size();
    evaluateTotalCost();
}

vector<RequestId> Simulation::reqsByCollCnt()
{
    vector<RequestId> reqs;
    RequestId request;
    int overflowCnt;

    while (overflowEdgeCnt > 0 && !requestHeap.empty())
    {
        requestHeap.extract_max(request, overflowCnt);
        reqOverflowEdge[request] = 0;
        set<Edge> newUnderflowEdges = delTrajectoryInRN(request);
        // update number of collisions of each request
        for (const auto &changeEdge: newUnderflowEdges)
        {
            for (const auto &relatedReq: edgeFlowDist[changeEdge.first][changeEdge.second].requests)
            {
                reqOverflowEdge[relatedReq] -= 1;
                requestHeap.update(relatedReq, reqOverflowEdge[relatedReq]);
            }
        }

        overflowEdgeCnt -= newUnderflowEdges.size();
        reqs.push_back(request);
    }
    return reqs;
}

void Simulation::reroutePartialReqsByBlocking()
{
    clearTraffic();

    vector<RequestId> request;
    for (RequestId i = 0; i < traffic.requestIdMap.size(); i++)
        request.push_back(i);

    while (true)
    {
        set<Edge> newOverflowEdges = heuBasedSimulation(request);
        overflowEdgeCnt = newOverflowEdges.size();

        blockOverflowEdges(newOverflowEdges);

        if (overflowEdgeCnt == 0)
            break;

        for (const auto &changeEdge: newOverflowEdges)
        {
            for (const auto &relatedReq: edgeFlowDist[changeEdge.first][changeEdge.second].requests)
            {
                reqOverflowEdge[relatedReq] += 1;
                requestHeap.update(relatedReq, reqOverflowEdge[relatedReq]);
            }
        }

        request = reqsByCollCnt();
        assert(overflowEdgeCnt == 0);
    }

    evaluateTotalCost();
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

void Simulation::blockOverflowEdges(set<Edge> &newOverflowEdges)
{
    for (const auto &edge: newOverflowEdges)
    {
        if (edgeFlowDist[edge.first][edge.second].flow > traffic.capacity)
        {
            traffic.rN.adjListInHeu[edge.second].erase(edge.first);
        }
    }
}

set<Edge> Simulation::addTrajectoryInRN(RequestId request)
{
    set<Edge> newOverflowEdges;
    auto *curLabel = trajectories[request];
    while (curLabel->previous != nullptr)
    {
        edgeFlowDist[curLabel->node_id][curLabel->previous->node_id].requests.insert(request);
        edgeFlowDist[curLabel->node_id][curLabel->previous->node_id].flow += 1;

        if (edgeFlowDist[curLabel->node_id][curLabel->previous->node_id].flow == traffic.capacity + 1)
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
    while (curLabel->previous != nullptr)
    {
        edgeFlowDist[curLabel->node_id][curLabel->previous->node_id].requests.erase(request);
        edgeFlowDist[curLabel->node_id][curLabel->previous->node_id].flow -= 1;

        if (edgeFlowDist[curLabel->node_id][curLabel->previous->node_id].flow == traffic.capacity)
        {
            newUnderflowEdges.insert(make_pair(curLabel->node_id, curLabel->previous->node_id));
        }

        curLabel = curLabel->previous;
    }

    return newUnderflowEdges;
}

float Simulation::trajCostFun1(Label *curLabel)
{
    float cost = 0;
    assert(curLabel != nullptr);
    while (curLabel->previous != nullptr)
    {
        int collision = edgeFlowDist[curLabel->node_id][curLabel->previous->node_id].flow - 1;
        if (collision > 1)
        {
            int base = traffic.rN.getEdgeWeight(curLabel->node_id, curLabel->previous->node_id);
            cost += base * (1 + 0.15 * pow(
                    edgeFlowDist[curLabel->node_id][curLabel->previous->node_id].flow / traffic.capacity, 4));
        }
        curLabel = curLabel->previous;
    }
    return cost;
}


void Simulation::evaluateTotalCost()
{
    for (const auto &sourceLabel: trajectories)
    {
        costs += trajCostFun1(sourceLabel);
    }
}

float Simulation::getCost()
{
    float c = costs;
    return c;
}

void Simulation::clearTraffic()
{
    overflowEdgeCnt = 0;
    costs = 0;

    for (int req = 0; req < traffic.requestIdMap.size(); req++)
    {
        trajectories[req] = nullptr;
        reqOverflowEdge[req] = 0;
    }

    for (NodeId lnode = 0; lnode <= traffic.rN.numNodes; lnode++)
    {
        for (const auto &rnodeW: traffic.rN.adjListOut[lnode])
        {
            edgeFlowDist[lnode][rnodeW.first].requests.clear();
            edgeFlowDist[lnode][rnodeW.first].flow = 0;
        }
    }

    RequestId r;
    int cnt;
    while (!requestHeap.empty())
    {
        requestHeap.extract_max(r, cnt);
    }
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
            ofstream << edgeFlowDist[lnode][rnodeW.first].flow << endl;
        }
    }

    ofstream.close();
}

void Simulation::writeCollision(const basic_string<char> &path, vector<Edge> &overflowEdges)
{
    ofstream ofstream(path);
    float totalColCnt = 0;
    for (const auto edge:overflowEdges)
    {
        totalColCnt += edgeFlowDist[edge.first][edge.second].flow;
        ofstream << edgeFlowDist[edge.first][edge.second].flow << endl;
    }

    ofstream << totalColCnt/overflowEdges.size() << endl;
}