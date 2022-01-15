//
// Created by yehong.xu on 28/12/21.
//

#include "Traffic.h"
#include <fstream>


void Traffic::writeSetting(const basic_string<char> &path, const basic_string<char> &rNName) const
{
    ofstream ofstream(path);
    ofstream << "capacity: " << capacity << "; no. requests: " << reqNo << endl;
    for (const auto od: requestIdMap)
    {
        ofstream << rN.coords[od.second.first].first << "\t" << rN.coords[od.second.first].second << endl;
        ofstream << rN.coords[od.second.second].first << "\t" << rN.coords[od.second.second].second << endl;
    }

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

void Simulation::routing(unordered_set<RequestId> &reqs)
{
    for (const auto &req: reqs)
    {
        auto *sourceLabel = new Label(traffic.requestIdMap[req].first, 0);
        dijkstra_label_timedep(trafficStat, timeReslo, timeIntNum, traffic.requestIdMap[req].first,
                               traffic.requestIdMap[req].second, sourceLabel);

        assert(sourceLabel != nullptr);
        trajectories[req] = sourceLabel;
    }
}

long long int Simulation::test()
{
    /**
     * Based on base cost // first iteration
     */

    cout << "test" << endl;

    unordered_set<RequestId> reqs(traffic.reqNo);
    benchmark::heap<2, int, RequestId> reqHeap(traffic.reqNo);
    vector<int> reqCNT(traffic.reqNo, 0);

    for (RequestId req = 0; req < traffic.reqNo; req++)
        reqs.insert(req);
    routing(reqs);

    set<pair<Edge, TimeIntIdx>> overflowETs;
    for (const auto &req: reqs)
    {
        addTrajectoryInRN(req, overflowETs);
    }

//    updateEdgeCost();
    long int curCost = getCost();

    // update rank of reqs
    set<pair<Edge, TimeIntIdx>>::iterator et;
    for (et = overflowETs.begin(); et != overflowETs.end(); et++)
    {
        for (const auto req: trafficStat[et->first.first][et->first.second].tempReqs[et->second])
        {
            reqCNT[req] += 1;
            reqHeap.update(req, reqCNT[req]);
        }
    }

    for (const auto &req: reqs)
    {
        vector<pair<Edge, TimeIntIdx>> del = delTrajectoryInRN(req);
        for (const auto et: del)
        {
            for (const auto req: trafficStat[et.first.first][et.first.second].tempReqs[et.second])
            {
                reqCNT[req] -= 1;
                reqHeap.update(req, reqCNT[req]);
            }
        }
    }
    return curCost;
}


long long int Simulation::rerouteAllPaths()
{
    // do not voluntary to adjust edge weight, just update edge weight based on previous traffic status
    cout << "basic simulation" << endl;

    unordered_set<RequestId> reqs(traffic.reqNo);
    for (RequestId req = 0; req < traffic.reqNo; req++)
        reqs.insert(req);

    int iteration = 0;
    long long int curCost;
    while (true)
    {
        routing(reqs);

        // clear traffic state of last iteration
        clearTraffic();

        // update the traffic state and edge cost based on given trajectories
        set<pair<Edge, TimeIntIdx>> overflowETs;
        for (const auto &req: reqs)
            addTrajectoryInRN(req, overflowETs);
        updateEdgeCost();

        // evaluate travel cost based on new traffic state
        curCost = getCost();
        cout << curCost << endl;

        iteration += 1;

        if (iteration == 100)
            return curCost;
    }
}

void Simulation::addTrajectoryInRN(RequestId request, set<pair<Edge, TimeIntIdx>> &newOverflowEdges)
{
    auto *curLabel = trajectories[request];
    assert(curLabel != nullptr);

    while (curLabel->previous != nullptr)
    {
        traversedEdges.insert(make_pair(curLabel->previous->node_id, curLabel->node_id));

        trafficStat[curLabel->previous->node_id][curLabel->node_id].totalFlow += 1;

        TimeIntIdx timeInt = curLabel->previous->length / timeReslo;
        TimeIntIdx timeInt2 = curLabel->length / timeReslo;

        for (TimeIntIdx i = timeInt; i <= timeInt2; i++)
        {
            trafficStat[curLabel->previous->node_id][curLabel->node_id].tempFlow[i] += 1;
            trafficStat[curLabel->previous->node_id][curLabel->node_id].tempReqs[i].insert(request);

            if (trafficStat[curLabel->previous->node_id][curLabel->node_id].tempFlow[i] == traffic.capacity)
            {
                newOverflowEdges.insert(make_pair(make_pair(curLabel->previous->node_id, curLabel->node_id), i));
            }
        }
        curLabel = curLabel->previous;
    }
}

long long int Simulation::rerouteAllBlockETs()
{
    cout << "Block overflow (e,t) and reroute all paths" << endl;

    unordered_set<RequestId> reqs(traffic.reqNo);
    for (RequestId req = 0; req < traffic.reqNo; req++)
        reqs.insert(req);

    int iteration = 0;
    long long int curCost;
    while (true)
    {
        routing(reqs);

        // clear (e,t)s from last iteration
        clearTraffic();

        // update the traffic state and edge cost based on given trajectories
        set<pair<Edge, TimeIntIdx>> overflowETs;
        for (const auto &req: reqs)
        {
            addTrajectoryInRN(req, overflowETs);
        }

        // update (e,t) based on current set of trajectories
        updateEdgeCost();

        // evaluate travel cost based on new traffic state
        curCost = getCost();
        cout << curCost << endl;

        iteration += 1;
        if (iteration == 100)
            return curCost;

        // voluntarily block (e,t)s that are overflow based on current set of trajectories
        blockOverflowETsRandom(overflowETs);
    }
}

long long int Simulation::reroutePartialBlockETs(double frac)
{
    cout << "Block overflow (e,t) and reroute partial paths" << endl;
    benchmark::heap<2, int, RequestId> reqHeap(traffic.reqNo);
    vector<int> reqCNT(traffic.reqNo, 0); // overflow et cnt of deleted paths are wrong (set to 0)

    unordered_set<RequestId> reqs(traffic.reqNo);
    vector<bool> fixed(traffic.reqNo, false);
    for (RequestId req = 0; req < traffic.reqNo; req++)
    {
        reqs.insert(req);
    }

    int iteration = 0, unfixedCnt = traffic.reqNo, preUnfixedCnt;
    long long curCost = 0;
    set<pair<Edge, TimeIntIdx>> overflowETs;

    while (iteration < 100)
    {
        routing(reqs); // reroute reqs that are selected to be reroute

        for (const auto req: reqs)
        {
            addTrajectoryInRN(req, overflowETs);
        }

        // update (e,t) based on current set of trajectories
        updateEdgeCost();

        // evaluate travel cost based on new traffic state
        curCost = getCost();

        // update rank of reqs
        set<pair<Edge, TimeIntIdx>>::iterator et;
        for (et = overflowETs.begin(); et != overflowETs.end(); et++)
        {
            for (const auto &req: trafficStat[et->first.first][et->first.second].tempReqs[et->second])
            {
                reqCNT[req] += 1;
                reqHeap.update(req, reqCNT[req]);
            }
        }

        cout << curCost << endl;
        preUnfixedCnt = unfixedCnt;
        reqs = selectReqsBasedOnETCnt(overflowETs, reqCNT, reqHeap, unfixedCnt, frac);
        unfixedCnt -= preUnfixedCnt - reqs.size();

        blockOverflowETsRandom(overflowETs);
        iteration += 1;
    }

    return curCost;
}

unordered_set<RequestId> Simulation::selectReqsBasedOnETCnt(
        set<pair<Edge, TimeIntIdx>> &overflowETs, vector<int> &reqCNT,
        benchmark::heap<2, int, RequestId> &reqHeap, int unfixedCnt, double frac)
{
    unordered_set<RequestId> reqs;
    int curReqCnt;
    RequestId maxReq;

    double cnt = 0;
    while (cnt / unfixedCnt < frac && !reqHeap.empty())
    {
        reqHeap.extract_max(maxReq, curReqCnt);
        reqCNT[maxReq] = 0;
        cnt += 1;

        // if (maxReq is fixed, do not )
        reqs.insert(maxReq);
        vector<pair<Edge, TimeIntIdx>> newUnderflowEdges = delTrajectoryInRN(maxReq);
        for (const auto &ET: newUnderflowEdges)
        {
            for (const auto &delReq: trafficStat[ET.first.first][ET.first.second].tempReqs[ET.second])
            {
                reqCNT[delReq] -= 1;
                reqHeap.update(delReq, reqCNT[delReq]);

                overflowETs.erase(ET);
            }
        }
    }
    for (RequestId req = 0; req < traffic.reqNo; req++)
    {
        if (reqs.find(req) == reqs.end())
            reqHeap.update(req, 0);
    }
    return reqs;
}

vector<pair<Edge, TimeIntIdx>> Simulation::delTrajectoryInRN(RequestId request)
{
    auto *curLabel = trajectories[request];

    vector<pair<Edge, TimeIntIdx>> newUnderflowEdges;
    while (curLabel->previous != nullptr)
    {
        assert(trafficStat[curLabel->previous->node_id][curLabel->node_id].totalFlow > 0);
        trafficStat[curLabel->previous->node_id][curLabel->node_id].totalFlow -= 1;
        if (trafficStat[curLabel->previous->node_id][curLabel->node_id].totalFlow == 0)
        {
            traversedEdges.erase(make_pair(curLabel->previous->node_id, curLabel->node_id));
        }

        TimeIntIdx timeInt = curLabel->previous->length / timeReslo;
        TimeIntIdx timeInt1 = curLabel->length / timeReslo;

        for (TimeIntIdx i = timeInt; i <= timeInt1; i++)
        {
            assert(trafficStat[curLabel->previous->node_id][curLabel->node_id].tempFlow[i] > 0);

            if (trafficStat[curLabel->previous->node_id][curLabel->node_id].tempFlow[i] == traffic.capacity)
            {
                newUnderflowEdges.emplace_back(make_pair(make_pair(curLabel->previous->node_id, curLabel->node_id), i));
            }
            trafficStat[curLabel->previous->node_id][curLabel->node_id].tempReqs[i].erase(request);

            trafficStat[curLabel->previous->node_id][curLabel->node_id].tempFlow[i] -= 1;
        }
        curLabel = curLabel->previous;
    }

    return newUnderflowEdges;
}

void Simulation::blockOverflowETs(set<pair<Edge, TimeIntIdx>> &overflowETs)
{
    set<pair<Edge, TimeIntIdx>>::iterator et;
    for (et = overflowETs.begin(); et != overflowETs.end(); et++)
    {
        int cnt = trafficStat[et->first.first][et->first.second].tempFlow[et->second];
        assert (cnt >= traffic.capacity);

        trafficStat[et->first.first][et->first.second].tempWeight[et->second] = INT_MAX;
    }
}

void Simulation::blockOverflowETsRandom(set<pair<Edge, TimeIntIdx>> &overflowETs)
{
    set<pair<Edge, TimeIntIdx>>::iterator et;
    for (et = overflowETs.begin(); et != overflowETs.end(); et++)
    {
        int cnt = trafficStat[et->first.first][et->first.second].tempFlow[et->second];
        assert (cnt >= traffic.capacity);
        if (randomBool(cnt - traffic.capacity + 1, 0.5))
            trafficStat[et->first.first][et->first.second].tempWeight[et->second] = INT_MAX;
    }
}

void Simulation::clearTraffic()
{
    for (const auto &edge: traversedEdges)
    {
        vector<unordered_set<RequestId>> s(timeIntNum);
        trafficStat[edge.first][edge.second].tempReqs.swap(s);
        trafficStat[edge.first][edge.second].totalFlow = 0;
        fill(trafficStat[edge.first][edge.second].tempFlow.begin(),
             trafficStat[edge.first][edge.second].tempFlow.end(), 0);
        fill(trafficStat[edge.first][edge.second].tempWeight.begin(),
             trafficStat[edge.first][edge.second].tempWeight.end(), trafficStat[edge.first][edge.second].weight);
    }
}

void Simulation::updateEdgeCost()
{
    // update heuweight based on real edge cost
    for (const auto &edge: traversedEdges)
    {
        for (int i = 0; i < timeIntNum; i++)
        {
            if (trafficStat[edge.first][edge.second].tempFlow[i] == 0)
                continue;

            long int cost = trajCostFun1(trafficStat[edge.first][edge.second].weight,
                                         trafficStat[edge.first][edge.second].tempFlow[i]);
            trafficStat[edge.first][edge.second].tempWeight[i] = cost;
        }
    }
}

int Simulation::trajCostFun1(int baseCost, int edgeFlow)
{
    int cost = (int) (baseCost * (1 + 0.15 * pow(edgeFlow / traffic.capacity, 4)));
    return cost;
}

long long int Simulation::getCost()
{
    long long int cost = 0;
    for (auto label: trajectories)
    {
        while (label->previous != nullptr)
        {
            TimeIntIdx timeInt = label->previous->length / timeReslo;
            assert(timeInt < timeIntNum && timeInt >= 0);

            cost += trafficStat[label->previous->node_id][label->node_id].tempWeight[timeInt];
            label = label->previous;
        }
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
    for (const auto &edge: overflowEdges)
    {
        totalColCnt += trafficStat[edge.first][edge.second].totalFlow;
        ofstream << trafficStat[edge.first][edge.second].totalFlow << endl;
    }

    ofstream << totalColCnt / overflowEdges.size() << endl;
}

