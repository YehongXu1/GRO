//
// Created by yehong.xu on 28/12/21.
//

#include "Traffic.h"
#include <fstream>


Traffic::Traffic(RoadNetwork &rN, vector<Request> &requestMap,
                 int timeIntNum, int timeReslo, int penalR) : rN(rN), requestODs(requestMap)
{
    this->timeIntNum = timeIntNum;
    this->timeReslo = timeReslo;

    unordered_map<NodeId, int> endPointKeyMap;

    this->reqNo = requestODs.size();
    this->penalR = penalR;

    trafficStat.resize(rN.numNodes + 1);
    EdgeList::iterator iterAdj;
    for (NodeId i = 0; i <= rN.numNodes; i++)
    {
        for (iterAdj = rN.adjListOut[i].begin();
             iterAdj != rN.adjListOut[i].end(); iterAdj++)
        {
            trafficStat[i][iterAdj->first].tempFlow.resize(timeIntNum, 0);
            trafficStat[i][iterAdj->first].tempReqs.resize(timeIntNum);
            trafficStat[i][iterAdj->first].tempWeight.resize(timeIntNum, iterAdj->second);
            trafficStat[i][iterAdj->first].modified.resize(timeIntNum, false);
            trafficStat[i][iterAdj->first].weight = iterAdj->second;
            assert(iterAdj->second >= 0);
        }
    }

    trajectories.assign(reqNo, nullptr);
    labelCollection.resize(reqNo);
}

void Traffic::temporalDij(unordered_set<RequestId> &reqs)
{
    for (const auto &req: reqs)
    {
        deleteLabels(req);
        labelCollection[req] = dijkstra_label_timedep(
                trafficStat, timeReslo, timeIntNum,
                requestODs[req].o, requestODs[req].d);

        trajectories[req] = labelCollection[req][requestODs[req].o];
    }
}

void Traffic::deleteLabels(RequestId req)
{
    if (!labelCollection[req].empty())
    {
        for (const auto &ptr: labelCollection[req])
        {
            delete ptr;
        }
    }
}

long long int Traffic::simulateTraffic()
{
    traversedEdges.clear();
    benchmark2::heapEval<2> queue(reqNo);
    // records request on each edge and sort them by their leaving time
    unordered_map<Edge, benchmark2::heapEval<2>, hash_edge> signList;

    for (RequestId i = 0; i < reqNo; i++)
    {
        Label *srcL = trajectories[i];
        assert(srcL != nullptr);
        srcL->length = requestODs[i].departT; // re-start lengths for all requsts

        queue.update(srcL, i);

        Label *label = srcL;
        while (label->nextL != nullptr)
        {
            clearEdgeProfile(label->node_id, label->nextL->node_id);
            benchmark2::heapEval<2> _queue(reqNo);
            signList[Edge(label->node_id, label->nextL->node_id)] = _queue;
            label = label->nextL;
        }
    }

    auto *curL = new Label();
    auto *curLS = new Label();
    RequestId i, j;
    long long totalC = 0;
    while (!queue.empty())
    {
        queue.extract_min(curL, i);

        if (curL->previous != nullptr)
        {
            benchmark2::heapEval<2> &_queue = signList[Edge(curL->previous->node_id, curL->node_id)];
            assert(!_queue.empty());
            // here we need to ensure that current req is extracted
            vector<pair<Label *, RequestId>> buffer;
            long long int minLen = INT_MAX;
            while (true)
            {
                _queue.extract_min(curLS, j);
                if (curLS->length < minLen)
                    minLen = curLS->length;
                if (i == j)
                    break;
                buffer.emplace_back(curLS, j);
            }
            for (const auto &pair: buffer)
            {
                _queue.update(pair.first, pair.second);
            }
            assert(curL->node_id == curLS->node_id);
            assert(curL->length == curLS->length && curL->length == minLen);
            trafficStat[curL->previous->node_id][curL->node_id].liveFlow -= 1; // leaves previous edge, may reassign later
        } else
        {
            requestODs[i].waitTimes = 0; // recalculate wait times
        }

        if (curL->nextL == nullptr)
        {
            totalC += curL->length;
            continue;
        }

        int capacity = rN.capacity[rN.edgeMap[curL->node_id][curL->nextL->node_id]];
        if (trafficStat[curL->node_id][curL->nextL->node_id].liveFlow < capacity)
        {
            // leave from current edge, enter next edge, record the time that leaves next edge
            long long cost = costFunc(
                    trafficStat[curL->node_id][curL->nextL->node_id].weight,
                    trafficStat[curL->node_id][curL->nextL->node_id].liveFlow, capacity);

            curL->nextL->length = curL->length + cost; // did update length of our labels arrival time of curL.nextL
            queue.update(curL->nextL, i);

            //sort reqs on running on edge (cur, curNext) by their leaving time
            signList[Edge(curL->node_id, curL->nextL->node_id)].update(curL->nextL, i);
            trafficStat[curL->node_id][curL->nextL->node_id].liveFlow += 1; // enters this edge
        } else
        {
            // the edge about to enter is full, get the earliest time when a car on the edge leaves
            benchmark2::heapEval<2> _queue = signList[Edge(curL->node_id, curL->nextL->node_id)];
            assert(_queue.size() == capacity);
            _queue.top(curLS, j); // curLS.previous.nodeId = curL.nodeId;
            assert(curLS->length >= curL->length);
            requestODs[i].waitTimes += curLS->length - curL->length + 1;
            curL->length =
                    curLS->length + 1; // add 1 is to avoid non-stop recursive, to wait until a car leaves the edge
            queue.update(curL, i);

            if (curL->previous != nullptr) // this request should re-sign this edge
            {
                signList[Edge(curL->previous->node_id, curL->node_id)].update(curL, i);
                trafficStat[curL->previous->node_id][curL->node_id].liveFlow += 1;
            }
        }
    }

    return totalC;
}

void Traffic::rankReqsByOverFlowETs(
        vector<long long int> &reqColli, benchmark2::heapSelectQ<2, long long int, RequestId> &reqHeap, vector<ET> &overflowETs)
{
    for (const auto &et: overflowETs)
    {
        for (const auto &request: trafficStat[et.v1][et.v2].tempReqs[et.i])
        {
            if (reqColli[request] < 0)
                continue;
            reqColli[request] += 1;
            reqHeap.update(request, reqColli[request]);
        }
    }
}

void Traffic::UpdateTrafficFlow(RequestId request, vector<ET> &overflowETs)
{
    auto *curLabel = trajectories[request];
    assert(curLabel != nullptr);

    while (curLabel->nextL != nullptr)
    {
        clearEdgeProfile(curLabel->node_id, curLabel->nextL->node_id);

        TimeIntIdx timeInt1 = floor(curLabel->length / timeReslo);
        TimeIntIdx timeInt2 = floor(curLabel->nextL->length / timeReslo);

        for (TimeIntIdx i = timeInt1; i <= timeInt2; i++)
        {
            assert(i < timeIntNum);
            traversedEdges[Edge(curLabel->node_id, curLabel->nextL->node_id)].insert(i);
            trafficStat[curLabel->node_id][curLabel->nextL->node_id].tempFlow[i] += 1;
            trafficStat[curLabel->node_id][curLabel->nextL->node_id].tempReqs[i].insert(request);

            int roadId = rN.edgeMap[curLabel->node_id][curLabel->nextL->node_id];
            if (rN.capacity[roadId] == trafficStat[curLabel->node_id][curLabel->nextL->node_id].tempFlow[i])
            {
                overflowETs.emplace_back(ET(curLabel->node_id, curLabel->nextL->node_id, i));
            }
        }
        curLabel = curLabel->nextL;
    }
}

void Traffic::delTrajectoryInRN(RequestId request, vector<ET> &newUnderflowEdges)
{
    auto *curLabel = trajectories[request];
    while (curLabel->nextL != nullptr)
    {
        TimeIntIdx timeInt1 = floor(curLabel->length / timeReslo);
        TimeIntIdx timeInt2 = floor(curLabel->nextL->length / timeReslo);

        for (TimeIntIdx i = timeInt1; i <= timeInt2; i++)
        {
            assert(trafficStat[curLabel->node_id][curLabel->nextL->node_id].tempFlow[i] > 0);
            int roadId = rN.edgeMap[curLabel->node_id][curLabel->nextL->node_id];

            if (trafficStat[curLabel->node_id][curLabel->nextL->node_id].tempFlow[i] == rN.capacity[roadId])
            {
                newUnderflowEdges.emplace_back(ET(curLabel->node_id, curLabel->nextL->node_id, i));
            }
            trafficStat[curLabel->node_id][curLabel->nextL->node_id].tempReqs[i].erase(request);
            trafficStat[curLabel->node_id][curLabel->nextL->node_id].tempFlow[i] -= 1;
        }
        curLabel = curLabel->nextL;
    }
}

void Traffic::updateHeuWeights()
{
    // update et based on edge cost for further temporalDij
    for (const auto &et: traversedEdges)
    {
        for (const auto &i: et.second)
        {
            int flow = trafficStat[et.first.first][et.first.second].tempFlow[i];
            if (flow == 0)
                continue;

            // do not unblock blocked edges
            if (trafficStat[et.first.first][et.first.second].modified[i])
                continue;

            int roadId = rN.edgeMap[et.first.first][et.first.second];
            // does not necessarily, because we only ensure at each time instance flow is under capacity, not each time interval
            // assert(flow <= rN.capacity[roadId]);
            long long int cost = costFunc(
                    trafficStat[et.first.first][et.first.second].weight, flow, rN.capacity[roadId]);
            trafficStat[et.first.first][et.first.second].tempWeight[i] = cost * 0.1; // to avoid
        }
    }
}

long long int Traffic::costFunc(int baseCost, int edgeFlow, int capacity) const
{
    long long int cost = floor((baseCost * (1 + 0.15 * pow(penalR * edgeFlow / capacity, 4))));
    assert(cost >= 0);
    return cost;
}

void Traffic::clearEdgeProfile(NodeId v1, NodeId v2)
{
    vector<unordered_set<RequestId>> s(timeIntNum);
    trafficStat[v1][v2].tempReqs.swap(s);
    trafficStat[v1][v2].liveFlow = 0;
    fill(trafficStat[v1][v2].tempFlow.begin(),
         trafficStat[v1][v2].tempFlow.end(), 0);
    int weight = trafficStat[v1][v2].weight;
    fill(trafficStat[v1][v2].tempWeight.begin(),
         trafficStat[v1][v2].tempWeight.end(), weight);
}

void Traffic::writeSetting(const basic_string<char> &path, const basic_string<char> &rNName) const
{
    ofstream ofstream(path);
    ofstream << "map: " << rNName << "; no. requests: " << reqNo << endl;
    for (const auto &req: requestODs)
    {
        ofstream << req.o << "\t" << req.d << endl;
    }

    ofstream.close();
}

void Traffic::writeTrajectories(const basic_string<char> &path)
{
    ofstream ofstream(path);

    if (trajectories.empty())
    {
        ofstream << "no result" << endl;
        return;
    }

    for (const auto &targetL: trajectories)
    {
        Label *label = targetL;
        while (label != nullptr)
        {
            ofstream << rN.coords[label->node_id].first << "," << rN.coords[label->node_id].second
                     << endl;
            label = label->previous;
        }
    }
    ofstream.close();
}

void Traffic::writeCollision(const basic_string<char> &path, vector<Edge> &overflowEdges)
{
    ofstream ofstream(path);
    float totalColCnt = 0;
    for (const auto &edge: overflowEdges)
    {
        totalColCnt += trafficStat[edge.first][edge.second].liveFlow;
        ofstream << trafficStat[edge.first][edge.second].liveFlow << endl;
    }

    ofstream << totalColCnt / overflowEdges.size() << endl;
    ofstream.close();
}
