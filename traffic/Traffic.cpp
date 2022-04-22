//
// Created by yehong.xu on 28/12/21.
//

#include "Traffic.h"
#include <fstream>

Traffic::Traffic(RoadNetwork &rN, vector<Request> &requestMap,
                 int timeIntNum, int timeReslo, int penalR, int threadNum, double threshold) : rN(rN),
                                                                                               requestODs(requestMap)
{
    this->timeIntNum = timeIntNum;
    this->timeReslo = timeReslo;
    this->threadNum = threadNum;
    this->threshold = threshold;

    this->reqNo = requestODs.size();
    this->penalR = penalR;

    trafficStat.resize(rN.numNodes + 1);

    boost::thread_group tGroup;
    int interval = ceil(1.0 * (1 + rN.numNodes) / threadNum);
    for (int i = 0; i < this->threadNum; ++i)
    {
        int begin = i * interval, end = (i + 1) * interval;
        if (end > rN.numNodes + 1)
            end = rN.numNodes + 1;
        tGroup.create_thread(boost::bind(&Traffic::initialize, this, begin, end));
    }
    tGroup.join_all();

    trajectories.assign(reqNo, nullptr);

    unordered_set<CongInfo *> v2;
    reqCongCnt.assign(reqNo, v2);
    vector<Label *> v;
    labelCollection.assign(reqNo, v);
}

void Traffic::initialize(int begin, int end)
{
    for (int idx = begin; idx < end; idx++)
    {
        for (const auto &edgeList: rN.adjListOut[idx])
        {
            trafficStat[idx][edgeList.first].tempFlow.assign(timeIntNum, 0);
            trafficStat[idx][edgeList.first].tempWeight.assign(timeIntNum, edgeList.second);
            trafficStat[idx][edgeList.first].baseCost = edgeList.second;
        }
    }
}

void Traffic::allTempDij(vector<RequestId> &reqs)
{
    boost::thread_group tGroup;
    int interval = ceil(1.0 * reqs.size() / threadNum);
    for (int i = 0; i < threadNum; ++i)
    {
        int begin = i * interval, end = (i + 1) * interval;
        if (end >= reqs.size())
            end = reqs.size();
        tGroup.create_thread(boost::bind(
                &Traffic::rangeTempDij, this, boost::ref(reqs), begin, end));
    }
    tGroup.join_all();
}

void ::Traffic::rangeTempDij(vector<RequestId> &reqs, int begin, int end)
{
    for (int i = begin; i < end; i++)
    {
        tempDij(reqs[i]);
    }
}

void Traffic::tempDij(RequestId requestId)
{
    NodeId source = requestODs[requestId].o, target = requestODs[requestId].d;
    int departT = requestODs[requestId].departT;

    benchmark::heap<2, int, NodeId> queue(rN.numNodes + 1);

    unordered_map<NodeId, EdgeProfile>::iterator iterAdj;

    vector<int> heuDistances(rN.numNodes + 1, INT_MAX);
    vector<NodeId> parents(rN.numNodes + 1, -1);
    vector<bool> visited(rN.numNodes + 1, false);
    heuDistances[source] = departT;

    queue.update(source, departT);

    NodeId curNode;
    int heuLength;
    while (!queue.empty())
    {
        queue.extract_min(curNode, heuLength);
        if (curNode == target)
        {
            vector<Label *> labsv;
            NodeId node = curNode;
            while (node != source)
            {
                labsv.emplace_back(new Label(node, 0, nullptr));
                node = parents[node];
            }
            labsv.emplace_back(new Label(node, departT, nullptr));

            for (int i = 0; i < labsv.size() - 1; i++)
            {
                labsv[i]->previous = labsv[i + 1];
            }

            for (int i = 1; i < labsv.size(); i++)
            {
                labsv[i]->nextL = labsv[i - 1];
            }

            trajectories[requestId] = labsv[labsv.size() - 1];
            labelCollection[requestId] = labsv;
            return;
        }

        visited[curNode] = true;

        int timeInt = floor(heuLength / timeReslo);

        if (timeInt >= timeIntNum)
            cout << timeInt << endl;
        assert(timeInt < timeIntNum);

        for (iterAdj = trafficStat[curNode].begin(); iterAdj != trafficStat[curNode].end(); iterAdj++)
        {
            if (visited[iterAdj->first])
                continue;

            int heuNewTravelTime = heuLength + iterAdj->second.tempWeight[timeInt];
            assert(heuNewTravelTime >= 0);

            if (heuDistances[iterAdj->first] > heuNewTravelTime)
            {
                parents[iterAdj->first] = curNode;
                queue.update(iterAdj->first, heuNewTravelTime);
                heuDistances[iterAdj->first] = heuNewTravelTime;
            }
        }
    }
    cout << "target not  found :" << source << " - " << target << endl;
    assert(curNode == target);
}

bool Traffic::detectCycles()
{
    unordered_map<NodeId, unordered_set<NodeId>> adjList(rN.numNodes + 1);
    unordered_set<NodeId> nset;
    for (const auto &label: trajectories)
    {
        if (label == nullptr)
            continue;

        Label *curL = label;
        while (curL->nextL != nullptr)
        {
            adjList[curL->node_id].insert(curL->nextL->node_id);
            nset.insert(curL->node_id);
            curL = curL->nextL;
        }
        nset.insert(curL->node_id);
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

void Traffic::detectCycleRec(NodeId o, unordered_set<NodeId> recS, bool &breakOut,
                             unordered_map<NodeId, bool> &visited,
                             unordered_map<NodeId, unordered_set<NodeId>> &adjList)
{
    if (breakOut)
        return;

    recS.insert(o);
    for (const auto nei: adjList[o])
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

int Traffic::simulateTraffic()
{
    int tempThreshold = 180;

    benchmark2::heapEval<2> queue(reqNo);
    // records request on each edge and sort them by their leaving time
    unordered_map<Edge, benchmark2::heapEval<2>, hash_edge> signList;

    for (RequestId i = 0; i < reqNo; i++)
    {
        Label *srcL = trajectories[i];
        assert (srcL != nullptr);
        srcL->length = requestODs[i].departT; // re-start lengths for all requests

        queue.update(srcL, i);

        Label *label = srcL;
        while (label->nextL != nullptr)
        {
            benchmark2::heapEval<2> _queue(reqNo);
            assert(trafficStat[label->node_id][label->nextL->node_id].liveFlow == 0);
            signList[Edge(label->node_id, label->nextL->node_id)] = _queue;
            edgeCongList[Edge(label->node_id, label->nextL->node_id)] = list<CongInfo>();
            label = label->nextL;
        }
    }

    auto *curL = new Label();
    auto *curLS = new Label();
    RequestId i, j;
    int totalC = 0, id = 0;

    bool deadlock = false;
    chrono::steady_clock::time_point begin = chrono::steady_clock::now();
    while (!queue.empty())
    {
        queue.extract_min(curL, i);
        if (curL->previous != nullptr)
        {
            benchmark2::heapEval<2> &_queue = signList[Edge(curL->previous->node_id, curL->node_id)];
            // here we need to ensure that current req is extracted
            _queue.extract_min(curLS, j);
            assert(i == j && curL->length == curLS->length);
        } else
        {
            requestODs[i].sortBy = 0; // recalculate wait times
        }

        if (curL->nextL == nullptr)
        {
            int &toLeave = trafficStat[curL->previous->node_id][curL->node_id].liveFlow;
            toLeave -= 1;

            list<CongInfo> &congs = edgeCongList[Edge(curL->previous->node_id, curL->node_id)];
            int cap = rN.capacity[rN.edgeMap[curL->previous->node_id][curL->node_id]];
            if (toLeave <= threshold * cap && !congs.empty() && congs.back().t2 == -1)
                congs.back().t2 = curL->length;

            totalC += curL->length - requestODs[i].departT;
            continue;
        }

        int capacity = rN.capacity[rN.edgeMap[curL->node_id][curL->nextL->node_id]];
        int &toEnterFlow = trafficStat[curL->node_id][curL->nextL->node_id].liveFlow;
        benchmark2::heapEval<2> &queue_ = signList[Edge(curL->node_id, curL->nextL->node_id)];
        assert(queue_.size() == toEnterFlow);
        if (toEnterFlow < capacity)
        {

            // leave from current edge, enter next edge, record the time that leaves next edge
            int cost = costFunc(
                    trafficStat[curL->node_id][curL->nextL->node_id].baseCost, toEnterFlow, capacity);

            // did update length of our lables arrival time of curL.nextL
            curL->nextL->length = curL->length + cost;
            queue.update(curL->nextL, i);
            toEnterFlow += 1; // enters this edge

            // sort reqs on running on edge (cur, curNext) by the time they arrive at curL->nextL.node
            queue_.update(curL->nextL, i);
            if (toEnterFlow > capacity * threshold && toEnterFlow > 1)
            {
                list<CongInfo> &congs = edgeCongList[Edge(curL->node_id, curL->nextL->node_id)];

                if (congs.empty() || congs.back().t2 != -1)
                {
                    CongInfo congInfo(
                            id, curL->node_id, curL->nextL->node_id, capacity * threshold, curL->length);
                    id += 1;
                    congs.emplace_back(congInfo);
                    for (int k = 0; k < queue_.size(); k++)
                    {
                        RequestId req = queue_.elements[k].req;
                        congs.back().relatedReqs.insert(req);
                        reqCongCnt[req].insert(&congs.back());
                    }
                } else if (!congs.empty() && congs.back().t2 == -1)
                {
                    congs.back().relatedReqs.insert(i);
                    reqCongCnt[i].insert(&congs.back());
                }
            }

            if (curL->previous != nullptr)
            {
                benchmark2::heapEval<2> &_queue = signList[Edge(curL->previous->node_id, curL->node_id)];
                int &toLeaveFlow = trafficStat[curL->previous->node_id][curL->node_id].liveFlow;
                toLeaveFlow -= 1; // leaves previous edge, may reassign later
                assert(_queue.size() == toLeaveFlow);

                list<CongInfo> &congs = edgeCongList[Edge(curL->previous->node_id, curL->node_id)];
                int cap = rN.capacity[rN.edgeMap[curL->previous->node_id][curL->node_id]];
                if (toLeaveFlow <= threshold * cap && !congs.empty() && congs.back().t2 == -1)
                    congs.back().t2 = curL->length;
            }

        } else
        {
            // curLS.previous.nodeId = curL.nodeId;
            queue_.top(curLS, j);

            requestODs[i].sortBy += curLS->length - curL->length + 1;

            // add 1 is to avoid non-stop recursive, to wait until a car leaves the edge
            curL->length = curLS->length + 1;
            queue.update(curL, i);
            if (curL->previous != nullptr) // this request should re-sign this edge
            {
                benchmark2::heapEval<2> &_queue = signList[Edge(curL->previous->node_id, curL->node_id)];
                signList[Edge(curL->previous->node_id, curL->node_id)].update(curL, i);
            }
        }

        requestODs[i].preStatyAt = curL->node_id;
        chrono::steady_clock::time_point end = chrono::steady_clock::now();
        int time = chrono::duration_cast<chrono::seconds>(end - begin).count();
        if (time > tempThreshold)
        {
            deadlock = true;
            break;
        }
    }

    if (deadlock)
        return -1;

    return totalC;
}

void Traffic::clearEdgeProfile()
{
    boost::thread_group tGroup;
    int interval = ceil(1.0 * reqNo / threadNum);
    for (int i = 0; i < threadNum; i++)
    {
        int begin = i * interval, end = (i + 1) * interval;
        if (end >= reqNo)
            end = reqNo;
        tGroup.create_thread(boost::bind(&Traffic::rangeClearEdgeProfile, this, begin, end));
    }
    tGroup.join_all();

    traversedEdges.clear();

    stuckODs.clear();
}

void Traffic::rangeClearEdgeProfile(int begin, int end)
{
    for (int i = begin; i < end; i++)
    {
        requestODs[i].stuckO = -1;
        requestODs[i].stuckD = -1;
        requestODs[i].preStatyAt = -1;
        Label *curLabel = trajectories[i]; // trajectories[i] of reroute i is null
        if (curLabel == nullptr)
            continue;

        while (curLabel->nextL != nullptr)
        {
            NodeId v1 = curLabel->node_id, v2 = curLabel->nextL->node_id;
            if (rN.edgeSM[rN.edgeMap[v1][v2]].isLocked())
            {
                curLabel = curLabel->nextL;
                continue;
            }
            clearAnEdgeProfile(v1, v2);

            curLabel = curLabel->nextL;
            rN.edgeSM[rN.edgeMap[v1][v2]].notify();
        }
    }
}

void Traffic::clearAnEdgeProfile(NodeId v1, NodeId v2)
{

    rN.edgeSM[rN.edgeMap[v1][v2]].wait();

    trafficStat[v1][v2].liveFlow = 0;
    fill(trafficStat[v1][v2].tempFlow.begin(),
         trafficStat[v1][v2].tempFlow.end(), 0);
    int weight = trafficStat[v1][v2].baseCost;
    fill(trafficStat[v1][v2].tempWeight.begin(),
         trafficStat[v1][v2].tempWeight.end(), weight);

    edgeCongList[Edge(v1, v2)].clear();
}

void Traffic::updateTraversedET()
{
    for (RequestId requestId = 0; requestId < reqNo; requestId++)
    {
        auto *curLabel = trajectories[requestId];
        assert(curLabel != nullptr);
        while (curLabel->nextL != nullptr)
        {
            NodeId v1 = curLabel->node_id, v2 = curLabel->nextL->node_id;
            TimeIntIdx timeInt = floor(curLabel->length / timeReslo);

            if (timeInt >= timeIntNum)
                continue;

            trafficStat[v1][v2].tempWeight[timeInt] += curLabel->nextL->length - curLabel->length;

            trafficStat[v1][v2].tempFlow[timeInt] += 1;

            traversedEdges[Edge(v1, v2)].insert(timeInt);
            curLabel = curLabel->nextL;
        }
    }
}

void Traffic::updateTrafficLoading()
{
    updateTraversedET();
    vector<Edge> record;
    record.reserve(traversedEdges.size());
    for (const auto &i: traversedEdges)
        record.emplace_back(i.first);

    updateHeuWeight(record);
}

void Traffic::updateHeuWeight(vector<Edge> &record)
{
    boost::thread_group tGroup;
    int interval = ceil(1.0 * record.size() / threadNum);
    for (int i = 0; i < this->threadNum; ++i)
    {
        int begin = i * interval, end = (i + 1) * interval;
        if (end > record.size())
            end = record.size();
        tGroup.create_thread(boost::bind(
                &Traffic::updateWeight, this, boost::ref(record), begin, end));
    }
    tGroup.join_all();
}

void Traffic::updateWeight(vector<Edge> &record, int begin, int end)
{
    for (int j = begin; j < end; j++)
    {
        Edge e = record[j];
        NodeId v1 = e.first, v2 = e.second;
        for (const auto &i: traversedEdges[e])
        {
            int flow = trafficStat[v1][v2].tempFlow[i];
            assert (flow > 0);
            int cost = trafficStat[v1][v2].tempWeight[i];
            trafficStat[v1][v2].tempWeight[i] = (cost - trafficStat[v1][v2].baseCost) / flow;
        }
    }
}

int Traffic::costFunc(int baseCost, int edgeFlow, int capacity) const
{
    int cost = floor((baseCost * (1 + 0.15 * pow(penalR * edgeFlow / capacity, 4))));
    assert(cost >= 0);
    return cost;
}

void Traffic::rangeDeleteLabels(vector<RequestId> &reqs, int begin, int end)
{
    for (int idx = begin; idx < end; idx++)
    {
        vector<Label *> &labels = labelCollection[reqs[idx]];
        for (auto &label: labels)
        {
            delete label;
            label = nullptr;
        }
    }
}

void Traffic::deleteLabels(vector<RequestId> &reqs)
{
    boost::thread_group tGroup;
    int interval = ceil(1.0 * reqs.size() / threadNum);
    for (int i = 0; i < threadNum; ++i)
    {
        int begin = i * interval, end = (i + 1) * interval;
        if (end >= reqs.size())
            end = reqs.size();
        tGroup.create_thread(boost::bind(
                &Traffic::rangeDeleteLabels, this, boost::ref(reqs), begin, end));
    }
    tGroup.join_all();
}

void Traffic::rangeClearReqCngs(int begin, int end)
{
    for (RequestId idx = begin; idx < end; idx++)
    {
        unordered_set<CongInfo *> v;
        reqCongCnt[idx].swap(v);
    }
}

void Traffic::clearReqCngs()
{
    boost::thread_group tGroup;
    int interval = ceil(1.0 * reqNo / threadNum);
    for (int i = 0; i < threadNum; ++i)
    {
        int begin = i * interval, end = (i + 1) * interval;
        if (end >= reqNo)
            end = reqNo;
        tGroup.create_thread(boost::bind(&Traffic::rangeClearReqCngs, this, begin, end));
    }
    tGroup.join_all();
}