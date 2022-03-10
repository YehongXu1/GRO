//
// Created by yehong.xu on 28/12/21.
//

#include "Traffic.h"
#include <fstream>

Traffic::Traffic(RoadNetwork &rN, vector<Request> &requestMap,
                 int timeIntNum, int timeReslo, int penalR, int threadNum) : rN(rN), requestODs(requestMap)
{
    this->timeIntNum = timeIntNum;
    this->timeReslo = timeReslo;
    this->threadNum = threadNum;

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
    vector<Label *> v;
    labelCollection.assign(reqNo, v);
    for (RequestId i = 0; i < reqNo; i++)
        labelCollection[i].assign(rN.numNodes + 1, nullptr);
}

void Traffic::initialize(int begin, int end)
{
    for (int idx = begin; idx < end; idx++)
    {
        for (const auto &edgeList: rN.adjListOut[idx])
        {
            trafficStat[idx][edgeList.first].tempFlow.resize(timeIntNum, 0);
            trafficStat[idx][edgeList.first].tempReqs.resize(timeIntNum);
            trafficStat[idx][edgeList.first].tempWeight.resize(timeIntNum, edgeList.second);
            trafficStat[idx][edgeList.first].baseCost = edgeList.second;
            assert(edgeList.second >= 0);
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
        tGroup.create_thread(boost::bind(&Traffic::rangeTempDij, this, boost::ref(reqs), begin, end));
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
    vector<Label *> &labels = labelCollection[requestId];

    NodeId source = requestODs[requestId].o, target = requestODs[requestId].d;

    benchmark::heapDij<2> queue(trafficStat.size());

    unordered_map<NodeId, EdgeProfile>::iterator iterAdj;

    long long int heuNewTravelTime;

    vector<long long int> heuDistances(trafficStat.size(), INT_MAX);

    heuDistances[source] = 0;

    vector<bool> visited(trafficStat.size(), false);
    auto *curLabel = new Label(source, 0);
    curLabel->heuLength = 0;
    queue.update(curLabel);
    labels[source] = curLabel;

    while (!queue.empty())
    {
        queue.extract_min(curLabel);
        if (curLabel->node_id == target)
        {
            Label *ll = curLabel;
            while (ll->previous != nullptr)
            {
                ll->previous->nextL = ll;
                ll = ll->previous;
            }
            trajectories[requestId] = labels[source];
            return;
        }

        visited[curLabel->node_id] = true;

        int timeInt = floor(curLabel->heuLength / timeReslo);

        if (timeInt >= timeIntNum)
            cout << timeInt << endl;
        assert(timeInt < timeIntNum);

        assert(labels[curLabel->node_id] != nullptr);
        labels[curLabel->node_id]->copy(curLabel);

        for (iterAdj = trafficStat[curLabel->node_id].begin();
             iterAdj != trafficStat[curLabel->node_id].end(); iterAdj++)
        {
            if (visited[iterAdj->first])
                continue;

            heuNewTravelTime = curLabel->heuLength + iterAdj->second.tempWeight[timeInt];
            assert(heuNewTravelTime >= 0);

            if (heuDistances[iterAdj->first] > heuNewTravelTime)
            {
                if (labels[iterAdj->first] == nullptr)
                {
                    labels[iterAdj->first] = new Label(iterAdj->first, 0, labels[curLabel->node_id]);
                } else
                {
                    labels[iterAdj->first]->previous = labels[curLabel->node_id];
                }
                labels[iterAdj->first]->heuLength = heuNewTravelTime;
                heuDistances[iterAdj->first] = heuNewTravelTime;
                queue.update(labels[iterAdj->first]);
            }
        }
    }
    cout << "target not  found :" << source << " - " << target << endl;
    assert(curLabel->node_id == target);
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
        tGroup.create_thread(boost::bind(&Traffic::rangeTempDij, this, boost::ref(reqs), begin, end));
    }
    tGroup.join_all();
}

void Traffic::rangeDeleteLabels(vector<RequestId> &reqs, int begin, int end)
{
    for (int idx = begin; idx < end; idx ++)
    {
        vector<Label *> &labels = labelCollection[reqs[idx]];
        for (auto &label:labels)
        {
            delete label;
            label = nullptr;
        }
    }
}

void Traffic::rangeDeleteLabels(vector<Label *> &labels, int begin, int end)
{
    for (NodeId i = 0; i <= rN.numNodes; i++)
    {
        delete labels[i];
        labels[i] = nullptr;
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
        if (srcL == nullptr)
            continue;
        srcL->length = requestODs[i].departT; // re-start lengths for all requsts

        queue.update(srcL, i);

        Label *label = srcL;
        while (label->nextL != nullptr)
        {
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
                    trafficStat[curL->node_id][curL->nextL->node_id].baseCost,
                    trafficStat[curL->node_id][curL->nextL->node_id].liveFlow, capacity);

            curL->nextL->length = curL->length + cost; // did update length of our labels arrival time of curL.nextL
            queue.update(curL->nextL, i);

            // sort reqs on running on edge (cur, curNext) by their arriving time
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

void Traffic::updateTraversedET()
{
    for (RequestId requestId = 0; requestId < reqNo; requestId++)
    {
        if (trajectories[requestId] == nullptr)
            continue;
        auto *curLabel = trajectories[requestId];

        while (curLabel->nextL != nullptr)
        {
            TimeIntIdx timeInt1 = floor(curLabel->length / timeReslo);
            TimeIntIdx timeInt2 = floor(curLabel->nextL->length / timeReslo);

            for (TimeIntIdx i = timeInt1; i < timeInt2; i++)
            {
                assert(i < timeIntNum);
                traversedEdges[Edge(curLabel->node_id, curLabel->nextL->node_id)].emplace_back(
                        make_pair(i, requestId));
            }
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

    updateETProfiles(record);

    updateHeuWeight(record);
    traversedEdges.clear();
}

void Traffic::updateETProfiles(vector<Edge> &record)
{
    boost::thread_group tGroup;
    int interval = ceil(1.0 * record.size() / threadNum);
    for (int i = 0; i < this->threadNum; ++i)
    {
        int begin = i * interval, end = (i + 1) * interval;
        if (end > record.size())
            end = record.size();
        tGroup.create_thread(boost::bind(
                &Traffic::updateRangeETProfiles, this, boost::ref(record), begin, end));
    }
    tGroup.join_all();
}

void Traffic::updateRangeETProfiles(vector<Edge> &record, int begin, int end)
{
    for (int j = begin; j < end; j++)
    {
        Edge e = record[j];
        NodeId v1 = e.first, v2 = e.second;

        for (const auto &pair: traversedEdges[e])
        {
            TimeIntIdx i = pair.first;
            RequestId req = pair.second;
            trafficStat[v1][v2].tempFlow[i] += 1;
            trafficStat[v1][v2].tempReqs[i].insert(req);
        }
    }
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
                &Traffic::updateRangeHeuWeight, this, boost::ref(record), begin, end));
    }
    tGroup.join_all();
}

void Traffic::updateRangeHeuWeight(vector<Edge> &record, int begin, int end)
{
    for (int j = begin; j < end; j++)
    {
        Edge e = record[j];
        NodeId v1 = e.first, v2 = e.second;
        for (const auto &pair: traversedEdges[e])
        {
            TimeIntIdx i = pair.first;
            int flow = trafficStat[v1][v2].tempFlow[i];
            assert (flow > 0);
            int roadId = rN.edgeMap[v1][v2];
            long long int cost = costFunc(trafficStat[v1][v2].baseCost, flow, rN.capacity[roadId]);
            trafficStat[v1][v2].tempWeight[i] = cost;
        }
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

long long int Traffic::costFunc(int baseCost, int edgeFlow, int capacity) const
{
    long long int cost = floor((baseCost * (1 + 0.15 * pow(penalR * edgeFlow / capacity, 4))));
    assert(cost >= 0);
    return cost;
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
}

void Traffic::rangeClearEdgeProfile(int begin, int end)
{
    for (int i = begin; i < end; i++)
    {
        Label *curLabel = labelCollection[i][requestODs[i].o]; // trajectories[i] of reroute i is null
        while (curLabel->nextL != nullptr)
        {
            NodeId v1 = curLabel->node_id, v2 = curLabel->nextL->node_id;
            assert(rN.edgeMap[v1].find(v2) != rN.edgeMap[v1].end());
            if (rN.edgeSM[rN.edgeMap[v1][v2]].isLocked())
            {
                curLabel = curLabel->nextL;
                continue;
            }
            rN.edgeSM[rN.edgeMap[v1][v2]].wait();
            clearAnEdgeProfile(v1, v2);
            curLabel = curLabel->nextL;
            rN.edgeSM[rN.edgeMap[v1][v2]].notify();
        }
    }
}

void Traffic::clearAnEdgeProfile(NodeId v1, NodeId v2)
{
    vector<unordered_set<RequestId>> s(timeIntNum);
    trafficStat[v1][v2].tempReqs.swap(s);
    trafficStat[v1][v2].liveFlow = 0;
    fill(trafficStat[v1][v2].tempFlow.begin(),
         trafficStat[v1][v2].tempFlow.end(), 0);
    int weight = trafficStat[v1][v2].baseCost;
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
