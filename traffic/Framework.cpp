//
// Created by yehong.xu on 1/2/2022.
//

#include "Traffic.h"

void Framework::allTempDij(TrafficMaintain &traffic, vector<RequestId> &reqs, bool &allGood) {
    boost::thread_group tGroup;
    int interval = ceil(1.0 * reqs.size() / traffic.threadNum);

    for (int i = 0; i < traffic.threadNum; ++i) {
        int begin = i * interval, end = (i + 1) * interval;
        if (end >= reqs.size()) end = reqs.size();
        tGroup.create_thread(
                boost::bind(&Framework::rangeTempDij, boost::ref(traffic),
                            boost::ref(reqs), begin, end, boost::ref(allGood)));
//        if (!allGood) return;

    }
    tGroup.join_all();
}

void Framework::rangeTempDij(TrafficMaintain &traffic, vector<RequestId> &reqs, int begin, int end, bool &allGood) {
    for (int i = begin; i < end; i++) {
        tempDij(traffic, reqs[i]);
        if (traffic.footprints[reqs[i]].empty()) { // did not find a valid path
            allGood = false;
            int c = Dij(traffic, reqs[i]);
        }
    }
}

void Framework::tempDij(TrafficMaintain &traffic, RequestId requestId) {
    traffic.footprints[requestId].clear();

    NodeId source = traffic.requestODs[requestId].o, target = traffic.requestODs[requestId].d;
    int departT = traffic.requestODs[requestId].departT;

    benchmark::heap<2, long int, NodeId> queue(traffic.rN.numNodes + 1);

    vector<long int> heuDistances(traffic.rN.numNodes + 1, UINT_MAX);
    vector<NodeId> parents(traffic.rN.numNodes + 1, -1);
    vector<bool> visited(traffic.rN.numNodes + 1, false);
    heuDistances[source] = departT;

    queue.update(source, departT);
    Timer clock;
    NodeId curNode;
    long int heuLength;

    clock.tick();
    while (!queue.empty()) {
        queue.extract_min(curNode, heuLength);
        if (curNode == target) {
            clock.tock();
            traffic.requestODs[requestId].temDijTime = clock.duration().count();
            vector<Label> labsv;
            NodeId node = curNode;
            while (node != source) {
                // this length maybe useless
                labsv.emplace_back(Label(node, 0, nullptr));
                node = parents[node];
            }
            labsv.emplace_back(Label(node, departT, nullptr));

            vector<Label> &path = traffic.footprints[requestId];
            path.clear();
            for (int i = labsv.size() - 1; i >= 0; i--) {
                path.emplace_back(labsv[i]);
            }
            return;
        }

        visited[curNode] = true;

        TimeIntIdx timeInt = traffic.getTimeInt(heuLength);
        if (timeInt >= traffic.timeIntNum)
            return;
//        assert(timeInt < traffic.timeIntNum);

        for (const auto &oppV: traffic.rN.adjListOut[curNode]) {
            if (visited[oppV.first])
                continue;

            long int heuNewTravelTime = heuLength + traffic.trafficStat[curNode][oppV.first].tempWeight[timeInt];

            if (heuNewTravelTime < 0) {
                cout << "error" << endl;
                return;
            }

            if (heuDistances[oppV.first] > heuNewTravelTime) {
                parents[oppV.first] = curNode;
                queue.update(oppV.first, heuNewTravelTime);
                heuDistances[oppV.first] = heuNewTravelTime;
            }

        }
    }
    clock.tock();
}

void Framework::allDij(TrafficMaintain &traffic, vector<RequestId> &reqs, long int &cost) {
    boost::thread_group tGroup;
    int interval = ceil(1.0 * reqs.size() / traffic.threadNum);

    vector<long int> eachReqCost(traffic.reqNo, 0);

    for (int i = 0; i < traffic.threadNum; ++i) {
        int begin = i * interval, end = (i + 1) * interval;
        if (end >= reqs.size()) end = reqs.size();
        tGroup.create_thread(
                boost::bind(&Framework::rangeDij, boost::ref(traffic),
                            boost::ref(reqs), begin, end, boost::ref(eachReqCost)));

    }
    tGroup.join_all();

    for (auto const c: eachReqCost) {
        cost += c;
    }
}

void Framework::rangeDij(TrafficMaintain &traffic,
                         vector<RequestId> &reqs, int begin, int end, vector<long int> &cost) {
    vector<long int> times;
    for (int i = begin; i < end; i++) {
        cost[reqs[i]] = Dij(traffic, reqs[i]);
    }
}

int Framework::Dij(TrafficMaintain &traffic, RequestId requestId) {
    traffic.footprints[requestId].clear();

    NodeId source = traffic.requestODs[requestId].o, target = traffic.requestODs[requestId].d;
    int departT = traffic.requestODs[requestId].departT;

    benchmark::heap<2, int, NodeId> queue(traffic.rN.numNodes + 1);

    vector<int> distances(traffic.rN.numNodes + 1, INT32_MAX);
    vector<NodeId> parents(traffic.rN.numNodes + 1, -1);
    vector<bool> visited(traffic.rN.numNodes + 1, false);
    distances[source] = departT;

    queue.update(source, departT);

    NodeId curNode;
    int distFromO;
    while (!queue.empty()) {
        queue.extract_min(curNode, distFromO);
        if (curNode == target) {
            vector<Label> labsv;
            NodeId node = curNode;
            while (node != source) {
                // this length maybe useless
                labsv.emplace_back(Label(node, 0, nullptr));
                node = parents[node];
            }
            labsv.emplace_back(Label(node, departT, nullptr));

            vector<Label> &path = traffic.footprints[requestId];
            path.clear();
            for (int i = labsv.size() - 1; i >= 0; i--) {
                path.emplace_back(labsv[i]);
            }
            return distances[curNode] - departT;
        }

        visited[curNode] = true;

        for (const auto &oppV: traffic.rN.adjListOut[curNode]) {
            if (visited[oppV.first])
                continue;

            int heuNewTravelTime = distFromO + traffic.trafficStat[curNode][oppV.first].baseCost;

            if (distances[oppV.first] > heuNewTravelTime) {
                parents[oppV.first] = curNode;
                queue.update(oppV.first, heuNewTravelTime);
                distances[oppV.first] = heuNewTravelTime;
            }
        }
    }
    return -100;
}

long int Framework::simulateTraffic(TrafficMaintain &traffic) {
    traffic.clearCongs();
    traffic.revertTrafficCondition();

    benchmark2::heapMin<2, long int, RequestId> queue(traffic.reqNo);

    vector<unordered_map<NodeId, unordered_set<RequestId>>> signList(traffic.rN.numNodes + 1);

    for (RequestId i = 0; i < traffic.reqNo; i++) {
        vector<Label> &path = traffic.footprints[i];
        if (path.empty()) continue;
        path[0].length = traffic.requestODs[i].departT; // re-start lengths for all requests
        traffic.requestODs[i].waitingTime = 0;
        traffic.requestODs[i].cost = 0;

        queue.update(i, path[0].length);
        for (int j = 0; j < path.size() - 1; j++) {
            assert(traffic.trafficStat[j][j + 1].liveFlow == 0);
            // initialize a congestion list for every traversed edge
            traffic.edgeCongList[Edge(path[j].node_id, path[j + 1].node_id)] = list<CongInfo>();
            signList[path[j].node_id][path[j + 1].node_id] = unordered_set<RequestId>{};
        }
    }

    long int totalC = 0;
    int congId = 0;
    // record the progress of every query, e.g. positions[1] = 0, 1st query is visiting 0-th vertex of its footprint
    vector<int> positions(traffic.reqNo, 0);

    long int curL;
    RequestId i = -1;
    vector<unordered_set<NodeId>> affectedEdges(traffic.rN.numNodes + 1);
    while (!queue.empty()) {
        queue.extract_min(i, curL);
        int &pos = positions[i];
        NodeId preVId = -1, curVId = traffic.footprints[i][pos].node_id, nextVId = -1;

        if (pos > 0) {
            preVId = traffic.footprints[i][pos - 1].node_id;

            // exit previous edge
            signList[preVId][curVId].erase(i);

            int &toLeaveFlow = traffic.trafficStat[preVId][curVId].liveFlow;
            toLeaveFlow -= 1;

            int cap = traffic.rN.capacity[preVId][curVId];
            list<CongInfo> &congs = traffic.edgeCongList[Edge(preVId, curVId)];
            if (!congs.empty() && congs.back().t2 == -1) {
                // there is an ongoing congestion

                if (toLeaveFlow <= cap * traffic.threshold) {
                    // this congestion is terminated by current req, all requests in it has same exit time
                    congs.back().t2 = curL;
                    congs.back().lastingTime = curL - congs.back().t1;
                    for (const auto &reqId: signList[preVId][curVId]) {
                        assert(congs.back().relatedReqs.find(reqId) != congs.back().relatedReqs.end());
                        congs.back().relatedReqs[reqId] = curL - congs.back().relatedReqs[reqId];
                    }
                } else {
                    // this congestion has not terminated, but current req exits
                    congs.back().relatedReqs[i] = curL - congs.back().relatedReqs[i];
                }
            }
            // otherwise, req is not in any congestion
        }

        if (pos == traffic.footprints[i].size() - 1) {
            // this query reaches its destination
            totalC += curL - traffic.requestODs[i].departT;
            traffic.requestODs[i].cost = curL - traffic.requestODs[i].departT;
            continue;
        }

        nextVId = traffic.footprints[i][pos + 1].node_id;

        int capacity = traffic.rN.capacity[curVId][nextVId];
        int &toEnterFlow = traffic.trafficStat[curVId][nextVId].liveFlow;
        pos += 1;
        // leave from current edge, enter next edge, record the time that leaves next edge
        int baseCost = traffic.trafficStat[curVId][nextVId].baseCost;
        long int cost = costFunc(baseCost, toEnterFlow, capacity, traffic.capThres);
        traffic.footprints[i][pos].length = curL + cost;
        traffic.requestODs[i].waitingTime += cost - baseCost;

        signList[curVId][nextVId].insert(i);

        if (toEnterFlow > capacity * traffic.threshold) {
            list<CongInfo> &congs = traffic.edgeCongList[Edge(curVId, nextVId)];
            if (congs.empty() || congs.back().t2 != -1) {
                CongInfo congInfo(
                        congId, curVId, nextVId, capacity * traffic.threshold, curL);

                congId += 1;

                congs.emplace_back(congInfo);
                for (auto &relatedReq: signList[curVId][nextVId]) {
                    congs.back().relatedReqs[relatedReq] = curL;
                    traffic.requestODs[relatedReq].congs.insert(congs.back().id);
                }
                traffic.congMap.insert(make_pair(congs.back().id, &congs.back()));

            } else if (!congs.empty() && congs.back().t2 == -1) {
                congs.back().relatedReqs[i] = curL;
                traffic.requestODs[i].congs.insert(congs.back().id);
            }
        }

        queue.update(i, traffic.footprints[i][pos].length);
        toEnterFlow += 1;

        TimeIntIdx timeInt = traffic.getTimeInt(curL);

        if (timeInt >= traffic.timeIntNum)
            continue;

        traffic.trafficStat[curVId][nextVId].tempWeight[timeInt] += cost;
        traffic.trafficStat[curVId][nextVId].tempFlow[timeInt] += 1;

        traffic.affectedHistBins[curVId][nextVId].emplace(timeInt);
        affectedEdges[curVId].emplace(nextVId);

    }

    for (int leftV = 0; leftV < affectedEdges.size(); leftV++)
    {
        for (const auto &rightV: affectedEdges[leftV])
        {
            traffic.traversedE.emplace_back(leftV, rightV);
        }
    }

    traffic.calculateTimeDepEdgeWeight();
    return totalC;
}

long int Framework::simulateTrafficNoCong(TrafficMaintain &traffic) {
    traffic.clearCongs();
    traffic.revertTrafficCondition();

    benchmark2::heapMin<2, long int, RequestId> queue(traffic.reqNo);


    for (RequestId i = 0; i < traffic.reqNo; i++) {
        vector<Label> &path = traffic.footprints[i];
        if (path.empty()) continue;
        path[0].length = traffic.requestODs[i].departT; // re-start lengths for all requests
        traffic.requestODs[i].waitingTime = 0;
        traffic.requestODs[i].cost = 0;
        queue.update(i, path[0].length);
    }

    long int totalC = 0;
    // record the progress of every query, e.g. positions[1] = 0, 1st query is visiting 0-th vertex of its footprint
    vector<int> positions(traffic.reqNo, 0);

    long int curL;
    RequestId i = -1;

    vector<unordered_set<NodeId>> affectedEdges(traffic.rN.numNodes + 1);
    while (!queue.empty()) {
        queue.extract_min(i, curL);
        int &pos = positions[i];
        NodeId preVId = -1, curVId = traffic.footprints[i][pos].node_id, nextVId = -1;

        if (pos > 0) {
            preVId = traffic.footprints[i][pos - 1].node_id;
            traffic.trafficStat[preVId][curVId].liveFlow -= 1;
        }

        if (pos == traffic.footprints[i].size() - 1) {
            // this query reaches its destination
            totalC += curL - traffic.requestODs[i].departT;
            traffic.requestODs[i].cost = curL - traffic.requestODs[i].departT;
            continue;
        }

        nextVId = traffic.footprints[i][pos + 1].node_id;

        int capacity = traffic.rN.capacity[curVId][nextVId];
        int &toEnterFlow = traffic.trafficStat[curVId][nextVId].liveFlow;
        // leave from current edge, enter next edge, record the time that leaves next edge
        int baseCost = traffic.trafficStat[curVId][nextVId].baseCost;
        long int cost = costFunc(baseCost, toEnterFlow, capacity, traffic.capThres);

        pos += 1;
        traffic.footprints[i][pos].length = curL + cost;
        traffic.requestODs[i].waitingTime += cost - baseCost;

        queue.update(i, curL + cost);
        toEnterFlow += 1;

        TimeIntIdx timeInt = traffic.getTimeInt(curL);

        if (timeInt >= traffic.timeIntNum)
            continue;

        traffic.trafficStat[curVId][nextVId].tempWeight[timeInt] += cost;
        traffic.trafficStat[curVId][nextVId].tempFlow[timeInt] += 1;

        traffic.affectedHistBins[curVId][nextVId].emplace(timeInt);
        affectedEdges[curVId].emplace(nextVId);
    }

    for (int leftV = 0; leftV < affectedEdges.size(); leftV++)
    {
        for (const auto &rightV: affectedEdges[leftV])
        {
            traffic.traversedE.emplace_back(leftV, rightV);
        }
    }

    traffic.calculateTimeDepEdgeWeight();
    return totalC;
}

long int Framework::costFunc(int baseCost, int edgeFlow, int capacity, int capThres) {
    long int cost;
    if (capacity <= capThres) {
        cost = baseCost;
    } else {
        cost = floor((baseCost * (1 + 0.15 * pow(edgeFlow / capacity, 4))));
    }
    assert(cost >= 0);
    return cost;
}

