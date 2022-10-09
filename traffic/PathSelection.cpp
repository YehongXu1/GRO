//
// Created by yehong.xu on 25/4/2022.
//

#include "Traffic.h"

vector<RequestId> PathSelection::selectRandomReqs(const TrafficMaintain &traffic, vector<bool> &selected, int rerouteNum) {
    vector<int> toReroute;
    random_device rd; // obtain a random number from hardware
    mt19937 gen(rd()); // seed the generator
    uniform_int_distribution<> distr(0, traffic.reqNo - 1); // define the range

    while (toReroute.size() < rerouteNum) {
        RequestId req = distr(gen);
        while (selected[req])
            req = distr(gen);
        selected[req] = true;
        toReroute.emplace_back(req);
    }

    return toReroute;
}

vector<RequestId> PathSelection::selectWaitingT(const TrafficMaintain &traffic, vector<bool> &selected, int rerouteNum) {
    benchmark2::heapMax<2, unsigned int, RequestId> myReqHeap(traffic.reqNo);
    for (RequestId i = 0; i < traffic.reqNo; i++) {
        if (selected[i])
            continue;

        unsigned int key = traffic.requestODs[i].waitingTime;
        myReqHeap.update(i, key);
    }

    RequestId requestId;
    unsigned int reqKey;
    vector<RequestId> rerouteReqs;
    while (!myReqHeap.empty() && rerouteReqs.size() < rerouteNum) {
        myReqHeap.extract_max(requestId, reqKey);
        if (selected[requestId])
            continue;
        selected[requestId] = true;
        rerouteReqs.emplace_back(requestId);
    }
    return rerouteReqs;
}

vector<RequestId> PathSelection::selectReqPers(TrafficMaintain &traffic, vector<bool> &selected, int rerouteNum) {
    // from req perspective
    benchmark2::heapMax<2, unsigned int, RequestId> myReqHeap(traffic.reqNo);
    for (RequestId i = 0; i < traffic.reqNo; i++) {
        if (selected[i])
            continue;
        unsigned int key = traffic.requestODs[i].waitingTime;
        myReqHeap.update(i, key);
    }

    RequestId requestId;
    unsigned int key;
    vector<RequestId> selectedReqs;
    bool shouldUpdate = false;
    while (selectedReqs.size() < rerouteNum) {
        myReqHeap.extract_max(requestId, key);
        selectedReqs.emplace_back(requestId);
        selected[requestId] = true;

        RequestId nextTop = myReqHeap.top();
        unordered_set<int> &congInfos = traffic.requestODs[requestId].congs;
        for (const auto &congId: congInfos) {
//            int congId = congPair.first;

            traffic.congMap[congId]->relatedReqs.erase(requestId); // no congId would contain selected request anymore
            if (traffic.congMap[congId]->relatedReqs.size() > traffic.congMap[congId]->thresCnt)
                continue;

            // Here, we missed to update requestId's conglist, doing this would incur errors because we are in the loop of its conglist
            for (const auto req: traffic.congMap[congId]->relatedReqs) {
                RequestId reqId = req.first;
                traffic.requestODs[reqId].congs.erase(congId);
                traffic.requestODs[reqId].waitingTime -= traffic.congMap[congId]->relatedReqs[reqId];
            }

            if (!shouldUpdate &&
                traffic.congMap[congId]->relatedReqs.find(nextTop) != traffic.congMap[congId]->relatedReqs.end()) {
                // the rank of nextTop maybe changed
                shouldUpdate = true;
            }
        }

        if (shouldUpdate) {
            myReqHeap.clear();
            for (RequestId i = 0; i < traffic.reqNo; i++) {
                if (selected[i])
                    continue;
                myReqHeap.update(i, traffic.requestODs[i].waitingTime);
            }
            shouldUpdate = false;
        }
    }

    return selectedReqs;
}

vector<RequestId> PathSelection::selectConETPers(TrafficMaintain &traffic, vector<bool> &selected, int rerouteNum) {
    // from congestion site perspective, this one seems better

    benchmark2::heapMax<2, unsigned int, int> congHeap(traffic.congMap.size());
    for (const auto &cong: traffic.congMap) {
        congHeap.update(cong.first, cong.second->lastingTime);
    }

    int congId;
    unsigned int congkey;
    vector<RequestId> selectedReqs;

    // count the number of times a congestion has been selected
    while (selectedReqs.size() < rerouteNum && !congHeap.empty()) {
        congHeap.extract_max(congId, congkey);

        if (congkey == -1)
            continue;

//        congSelectedCnt[congId] += 1;
        // get the biggest congestion contribution query in the congestion
        benchmark2::heapMax<2, unsigned int, RequestId> myReqHeap(traffic.reqNo);
        for (const auto &req: traffic.congMap[congId]->relatedReqs) {
            RequestId reqId = req.first;
            if (selected[reqId]) // have been selected
                continue;

            myReqHeap.update(reqId, req.second);
        }

        RequestId reqId;
        unsigned int reqKey;
        myReqHeap.extract_max(reqId, reqKey);
        selected[reqId] = true;
        selectedReqs.emplace_back(reqId);

        unordered_set<int> &congPairs = traffic.requestODs[reqId].congs;
        for (const auto &curReqCongId: congPairs) {

            CongInfo *curCongInfo = traffic.congMap[curReqCongId];

            if (curCongInfo->relatedReqs.size() - 1 > curCongInfo->thresCnt) {

                // reqKey may be greater than cong's lasting time, as this req may overlap with
                double savedCost = 1.0 * curCongInfo->relatedReqs[reqId] / curCongInfo->relatedReqs.size();
                curCongInfo->lastingTime -= round(savedCost);
                congHeap.update(curReqCongId, curCongInfo->lastingTime);
                curCongInfo->relatedReqs.erase(reqId);
            } else {
                curCongInfo->relatedReqs.erase(reqId);
                congHeap.update(curReqCongId, -1);
                for (const auto req: curCongInfo->relatedReqs) {
                    RequestId reqId = req.first;
                    traffic.requestODs[reqId].congs.erase(curReqCongId);
                }
            }
        }
    }

    return selectedReqs;
}

vector<RequestId> PathSelection::selectionGreedy(const TrafficMaintain &traffic, vector<bool> &selected, int rerouteNum) {
    // the greedy function to find which k paths such that the traffic cost of remaining requests is minimized
    vector<RequestId> reqsToReroute;

    unsigned int leftCost = INT_MAX;
    RequestId reqToReroute = -1;
    while (reqsToReroute.size() < rerouteNum) {
        PathSelection::selectReq(traffic, selected, reqsToReroute, leftCost, reqToReroute);
    }

    return reqsToReroute;
}

void PathSelection::selectReq(
        const TrafficMaintain &traffic, vector<bool> &selected, vector<RequestId> &reqsToReroute,
        unsigned int &leftCost, RequestId &reqToReroute) {
    boost::thread_group tGroup;
    int threadNum = traffic.threadNum;
    int interval = ceil(1.0 * traffic.reqNo / threadNum);
    for (int i = 0; i < threadNum; ++i) {
        int begin = i * interval, end = (i + 1) * interval;
        if (end > traffic.reqNo)
            end = traffic.reqNo;

        tGroup.create_thread(
                [&traffic, &selected, begin, end, &reqToReroute, &leftCost] {
                    return PathSelection::paraRemoveReq(traffic, selected, begin, end, reqToReroute, leftCost);
                }
        );
    }
    tGroup.join_all();

    reqsToReroute.emplace_back(reqToReroute);

    selected[reqToReroute] = true;

    leftCost = INT_MAX;
}


void PathSelection::paraRemoveReq(
        const TrafficMaintain &traffic, vector<bool> &selected, RequestId begin, RequestId end, RequestId &reqToReroute,
        unsigned int &leftCost) {
    for (RequestId req = begin; req < end; req++) {
        PathSelection::removeReq(traffic, selected, req, reqToReroute, leftCost);
    }
}

void PathSelection::removeReq(
        const TrafficMaintain &traffic, vector<bool> &selected, RequestId req, RequestId &reqToReroute, unsigned int &leftCost) {
    unsigned int cost;
    if (selected[req])
        return;

//    chrono::steady_clock::time_point begin = chrono::steady_clock::now();
    cost = PathSelection::simulateTraffic(traffic, selected, req);
//    chrono::steady_clock::time_point end = chrono::steady_clock::now();
//    cout << chrono::duration_cast<chrono::milliseconds>(end - begin).count() << endl;

    Semaphore semaphore;
    semaphore.initialize(1);
    semaphore.wait();
    if (cost < leftCost) {
        leftCost = cost;
        reqToReroute = req;
    }
    semaphore.notify();
}

unsigned int PathSelection::simulateTraffic(const TrafficMaintain &traffic, vector<bool> &selected, RequestId req) {
    // copy footprints of trafficCopy
    vector<unordered_map<NodeId, int>> trafficFlow(traffic.rN.numNodes + 1);
    vector<vector<Label>> footprints = traffic.footprints;

    benchmark2::heapMin<2, unsigned int, RequestId> queue(traffic.reqNo);
    for (RequestId i = 0; i < traffic.reqNo; i++) {
        if (selected[i] || i == req)
            continue;

        vector<Label> &path = footprints[i];

        assert (!path.empty());
        Label *srcL = &path[0];

        queue.update(i, srcL->length);
        for (int j = 0; j < path.size() - 1; j++) {
            trafficFlow[path[j].node_id][path[j + 1].node_id] = 0;
        }
    }

    unsigned int totalC = 0;
    vector<int> positions(traffic.reqNo, 0); // positions[reqId] = i (ith label in footprints[reqId])

    unsigned int curL;
    RequestId i = -1;

    while (!queue.empty()) {
        queue.extract_min(i, curL);

        if (selected[i] || req == i)
            cout << 1;

        int &pos = positions[i];
        NodeId curVId = footprints[i][pos].node_id;

        if (pos > 0) {
            NodeId preVId = footprints[i][pos - 1].node_id;
            trafficFlow[preVId][curVId] -= 1;
        }

        if (pos >= footprints[i].size() - 1) {
            totalC += curL - traffic.requestODs[i].departT;
            continue;
        }

        pos += 1;
        NodeId nextVId = footprints[i][pos].node_id;
        int capacity = traffic.rN.capacity[curVId][nextVId];
        int &toEnterFlow = trafficFlow[curVId][nextVId];
        // leave from current edge, enter next edge, record the time that leaves next edge

        int basecost = traffic.trafficStat[curVId].find(nextVId)->second.baseCost;
        unsigned int cost = Framework::costFunc(
                basecost, toEnterFlow, capacity, traffic.capThres);
        footprints[i][pos].length = curL + cost;
        queue.update(i, footprints[i][pos].length);
        toEnterFlow += 1;
    }

    vector<unsigned int> costs;
    unsigned total = 0;
    for (const auto &path: footprints) {
        total += path[path.size() - 1].length;
        costs.emplace_back(path[path.size() - 1].length);
    }
    return totalC;
}

void PathSelection::writeCongestion(TrafficMaintain &traffic) {
    for (int i = 0; i < traffic.congMap.size(); i++) {
        CongInfo *cong = traffic.congMap[i];
        cout << i << "\t" << cong->relatedReqs.size() << "\t" << cong->t2 - cong->t1 << "\t" << cong->thresCnt << endl;
    }
}

void PathSelection::writeGreedyPath(TrafficMaintain &traffic, vector<RequestId> &toReroute) {
    for (const auto req: toReroute) {
        unordered_set<int> congIds = traffic.requestODs[req].congs;
        cout << traffic.requestODs[req].waitingTime << "\t";

        for (const auto congPair: congIds) {
            cout << congPair << "\t";
        }

        cout << endl;
    }
}