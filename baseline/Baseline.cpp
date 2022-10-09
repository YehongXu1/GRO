//
// Created by Yehong Xu on 4/10/2022.
//

#include "Baseline.h"
Baseline::Baseline(RoadNetwork &rN, int threadNum, int capThreshold) : rN(rN) {
    this->threadNum = threadNum;
    this->capThres = capThreshold;
}

void Baseline::getRequests(const vector<Request> &requests) {
    this->requests = requests;
}

void Baseline::getUniqODs() {
    assert(!requests.empty());

    unordered_map<NodeId, unordered_map<NodeId, vector<RequestId>>> ods;
    for (const auto &req: requests) {
        if (ods.find(req.o) == ods.end()) {
            ods[req.o] = unordered_map<NodeId, vector<RequestId>>();
        }
        ods[req.o][req.d].push_back(req.id);
    }

    reqInvLst.resize(requests.size());
    int odIDd = 0;
    for (const auto &od: ods) {
        for (const auto &d: od.second) {
            uniqODs.emplace_back(od.first, d.first);
            for (const auto &reqID: d.second) {
                reqInvLst[reqID] = odIDd;
            }
            odIDd++;
        }
    }

}

void Baseline::svpBaseLine(vector<vector<NodeId>> &pathResults, int k, double theta) {
    if (uniqODs.empty())
        getUniqODs();

    vector<Path> v;
    vector<vector<Path>> candiPathsForAllODs(uniqODs.size(), v);
    svpKSP(candiPathsForAllODs, k, theta);

    vector<NodeId> x;
    pathResults.assign(requests.size(), x);
    assignPath(candiPathsForAllODs, pathResults);
}

void Baseline::allDij(vector<vector<int>> &dists) {
    dists.clear();
    vector<int> v;
    dists.assign(rN.numNodes + 1, v);
    boost::thread_group tGroup;
    int interval = ceil((1.0 * rN.numNodes + 1) / threadNum);
    for (int i = 0; i < threadNum; ++i) {
        NodeId begin = i * interval, end = (i + 1) * interval;
        if (end > rN.numNodes + 1) end = rN.numNodes + 1;
        tGroup.create_thread(
                boost::bind(
                        &Baseline::rangeDij, this, begin, end, dists));

    }
    tGroup.join_all();
}

void Baseline::greedyExpansion(const vector<vector<int>> &dists, vector<vector<NodeId>> &paths) {
    int reqNo = requests.size();
    benchmark2::heapMin<2, long int, RequestId> queue(reqNo);
    vector<unordered_map<NodeId, int>> liveFlow(rN.numNodes + 1);

    for (RequestId i = 0; i < reqNo; i++) {
        queue.update(i, requests[i].departT);
        paths[i].clear();
        paths[i].emplace_back(requests[i].o);
    }

    long int curL;
    RequestId qId = -1;

    while (!queue.empty()) {
        queue.extract_min(qId, curL);

        NodeId curNode = paths[qId].back();
        int fromD = dists[curNode][requests[qId].d];
        assert(fromD >= 0);

        /* leave current edge */
        if (curNode != requests[qId].o) {
            NodeId prevNode = paths[qId][paths[qId].size() - 2];
            liveFlow[prevNode][curNode] -= 1;
        }

        if (curNode == requests[qId].d) {
            continue;
        }

        long int cheapestOutW = INT32_MAX;
        NodeId cheapestOutN;
        for (const auto &e: rN.adjListOut[curNode]) {

            /* 1. cannot appear twice in the path */
            bool nInPath = false;
            for (const auto &v: paths[qId]) {
                if (e.first == v) {
                    nInPath = true;
                    break;
                }
            }
            if (nInPath) continue;

            /* 2. adj v's min travel time to d cannot >= curV's min travel time to d */
            int nFromD = dists[e.first][requests[qId].d];
            if (nFromD > fromD)
                continue;

            if (liveFlow[curNode].find(e.first) == liveFlow[curNode].end()) {
                liveFlow[curNode][e.first] = 0;
            }
            int flow = liveFlow[curNode][e.first];

            long int edgeW = Framework::costFunc(
                    e.second, flow, rN.capacity[curNode][e.first], capThres);

            if (edgeW < cheapestOutW) {
                cheapestOutW = edgeW;
                cheapestOutN = e.first;
            }
        }

        if (cheapestOutW != INT32_MAX) {
            /* enter next edge */
            liveFlow[curNode][cheapestOutN] += 1;

            /* update queue and path*/
            queue.update(qId, curL + cheapestOutW);
            paths[qId].emplace_back(cheapestOutN);
        } else {
            /* all neighbours are passed */
            getMinTravelPath(curNode, requests[qId].d, paths[qId]);
        }
    }
}

void Baseline::svpKSP(vector<vector<Path>> &candiPathsForAllODs, int k, double theta) {
    boost::thread_group tGroup;
    int interval = ceil(1.0 * uniqODs.size() / threadNum);
    for (int i = 0; i < threadNum; ++i) {
        int begin = i * interval, end = (i + 1) * interval;
        if (end >= uniqODs.size())
            end = uniqODs.size();

//        cout << "thread " << i << " begin " << begin << " end " << end << endl;
        tGroup.create_thread(
                boost::bind(&Baseline::svpKSP, this, boost::ref(candiPathsForAllODs),
                            k, theta, begin, end));
    }
    tGroup.join_all();
}

void Baseline::svpKSP(vector<vector<Path>> &allCandidates, int k, float theta, int begin, int end) {
    for (int odId = begin; odId < end; odId++) {
        allCandidates[odId] = svp_plus_complete(
                &rN, uniqODs[odId].first, uniqODs[odId].second, k, theta).first;
//        cout << odId << "\t" << allCandidates[odId].size() << endl;
    }
}

void Baseline::assignPath(const vector<vector<Path>> &allCandidates, vector<vector<NodeId>> &pathResults) {
    for (RequestId reqId = 0; reqId < requests.size(); reqId++) {
        int odId = reqInvLst[reqId];
        if (allCandidates[odId].empty()) {
            vector<NodeId> path;
            getMinTravelPath(requests[reqId].o, requests[reqId].d, path);
            pathResults[reqId] = path;
        } else {
            int theP = selectAPath(allCandidates[odId].size());
            assert(theP < allCandidates[odId].size());
            pathResults[reqId] = allCandidates[odId][theP].nodes;
        }
    }
}

void Baseline::assignPath(const vector<vector<vector<NodeId>>> &allCandidates, vector<vector<NodeId>> &pathResults) {
    for (RequestId reqId = 0; reqId < requests.size(); reqId++) {
        int odId = reqInvLst[reqId];
        int theP = selectAPath(allCandidates[odId].size());
        if (theP == -1) {
            vector<NodeId> path;
            getMinTravelPath(requests[reqId].o, requests[reqId].d, path);
            pathResults[odId] = path;
        } else {
            assert(theP < allCandidates[odId].size());
            pathResults[odId] = allCandidates[odId][theP];
        }
    }
}

int Baseline::selectAPath(const int k) {
    random_device rd; // obtain a random number from hardware
    mt19937 gen(rd()); // seed the generator
    if (k < 1)
        return -1;

    uniform_int_distribution<> distr(0, k - 1); // define the range
    return distr(gen); // generate numbers
}

void Baseline::getMinTravelPath(NodeId ID1, NodeId ID2, vector<NodeId> &path) {
    benchmark::heap<2, int, NodeId> pqueue(rN.numNodes + 1);
    pqueue.update(ID1, 0);

    vector<int> distance(rN.numNodes + 1, INT32_MAX);
    vector<int> parent(rN.numNodes + 1, -1);
    vector<bool> closed(rN.numNodes + 1, false);

    distance[ID1] = 0;
    NodeId topNodeID, NNodeID;
    int NWeigh, topNodeDis;

    int d = INT32_MAX; //initialize d to infinite for the unreachable case

    while (!pqueue.empty()) {
        pqueue.extract_min(topNodeID, topNodeDis);
        if (topNodeID == ID2) {
            d = distance[ID2];
            break;
        }
        closed[topNodeID] = true;

        for (const auto &it: rN.adjListOut[topNodeID]) {
            NNodeID = it.first;
            NWeigh = it.second + topNodeDis;
            if (!closed[NNodeID]) {
                if (distance[NNodeID] > NWeigh) {
                    distance[NNodeID] = NWeigh;
                    pqueue.update(NNodeID, NWeigh);
                    parent[NNodeID] = topNodeID;
                }
            }
        }
    }

    NodeId curNode = ID2;
    vector<NodeId> subpath;
    while (curNode != -1) {
        subpath.emplace_back(curNode);
        curNode = parent[curNode];
    }

    for (int i = subpath.size() - 1; i >= 0; i--) {
        path.emplace_back(subpath[i]);
    }
}

long Baseline::evaluate(vector<vector<NodeId>> &paths) {
    int reqNo = requests.size();
    benchmark2::heapMin<2, long int, RequestId> queue(reqNo);
    vector<unordered_map<NodeId, int>> liveFlow(rN.numNodes + 1);
    vector<int> posInPath(reqNo, 0);

    for (RequestId i = 0; i < reqNo; i++) {
        assert(!paths[i].empty());
        assert(paths[i][0] == requests[i].o && paths[i].back() == requests[i].d);
        queue.update(i, requests[i].departT);
    }

    long int curL;
    RequestId qId = -1;

    long int totalCost = 0;

    while (!queue.empty()) {
        queue.extract_min(qId, curL);
        int pos = posInPath[qId];

        NodeId curNode = paths[qId][pos], nextNode;

        /* leave current edge */
        if (pos > 0) {
            NodeId prevNode = paths[qId][pos - 1];
            liveFlow[prevNode][curNode] -= 1;
        }

        if (pos == paths[qId].size() - 1) {
            totalCost += curL - requests[qId].departT;
            continue;
        }

        nextNode = paths[qId][pos + 1];

        /* enter next edge */
        int flow = 0;
        if (liveFlow[curNode].find(nextNode) != liveFlow[curNode].end()) {
            flow = liveFlow[curNode][nextNode];
        } else
            liveFlow[curNode][nextNode] = 0;

        liveFlow[curNode][nextNode] += 1;
        long int cost = Framework::costFunc(
                rN.adjListOut[curNode][nextNode], flow,
                rN.capacity[curNode][nextNode], capThres);

        /* update queue */
        queue.update(qId, curL + cost);
        posInPath[qId] += 1;
    }

    return totalCost;
}

void Baseline::rangeDij(NodeId begin, NodeId end, vector<vector<int>> &dists) {
    vector<long int> times;

    for (NodeId i = begin; i < end; i++) {
        Dij(i, dists[i]);
    }
}

void Baseline::Dij(NodeId source, vector<int> &distances) {
    int nodeNum = rN.numNodes + 1;
    benchmark::heap<2, int, NodeId> queue(nodeNum);
    vector<bool> visited(nodeNum, false);
    distances[source] = 0;

    queue.update(source, 0);

    NodeId curNode;
    int distFromO;
    while (!queue.empty()) {
        queue.extract_min(curNode, distFromO);

        visited[curNode] = true;

        for (const auto &oppV: rN.adjListOut[curNode]) {
            if (visited[oppV.first])
                continue;

            int newW = distFromO + rN.adjListOut[curNode][oppV.first];

            if (distances[oppV.first] > newW) {
                queue.update(oppV.first, newW);
                distances[oppV.first] = newW;
            }
        }
    }
}

int Baseline::getMinTravelTime(NodeId ID1, NodeId ID2) {
    benchmark::heap<2, int, NodeId> pqueue(rN.numNodes + 1);
    pqueue.update(ID1, 0);

    vector<int> distance(rN.numNodes + 1, INT32_MAX);
    vector<bool> closed(rN.numNodes + 1, false);

    distance[ID1] = 0;
    NodeId topNodeID, NNodeID;
    int NWeigh, topNodeDis;

    int d = INT32_MAX; //initialize d to infinite for the unreachable case

    while (!pqueue.empty()) {
        pqueue.extract_min(topNodeID, topNodeDis);
        if (topNodeID == ID2) {
            d = distance[ID2];
            break;
        }
        closed[topNodeID] = true;

        for (const auto &it: rN.adjListOut[topNodeID]) {
            NNodeID = it.first;
            NWeigh = it.second + topNodeDis;
            if (!closed[NNodeID]) {
                if (distance[NNodeID] > NWeigh) {
                    distance[NNodeID] = NWeigh;
                    pqueue.update(NNodeID, NWeigh);
                }
            }
        }
    }
    return d;
}

void Baseline::correctnessCheck(const vector<vector<int>> &dists) {
    vector<int> v;
    for (int i = 0; i < rN.numNodes + 1; ++i) {
        for (int j = 0; j < rN.numNodes + 1; ++j) {
            if (i == j) continue;
            int d = getMinTravelTime(i, j);
            if (d != dists[i][j]) {
                cout << "error: " << i << " " << j << " " << d << " " << dists[i][j] << endl;
            }
        }
    }

    cout << "correctness check finished" << endl;
}