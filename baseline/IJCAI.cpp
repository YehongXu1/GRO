//
// Created by Yehong Xu on 1/10/2022.
//

#include "IJCAI.h"

IJCAI::IJCAI(RoadNetwork &roadNetwork) : roadNetwork(roadNetwork) {
    vector<int> v(roadNetwork.numNodes + 1, INT32_MAX);
    dists.assign(roadNetwork.numNodes + 1, v);
}

void IJCAI::getRequests(vector<Request> &requests) {
    this->requests = requests;

}

void IJCAI::greedyExpansion(int capThres, vector<vector<NodeId>> &paths) {
    int reqNo = requests.size();
    benchmark2::heapMin<2, long int, RequestId> queue(reqNo);
    vector<unordered_map<NodeId, int>> liveFlow(roadNetwork.numNodes + 1);

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
        for (const auto &e: roadNetwork.adjListOut[curNode]) {

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
                    e.second, flow, roadNetwork.capacity[curNode][e.first], capThres);

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

long int IJCAI::evaluate(int capThres, vector<vector<NodeId>> &paths) {
    int reqNo = requests.size();
    benchmark2::heapMin<2, long int, RequestId> queue(reqNo);
    vector<unordered_map<NodeId, int>> liveFlow(roadNetwork.numNodes + 1);
    vector<int> posInPath(reqNo, 0);

    for (RequestId i = 0; i < reqNo; i++) {
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
                roadNetwork.adjListOut[curNode][nextNode], flow,
                roadNetwork.capacity[curNode][nextNode], capThres);

        /* update queue */
        queue.update(qId, curL + cost);
        posInPath[qId] += 1;
    }

    return totalCost;
}

int IJCAI::getMinTravelTime(NodeId ID1, NodeId ID2) {

    benchmark::heap<2, int, NodeId> pqueue(roadNetwork.numNodes + 1);
    pqueue.update(ID1, 0);

    vector<int> distance(roadNetwork.numNodes + 1, INT32_MAX);
    vector<bool> closed(roadNetwork.numNodes + 1, false);

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

        for (const auto &it: roadNetwork.adjListOut[topNodeID]) {
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

void IJCAI::getMinTravelPath(NodeId ID1, NodeId ID2, vector<NodeId> &path) {

    benchmark::heap<2, int, NodeId> pqueue(roadNetwork.numNodes + 1);
    pqueue.update(ID1, 0);

    vector<int> distance(roadNetwork.numNodes + 1, INT32_MAX);
    vector<int> parent(roadNetwork.numNodes + 1, -1);
    vector<bool> closed(roadNetwork.numNodes + 1, false);

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

        for (const auto &it: roadNetwork.adjListOut[topNodeID]) {
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
    while (curNode != ID1) {
        subpath.emplace_back(curNode);
        curNode = parent[curNode];
    }

    for (int i = subpath.size() - 1; i >= 0; i--) {
        path.emplace_back(subpath[i]);
    }
}

void IJCAI::allDij(int threadNum) {
    boost::thread_group tGroup;
    int interval = ceil((1.0 * roadNetwork.numNodes + 1) / threadNum);

    for (int i = 0; i < threadNum; ++i) {
        int begin = i * interval, end = (i + 1) * interval;
        if (end > roadNetwork.numNodes + 1) end = roadNetwork.numNodes + 1;
        tGroup.create_thread(
                boost::bind(&IJCAI::rangeDij, this, begin, end));

    }
    tGroup.join_all();
    cout << "all Dij finished" << endl;
}

void IJCAI::rangeDij(int begin, int end) {
    vector<long int> times;

    for (int i = begin; i < end; i++) {
        Dij(i);
    }
}

void IJCAI::Dij(NodeId source) {
    int nodeNum = roadNetwork.numNodes + 1;
    benchmark::heap<2, int, NodeId> queue(nodeNum);
    vector<bool> visited(nodeNum, false);
    vector<int> &distances = dists[source];
    distances[source] = 0;

    queue.update(source, 0);

    NodeId curNode;
    int distFromO;
    while (!queue.empty()) {
        queue.extract_min(curNode, distFromO);

        visited[curNode] = true;

        for (const auto &oppV: roadNetwork.adjListOut[curNode]) {
            if (visited[oppV.first])
                continue;

            int newW = distFromO + roadNetwork.adjListOut[curNode][oppV.first];

            if (distances[oppV.first] > newW) {
                queue.update(oppV.first, newW);
                distances[oppV.first] = newW;
            }
        }
    }
}

void IJCAI::writeAllSDs(const string &fileName) {
    ofstream outFile(fileName);
    for (int i = 0; i < dists.size(); i++) {
        outFile << i << " ";
        for (int j = 0; j < dists[i].size(); j++) {
            outFile << dists[i][j] << " ";
        }
        outFile << endl;
    }
    outFile.close();
    cout << "write all SDs finished" << endl;
}

void IJCAI::readAllSDs(const string &fileName) {
    ifstream inFile(fileName);
    int nodeNum = roadNetwork.numNodes + 1;
    dists.resize(nodeNum);
    for (int i = 0; i < nodeNum; i++) {
        dists[i].resize(nodeNum);
    }

    while (!inFile.eof()) {
        int nodeID;
        inFile >> nodeID;
        for (int i = 0; i < nodeNum; i++) {
            inFile >> dists[nodeID][i];
        }
    }
    inFile.close();

    cout << "finish reading all SDs" << endl;
}

void IJCAI::correctnessCheck() {
    int nodenum = roadNetwork.numNodes + 1, s, t, d1, d2;
    for (int i = 0; i < 1000; i++) {
        s = rand() % nodenum;
        t = rand() % nodenum;
        d1 = dists[s][t];
        d2 = getMinTravelTime(s, t);
        if (d1 != d2)
            cout << "InCorrect!" << s << " " << t << " " << d1 << " " << d2 << endl;
    }
    cout << "Correctness check finished" << endl;
}
