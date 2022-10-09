//
// Created by Yehong Xu on 4/10/2022.
//

#ifndef TRAFFIC_ASSIGNMENT_BASELINE_H
#define TRAFFIC_ASSIGNMENT_BASELINE_H

#include <random>
#include "../traffic/Traffic.h"
#include "../KSP/kspwlo.h"

class Baseline {
public:
    RoadNetwork &rN;
    vector<Request> requests;
    vector<int> reqInvLst; //idx: reqID, content: idx in uniqODs
    vector<pair<NodeId, NodeId>> uniqODs; // unique OD pairs
    int capThres;
    int threadNum;

    Baseline(RoadNetwork &rN, int threadNum, int capThreshold);

    void getRequests(const vector<Request> &requests);

    void svpBaseLine(vector<vector<NodeId>> &pathResults, int k, double theta);

    void getUniqODs();

    void svpKSP(vector<vector<Path>> &candiPathsForAllODs, int k, double theta);

    void svpKSP(vector<vector<Path>> &allCandidates, int k, float theta, int begin, int end);

    void assignPath(const vector<vector<Path>> &allCandidates, vector<vector<NodeId>> &pathResults);

    void assignPath(const vector<vector<vector<NodeId>>> &allCandidates, vector<vector<NodeId>> &pathResults);

    static int selectAPath(const int k);

    void getMinTravelPath(NodeId ID1, NodeId ID2, vector<NodeId> &path);

    long evaluate(vector<vector<NodeId>> &paths);

    void greedyExpansion(const vector<vector<int>> &dists, vector<vector<NodeId>> &paths);

    void allDij(vector<vector<int>> &dists);

    void rangeDij(NodeId begin, NodeId end, vector<vector<int>> &dists);

    void Dij(NodeId source, vector<int> &distances);

    int getMinTravelTime(NodeId ID1, NodeId ID2);

    void correctnessCheck(const vector<vector<int>> &dists);

};

#endif //TRAFFIC_ASSIGNMENT_BASELINE_H