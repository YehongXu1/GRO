//
// Created by Yehong Xu on 1/10/2022.
//

#ifndef TRAFFIC_ASSIGNMENT_IJCAI_H
#define TRAFFIC_ASSIGNMENT_IJCAI_H
#include "../traffic/Traffic.h"

class IJCAI {
public:
    RoadNetwork &roadNetwork;

    vector<Request> requests;
    vector<vector<int>> dists;

    IJCAI(RoadNetwork &roadNetwork);

    void getRequests(vector<Request> &requests);

    void greedyExpansion(int capThres, vector<vector<NodeId>> &paths);

    void getMinTravelPath(NodeId ID1, NodeId ID2, vector<NodeId> &path);

    long evaluate(int capThres, vector<vector<NodeId>> &paths);

    void allDij(int threadNum);

    void rangeDij(int begin, int end);

    void Dij(NodeId source);

    void writeAllSDs(const string &fileName);

    void readAllSDs(const string &fileName);

    int getMinTravelTime(NodeId ID1, NodeId ID2);

    void correctnessCheck();
};

#endif //TRAFFIC_ASSIGNMENT_IJCAI_H
