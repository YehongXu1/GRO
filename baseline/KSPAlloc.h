//
// Created by yehong.xu on 31/1/2022.
//

#ifndef TRAFFIC_ASSIGNMENT_KSPALLOC_H
#define TRAFFIC_ASSIGNMENT_KSPALLOC_H

#include <random>
#include "../traffic/Traffic.h"
class KSPAlloc
{
public:
    TrafficMaintain &traffic;
    vector<vector<Path>> candidates;
    int selectAPath(RequestId req);
    void pathToLabelV(RequestId req, Path &path);
    void getCandiPaths();
    unsigned int cost = INT_MAX;
    int k = 5;
    double theta = 0.75;

    explicit KSPAlloc(TrafficMaintain &traffic, int k, double theta);
    explicit KSPAlloc(TrafficMaintain &traffic);
    void assignPath();

    void getCandiPaths(int begin, int end);
};


#endif //TRAFFIC_ASSIGNMENT_KSPALLOC_H
