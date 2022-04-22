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
    Traffic &traffic;
    vector<vector<Path>> candidates;
    int selectAPath(RequestId req);
    Label* pathToLabel(Path &path);
    void getCandiPaths();
    long long int cost = INT_MAX;
    int k;
    double theta;

    explicit KSPAlloc(Traffic &traffic, int k, double theta);
    void assignPath();
    long long int getCost();

    void getCandiPaths(int begin, int end);
};


#endif //TRAFFIC_ASSIGNMENT_KSPALLOC_H
