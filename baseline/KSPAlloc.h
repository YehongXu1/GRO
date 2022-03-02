//
// Created by yehong.xu on 31/1/2022.
//

#ifndef TRAFFIC_ASSIGNMENT_KSPALLOC_H
#define TRAFFIC_ASSIGNMENT_KSPALLOC_H

#include <random>
#include "../traffic/Traffic.h"
class KSPAlloc
{
private:
    Traffic traffic;
    unordered_map<RequestId, vector<Path>> candidates;
    int selectAPath(RequestId req);
    Label* pathToLabel(Path &path);
    long long int cost = INT_MAX;
public:

    KSPAlloc(Traffic &traffic, const basic_string<char> &paths);
    explicit KSPAlloc(Traffic &traffic, int k, double theta);
    void assignPath();
    long long int getCost();

};


#endif //TRAFFIC_ASSIGNMENT_KSPALLOC_H
