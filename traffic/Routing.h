//
// Created by yehong.xu on 1/2/2022.
//

#ifndef TRAFFIC_ASSIGNMENT_ROUTING_H
#define TRAFFIC_ASSIGNMENT_ROUTING_H

#include "Traffic.h"

class Routing
{
public:
    explicit Routing(Traffic &traffic);

    long long int mainAlg(double frac, bool fix);

    void writeEdgeFlowDist(const basic_string<char> &path);


private:
    Traffic &traffic;

    unordered_set<RequestId> selectRerouteReqsFixPaths(
            vector<long long int> &reqColli, benchmark2::heapSelectQ<2, long long int, RequestId> &reqHeap, int selectedP);

    unordered_set<RequestId> selectReqsBasedOnETCnt(
            vector<long long int> &reqCNT, benchmark2::heapSelectQ<2, long long int, RequestId> &reqHeap,
            int selectNum);

    // return ETs become underflow by due to removal of this request from traffic

    void blockOverflowETs(vector<ET> &overflowETs);

    void blockOverflowETsRandom(vector<ET> &overflowETs);

    void doubleWeightOverflowETsRandom(vector<ET> &overflowETs);

    void zeroWeighOverflowETs(vector<ET> &overflowETs);

    unordered_set<RequestId> selectRerouteReqsUnfixPaths(vector<long long int> &reqColli, int selectedP);
};


#endif //TRAFFIC_ASSIGNMENT_ROUTING_H
