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

    void mainAlg(double frac, bool fix);

    void writeEdgeFlowDist(const basic_string<char> &path);


private:
    Traffic &traffic;

    int curCost;

    vector<RequestId> selectRerouteReqsFixPaths(vector<RequestId> &reqs, double frac);

    // return ETs become underflow by due to removal of this request from traffic

    vector<RequestId> selectRerouteReqsUnfixPaths(double frac);

    vector<RequestId> selectRerouteReqsCongET(double frac);

    vector<RequestId> selectRerouteReqsCongET2(double frac);
};


#endif //TRAFFIC_ASSIGNMENT_ROUTING_H
