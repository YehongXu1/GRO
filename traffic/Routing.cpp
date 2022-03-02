//
// Created by yehong.xu on 1/2/2022.
//

#include "Routing.h"

Routing::Routing(Traffic &traffic) : traffic(traffic) {}

long long int Routing::mainAlg(double frac, bool fix)
{
    unordered_set<RequestId> reqs(traffic.reqNo);
    for (RequestId req = 0; req < traffic.reqNo; req++)
    {
        reqs.insert(reqs.end(), req);
    }

    int iteration = 0;
    long long int curCost;

    benchmark2::heapSelectQ<2, long long int, RequestId> reqHeap(traffic.reqNo);
    vector<long long int> reqColli(traffic.reqNo, INT_MIN);
    while (iteration < 100)
    {
        // reroute reqs that are selected to be rerouted
        traffic.temporalDij(reqs);

        // evaluation is based on entire traffic
        curCost = traffic.simulateTraffic();

        // evaluate travel cost based on new traffic state
        cout << iteration << "," << reqs.size() << "," << curCost << endl;

        if (fix)
        {
            reqs = selectRerouteReqsFixPaths(reqColli, reqHeap, reqs.size() * frac); // fix some routes
            if (reqs.size() == 0)
                break;
        } else
        {
            reqs = selectRerouteReqsUnfixPaths(reqColli, traffic.reqNo);
            if (iteration == 50)
                break;
        }

        for (RequestId requestId = 0; requestId < traffic.reqNo; requestId++)
        {
            if (reqColli[requestId] != INT_MIN)
            {
                // req will not be rerouted in this iteration
                vector<ET> s;
                traffic.UpdateTrafficFlow(requestId, s);
            }
        }

        traffic.updateHeuWeights();

        // some ETs in overflowETs maybe not overflow due to the deletions of some trips from RN
        iteration += 1;

    }
    return curCost;
}

unordered_set<RequestId> Routing::selectRerouteReqsFixPaths(
        vector<long long int> &reqColli, benchmark2::heapSelectQ<2, long long int, RequestId> &reqHeap, int selectedP)
{
    assert(reqColli.size() == traffic.reqNo);
    for (RequestId i = 0; i < traffic.reqNo; i++)
    {
        if (reqColli[i] > 0)
        {
            // these are reqs that are not rerourted in previous iteration, they are fixed
            reqColli[i] = INT_MIN;
        } else
        {
            reqColli[i] = traffic.requestODs[i].waitTimes;
            reqHeap.update(i, reqColli[i]);
        }
    }

    RequestId requestId;
    long long int i;
    unordered_set<RequestId> rerouteReqs;
    while (!reqHeap.empty() && rerouteReqs.size() < selectedP)
    {
        reqHeap.extract_max(requestId, i);
        rerouteReqs.insert(requestId);

        // to mark reqs that are to be routed in this iteration
        reqColli[requestId] = INT_MIN;
        reqHeap.update(requestId, reqColli[requestId]);
    }
    return rerouteReqs;
}

unordered_set<RequestId> Routing::selectRerouteReqsUnfixPaths(vector<long long int> &reqColli, int selectedP)
{
    benchmark2::heapSelectQ<2, long long int, RequestId> myReqHeap(traffic.reqNo);
    vector<long long int> myReqColli(traffic.reqNo, INT_MIN);
    assert(reqColli.size() == traffic.reqNo);
    for (RequestId i = 0; i < traffic.reqNo; i++)
    {
        myReqColli[i] = traffic.requestODs[i].waitTimes;
        myReqHeap.update(i, reqColli[i]);
    }

    RequestId requestId;
    long long int i;
    unordered_set<RequestId> rerouteReqs;
    while (!myReqHeap.empty() && rerouteReqs.size() < selectedP)
    {
        myReqHeap.extract_max(requestId, i);
        rerouteReqs.insert(requestId);

        // to mark reqs that are to be routed in this iteration
        reqColli[requestId] = INT_MIN;
        myReqHeap.update(requestId, reqColli[requestId]);
    }
    return rerouteReqs;
}

unordered_set<RequestId> Routing::selectReqsBasedOnETCnt(
        vector<long long int> &reqCNT, benchmark2::heapSelectQ<2, long long int, RequestId> &reqHeap, int selectNo)
{
    unordered_set<RequestId> reqs;
    long long int curReqCnt;
    RequestId maxReq;

    while (reqs.size() < selectNo && !reqHeap.empty())
    {
        reqHeap.extract_max(maxReq, curReqCnt);
        // if (maxReq is fixed, do not )
        reqs.insert(maxReq);

        vector<ET> newUnderflowEdges;
        traffic.delTrajectoryInRN(maxReq, newUnderflowEdges);
        for (const auto &et: newUnderflowEdges)
        {
            for (const auto &delReq: traffic.trafficStat[et.v1][et.v2].tempReqs[et.i])
            {
                reqCNT[delReq] -= 1;
                reqHeap.update(delReq, reqCNT[delReq]);
            }
        }
    }
    for (RequestId req = 0; req < traffic.reqNo; req++)
    {
        // requests not selected are regarded as fixed and should not be rerouted
        if (reqCNT[req] > 0 && reqs.find(req) == reqs.end())
        {
            reqCNT[req] = -1;
            reqHeap.update(req, -1);
        }
    }
    return reqs;
}

void Routing::blockOverflowETs(vector<ET> &overflowETs)
{
    vector<ET>::iterator et;
    for (et = overflowETs.begin(); et != overflowETs.end(); et++)
    {
        int cnt = traffic.trafficStat[et->v1][et->v2].tempFlow[et->i];
        int roadId = traffic.rN.edgeMap[et->v1][et->v2];
        if (cnt < traffic.rN.capacity[roadId])
            continue;

        traffic.trafficStat[et->v1][et->v2].tempWeight[et->i] = INT_MAX;
        traffic.trafficStat[et->v1][et->v2].modified[et->i] = true;
    }
}

void Routing::blockOverflowETsRandom(vector<ET> &overflowETs)
{
    vector<ET>::iterator et;
    for (et = overflowETs.begin(); et != overflowETs.end(); et++)
    {
        int cnt = traffic.trafficStat[et->v1][et->v2].tempFlow[et->i];
        int roadId = traffic.rN.edgeMap[et->v1][et->v2];
        if (cnt < traffic.rN.capacity[roadId])
            continue;

        if (randomBool(cnt - traffic.rN.capacity[roadId] + 1, 0.5))
        {
            traffic.trafficStat[et->v1][et->v2].tempWeight[et->i] = INT_MAX;
            traffic.trafficStat[et->v1][et->v2].modified[et->i] = true;
        }
    }
}

void Routing::doubleWeightOverflowETsRandom(vector<ET> &overflowETs)
{
    vector<ET>::iterator et;
    for (et = overflowETs.begin(); et != overflowETs.end(); et++)
    {
        int roadId = traffic.rN.edgeMap[et->v1][et->v2];
        int cnt = traffic.trafficStat[et->v1][et->v2].tempFlow[et->i];
        if (cnt < traffic.rN.capacity[roadId])
            continue;

        if (randomBool(cnt - traffic.rN.capacity[roadId] + 1, 0.5))
        {
            long long int tempWeight = traffic.trafficStat[et->v1][et->v2].tempWeight[et->i];
            assert(tempWeight * 2 >= 0);
            traffic.trafficStat[et->v1][et->v2].tempWeight[et->i] = floor(tempWeight * 2);
            traffic.trafficStat[et->v1][et->v2].modified[et->i] = true;
        }
    }
}

void Routing::zeroWeighOverflowETs(vector<ET> &overflowETs)
{
    vector<ET>::iterator et;
    for (et = overflowETs.begin(); et != overflowETs.end(); et++)
    {
        int cnt = traffic.trafficStat[et->v1][et->v2].tempFlow[et->i];
        int roadId = traffic.rN.edgeMap[et->v1][et->v2];
        if (cnt < traffic.rN.capacity[roadId])
            continue;

        traffic.trafficStat[et->v1][et->v2].tempWeight[et->i] = 0;
        traffic.trafficStat[et->v1][et->v2].modified[et->i] = true;
    }
}

void Routing::writeEdgeFlowDist(const basic_string<char> &path)
{
    ofstream ofstream(path);
    for (NodeId lnode = 0; lnode <= traffic.rN.numNodes; lnode++)
    {
        for (const auto &rnodeW: traffic.rN.adjListOut[lnode])
        {
            ofstream << traffic.trafficStat[lnode][rnodeW.first].liveFlow << endl;
        }
    }

    ofstream.close();
}
