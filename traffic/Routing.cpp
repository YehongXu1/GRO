//
// Created by yehong.xu on 1/2/2022.
//

#include "Routing.h"

Routing::Routing(Traffic &traffic) : traffic(traffic) {}

long long int Routing::mainAlg(double frac, bool fix)
{
    vector<RequestId> reqs;
    boost::push_back(reqs, boost::irange(0, traffic.reqNo));

    int iteration = 0;
    long long int curCost;

    while (true)
    {
        // reroute reqs that are selected to be rerouted
        traffic.allTempDij(reqs);

        // evaluation is based on entire traffic
        curCost = traffic.simulateTraffic();

        // evaluate travel cost based on new traffic state
        cout << iteration << "," << reqs.size() << "," << curCost << endl;

        vector<bool> toBeRerouted(traffic.reqNo, false);
        if (fix)
        {
            reqs = selectRerouteReqsFixPaths(reqs, frac); // fix some routes
            if (reqs.empty())
                break;
        } else
        {
            reqs = selectRerouteReqsUnfixPaths(frac); // select from all reqs
            if (iteration == 50)
                break;
        }

//        traffic.simulateTraffic();

        traffic.clearEdgeProfile();
        traffic.updateTrafficLoading();
        traffic.deleteLabels(reqs);
        // some ETs in overflowETs maybe not overflow due to the deletions of some trips from RN
        iteration += 1;
    }
    return curCost;
}

vector<RequestId> Routing::selectRerouteReqsFixPaths(vector<int> &reqs, double frac)
{
    benchmark2::heapSelectQ<2, long long int, RequestId> reqHeap(traffic.reqNo);

    for (const auto &req: reqs)
    {
        long long int key = traffic.requestODs[req].waitTimes;
        reqHeap.update(req, key);
    }

    int selectedP = floor(1.0 * frac * reqs.size());
    RequestId requestId;
    long long int i;
    vector<RequestId> rerouteReqs;
    while (!reqHeap.empty() && rerouteReqs.size() < selectedP)
    {
        reqHeap.extract_max(requestId, i);
        rerouteReqs.emplace_back(requestId);
        traffic.trajectories[requestId] = nullptr;
    }
    return rerouteReqs;
}

vector<RequestId> Routing::selectRerouteReqsUnfixPaths(double frac)
{
    benchmark2::heapSelectQ<2, long long int, RequestId> myReqHeap(traffic.reqNo);
    for (RequestId i = 0; i < traffic.reqNo; i++)
    {
        long long int key = traffic.requestODs[i].waitTimes;
        myReqHeap.update(i, key);
    }

    int selectedP = floor(1.0 * frac * traffic.reqNo);
    RequestId requestId;
    long long int i;
    vector<RequestId> rerouteReqs;
    while (!myReqHeap.empty() && rerouteReqs.size() < selectedP)
    {
        myReqHeap.extract_max(requestId, i);
        rerouteReqs.emplace_back(requestId);

        traffic.trajectories[requestId] = nullptr;
    }
    return rerouteReqs;
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
