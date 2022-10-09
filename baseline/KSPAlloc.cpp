//
// Created by yehong.xu on 31/1/2022.
//

#include "KSPAlloc.h"
#include "../KSP/kspwlo.h"

KSPAlloc::KSPAlloc(TrafficMaintain &traffic, int k, double theta) : traffic(traffic) {
    this->k = k;
    this->theta = theta;
    vector<Path> v;
    candidates.assign(traffic.reqNo, v);
    getCandiPaths();
}

KSPAlloc::KSPAlloc(TrafficMaintain &traffic) : traffic(traffic) {
    vector<Path> v;
    candidates.assign(traffic.reqNo, v);
    getCandiPaths();
}

void KSPAlloc::getCandiPaths() {
    boost::thread_group tGroup;
    int interval = ceil(1.0 * traffic.reqNo / traffic.threadNum);
    for (int i = 0; i < traffic.threadNum; ++i) {
        int begin = i * interval, end = (i + 1) * interval;
        if (end >= traffic.reqNo)
            end = traffic.reqNo;
        tGroup.create_thread([this, begin, end] { getCandiPaths(begin, end); });
    }
    tGroup.join_all();
}

void KSPAlloc::getCandiPaths(int begin, int end) {
    for (RequestId req = begin; req < end; req++) {
        candidates[req] = svp_plus_complete(
                &traffic.rN, traffic.requestODs[req].o, traffic.requestODs[req].d, k, theta).first;
    }
}

int KSPAlloc::selectAPath(RequestId req) {
    random_device rd; // obtain a random number from hardware
    mt19937 gen(rd()); // seed the generator
    if(candidates[req].empty())
        return -1;

    uniform_int_distribution<> distr(0, candidates[req].size() - 1); // define the range
    return distr(gen); // generate numbers
}

void KSPAlloc::assignPath() {
    for (RequestId id = 0; id < traffic.reqNo; id++) {
        int theP = selectAPath(id);
        if (theP == -1) {
            Framework::tempDij(traffic, id);
        } else {
            assert(theP < k);
            pathToLabelV(id, candidates[id][theP]);
        }
    }
    cost = Framework::simulateTrafficNoCong(traffic);
}

void KSPAlloc::pathToLabelV(RequestId req, Path &path) {
    int length = traffic.requestODs[req].departT;
    vector<Label> &v = traffic.footprints[req];
    for (unsigned int node: path.nodes) {
        Label prevL = Label(node, length, nullptr);
        v.emplace_back(prevL);
    }
}

