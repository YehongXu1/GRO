//
// Created by yehong.xu on 28/5/2022.
//

#include "Traffic.h"

// load path to road network one by one

void MainAlg::greedyMain(TrafficMaintain &traffic) {
    chrono::steady_clock::time_point begin = chrono::steady_clock::now();
    long int t1 = 0, t2 = 0;
    for (RequestId requestId = 0; requestId < traffic.reqNo; requestId++) {
        Timer clock;
        clock.tick();
        Framework::tempDij(traffic, requestId);
        clock.tock();
        t1 += clock.duration().count();

        clock.tick();
        Framework::simulateTrafficNoCong(traffic);
        clock.tock();
        t2 += clock.duration().count();
    }

    bool allGood = true;
    for (RequestId i = 0; i < traffic.reqNo; i++) {
        if (traffic.footprints[i].empty()) {
            allGood = false;
        }
    }

    long int trafficCost;
    if (true) trafficCost = Framework::simulateTrafficNoCong(traffic);
    else trafficCost = 0;

    t1 = t1 / 1000, t2 = t2 / 1000;

    chrono::steady_clock::time_point end = chrono::steady_clock::now();
    cout << chrono::duration_cast<chrono::milliseconds>(end - begin).count() << "\t"
         << t1 << "\t" << t2 << "\t" << trafficCost << endl;
}

void MainAlg::batchMain(TrafficMaintain &traffic, float sigma, float tau) {
    DisGraph disGraph(traffic.rN, sigma, tau, traffic.threadNum);

    chrono::steady_clock::time_point begin = chrono::steady_clock::now();
    long int t1 = 0, t2 = 0, t3 = 0;

    Timer clock;
    clock.tick();
    for (auto &req: traffic.requestODs) {
        disGraph.findNearbyGrids(req);
    }
    clock.tock();
    t1 += clock.duration().count();

    bool allGood = true;
    long int trafficCost;
    for (auto &batch: disGraph.batches) {
//        if (!allGood) break;
        clock.tick();
        Framework::allTempDij(traffic, batch, allGood);
        clock.tock();
        t2 += clock.duration().count();

        clock.tick();
        trafficCost = Framework::simulateTrafficNoCong(traffic);
        clock.tock();
        t3 += clock.duration().count();
    }

    t1 = t1 / 1000, t2 = t2 / 1000, t3 = t3 / 1000;
    chrono::steady_clock::time_point end = chrono::steady_clock::now();
    cout << chrono::duration_cast<chrono::milliseconds>(end - begin).count() << "\t"
         << disGraph.batches.size() << "\t" << trafficCost << endl;
}


void MainAlg::batchMainIter(TrafficMaintain &traffic, float sigma, float tau, float frac) {
    DisGraph disGraph(traffic.rN, sigma, tau, traffic.threadNum);
    bool allGood = true;
    long int cost;
    chrono::steady_clock::time_point start = chrono::steady_clock::now();

    long int t1 = 0, t2 = 0, t3 = 0, t4 = 0;
    Timer clock;

    clock.tick();
    for (auto &req: traffic.requestODs) {
        disGraph.findNearbyGrids(req);
    }
    clock.tock();
    t4 = clock.duration().count();

    cout << "batch nb: " << disGraph.batches.size() << "\n";

    int batch_nb = 0;
    for (auto &batch: disGraph.batches) {
        batch_nb++;
        if (batch.empty()) continue;

        clock.tick();
        Framework::allTempDij(traffic, batch, allGood);
        clock.tock();
        t2 += clock.duration().count();

        clock.tick();
        cost = Framework::simulateTrafficNoCong(traffic);
        clock.tock();
        t3 += clock.duration().count();
    }

    chrono::steady_clock::time_point end1 = chrono::steady_clock::now();

    int iteration = 0;
    while (iteration < 20) {
        cout << cost << "\n";

        clock.tick();
        vector<bool> selected(traffic.reqNo, false); // unfix: assign false; fix: assign true
        vector<RequestId> reRoutes = PathSelection::selectWaitingT(
                traffic, selected, floor(frac * traffic.reqNo));
        clock.tock();
        t1 += clock.duration().count();

        clock.tick();
        Framework::allTempDij(traffic, reRoutes, allGood);
        clock.tock();
        t2 += clock.duration().count();

        clock.tick();
        cost = Framework::simulateTrafficNoCong(traffic);
        clock.tock();
        t3 += clock.duration().count();

        iteration += 1;
    }

    t1 = t1 / 1000, t2 = t2 / 1000, t3 = t3 / 1000, t4 = t4 / 1000;

    chrono::steady_clock::time_point end2 = chrono::steady_clock::now();

    int hop_num = 0;
    for (const auto &footprint: traffic.footprints) {
        hop_num += footprint.size();
    }
    cout << "avg hop num: " << int(hop_num / traffic.reqNo) << "\n";

    cout << "time: " << chrono::duration_cast<chrono::milliseconds>(end1 - start).count()
    << "\t" << chrono::duration_cast<chrono::milliseconds>(end2 - start).count()
    << "\t" << t4 << "\t" << t1 << "\t" << t2 << "\t" << t3 << endl;
}
