#include "traffic/Traffic.h"
#include "KSP/kspwlo.h"
#include "baseline/Baseline.h"
#include <filesystem>

using namespace std;
namespace fs = std::filesystem;

void writeRoutingResult(TrafficMaintain &traffic, const string &fileName) {
    ofstream outFile(fileName);
    const vector<vector<Label>> footprints = traffic.footprints;

    for (int i = 0; i < traffic.reqNo; ++i) {
        Timer clock;
        long int sd = Framework::Dij(traffic, i);
        clock.tock();
        long int dijT = clock.duration().count();

        outFile << i << "|" << sd << "|" << dijT << "|" << traffic.requestODs[i].cost << "|";
//        for (const auto &j: footprints[i]) {
//            outFile << j.node_id << " ";
//        }
        outFile << endl;
    }
    outFile.close();
}

vector<vector<NodeId>> runSVPBaseline(RoadNetwork &rN, vector<Request> &requestODs, int k, int threadNum, int capThres) {
    Baseline baseline(rN, threadNum, capThres);
    baseline.getRequests(requestODs);

    baseline.getUniqODs();

    vector<vector<NodeId>> pathResults;
    Timer clock;
    clock.tick();
    baseline.svpBaseLine(pathResults, k, 0.75);

    long int cost = baseline.evaluate(pathResults);

    clock.tock();
    cout << clock.duration().count()/1000 << "\t" << cost << endl;

    return pathResults;
}

TrafficMaintain runBaselineAllSP(RoadNetwork &rN, vector<Request> &requestODs, int threadNum, int capThres) {
//    cout << "AllSP" << endl;
    TrafficMaintain traffic(rN, requestODs, threadNum, capThres);
    vector<RequestId> reqs;
    boost::push_back(reqs, boost::irange(0, traffic.reqNo));
    long int optimalCost = 0;
    chrono::steady_clock::time_point begin = chrono::steady_clock::now();
    Timer clock;
    clock.tick();
    Framework::allDij(traffic, reqs, optimalCost);
    clock.tock();
    long int t1 = clock.duration().count();

    clock.tick();
    long int trafficCost = Framework::simulateTrafficNoCong(traffic);
    clock.tock();
    long int t2 = clock.duration().count();

    chrono::steady_clock::time_point end = chrono::steady_clock::now();
    cout << chrono::duration_cast<chrono::milliseconds>(end - begin).count() << "\t" << t1 / 1000 << "\t" << t2 / 1000
         << "\t" << optimalCost << "\t" << trafficCost << endl;

    return traffic;
}

TrafficMaintain runLinear(RoadNetwork &rN, vector<Request> &requestODs, int threadNum, int capThres) {
//    cout << "Linear" << endl;
    TrafficMaintain traffic(rN, requestODs, threadNum, capThres);
    MainAlg::greedyMain(traffic);

    return traffic;
}

TrafficMaintain
runBatch(RoadNetwork &rN, vector<Request> &requestODs, int threadNum, float sigma, float tau, int capThres) {
//    cout << "Batch" << " sigma:" << sigma << " tau:" << tau << endl;

    TrafficMaintain traffic(rN, requestODs, threadNum, capThres);
    MainAlg::batchMain(traffic, sigma, tau);

    return traffic;
}

TrafficMaintain runBatchIter(
        RoadNetwork &rN, vector<Request> &requestODs, int threadNum, float sigma, float tau, float frac, int capThres) {
    cout << "BatchIter" << " sigma:" << sigma << " tau:" << tau << " fra:" << frac << endl;
    TrafficMaintain traffic(rN, requestODs, threadNum, capThres);
    MainAlg::batchMainIter(traffic, sigma, tau, frac);

    return traffic;
}

void readVaryDisQsDirectly(
        string &basepath, RoadNetwork &rN,
        int threadNum, float sigma, float tau, float frac, int capThres, int rep) {

    /**
    IJCAI ijcai(rN);
    Timer clock;
    clock.tick();
    ijcai.allDij(threadNum);
    clock.tock();
    cout << "AllDij Time:" << clock.duration().count() / 1000 << endl;
    **/

    vector<string> distances{"5", "10", "20", "40"};
    for (int i = 0; i < distances.size(); i++) {
        for (int j = 0; j < 15; j++) {
            string path = basepath + "input3/dist" + distances[i] + "km_" + to_string(j) + ".txt";

            int o, d, departT;
            vector<Request> reqs;
            ifstream infile(path);
            while (infile >> o >> d >> departT) {
                assert(o <= rN.numNodes && d <= rN.numNodes);
                for (int k = 0; k < rep; k++) {
                    reqs.emplace_back(reqs.size(), o, d, departT);
                }

            }
            if (reqs.empty()) {
                cout << "No request in " << j << endl;
                continue;
            }
            /**
            ijcai.getRequests(requests);
            vector<NodeId> nodeSeq;
            vector<vector<NodeId>> paths(requests.size(), nodeSeq);
            clock.tick();
            ijcai.greedyExpansion(capThres,paths);
            clock.tock();
            long int trafficCost = ijcai.evaluate(capThres, paths);

            cout << clock.duration().count()/1000 << "\t" << trafficCost << endl;
            **/
//            runBaselineAllSP(rN, reqs, threadNum, capThres);
            TrafficMaintain traffic = runBaselineAllSP(rN, reqs, threadNum, capThres);
//            int candiPNum = 10;
//            vector<vector<NodeId>> paths = runSVPBaseline(rN, reqs, candiPNum, threadNum, capThres);
//            TrafficMaintain traffic = runLinear(rN, requests, threadNum, capThres);
//            TrafficMaintain traffic = runBatch(rN, reqs, threadNum, sigma, tau, capThres);
//            TrafficMaintain traffic = runBatchIter(rN, requests, threadNum, sigma, tau, frac, capThres);
        }
    }
}


vector<Request> readRealLifeData(
        string &basepath, RoadNetwork &rN,
        int threadNum, float sigma, float tau, float frac, int capThres) {
    int o, d, departT;
    vector<Request> reqs;
    ifstream infile(basepath);
    while (infile >> o >> d >> departT) {
        assert(o <= rN.numNodes && d <= rN.numNodes);
        reqs.emplace_back(reqs.size(), o, d, departT);
    }
    infile.close();
    return reqs;
}

int main() {

    string basepath1 = "/export/project/yehong/traffic-assignment/data/";
    string basepath2 = "/Users/xyh/Desktop/traffic-assignment/data/";
    string basepath3 = "/home/yxudi/GRO/data/";

    string basepath = basepath3, location = "BJ";
    int threadNum = 160, capThres = 2, rep = 1; // c <=2 15%: ; c <= 5: 26%; c <= 10: 41%; c <= 20: 61%
    float frac = 0.005, sigma = 250, tau = 0.05;

    string map, coords, realLifePath, varyDistPath, sdPath = basepath + location + "_allSDs.txt";
    vector<string> hours;
    RoadNetwork rN;
    if (location == "NY" or location == "Wuhan") {
        map = basepath + location + ".txt", coords = basepath + "NY_NodeIDLonLat.txt";
        rN.readGraph(map, coords, 2);
        hours = {"7x5", "67x3", "678x2"};
        realLifePath = basepath + "real_life/NY/input/req";

    } else if (location == "BJ") {
        map = basepath + "BJ_map.txt", coords = basepath + "BJ_NodeIDLonLat.txt";
        rN.readGraph(map, coords, 1);
        varyDistPath = basepath + "extreme_congested/";
        hours = {"6x4", "6x3&78x1", "678x2"};
        realLifePath = basepath + "real_life/BJ/input/";
    }


//    for (const auto &i: {1, 2, 4}) {
//        rep = i;
//        cout << "BAT rep: " << rep  << endl;
//        readVaryDisQsDirectly(varyDistPath, rN, threadNum, sigma, tau, frac, capThres, rep);
//    }

//    /**
    Baseline baseline(rN, threadNum, capThres);
    Timer clock;
    clock.tick();
    vector<vector<int>> distances(rN.numNodes + 1, vector<int>(rN.numNodes + 1, 0));
    baseline.allDij(distances);
    clock.tock();
    cout << location << " AllDij Time:" << clock.duration().count() / 1000 << endl;
//    **/

//    /**
    for (auto const &hour: hours) {
        int numCandi = 10;
        cout << "B3 dataset: " << hour << endl;
        string realLifeD = realLifePath + hour + ".txt";
        vector<Request> requests = readRealLifeData(realLifeD, rN, threadNum, sigma, tau, frac, capThres);
        assert(!requests.empty());
//        vector<vector<NodeId>> paths = runSVPBaseline(rN, requests, numCandi, threadNum, capThres);
        vector<vector<NodeId>> paths(requests.size(), vector<NodeId>());
        clock.tick();
        baseline.greedyExpansion(distances, paths);
        clock.tock();
        long int cost = baseline.evaluate(paths);
        cout << clock.duration().count() / 1000 << "\t" << cost << endl;
//        for (const auto &sig: {8000, 16000, 32000, 64000}) {
//            sigma = sig;
//            for (const auto &reFrac: {0.0005, 0.001, 0.01}) {
//                frac = reFrac;
//                runBatchIter(rN, requests, threadNum, sigma, tau, frac, capThres);
//            }
//        }
    }
//    **/
    cout << "done" << endl;
    return 0;
}
