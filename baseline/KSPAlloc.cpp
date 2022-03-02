//
// Created by yehong.xu on 31/1/2022.
//

#include "KSPAlloc.h"
#include "../KSP/kspwlo.h"

KSPAlloc::KSPAlloc(Traffic &traffic, const basic_string<char> &output) : traffic(traffic)
{
    string delimiter = "\t";
    for (RequestId id = 0; id < traffic.reqNo; id ++)
    {
        Request od = traffic.requestODs[id];
        string fileName = output + "esx/" + to_string(od.o) + "-" + to_string(od.d) + ".txt";
        ifstream file(fileName);
        assert(file.is_open());
        string line;
        while (getline(file, line))
        {
            Path path;
            size_t pos = 0;
            string token, s = line;
            while ((pos = s.find(delimiter)) != string::npos)
            {
                path.nodes.emplace_back(stoi(s.substr(0, pos)));
                s.erase(0, pos + delimiter.length());
            }
            assert(od.o == path.nodes[0] && od.d == path.nodes[path.nodes.size() - 1]);
            candidates[id].emplace_back(path);
        }
    }
}

KSPAlloc::KSPAlloc(Traffic &traffic, int k, double theta) : traffic(traffic)
{
    for (RequestId id =0; id < traffic.reqNo; id++)
    {
        candidates[id] = esx(
                &traffic.rN, traffic.requestODs[id].o, traffic.requestODs[id].d, k, theta);
    }
}

int KSPAlloc::selectAPath(RequestId req)
{
    random_device rd; // obtain a random number from hardware
    mt19937 gen(rd()); // seed the generator
    uniform_int_distribution<> distr(0, candidates[req].size() - 1); // define the range

    return distr(gen); // generate numbers
}

void KSPAlloc::assignPath()
{
    unordered_map<Edge, unordered_set<TimeIntIdx>, hash_edge> traversedEdges;
    for (RequestId id = 0; id < traffic.reqNo; id++)
    {
        int theP = selectAPath(id);
        traffic.trajectories[id] = pathToLabel(candidates[id][theP]);
    }
    cost = traffic.simulateTraffic();

    for (RequestId id = 0; id < traffic.reqNo; id++)
    {
        Label *label = traffic.trajectories[id];
        while (label != nullptr)
        {
            Label *nextL = label->nextL;
            delete label;
            label = nextL;
        }
    }
}

Label *KSPAlloc::pathToLabel(Path &path)
{
    int length = 0;
    auto *prevL = new Label(path.nodes[0], length);
    Label *firstL = prevL;
    for (int i = 1; i < path.nodes.size(); i++)
    {
        NodeId curId = path.nodes[i];
        auto * curL = new Label(curId, 0, prevL);
        prevL->nextL = curL;
        prevL = curL;
    }
    return firstL;
}

long long int KSPAlloc::getCost()
{
    return cost;
}
