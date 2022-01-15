#include "tools.h"
#include <random>

/*
 *
 *	dijkstra_path_and_bounds(RoadNetwork, NodeId, NodeId, vector<int>, unordered_set<Edge,boost::hash<Edge>>)
 *	-----
 *	This KSP is essentially an all-to-one Dijkstra and returns at the same
 *	time the shortest path from the source to the target AND the distances of
 *	all nodes to the target. This KSP is used way to compute the shortest
 *	path along with exact lower bounds for OnePas, MultiPass and OnePass+.
 *
 */


pair<Path, vector<int>> dijkstra_path_and_bounds(RoadNetwork *rN, NodeId source, NodeId target)
{
    unsigned int count = 0;
    benchmark::heapCust<2> heap(rN->numNodes + 1);

    Path resPath;
    int newLength = 0;
    EdgeList::iterator iterAdj;
    vector<int> distances(rN->numNodes + 1, INT_MAX);
    vector<bool> visited(rN->numNodes + 1);
    Label *targetLabel = nullptr;
    auto *curLabel = new Label(-1, -1);
    distances[target] = 0;
    auto *srcLabel = new Label(target, newLength);
    heap.update(srcLabel);

    while (!heap.empty())
    {
        heap.extract_min(curLabel);

        visited[curLabel->node_id] = true;
        distances[curLabel->node_id] = curLabel->length;

        if (curLabel->node_id == source)
        { // Destination has been found
            targetLabel = curLabel;
            resPath.length = curLabel->length;
            while (targetLabel != nullptr)
            {
                resPath.nodes.push_back(targetLabel->node_id);
                targetLabel = targetLabel->previous;
            }

        }

        if (++count == rN->numNodes)
            break;

        else
        { // Expand search
            // For each incoming edge.
            for (iterAdj = rN->adjListInc[curLabel->node_id].begin();
                 iterAdj != rN->adjListInc[curLabel->node_id].end(); iterAdj++)
            {
                if (visited[iterAdj->first])
                    continue;

                newLength = curLabel->length + iterAdj->second;
                Label *newPrevious = curLabel;
                if (distances[iterAdj->first] > newLength)
                {
                    auto *label = new Label(iterAdj->first, newLength, newPrevious);
                    distances[iterAdj->first] = newLength;
                    heap.update(label);
                }
            }
        }
    }

    return make_pair(resPath, distances);
}

void dijkstra_label(RoadNetwork *rN, NodeId source, NodeId target, Label *curLabel)
{
    unsigned int count = 0;
    benchmark::heapCust<2> heap(rN->numNodes + 1);

    int newLength = 0;
    EdgeList::iterator iterAdj;
    vector<int> distances(rN->numNodes + 1, INT_MAX);
    vector<bool> visited(rN->numNodes + 1);
    Label *targetLabel = nullptr;
    distances[target] = 0;
    auto *srcLabel = new Label(target, newLength);
    heap.update(srcLabel);

    while (!heap.empty())
    {
        heap.extract_min(curLabel);

        visited[curLabel->node_id] = true;
        distances[curLabel->node_id] = curLabel->length;

        if (curLabel->node_id == source)
            return;

        if (++count == rN->numNodes)
            break;

        else
        { // Expand search
            // For each incoming edge.
            for (iterAdj = rN->adjListInc[curLabel->node_id].begin();
                 iterAdj != rN->adjListInc[curLabel->node_id].end(); iterAdj++)
            {
                if (visited[iterAdj->first])
                    continue;

                newLength = curLabel->length + iterAdj->second;
                auto *newPrevious = new Label(curLabel);
                if (distances[iterAdj->first] > newLength)
                {
                    auto *label = new Label(iterAdj->first, newLength, newPrevious);
                    distances[iterAdj->first] = newLength;
                    heap.update(label);
                }
            }
        }
    }
    curLabel = nullptr;
}

void dijkstra_label_timedep(vector<unordered_map<NodeId, EdgeFlowInfo>> &trafficStat, int timeReslo, int timeIntNum,
                            NodeId source, NodeId target, Label *curLabel)
{
    benchmark::heapCust<2> heap(trafficStat.capacity());

    int newTravelTime = 0;
    unordered_map<NodeId, EdgeFlowInfo>::iterator iterAdj;
    vector<int> distances(trafficStat.capacity(), INT_MAX);
    vector<bool> visited(trafficStat.capacity());
    distances[source] = 0;
    heap.update(curLabel);

    while (!heap.empty())
    {
        heap.extract_min(curLabel);

        visited[curLabel->node_id] = true;
        distances[curLabel->node_id] = curLabel->length;

        if (curLabel->node_id == target)
        {
            return;
        } else
        { // Expand search
            // For each incoming edge.
            for (iterAdj = trafficStat[curLabel->node_id].begin();
                 iterAdj != trafficStat[curLabel->node_id].end(); iterAdj++)
            {
                int timeInt = curLabel->length / timeReslo;
                assert(timeInt < timeIntNum);

//                cout << iterAdj->second.tempWeight[timeInt] << endl;
                if (iterAdj->second.tempWeight[timeInt] == INT_MAX)
                    continue; // this (e,t) is blocked

                newTravelTime = curLabel->length + iterAdj->second.tempWeight[timeInt];

                assert(newTravelTime >= 0 && newTravelTime < INT_MAX);

                auto *newPrevious = new Label(curLabel);
                if (distances[iterAdj->first] > newTravelTime)
                {
                    auto *label = new Label(iterAdj->first, newTravelTime, newPrevious);
                    distances[iterAdj->first] = newTravelTime;
                    heap.update(label);
                }
            }
        }
    }
    curLabel = nullptr;
}

int dijkstra_dist_del(RoadNetwork *rN, NodeId source, NodeId target)
{
    int newLength = 0, resDist = -1;
    EdgeList::iterator iterAdj;

    benchmark::heapCust<2> heap(rN->numNodes + 1);
    vector<int> distances(rN->numNodes + 1, INT_MAX);
    vector<bool> visited(rN->numNodes + 1);
    auto *curLabel = new Label(-1, -1);
    distances[target] = 0;
    auto *srcLabel = new Label(target, newLength);
    heap.update(srcLabel);

    while (!heap.empty())
    {
        heap.extract_min(curLabel);

        visited[curLabel->node_id] = true;
        distances[curLabel->node_id] = curLabel->length;

        if (curLabel->node_id == source)
        { // Destination has been found
            resDist = curLabel->length;
            return resDist;
        } else
        { // Expand search
            // For each incoming edge.
            for (iterAdj = rN->adjListInc[curLabel->node_id].begin();
                 iterAdj != rN->adjListInc[curLabel->node_id].end(); iterAdj++)
            {
                if (visited[iterAdj->first])
                    continue;

                newLength = curLabel->length + iterAdj->second;
                Label *newPrevious = curLabel;
                if (distances[iterAdj->first] > newLength)
                {
                    auto *label = new Label(iterAdj->first, newLength, newPrevious);
                    distances[iterAdj->first] = newLength;
                    heap.update(label);
                }
            }
        }
    }

    return resDist;
}

bool randomBool(int trialNum, double sucRateEachTrial)
{
    // sucRateEachTrial: prob not to add weight
    random_device rd;
    mt19937 gen(rd());
    // give "true" sucRateEachTrial of the time
    // give "false" (1-sucRateEachTrial) of the time
    bernoulli_distribution d(sucRateEachTrial);
    bool notToAddValue = true;
    for (int i = 0; i < trialNum; i++)
    {
        notToAddValue = notToAddValue & d(gen);
    }

    return not notToAddValue;
}