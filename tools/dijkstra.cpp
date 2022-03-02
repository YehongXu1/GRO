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
//    PriorityQueue queue;
    benchmark::heapDij<2> queue(rN->numNodes + 1);

    Path resPath;
    int newLength = 0;
    EdgeList::iterator iterAdj;

    vector<int> distances(rN->numNodes + 1, INT_MAX);
    vector<bool> visited(rN->numNodes + 1);
    Label *targetLabel = nullptr;
    distances[target] = 0;
    vector<Label *> allCreatedLabels;
    auto *srcLabel = new Label(target, newLength);
    queue.update(srcLabel);
    allCreatedLabels.push_back(srcLabel);

    auto *curLabel = new Label();
    while (!queue.empty())
    {
        queue.extract_min(curLabel);
//        queue.pop();

        visited[curLabel->node_id] = true;

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

        auto *newPrevious = new Label(curLabel);

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

                if (distances[iterAdj->first] > newLength)
                {
                    auto *label = new Label(iterAdj->first, newLength, newPrevious);
                    allCreatedLabels.push_back(label);
                    distances[iterAdj->first] = newLength;
                    queue.update(label);
                }
            }
        }
    }

    for (auto &allCreatedLabel: allCreatedLabels)
        delete allCreatedLabel;

    return make_pair(resPath, distances);
}

vector<Label *> dijkstra_label_timedep(
        vector<unordered_map<NodeId, EdgeFlowInfo>> &trafficStat,
        int timeReslo, int timeIntNum, NodeId source, NodeId target)
{
    benchmark::heapDij<2> queue(trafficStat.size());

    unordered_map<NodeId, EdgeFlowInfo>::iterator iterAdj;

    long long int heuNewTravelTime;

    vector<long long int> heuDistances(trafficStat.size(), INT_MAX);

    heuDistances[source] = 0;

    vector<bool> visited(trafficStat.size(), false);
    vector<Label *> labels(trafficStat.size(), nullptr);
    auto *curLabel = new Label(source, 0);
    curLabel->heuLength = 0;
    queue.update(curLabel);
    labels[source] = new Label();

    while (!queue.empty())
    {
        queue.extract_min(curLabel);
        if (curLabel->node_id == target)
        {
            Label *ll = curLabel;
            while (ll->previous != nullptr)
            {
                ll->previous->nextL = ll;
                ll = ll->previous;
            }
            return labels;
        }

        visited[curLabel->node_id] = true;

        int timeInt = floor(curLabel->heuLength / timeReslo);

        if (timeInt >= timeIntNum)
            cout << timeInt << endl;
        assert(timeInt < timeIntNum);

        assert(labels[curLabel->node_id] != nullptr);
        labels[curLabel->node_id]->copy(curLabel);

        for (iterAdj = trafficStat[curLabel->node_id].begin();
             iterAdj != trafficStat[curLabel->node_id].end(); iterAdj++)
        {
            if (visited[iterAdj->first])
                continue;

            heuNewTravelTime = curLabel->heuLength + iterAdj->second.tempWeight[timeInt];
            assert(heuNewTravelTime >= 0);

            if (heuDistances[iterAdj->first] > heuNewTravelTime)
            {
                if (labels[iterAdj->first] == nullptr)
                {
                    labels[iterAdj->first] = new Label(iterAdj->first, 0, labels[curLabel->node_id]);
                } else
                {
                    labels[iterAdj->first]->previous = labels[curLabel->node_id];
                }
                labels[iterAdj->first]->heuLength = heuNewTravelTime;
                heuDistances[iterAdj->first] = heuNewTravelTime;
                queue.update(labels[iterAdj->first]);
            }
        }
    }
    assert(cout << "target not  found" << endl);
    return labels;
}

int dijkstra_dist_del(RoadNetwork *rN, NodeId source, NodeId target)
{
    int newLength = 0, resDist = -1;
    EdgeList::iterator iterAdj;

    benchmark::heapDij<2> heap(rN->numNodes + 1);
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