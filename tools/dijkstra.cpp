#include "tools.h"

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

void dijkstra_label_heu(RoadNetwork *rN, NodeId source, NodeId target, Label *curLabel)
{
    unsigned int count = 0;
    benchmark::heapCust<2> heap(rN->numNodes + 1);

    int newLength = 0;
    EdgeList::iterator iterAdj;
    vector<int> distances(rN->numNodes + 1, INT_MAX);
    vector<bool> visited(rN->numNodes + 1);
    distances[target] = 0;
    auto *srcLabel = new Label(target, newLength);
    heap.update(srcLabel);

    while (!heap.empty())
    {
        heap.extract_min(curLabel);

        visited[curLabel->node_id] = true;
        distances[curLabel->node_id] = curLabel->length;

        if (curLabel->node_id == source)
        {
            return;
        }

        if (++count == rN->numNodes)
            break;

        else
        { // Expand search
            // For each incoming edge.
            for (iterAdj = rN->adjListInHeu[curLabel->node_id].begin();
                 iterAdj != rN->adjListInHeu[curLabel->node_id].end(); iterAdj++)
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