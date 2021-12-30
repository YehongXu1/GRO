//
// Created by yehong.xu on 28/12/21.
//

#include "Traffic.h"
#include <fstream>

vector<NodeId> Traffic::getNodesInR(Coord &center, int r)
{
    vector<NodeId> nodes;

    for (NodeId i = 0; i <= rN.numNodes; i++)
    {
        if (RoadNetwork::distance(center, rN.coords[i]) <= r)
        {
            nodes.push_back(i);
        }
    }
    return nodes;
}


void Simulation::basicSimulation()
{
    /**
     * Based on base cost
     */

    trajs.clear();
    for (const auto &source: traffic.sources)
    {
        for (const auto &target: traffic.targets)
        {
            auto *sourceLabel = new Label();
            dijkstra_label(&traffic.rN, source, target, sourceLabel);
            trajs.push_back(sourceLabel);
        }
    }

    getEdgeFlowInRN();
}

void Simulation::heuBasedSimulation()
{
    trajs.clear();
    for (const auto &source: traffic.sources)
    {
        for (const auto &target: traffic.targets)
        {
            auto *sourceLabel = new Label();
            dijkstra_label_heu(&traffic.rN, source, target, sourceLabel);
            trajs.push_back(sourceLabel);
        }
    }

    getEdgeFlowInRN();
}

void Simulation::updateHeuWeight()
{
    for (const auto &edge: traversedEdges)
    {
        traffic.rN.adjListInHeu[edge.second][edge.first] += edgeFlowDist[edge.first][edge.second].first.size();
    }
}

void Simulation::getEdgeFlowInRN()
{
    for (const auto &sourceLabel: trajs)
    {
        int totalLength = sourceLabel->length;
        Label *curLabel = sourceLabel;
        curLabel->length = totalLength - curLabel->length;

        while (curLabel != nullptr)
        {
            auto *prevLabel = new Label(curLabel);

            curLabel = curLabel->previous;
            if (curLabel == nullptr)
                continue;

            curLabel->length = totalLength - curLabel->length;
            edgeFlowDist[prevLabel->node_id][curLabel->node_id].first.push_back(
                    new pair(prevLabel->length, curLabel->length));

            traversedEdges.emplace_back(prevLabel->node_id, curLabel->node_id);
        }
    }
}

int Simulation::getCollisionT(Label *label)
{
    // should improve
    int collision = 0;
    assert(label->previous != nullptr);
    for (const auto &interval: edgeFlowDist[label->node_id][label->previous->node_id].first)
    {
        if (interval->first <= label->length && interval->second >= label->length)
        {
            collision++;
        }
    }

    collision -= 1;
    if (collision > edgeFlowDist[label->previous->node_id][label->node_id].second)
        edgeFlowDist[label->node_id][label->previous->node_id].second = collision;

    return collision;
}

int Simulation::getCollision(Label *label)
{
    return edgeFlowDist[label->node_id][label->previous->node_id].first.size() - 1;
}

int Simulation::trajCostT(Label *sourceLabel)
{
    int cost = 0;
    while (sourceLabel->previous != nullptr)
    {
        int collision = getCollisionT(sourceLabel);
        if (collision > 1)
        {
            cost += (collision + traffic.rN.getEdgeWeight(sourceLabel->node_id, sourceLabel->previous->node_id));
        }
        sourceLabel = sourceLabel->previous;
    }
    return cost;
}

int Simulation::trajCost(Label *sourceLabel)
{
    int cost = 0;
    while (sourceLabel->previous != nullptr)
    {
        int collision = getCollision(sourceLabel);
        if (collision > 1)
        {
            cost += (collision + traffic.rN.getEdgeWeight(sourceLabel->node_id, sourceLabel->previous->node_id));
        }
        sourceLabel = sourceLabel->previous;
    }
    return cost;
}

int Simulation::trajCostCascade(Label *sourceLabel)
{
    // adjust arrival time

    return 0;
}

void Simulation::totalCostT()
{
    for (const auto &sourceLabel: trajs)
    {
        costsT += trajCostT(sourceLabel);
    }
}


void Simulation::totalCost()
{
    for (const auto &sourceLabel: trajs)
    {
        costs += trajCost(sourceLabel);
    }
}


int Simulation::totalCostCascade()
{
    int totalCost = 0;

    for (const auto &sourceLabel: trajs)
    {
        totalCost += trajCostCascade(sourceLabel);
    }

    return totalCost;
}


void Simulation::clearTraffic()
{
    for (const auto &edge: traversedEdges)
    {
        edgeFlowDist[edge.first][edge.second].first.clear();
    }
    trajs.clear();
}

void Simulation::writeResults(string &path)
{
    ofstream ofstream1(path + "/traj.csv");
    for (const auto &sourceLabel: trajs)
    {
        Label *label = sourceLabel;
        while (label != nullptr)
        {
            ofstream1 << traffic.rN.coords[label->node_id].second << "," << traffic.rN.coords[label->node_id].first
                      << endl;
            label = label->previous;
        }
    }
    ofstream1.close();

    ofstream ofstream2(path + "/collision.csv");
    int avgCollisionT = 0, avgCollision = 0;
    for (const auto &edge: traversedEdges)
    {
        avgCollision += edgeFlowDist[edge.first][edge.second].first.size();
        avgCollisionT += edgeFlowDist[edge.first][edge.second].second;
        ofstream2 << edge.first << "," << edge.second << "," << edgeFlowDist[edge.first][edge.second].second << endl;
    }
    avgCollisionT = round(avgCollisionT / traversedEdges.size());
    avgCollision = round(avgCollision / traversedEdges.size());
    ofstream2.close();

    ofstream ofstream3(path + "/ods.csv");
    for (const auto &item: traffic.sources)
    {
        ofstream3 << traffic.rN.coords[item].second << "," << traffic.rN.coords[item].first << endl;
    }

    for (const auto &item: traffic.targets)
    {
        ofstream3 << traffic.rN.coords[item].second << "," << traffic.rN.coords[item].first << endl;
    }
    ofstream2.close();

    ofstream ofstream4(path + "/setting.csv");
    ofstream4 << "r1 " << traffic.r1 << endl;
    ofstream4 << "r2 " << traffic.r2 << endl;
    ofstream4 << "costs " << costs << ", avg:" << avgCollision << endl;
    ofstream4 << "costsT " << costsT << ", avg:" << avgCollisionT << endl;
    ofstream4 << "sCenter " << traffic.sCenter.second << "," << traffic.sCenter.first << endl;
    ofstream4 << "sCenter " << traffic.tCenter.second << "," << traffic.tCenter.first << endl;
    ofstream4.close();
}



