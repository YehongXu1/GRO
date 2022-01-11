//
// Created by yehong.xu on 26/12/21.
//

#include "graph.h"
//
// Created by yehong.xu on 11/12/21.
//


/*
Copyright (c) 2017 Theodoros Chondrogiannis
*/


RoadNetwork::RoadNetwork(const char *filename)
{
    ifstream infile(filename);
    NodeId lnode, rnode, edgeId;
    int line_cnt = 0, w = -1;
    while (infile >> edgeId >> lnode >> rnode >> w)
    {
        line_cnt += 1;
        if (line_cnt == 1)
        {
            this->numNodes = lnode;
            this->numEdges = rnode;
            this->adjListOut = vector<EdgeList>(this->numNodes + 1);
            this->adjListInc = vector<EdgeList>(this->numNodes + 1);
            this->adjListInHeu = vector<EdgeList>(this->numNodes + 1);
        } else
        {
            if (lnode > this->numNodes || rnode > this->numNodes)
                continue;

            this->adjListOut[lnode][rnode] = w;
            this->adjListInc[rnode][lnode] = w;
            this->adjListInHeu[rnode][lnode] = w;
        }
    }

    infile.close();
    cout << "finish loading graph" << endl;
}

RoadNetwork::RoadNetwork(const char *filename, const char *filename1)
{
    ifstream infile(filename);
    NodeId lnode, rnode, edgeId;
    int line_cnt = 0, w = -1;
    while (infile >> edgeId >> lnode >> rnode >> w)
    {
        line_cnt += 1;
        if (line_cnt == 1)
        {
            this->numNodes = lnode;
            this->numEdges = rnode;
            this->adjListOut = vector<EdgeList>(this->numNodes + 1);
            this->adjListInc = vector<EdgeList>(this->numNodes + 1);

            this->adjListInHeu = vector<EdgeList>(this->numNodes + 1);
        } else
        {
            if (lnode > this->numNodes || rnode > this->numNodes)
                continue;

            this->adjListOut[lnode][rnode] = w;
            this->adjListInc[rnode][lnode] = w;

            this->adjListInHeu[rnode][lnode] = w;
        }
    }
    infile.close();

    coords = vector<Coord>(numNodes + 1);
    ifstream infile2(filename1);
    NodeId nodeId;
    double lat, lon;
    while (infile2 >> nodeId >> lon >> lat)
    {
        coords[nodeId] = make_pair(lat, lon);
    }
    infile2.close();

    cout << "finish loading graph" << endl;
}

int RoadNetwork::getEdgeWeight(NodeId lnode, NodeId rnode)
{
    return this->adjListOut[lnode][rnode];
}

RoadNetwork::~RoadNetwork()
{
    this->adjListOut.clear();
    this->adjListInc.clear();
}

void RoadNetwork::adjustWeight(NodeId lnode, NodeId rnode, int increment)
{
    adjListInHeu[rnode][lnode] += increment;
}

bool operator==(const Edge &le, const Edge &re)
{
    return (le.first == re.first && le.second == re.second) || (le.second == re.first && le.first == re.second);
}

bool Path::containsEdge(Edge &e)
{
    bool res = false;

    for (unsigned int i = 0; i < this->nodes.size() - 1; i++)
    {
        if (this->nodes[i] == e.first && this->nodes[i + 1] == e.second)
        {
            res = true;
            break;
        }
    }

    return res;
}

double Path::overlap_ratio(RoadNetwork *rN, Path &path2)
{
    double sharedLength = 0;

    for (unsigned int i = 0; i < path2.nodes.size() - 1; i++)
    {
        Edge e = make_pair(path2.nodes[i], path2.nodes[i + 1]);
        if (this->containsEdge(e))
            sharedLength += rN->getEdgeWeight(path2.nodes[i], path2.nodes[i + 1]);
    }

    return sharedLength / path2.length;
}

bool operator==(const Path &lp, const Path &rp)
{
    if (lp.length != rp.length || lp.nodes.size() != rp.nodes.size())
        return false;
    for (int i = 0; i < lp.nodes.size(); i++)
    {
        if (lp.nodes[i] != rp.nodes[i])
            return false;
    }
    return true;
}