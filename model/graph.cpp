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
    int roadId, lnode, rnode;
    int line_cnt = 0, w = 0, c= 0;
    while (infile >> roadId >> lnode >> rnode >> w >> c)
    {
        line_cnt += 1;
        if (line_cnt == 1)
        {
            this->numNodes = lnode;
            this->numEdges = rnode;
            this->adjListOut = vector<EdgeList>(this->numNodes + 1);
            this->adjListInc = vector<EdgeList>(this->numNodes + 1);
            this->capacity.resize(this->numEdges + 1);

            std::vector<Semaphore> sms(numEdges + 1);
            this->edgeSM.swap(sms);

            this->edgeMap = vector<EdgeList>(this->numNodes + 1);

        } else
        {
            if (lnode > this->numNodes || rnode > this->numNodes)
                continue;

            this->adjListOut[lnode][rnode] = w;
            this->edgeMap[lnode][rnode] = roadId;
            this->adjListInc[rnode][lnode] = w;

            this->capacity[roadId] = c;
        }
    }
    infile.close();
}

RoadNetwork::RoadNetwork(const char *filename, const char *filename1)
{
    ifstream infile(filename);
    int roadId, lnode, rnode;
    int line_cnt = 0, w = 0, c= 0;
    while (infile >> roadId >> lnode >> rnode >> w >> c)
    {
        line_cnt += 1;
        if (line_cnt == 1)
        {
            this->numNodes = lnode;
            this->numEdges = rnode;
            this->adjListOut = vector<EdgeList>(this->numNodes + 1);
            this->adjListInc = vector<EdgeList>(this->numNodes + 1);
            this->capacity.resize(this->numEdges + 1);
            this->edgeMap = vector<EdgeList>(this->numNodes + 1);

            this->capacity.resize(numEdges + 1);
            std::vector<Semaphore> sms(numEdges + 1);
            this->edgeSM.swap(sms);
        } else
        {
            if (lnode > this->numNodes || rnode > this->numNodes)
                continue;

            this->adjListOut[lnode][rnode] = w;
            this->edgeMap[lnode][rnode] = roadId;
            this->adjListInc[rnode][lnode] = w;

            this->capacity[roadId] = c;
            this->edgeSM[roadId].initialize(1);
        }
    }
    infile.close();

    coords = vector<Coord>(numNodes + 1);
    ifstream infile2(filename1);
    NodeId nodeId;
    double lat, lon;
    while (infile2 >> nodeId >> lon >> lat)
    {
        coords[nodeId] = make_pair(lon, lat);
    }
    infile2.close();
}

int RoadNetwork::getEdgeWeight(NodeId lnode, NodeId rnode)
{
    return this->adjListOut[lnode][rnode];
}

float RoadNetwork::distance(Coord p1, Coord p2)
{
    float lon1 = p1.first;
    float lat1 = p1.second;
    float lon2 = p2.first;
    float lat2 = p2.second;
    // Convert the latitudes
    // and longitudes
    // from degree to radians.
    lat1 = toRadians(lat1);
    lon1 = toRadians(lon1);
    lat2 = toRadians(lat2);
    lon2 = toRadians(lon2);

    // Haversine Formula
    float dlon = lon2 - lon1;
    float dlat = lat2 - lat1;

    float ans = pow(sin(dlat / 2), 2) +
                cos(lat1) * cos(lat2) *
                pow(sin(dlon / 2), 2);

    ans = 2 * asin(sqrt(ans));

    // Radius of Earth in
    // Kilometers, R = 6371
    // Use R = 3956 for miles
    float R = 6371000;

    // Calculate the result
    ans = ans * R;

    return ans;
}

float RoadNetwork::toRadians(const float degree)
{
    // cmath library in C++
    // defines the constant
    // M_PI as the value of
    // pi accurate to 1e-30
    float one_deg = (M_PI) / 180;
    return (one_deg * degree);
}

bool operator==(const Edge &le, const Edge &re)
{
//    return (le.v1 == re.v1 && le.v2 == re.v2) || (le.v2 == re.v1 && le.v1 == re.v2);
    return le.first == re.first && le.second == re.second;
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
        Edge e = Edge(path2.nodes[i], path2.nodes[i + 1]);
        if (this->containsEdge(e))
            sharedLength += rN->getEdgeWeight(path2.nodes[i], path2.nodes[i + 1]);
    }

    return sharedLength / path2.length;
}

//void Path::writePath(const basic_string<char> &file) const
//{
//    ofstream ofstream(file);
//    for (const auto node: this->nodes)
//    {
//        ofstream << node << "\t";
//    }
//    ofstream << endl;
//    ofstream.close();
//}

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