//
// Created by yehong.xu on 26/12/21.
//

#include "Graph.h"
//
// Created by yehong.xu on 11/12/21.
//


/*
Copyright (c) 2017 Theodoros Chondrogiannis
*/

float R = 6371000;

void RoadNetwork::readGraph(const string &filename, const string &filename1, int type) {
    if (type == 1)
        readGraph1(filename, filename1);
    if (type == 2)
        readGraph2(filename, filename1);
}

void RoadNetwork::readGraph1(const string &filename, const string &filename1) {

    ifstream infile(filename);
    int roadId, lnode, rnode;
    int line_cnt = 0, w = 0, c = 0;

    while (infile >> roadId >> lnode >> rnode >> w >> c) {
        line_cnt += 1;
        if (line_cnt == 1) {
            this->numNodes = lnode;
            this->numEdges = rnode;
            this->adjListOut = vector<EdgeList>(this->numNodes + 1);
            this->adjListInc = vector<EdgeList>(this->numNodes + 1);
            this->capacity = vector<EdgeList>(this->numNodes + 1);

        } else {
            if (lnode > this->numNodes || rnode > this->numNodes)
                continue;

            this->adjListOut[lnode][rnode] = w;
            this->adjListInc[rnode][lnode] = w;
            if (c == 0) c = 1; // avoid floating point exception
            this->capacity[lnode][rnode] = c;
        }
    }
    infile.close();

    coords = vector<Coord>(numNodes + 1);
    ifstream infile2(filename1);
    NodeId nodeId;
    float lat, lon;
    while (infile2 >> nodeId >> lon >> lat) {
        coords[nodeId] = make_pair(lat, lon);
        if (lon > maxLon)
            maxLon = lon;
        if (lon < minLon)
            minLon = lon;
        if (lat > maxLat)
            maxLat = lat;
        if (lat < minLat)
            minLat = lat;
    }
    infile2.close();
}

void RoadNetwork::readGraph2(const string &filename, const string &filename2) {
    ifstream IF(filename);
    if (!IF) {
        cout << "Cannot open Map " << filename << endl;
    }

    int lnode, rnode, w;

    for (string line; getline(IF, line);) {
        istringstream iss(line);
        vector<string> tokens;
        copy(istream_iterator<string>(iss),
             istream_iterator<string>(),
             back_inserter(tokens));

        if (tokens[0] == "%") {
            this->numNodes = stoi(tokens[1]);
            this->numEdges = stoi(tokens[2]);
            this->adjListOut.resize(this->numNodes + 1);
            this->adjListInc = vector<EdgeList>(this->numNodes + 1);
            this->capacity = vector<EdgeList>(this->numNodes + 1);

        } else {
            lnode = stoi(tokens[0]);
            rnode = stoi(tokens[1]);
            w = stod(tokens[2]);

            int t = round(w / 50000.0 * 3600.0);
            this->adjListOut[lnode][rnode] = t;
            this->adjListInc[rnode][lnode] = t;
            int c = round(w / 13.0);
            if (c == 0) c = 1; // avoid floating point exception
            this->capacity[lnode][rnode] = c;
        }
    }
    IF.close();

    coords = vector<Coord>(numNodes + 1);
    ifstream infile2(filename2);
    NodeId nodeId;
    float lat, lon;
    while (infile2 >> nodeId >> lon >> lat) {
        coords[nodeId] = make_pair(lat, lon);
        if (lon > maxLon)
            maxLon = lon;
        if (lon < minLon)
            minLon = lon;
        if (lat > maxLat)
            maxLat = lat;
        if (lat < minLat)
            minLat = lat;
    }
    infile2.close();
    cout << "finish loading graph" << endl;
}

int RoadNetwork::getEdgeWeight(NodeId lnode, NodeId rnode) {
    return this->adjListOut[lnode][rnode];
}

float RoadNetwork::distance(float lat1, float lon1, float lat2, float lon2) {
    // https://www.movable-type.co.uk/scripts/latlong.html
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

    float a = sin(dlat / 2) * sin(dlat / 2) +
              cos(lat1) * cos(lat2) * pow(sin(dlon / 2), 2);

    float c = 2 * atan2(sqrt(a), sqrt(1 - a));

    // Radius of Earth in Meters, R = 6371

    // Calculate the result
    return R * c;
}

float RoadNetwork::toRadians(const float degree) {
    // cmath library in C++
    // defines the constant
    // M_PI as the value of
    // pi accurate to 1e-30
    float one_deg = M_PI / 180;
    return (one_deg * degree);
}

Coord RoadNetwork::interPoint(float frac, float lat1, float lon1, float lat2, float lon2) {
    // fraction along great circle route (frac=0 is point 1, f=1 is point 2)
    float angDis = distance(lat1, lon1, lat2, lon2) / R;
    float phi1 = toRadians(lat1), phi2 = toRadians(lat2), lambda1 = toRadians(lon1), lambda2 = toRadians(lon2);
    float a = sin((1 - frac) * angDis) / sin(angDis);
    float b = sin(frac * angDis) / sin(angDis);
    float x = a * cos(phi1) * cos(lambda1) + b * cos(phi2) * cos(lambda2);
    float y = a * cos(phi1) * sin(lambda1) + b * cos(phi2) * sin(lambda2);
    float z = a * sin(phi1) + b * sin(phi2);
    float phi3 = atan2(z, sqrt(x * x + y * y)) / (M_PI / 180);
    float lambda3 = atan2(y, x) / (M_PI / 180);
    return Coord(phi3, lambda3);
}

float RoadNetwork::bearing(float lat1, float lon1, float lat2, float lon2) {
    lat1 = toRadians(lat1);
    lon1 = toRadians(lon1);
    lat2 = toRadians(lat2);
    lon2 = toRadians(lon2);

    // Haversine Formula
    float dlon = lon2 - lon1;
    float dlat = lat2 - lat1;

    float y = sin(dlon) * cos(lat2);
    float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon);
    float theta = atan2(y, x);
    return fmod((theta * 180 / M_PI + 360), 360); // in degrees
}

bool Path::containsEdge(Edge &e) {
    bool res = false;

    for (unsigned int i = 0; i < this->nodes.size() - 1; i++) {
        if (this->nodes[i] == e.first && this->nodes[i + 1] == e.second) {
            res = true;
            break;
        }
    }

    return res;
}

double Path::overlap_ratio(RoadNetwork *rN, Path &path2) {
    double sharedLength = 0;

    for (unsigned int i = 0; i < path2.nodes.size() - 1; i++) {
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

bool operator==(const Path &lp, const Path &rp) {
    if (lp.length != rp.length || lp.nodes.size() != rp.nodes.size())
        return false;
    for (int i = 0; i < lp.nodes.size(); i++) {
        if (lp.nodes[i] != rp.nodes[i])
            return false;
    }
    return true;
}

bool operator==(const Edge &e, const Edge &f) {
    return ((e.first == f.first) && (e.second == f.second));
}