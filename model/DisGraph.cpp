//
// Created by Yehong Xu on 18/7/2022.
//

#include "Graph.h"

DisGraph::DisGraph(RoadNetwork &rN, float sigma, float tau, int threadNum) : rN(rN) {
    this->sigma = sigma;
    this->tau = tau;
    this->vToGrid.resize(rN.numNodes + 1);
    int longMarkNum = ceil(RoadNetwork::distance(rN.minLat, rN.minLon, rN.minLat, rN.maxLon) / sigma);
    int latMarkNum = ceil(RoadNetwork::distance(rN.minLat, rN.minLon, rN.maxLat, rN.minLon) / sigma);

    longitutes.reserve(longMarkNum);
    longitutes.emplace_back(rN.minLon);
    latitudes.reserve(latMarkNum);
    latitudes.emplace_back(rN.minLat);

    for (int i = 1; i <= longMarkNum; i++) { //
        float frac = (float) i / ((float) longMarkNum - 1);
        Coord coord = RoadNetwork::interPoint(frac, rN.maxLat, rN.minLon, rN.maxLat, rN.maxLon);
        longitutes.emplace_back(coord.second);
    }

    for (int i = 1; i < latMarkNum; i++) {
        float frac = (float) i / ((float) latMarkNum - 1);
        Coord coord = RoadNetwork::interPoint(frac, rN.minLat, rN.minLon, rN.maxLat, rN.minLon);
        latitudes.emplace_back(coord.first);
    }

    vector<Grid> v;
    grids.assign(latMarkNum, v);
    for (int i = 0; i < latMarkNum; i++) {
        for (int j = 0; j < longMarkNum; j++) {
            grids[i].emplace_back(Grid(i, j));
        }
    }

    boost::thread_group tGroup;
    int interval = ceil(1.0 * rN.numNodes / threadNum);
    for (int i = 0; i < threadNum; ++i) {
        int begin = i * interval, end = (i + 1) * interval;
        if (end >= rN.numNodes + 1)
            end = rN.numNodes + 1;
        tGroup.create_thread(boost::bind(
                &DisGraph::rangeBinarySearch, begin, end, boost::ref(longitutes),
                boost::ref(latitudes), boost::ref(rN.coords), boost::ref(vToGrid)));
    }
    tGroup.join_all();

    for (NodeId i = 0; i <= rN.numNodes; i++) {
        pair<int, int> pair1 = vToGrid[i];
        if (pair1.first == -1 or pair1.second == -1)
            continue;
        grids[pair1.first][pair1.second].vertices.emplace_back(i);
    }

    newABatch();

}

void DisGraph::rangeBinarySearch(int begin, int end, vector<float> &longitutes, vector<float> &latitudes,
                                 vector<Coord> &coords, vector<pair<int, int>> &vToGrid) {
    for (int i = begin; i < end; i++) {
        Coord coord = coords[i];
        int longIdx = binarySearch(coord.second, longitutes), latIdx = binarySearch(coord.first, latitudes);
        vToGrid[i] = make_pair(latIdx, longIdx);
    }
}


int DisGraph::binarySearch(float x, vector<float> &arr) {
    if (x < arr[0] || x > arr[arr.size() - 1]) {
        return -1;
    }

    int low = 0, high = arr.size() - 1, mid = floor(((float) low + (float) high) / 2), premid = -1;

    while (low <= high && premid != mid) {
        if (arr[mid] <= x && arr[mid + 1] > x)
            return mid;

        if (arr[mid + 1] == x)
            return mid + 1;

        if (x < arr[mid])
            high = mid;
        else if (x > arr[mid + 1])
            low = mid + 1;

        premid = mid;
        mid = floor(((float) low + (float) high) / 2);
    }

    return -1;
}

bool DisGraph::evaluateQ(Request &req) {
    const int range = 2;

    pair<int, int> oGrid = this->vToGrid[req.o];
    pair<int, int> dGrid = this->vToGrid[req.d];

    int dLat = abs(oGrid.first - dGrid.first), dLon = abs(oGrid.second - dGrid.second);

    bool overLoad = false;
    if (dLat > dLon) { // vary lon

        int startLat, startLon, endLat, endLon;
        if (dGrid.first < oGrid.first) {
            startLat = dGrid.first;
            endLat = oGrid.first;
            startLon = dGrid.second;
            endLon = oGrid.second;
        } else {
            startLat = oGrid.first;
            endLat = dGrid.first;
            startLon = oGrid.second;
            endLon = dGrid.second;
        }

        float gradient = abs((float) (endLon - startLon) / (float) (endLat - startLat));

        countIntoG(1, startLat - 1, startLon, overLoad);
        for (int increLat = 0; increLat <= endLat - startLat; increLat++) {
            int increLon = floor(gradient * 1.0 * increLat);
            countIntoG(0, startLat + increLat, startLon + increLon, overLoad);
            for (int i = 1; i <= range; i++) {
                countIntoG(i, startLat + increLat, startLon + increLon - i, overLoad);
                countIntoG(i, startLat + increLat, startLon + increLon + i, overLoad);
            }
        }
        countIntoG(1, endLat + 1, endLon, overLoad);

    } else {
        int startLat, startLon, endLat, endLon;
        if (dGrid.second < oGrid.second) {
            startLon = dGrid.second;
            endLon = oGrid.second;
            startLat = dGrid.first;
            endLat = oGrid.first;
        } else {
            startLon = oGrid.second;
            endLon = dGrid.second;
            startLat = oGrid.first;
            endLat = dGrid.first;
        }

        float gradient = abs((float) (endLat - startLat) / (float) (endLon - startLon));

        countIntoG(1, startLat, startLon - 1, overLoad);
        for (int increLon = 0; increLon <= endLon - startLon; increLon++) {

            int increLat = floor(gradient * 1.0 * increLon);

            countIntoG(1, startLat + increLat, startLon + increLon, overLoad);
            for (int i = 1; i <= range; i++) {
                countIntoG(i, startLat + increLat + i, startLon + increLon, overLoad);
                countIntoG(i, startLat + increLat - i, startLon + increLon, overLoad);
            }
        }
        countIntoG(1, endLat, endLon + 1, overLoad);
    }
    return overLoad;
}

void DisGraph::findNearbyGrids(Request &req) {
    if (evaluateQ(req)) {
        newABatch();
        evaluateQ(req);
    }

    batches[batches.size()-1].emplace_back(req.id);
}

float DisGraph::occurProb(int i) {
    float value = 1.0 / (sigma * sqrt(2.0 * M_PI)) * exp(-0.5 * i * i);
    return value;
}

void DisGraph::countIntoG(int i, int lat, int lon, bool &overFload) {

    if (lat == -1 or lon == -1 or lat >= latitudes.size() or lon >= longitutes.size()) return;

    relatedG.insert(make_pair(lat, lon));
    grids[lat][lon].eFlow += occurProb(i);
    if (grids[lat][lon].eFlow > tau)
        overFload = true;
}

void DisGraph::newABatch() {
    vector<RequestId> v;
    batches.emplace_back(v);
    for (auto pair:relatedG) {
        grids[pair.first][pair.second].eFlow = 0;
    }
    relatedG.clear();
}
