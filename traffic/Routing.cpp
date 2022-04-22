//
// Created by yehong.xu on 1/2/2022.
//

#include "Routing.h"

Routing::Routing(Traffic &traffic) : traffic(traffic) {}

void Routing::mainAlg(double frac, bool fix)
{
    vector<RequestId> reqs;
    boost::push_back(reqs, boost::irange(0, traffic.reqNo));

    int iteration = 0;

    while (true)
    {
        // reroute reqs that are selected to be rerouted
        traffic.allTempDij(reqs);
        // evaluation is based on entire traffic
        curCost = traffic.simulateTraffic();
        // evaluate travel cost based on new traffic state
        cout << iteration << "," << reqs.size() << "," << curCost << endl;

        if (curCost == -1)
        {
            cout << iteration << "," << reqs.size() << "," << -1 << endl;
            return;
        }

        if (fix)
        {
            reqs = selectRerouteReqsFixPaths(reqs, frac); // fix some routes
            if (reqs.empty())
                break;
        } else
        {
            reqs = selectRerouteReqsCongET2(frac);
//            reqs = selectRerouteReqsUnfixPaths(frac); // select from all reqs
            if (iteration == 20)
                break;
        }

        traffic.clearEdgeProfile();
        traffic.updateTrafficLoading();
        traffic.deleteLabels(reqs);
        traffic.clearReqCngs();
        iteration += 1;
    }
}

vector<RequestId> Routing::selectRerouteReqsFixPaths(vector<int> &reqs, double frac)
{
    benchmark2::heapSelectQ<2, int, RequestId> reqHeap(traffic.reqNo);

    for (const auto &req: reqs)
    {
        int key = traffic.requestODs[req].sortBy;
        reqHeap.update(req, key);
    }

    int selectedP = floor(1.0 * frac * (int) reqs.size());
    RequestId requestId;
    int i;
    vector<RequestId> rerouteReqs;
    while (!reqHeap.empty() && rerouteReqs.size() < selectedP)
    {
        reqHeap.extract_max(requestId, i);
        rerouteReqs.emplace_back(requestId);
    }
    return rerouteReqs;
}

vector<RequestId> Routing::selectRerouteReqsUnfixPaths(double frac)
{
    benchmark2::heapSelectQ<2, int, RequestId> myReqHeap(traffic.reqNo);
    for (RequestId i = 0; i < traffic.reqNo; i++)
    {
        int key = traffic.requestODs[i].sortBy;
        myReqHeap.update(i, key);
    }

    int selectedP = floor(1.0 * frac * traffic.reqNo), i;
    RequestId requestId;
    vector<RequestId> rerouteReqs;
    while (!myReqHeap.empty() && rerouteReqs.size() < selectedP)
    {
        myReqHeap.extract_max(requestId, i);
        rerouteReqs.emplace_back(requestId);
    }
    return rerouteReqs;
}

vector<RequestId> Routing::selectRerouteReqsCongET(double frac)
{
    // from req perspective
    benchmark2::heapSelectQ<2, int, RequestId> myReqHeap(traffic.reqNo);
    for (RequestId i = 0; i < traffic.reqNo; i++)
    {
        int key = traffic.reqCongCnt[i].size();
        myReqHeap.update(i, key);
    }

    RequestId requestId;
    int key;
    vector<RequestId> selectedReqs;
    vector<bool> selected(traffic.reqNo, false);
    bool shouldUpdate = false;
    while (selectedReqs.size() < frac * traffic.reqNo)
    {
        myReqHeap.extract_max(requestId, key);
        if (selected[requestId])
            continue;
        selectedReqs.emplace_back(requestId);
        selected[requestId] = true;

        RequestId nextTop = myReqHeap.top();
        unordered_set<CongInfo *> &congInfos = traffic.reqCongCnt[requestId];
        for (auto &conginfo: congInfos)
        {
            conginfo->relatedReqs.erase(requestId); // no conginfo would contain selected request anymore
            if (conginfo->relatedReqs.size() > conginfo->thresCnt)
                continue;

            // Here, we missed to update requestId's conglist, doing this would incur errors because we are in the loop of its conglist
            for (const auto req: conginfo->relatedReqs)
            {
                traffic.reqCongCnt[req].erase(conginfo);
            }

            if (!shouldUpdate && conginfo->relatedReqs.find(nextTop) != conginfo->relatedReqs.end())
            {
                // the rank of nextTop maybe changed
                shouldUpdate = true;
            }
        }

        if (shouldUpdate)
        {
            myReqHeap.clear();
            for (RequestId i = 0; i < traffic.reqNo; i++)
            {
                int newKey = 0;
                for (const auto &cong : traffic.reqCongCnt[i])
                {
                    newKey += cong->relatedReqs.size();
                }

                myReqHeap.update(i, newKey);
            }
            shouldUpdate = false;
        }
    }

    return selectedReqs;
}

vector<RequestId> Routing::selectRerouteReqsCongET2(double frac)
{
    // from congestion site perspective, this one seems better
    int cnt = 0;
    for (const auto &congs: traffic.edgeCongList)
    {
        cnt += congs.second.size();
    }

    vector<CongInfo *> congCnt(cnt, nullptr);

    benchmark2::heapSelectQ<2, int, int> congHeap(cnt);

    for (auto &congs: traffic.edgeCongList)
    {
        for (auto &cong: congs.second)
        {
            congCnt[cong.id] = &cong;
            congHeap.update(cong.id, cong.relatedReqs.size());
        }
    }

    int congId, congkey;
    vector<RequestId> selectedReqs;
    while (selectedReqs.size() < floor(1.0 * traffic.reqNo * frac) && !congHeap.empty())
    {
        congHeap.extract_max(congId, congkey);
        if (congkey == -1)
            continue;

        benchmark2::heapSelectQ<2, int, RequestId> myReqHeap(traffic.reqNo);
        for (const auto &req: congCnt[congId]->relatedReqs)
        {
            int key = 0;
            for (const auto &cong : traffic.reqCongCnt[req])
            {
                key += cong->relatedReqs.size();
            }
            myReqHeap.update(req, key);
        }
        int reqId, reqKey;
        myReqHeap.extract_max(reqId, reqKey);
        selectedReqs.emplace_back(reqId);

        unordered_set<CongInfo *> &reqConInfos = traffic.reqCongCnt[reqId];
        for (auto &reqConInfo: reqConInfos)
        {
            reqConInfo->relatedReqs.erase(reqId);

            if (reqConInfo->relatedReqs.size() > reqConInfo->thresCnt)
            {
                congHeap.update(reqConInfo->id, reqConInfo->relatedReqs.size());
            } else
            {
                // Here, we missed to update reqId's conglist,
                // doing this would incur errors because we are in the loop of its conglist.
                // However, since reqId is removed
                // from all conginfo, we will not select reqId anymore
                congHeap.update(reqConInfo->id, -1);
                for (const auto req: reqConInfo->relatedReqs)
                {
                    traffic.reqCongCnt[req].erase(reqConInfo);
                }
            }
        }

    }

    return selectedReqs;
}
