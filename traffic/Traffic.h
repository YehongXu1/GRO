//
// Created by yehong.xu on 28/12/21.
//

#ifndef TRAFFIC_ASSIGNMENT_TRAFFIC_H
#define TRAFFIC_ASSIGNMENT_TRAFFIC_H

#include "../tools/tools.h"
#include <cmath>

namespace benchmark
{

#define NULLINDEX 0xFFFFFFFF

    template<int log_k, typename k_t, typename id_t>
    class heap
    {

    public:

        // Expose types.
        typedef k_t key_t;
        typedef id_t node_t;

        // Some constants regarding the elements.
        //static const node_t NULLINDEX = 0xFFFFFFFF;
        static const node_t k = 1 << log_k;

        // A struct defining a heapCust element.
        struct element_t
        {
            key_t key;
            node_t element;

            element_t() : key(0), element(0) {}

            element_t(const key_t k, const node_t e) : key(k), element(e) {}
        };


    public:

        // Constructor of the heapCust.
        explicit heap(node_t n) : n(0), max_n(n), elements(n), position(n, NULLINDEX)
        {
        }

        heap() = default;

        inline void resize(node_t n)
        {
            max_n = n;
            elements.reserve(n);
            position.assign(n, NULLINDEX);
        }

        // Size of the heapCust.
        inline node_t size() const
        {
            return n;
        }

        // Heap empty?
        [[nodiscard]] inline bool empty() const
        {
            return size() == 0;
        }

        // Extract min element.
        inline void extract_max(node_t &element, key_t &key)
        {
            assert(!empty());

            element_t &front = elements[0];

            // Assign element and key.
            element = front.element;
            key = front.key;

            // Replace elements[0] by last element.
            position[element] = NULLINDEX;
            --n;
            if (!empty())
            {
                front = elements[n];
                position[front.element] = 0;
                shift_down(0);
            }
        }

        inline key_t top()
        {
            assert(!empty());

            element_t &front = elements[0];

            return front.key;

        }

        inline node_t top_value()
        {

            assert(!empty());

            element_t &front = elements[0];

            return front.element;
        }

        // Update an element of the heap.
        inline void update(const node_t element, const key_t key)
        {
            if (position[element] == NULLINDEX)
            {
                element_t &back = elements[n];
                back.key = key;
                back.element = element;
                position[element] = n;
                shift_up(n++);
            } else
            {
                node_t el_pos = position[element];
                element_t &el = elements[el_pos];
                if (key < el.key)
                {
                    el.key = key;
                    shift_down(el_pos);
                } else
                {
                    el.key = key;
                    shift_up(el_pos);
                }
            }
        }


        // Clear the heapCust.
        inline void clear()
        {
            for (node_t i = 0; i < n; ++i)
            {
                position[elements[i].element] = NULLINDEX;
            }
            n = 0;
        }

        // Cheaper clear.
        inline void clear(node_t v)
        {
            position[v] = NULLINDEX;
        }

        inline void clear_n()
        {
            n = 0;
        }


        // Test whether an element is contained in the heapCust.
        [[nodiscard]] inline bool contains(const node_t element) const
        {
            return position[element] != NULLINDEX;
        }


    protected:

        // Sift up an element.
        inline void shift_up(node_t i)
        {
            assert(i < n);
            node_t cur_i = i;
            while (cur_i > 0)
            {
                node_t parent_i = (cur_i - 1) >> log_k;
                if (elements[parent_i].key < elements[cur_i].key)
                    swap(cur_i, parent_i);
                else
                    break;
                cur_i = parent_i;
            }
        }

        // Sift down an element at position i.
        inline void shift_down(node_t i)
        {
            assert(i < n);

            while (true)
            {
                node_t max_ind = i;
                key_t max_key = elements[i].key;

                node_t child_ind_l = (i << log_k) + 1;
                node_t child_ind_u = std::min(child_ind_l + k, n);

                for (node_t j = child_ind_l; j < child_ind_u; ++j)
                {
                    if (elements[j].key > max_key)
                    {
                        max_ind = j;
                        max_key = elements[j].key;
                    }
                }

                // Exchange?
                if (max_ind != i)
                {
                    swap(i, max_ind);
                    i = max_ind;
                } else
                {
                    break;
                }
            }
        }

        // Swap two elements in the heapCust.
        inline void swap(const node_t i, const node_t j)
        {
            element_t &el_i = elements[i];
            element_t &el_j = elements[j];

            // Exchange positions
            position[el_i.element] = j;
            position[el_j.element] = i;

            // Exchange elements
            element_t temp = el_i;
            el_i = el_j;
            el_j = temp;
        }


    private:

        // Number of elements in the heapCust.
        node_t n{};

        // Number of maximal elements.
        node_t max_n{};

        // Array of length heap_elements.
        vector<element_t> elements;

        // An array of positions for all elements.
        vector<node_t> position;
    };
}

typedef pair<int, int> Interval;
typedef int RequestId;

class Traffic
{
public:
    unordered_map<RequestId, pair<NodeId, NodeId>> requestIdMap; // request's s-t
    RoadNetwork &rN;
    int capacity{};

    Traffic(RoadNetwork &rN, Coord &sCenter, Coord &tCenter, int r1, int r2, int capacity) : rN(rN)
    {
        this->capacity = capacity;
        unordered_map<NodeId, int> endPointKeyMap;

        vector<NodeId> sources = getNodesInR(sCenter, r1);
        vector<NodeId> targets = getNodesInR(tCenter, r1);
        for (int i = 0; i < sources.size(); i++)
        {
            for (int j = 0; j < targets.size(); j++)
            {
                RequestId reqId = targets.size() * i + j;
                requestIdMap[reqId] = make_pair(sources[i], targets[j]);
            }
        }
    }

    Traffic(RoadNetwork &rN, string &OdPath, int capacity) : rN(rN)
    {
        int j = 100; // control number of requests
        this->capacity = capacity;
        RequestId id = 0;
        NodeId origin, dest;
        ifstream infile(OdPath);

        while (infile >> origin >> dest)
        {
            requestIdMap[id] = make_pair(origin, dest);
            id += 1;
            if (id == j)
                break;
        }
        cout << requestIdMap.size();
        infile.close();
    }


    void writeSetting(const basic_string<char> &path) const;

private:
    vector<NodeId> getNodesInR(Coord &center, int r);

};

struct EdgeFlowInfo
{
    set<RequestId> requests;
    int totalFlow = 0, weight = -1, maxTempFlow = INT_MIN, tempWeight = INT_MAX; // total totalFlow
    vector<int> tempFlow;

    EdgeFlowInfo()
    = default;
};

class Simulation
{

public:
    explicit Simulation(Traffic &traffic, int timeInts, int timeResl) : traffic(traffic)
    {
        this->timeInts = timeInts;
        this->timeResl = timeResl;

        EdgeList::iterator iterAdj;
        trafficStat.reserve(traffic.rN.numNodes + 1);
        for (NodeId i = 0; i <= traffic.rN.numNodes; i++)
        {
            for (iterAdj = traffic.rN.adjListOut[i].begin();
                 iterAdj != traffic.rN.adjListOut[i].end(); iterAdj++)
            {
                trafficStat[i][iterAdj->first].tempFlow.resize(timeInts, 0);
                trafficStat[i][iterAdj->first].weight = iterAdj->second;
            }
        }

        trajectories.reserve(traffic.requestIdMap.size());
    }

    int reroutePartialReqsByBlocking();

    int onePass();

    int basicSimulation();

    void writeTrajectories(const basic_string<char> &path);

    void writeEdgeFlowDist(const basic_string<char> &path);

    int getCost();

private:
    Traffic &traffic;
    vector<Label *> trajectories;
    vector<unordered_map<NodeId, EdgeFlowInfo>> trafficStat;
    set<Edge> traversedEdges;

    int overflowEdgeCnt = 0, timeInts = 1000, timeResl = 100;

    int trajCostFun1(Edge &edge, int edgeFlow);

    set<Edge> heuBasedSimulation(vector<RequestId> &request);

    void blockOverflowEdges(set<Edge> &newOverflowEdges);

    // return edges become overflow
    set<Edge> addTrajectoryInRN(RequestId request);

    // return edges become underflow
    set<Edge> delTrajectoryInRN(RequestId request);

    void updateTrafficStat();

    vector<RequestId> reqsByCollCnt(vector<int> &reqOverflowEdge, benchmark::heap<2, int, RequestId> &requestHeap);

    void clearTraffic();

    void writeCollision(const basic_string<char> &path, vector<Edge> &overflowEdges);

    void updateEdgeCost();
};

#endif //TRAFFIC_ASSIGNMENT_TRAFFIC_H
