//
// Created by yehong.xu on 28/12/21.
//

#ifndef TRAFFIC_ASSIGNMENT_TRAFFIC_H
#define TRAFFIC_ASSIGNMENT_TRAFFIC_H

#include "../tools/tools.h"
#include <cmath>

namespace benchmark2 {

#define NULLINDEX 0xFFFFFFFF

    template<int log_k, typename k_t, typename id_t>
    class heapMin {

    public:

        // Expose types.
        typedef k_t key_t;
        typedef id_t node_t;

        // Some constants regarding the elements.
        //static const node_t NULLINDEX = 0xFFFFFFFF;
        static const node_t k = 1 << log_k;

        // A struct defining a heap element.
        struct element_t {
            key_t key;
            node_t element;

            element_t() : key(0), element(0) {}

            element_t(const key_t k, const node_t e) : key(k), element(e) {}
        };


    public:

        // Constructor of the heap.
        explicit heapMin(node_t n) : n(0), max_n(n), elements(n), position(n, NULLINDEX) {
        }

        heapMin() = default;

        // Size of the heap.
        inline node_t size() const {
            return n;
        }

        // Heap empty?
        inline bool empty() const {
            return size() == 0;
        }

        // Extract min element.
        inline void extract_min(node_t &element, key_t &key) {
            assert(!empty());

            element_t &front = elements[0];

            // Assign element and key.
            element = front.element;
            key = front.key;

            // Replace elements[0] by last element.
            position[element] = NULLINDEX;
            --n;
            if (!empty()) {
                front = elements[n];
                position[front.element] = 0;
                shift_down(0);
            }
        }

        inline key_t top() {
            assert(!empty());

            element_t &front = elements[0];

            return front.key;

        }

        inline node_t top_value() {

            assert(!empty());

            element_t &front = elements[0];

            return front.element;
        }

        // Update an element of the heap.
        inline void update(const node_t element, const key_t key) {
            if (position[element] == NULLINDEX) {
                element_t &back = elements[n];
                back.key = key;
                back.element = element;
                position[element] = n;
                shift_up(n++);
            } else {
                node_t el_pos = position[element];
                element_t &el = elements[el_pos];
                if (key > el.key || (key == el.key && element > el.element)) {
                    el.key = key;
                    shift_down(el_pos);
                } else {
                    el.key = key;
                    shift_up(el_pos);
                }
            }
        }


        // Clear the heap.
        inline void clear() {
            for (node_t i = 0; i < n; ++i) {
                position[elements[i].element] = NULLINDEX;
            }
            n = 0;
        }

        // Cheaper clear.
        inline void clear(node_t v) {
            position[v] = NULLINDEX;
        }

        inline void clear_n() {
            n = 0;
        }


        // Test whether an element is contained in the heap.
        inline bool contains(const node_t element) const {
            return position[element] != NULLINDEX;
        }


    protected:

        // Sift up an element.
        inline void shift_up(node_t i) {
            assert(i < n);
            node_t cur_i = i;
            while (cur_i > 0) {
                node_t parent_i = (cur_i - 1) >> log_k;
                if (elements[parent_i].key > elements[cur_i].key ||
                    (elements[parent_i].key == elements[cur_i].key &&
                     elements[parent_i].element > elements[cur_i].element))
                    swap(cur_i, parent_i);
                else
                    break;
                cur_i = parent_i;
            }
        }

        // Sift down an element.
        inline void shift_down(node_t i) {
            assert(i < n);

            while (true) {
                node_t min_ind = i;
                key_t min_key = elements[i].key;
                node_t min_element = elements[i].element;

                node_t child_ind_l = (i << log_k) + 1;
                node_t child_ind_u = std::min(child_ind_l + k, n);

                for (node_t j = child_ind_l; j < child_ind_u; ++j) {
                    if (elements[j].key < min_key ||
                        (elements[j].key == min_key && elements[j].element < min_element)) {
                        min_ind = j;
                        min_key = elements[j].key;
                        min_element = elements[j].element;
                    }
                }

                // Exchange?
                if (min_ind != i) {
                    swap(i, min_ind);
                    i = min_ind;
                } else {
                    break;
                }
            }
        }

        // Swap two elements in the heap.
        inline void swap(const node_t i, const node_t j) {
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

        // Number of elements in the heap.
        node_t n;

        // Number of maximal elements.
        node_t max_n;

        // Array of length heap_elements.
        vector<element_t> elements;

        // An array of positions for all elements.
        vector<node_t> position;
    };


    template<int log_k, typename k_t, typename id_t>
    class heapMax {

    public:

        // Expose types.
        typedef k_t key_t;
        typedef id_t node_t;

        // Some constants regarding the elements.
        //static const node_t NULLINDEX = 0xFFFFFFFF;
        static const node_t k = 1 << log_k;

        // A struct defining a heap element.
        struct element_t {
            key_t key;
            node_t element;

            element_t() : key(0), element(0) {}

            element_t(const key_t k, const node_t e) : key(k), element(e) {}
        };

        // Constructor of the heap.
        explicit heapMax(node_t n) : n(0), max_n(n), elements(n), position(n, NULLINDEX) {}

        heapMax() = default;

        // Size of the heap.
        inline node_t size() const {
            return n;
        }

        // Heap empty?
        [[nodiscard]] inline bool empty() const {
            return size() == 0;
        }

        // Extract max element.
        inline void extract_max(node_t &element, key_t &key) {
            assert(!empty());

            element_t &front = elements[0];

            // Assign element and key.
            element = front.element;
            key = front.key;

            // Replace elements[0] by last element.
            position[element] = NULLINDEX;
            --n;
            if (!empty()) {
                front = elements[n];
                position[front.element] = 0;
                shift_down(0);
            }
        }

        inline node_t top() {
            assert(!empty());

            element_t &front = elements[0];

            return front.element;

        }

        // Update an element of the heap.
        inline void update(const node_t element, const key_t key) {
            if (position[element] == NULLINDEX) {
                element_t &back = elements[n];
                back.key = key;
                back.element = element;
                position[element] = n;
                shift_up(n++);
            } else {
                node_t el_pos = position[element];
                element_t &el = elements[el_pos];
                if (key < el.key) {
                    el.key = key;
                    shift_down(el_pos);
                } else {
                    el.key = key;
                    shift_up(el_pos);
                }
            }
        }


        // Clear the heap.
        inline void clear() {
            for (node_t i = 0; i < n; ++i) {
                position[elements[i].element] = NULLINDEX;
            }
            n = 0;
        }

        // Cheaper clear.
        inline void clear(node_t v) {
            position[v] = NULLINDEX;
        }

        inline void clear_n() {
            n = 0;
        }


        // Test whether an element is contained in the heap.
        inline bool contains(const node_t element) const {
            return position[element] != NULLINDEX;
        }


    protected:

        // Sift up an element.
        inline void shift_up(node_t i) {
            assert(i < n);
            node_t cur_i = i;
            while (cur_i > 0) {
                node_t parent_i = (cur_i - 1) >> log_k;
                if (elements[parent_i].key < elements[cur_i].key)
                    swap(cur_i, parent_i);
                else
                    break;
                cur_i = parent_i;
            }
        }

        // Sift down an element.
        inline void shift_down(node_t i) {
            assert(i < n);

            while (true) {
                node_t min_ind = i;
                key_t min_key = elements[i].key;

                node_t child_ind_l = (i << log_k) + 1;
                node_t child_ind_u = std::min(child_ind_l + k, n);

                for (node_t j = child_ind_l; j < child_ind_u; ++j) {
                    if (elements[j].key > min_key) {
                        min_ind = j;
                        min_key = elements[j].key;
                    }
                }

                // Exchange?
                if (min_ind != i) {
                    swap(i, min_ind);
                    i = min_ind;
                } else {
                    break;
                }
            }
        }

        // Swap two elements in the heap.
        inline void swap(const node_t i, const node_t j) {
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

        // Number of elements in the heap.
        node_t n;

        // Number of maximal elements.
        node_t max_n;

        // Array of length heap_elements.
        vector<element_t> elements;

        // An array of positions for all elements.
        vector<node_t> position;
    };
}

struct CongInfo {
    int id;
    unsigned int t1 = -1, t2 = -1, lastingTime = -1;
    NodeId v1, v2;
    unordered_map<RequestId, unsigned int> relatedReqs;
    double thresCnt;

    CongInfo(int id, NodeId v1, NodeId v2, double thresCnt, int t1, int t2 = -1) {
        this->id = id;
        this->v1 = v1;
        this->v2 = v2;
        this->thresCnt = thresCnt;
        this->t1 = t1;
        this->t2 = t2;
    }
};

struct EdgeProfile {
    int liveFlow = 0, baseCost = -1; // total liveFlow
    vector<int> tempFlow;
    vector<long int> tempWeight;

    EdgeProfile() = default;
};

class TrafficMaintain {
public:
    vector<Request> requestODs; // request's s-t
    RoadNetwork &rN;
    float threshold = 1;
    int reqNo = 1000, timeIntNum = 2, timeReslo = 600, capThres = 20, threadNum = 50;

    vector<vector<Label>> footprints;
    vector<unordered_map<NodeId, EdgeProfile>> trafficStat;
    unordered_map<Edge, list<CongInfo>, hash_edge> edgeCongList;

    unordered_map<int, CongInfo *> congMap;
    vector<Edge> traversedE;
    vector<unordered_map<NodeId, unordered_set<TimeIntIdx>>> affectedHistBins;

    TrafficMaintain(RoadNetwork &rN, vector<Request> &requestMap, int timeIntNum, int timeReslo, int capThres,
                    int threadNum, double threshold);

    TrafficMaintain(RoadNetwork &rN, vector<Request> &requestMap, int threadNum, int capThres);

    void revertTrafficCondition();

    void initialize(NodeId i, NodeId interval);

    void revertTrafficCondition(int begin, int end);

    void calculateTimeDepEdgeWeight();


    void calculateTimeDepEdgeWeight(int begin, int end);

    bool detectCycles();

    void detectCycleRec(NodeId o, unordered_set<NodeId> recS, bool &breakOut, unordered_map<NodeId, bool> &visited,
                        unordered_map<NodeId, unordered_set<NodeId>> &adjList);

    void rangeClearReqCngs(int begin, int end);

    void clearCongs();

    [[nodiscard]] TimeIntIdx getTimeInt(long int time) const;
};

class PathSelection {

public:
    PathSelection() = default;

    static vector<RequestId> selectWaitingT(const TrafficMaintain &traffic, vector<bool> &selected, int rerouteNum);

    static vector<RequestId> selectReqPers(TrafficMaintain &traffic, vector<bool> &selected, int rerouteNum);

    static vector<RequestId> selectConETPers(TrafficMaintain &traffic, vector<bool> &selected, int rerouteNum);

    static vector<RequestId> selectionGreedy(const TrafficMaintain &traffic, vector<bool> &selected, int rerouteNum);

    static vector<RequestId> selectRandomReqs(const TrafficMaintain &traffic, vector<bool> &selected, int rerouteNum);

private:
    static void removeReq(const TrafficMaintain &traffic, vector<bool> &selected, RequestId req,
                          RequestId &reqToReroute, unsigned int &leftCost);

    static void paraRemoveReq(
            const TrafficMaintain &traffic, vector<bool> &selected,
            RequestId begin, RequestId end, RequestId &reqToReroute, unsigned int &leftCost);

    static void selectReq(const TrafficMaintain &traffic, vector<bool> &selected, vector<RequestId> &reqsToReroute,
                          unsigned int &leftCost, RequestId &reqToReroute);


    static unsigned int simulateTraffic(const TrafficMaintain &traffic, vector<bool> &selected, RequestId req);

    static void writeCongestion(TrafficMaintain &traffic);

    static void writeGreedyPath(TrafficMaintain &traffic, vector<RequestId> &toReroute);
};

class Framework {
public:
    Framework() = default;

    static long int costFunc(int baseCost, int edgeFlow, int capacity, int capThres);

    static void allTempDij(TrafficMaintain &traffic, vector<RequestId> &reqs, bool &allGood);

    static void rangeTempDij(TrafficMaintain &traffic, vector<RequestId> &reqs, int begin, int end, bool &allGood);

    static void tempDij(TrafficMaintain &traffic, RequestId requestId);

    static void allDij(TrafficMaintain &traffic, vector<RequestId> &reqs, long int &totalCost);

    static void rangeDij(TrafficMaintain &traffic,
                         vector<RequestId> &reqs, int begin, int end, vector<long int> &threadCosts);

    static int Dij(TrafficMaintain &traffic, RequestId requestId);

    static long int simulateTraffic(TrafficMaintain &traffic, vector<bool> &selected, RequestId req);

    static long int simulateTraffic(TrafficMaintain &traffic);

    static long int simulateTrafficNoCong(TrafficMaintain &traffic);
};


class Batch {

    struct QueryNode {
        // Saturation degree of the vertex: # different colours currently assigned to neighbouring vertices.
        int sat;
        int deg; // Degree in the uncoloured subgraph
        RequestId id; // Index of vertex
    };

    typedef vector<vector<QueryNode>> QueryGraph;

    struct maxSat {
        bool operator()(const QueryNode &lhs, const QueryNode &rhs) const {
            // Compares two nodes by saturation degree, then degree in the subgraph, then vertex id
            return tie(lhs.sat, lhs.deg, lhs.id) > tie(rhs.sat, rhs.deg, rhs.id);
        }
    };

    static vector<vector<RequestId>> DSatur(QueryGraph &queryGraph) {
        int u, i, n = queryGraph.size();
        vector<bool> used(n, false);
        vector<int> c(n), d(n);
        vector<set<int> > adjCols(n);
        set<QueryNode, maxSat> Q;
        set<QueryNode, maxSat>::iterator maxPtr;

        // Initialise the data structures.
        // These are a (binary tree) priority queue, a set of colours adjacent to each uncoloured vertex
        // (initially empty) and the degree d(v) of each uncoloured vertex in the graph induced by uncoloured vertices
        for (u = 0; u < n; u++) {
            c[u] = -1;
            d[u] = n;
            adjCols[u] = set<int>();
            Q.emplace(QueryNode{0, d[u], u});
        }

        int noColor = 0;
        while (!Q.empty()) {

            // Choose the vertex u with highest saturation degree, breaking ties with d.
            // Remove u from the priority queue
            maxPtr = Q.begin();
            u = (*maxPtr).id;
            Q.erase(maxPtr);

            // Identify the lowest feasible colour i for vertex u
            for (const auto &v: queryGraph[u])
                if (c[v.id] != -1)
                    used[v.id] = true;
            for (i = 0; i < used.size(); i++)
                if (!used[i])
                    break; // the first unused color of u's neighbours
            for (const auto &v: queryGraph[u])
                if (c[v.id] != -1)
                    used[v.id] = false; // reset the colors

            // Assign vertex u to colour i
            c[u] = i;
            if (i > noColor)
                noColor = i;

            // Update the saturation degrees and degrees of all uncoloured neighbours;
            // hence modify their corresponding elements in the priority queue
            for (const auto &v: queryGraph[u]) {
                if (c[v.id] == -1) {
                    Q.erase({int(adjCols[v.id].size()), d[v.id], v.id});
                    adjCols[v.id].insert(i);
                    d[v.id]--;
                    Q.emplace(QueryNode{int(adjCols[v.id].size()), d[v.id], v.id});
                }
            }
        }

        vector<RequestId> group;
        vector<vector<RequestId>> groups(noColor, group);

        for (u = 0; u < n; u++) {
            groups[c[u]].emplace_back(u);
        }

        return groups;
    }
};

class MainAlg {
public:
    static void greedyMain(TrafficMaintain &traffic);

    static void batchMain(TrafficMaintain &traffic, float sigma, float tau);

    static void batchMainIter(TrafficMaintain &traffic, float sigma, float tau, float frac);

    static void batchMainIterFix(TrafficMaintain &traffic, float sigma, float tau, float frac);
};

#endif //TRAFFIC_ASSIGNMENT_TRAFFIC_H
