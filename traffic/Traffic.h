//
// Created by yehong.xu on 28/12/21.
//

#ifndef TRAFFIC_ASSIGNMENT_TRAFFIC_H
#define TRAFFIC_ASSIGNMENT_TRAFFIC_H

#include "../tools/tools.h"
#include <cmath>

namespace benchmark2
{

#define NULLINDEX 0xFFFFFFFF

    template<int log_k>
    class heapEval
    {

    public:

        // Expose types.

        // Some constants regarding the elements.
        //static const node_t NULLINDEX = 0xFFFFFFFF;
        static const int k = 1 << log_k;

        // A struct defining a heap element.
        struct element_t
        {
            RequestId req;
            Label *label{};

            element_t() : req(-1), label() {}

            element_t(RequestId req, Label *label) : req(req), label(label) {}
        };

        // Constructor of the heap.
        explicit heapEval(int n) : n(0), max_n(n), elements(n), position(n, NULLINDEX)
        {
        }

        heapEval() = default;

        // Size of the heap.
        [[nodiscard]] inline int size() const
        {
            return n;
        }

        // Heap empty?
        [[nodiscard]] inline bool empty() const
        {
            return size() == 0;
        }

        // Extract min element.
        inline void extract_min(Label *&inputLabel, RequestId &req)
        {
            assert(!empty());

            element_t &front = elements[0];

            // Assign element and key.
            inputLabel = front.label;
            req = front.req;

            // Replace elements[0] by last element.
            position[req] = NULLINDEX;
            --n;
            if (!empty())
            {
                front = elements[n];
                position[front.req] = 0;
                shift_down(0);
            }
        }

        inline void top(Label *&inputLabel, RequestId &req)
        {
            assert(!empty());

            inputLabel = elements[0].label;
            req = elements[0].req;
        }

        // Update an element of the heap.
        inline void update(Label *label, const RequestId req)
        {
            if (position[req] == NULLINDEX)
            {
                element_t &back = elements[n];
                back.label = label;
                back.req = req;
                position[req] = n;
                shift_up(n++);
            } else
            {
                int el_pos = position[req];
                element_t &el = elements[el_pos];
                if (label->length > el.label->length)
                {
                    el.label = label;
                    shift_down(el_pos);
                } else if (label->length < el.label->length)
                {
                    el.label = label;
                    shift_up(el_pos);
                } else
                {
                    if (req > el.req)
                    {
                        el.label = label;
                        shift_down(el_pos);
                    } else
                    {
                        el.label = label;
                        shift_up(el_pos);
                    }
                }
            }
        }

        // Array of length heap_elements.
        vector<element_t> elements;
    protected:

        // Sift up an element.
        inline void shift_up(int i)
        {
            assert(i < n);
            int cur_i = i;
            while (cur_i > 0)
            {
                int parent_i = (cur_i - 1) >> log_k;
                if (elements[parent_i].label->length > elements[cur_i].label->length)
                    swap(cur_i, parent_i);
                else if (elements[parent_i].label->length < elements[cur_i].label->length)
                    break;
                else
                {
                    if (elements[parent_i].req > elements[cur_i].req)
                        swap(cur_i, parent_i);
                    else
                        break;
                }
                cur_i = parent_i;
            }
        }

        // Sift down an element.
        inline void shift_down(int i)
        {
            assert(i < n);

            while (true)
            {
                int min_ind = i;
                key_t min_key = elements[i].label->length;
                RequestId min_req = elements[i].req;

                int child_ind_l = (i << log_k) + 1;
                int child_ind_u = std::min(child_ind_l + k, n);

                for (int j = child_ind_l; j < child_ind_u; ++j)
                {
                    if (elements[j].label->length < min_key)
                    {
                        min_ind = j;
                        min_key = elements[j].label->length;
                        min_req = elements[j].req;
                    } else if (elements[j].label->length == min_key)
                    {
                        if (elements[j].req < min_req)
                        {
                            min_ind = j;
                            min_req = elements[j].req;
//                            min_key = elements[j].label->length;
                        }
                    }
                }

                // Exchange?
                if (min_ind != i)
                {
                    swap(i, min_ind);
                    i = min_ind;
                } else
                {
                    break;
                }
            }
        }

        // Swap two elements in the heap.
        inline void swap(const int i, const int j)
        {
            element_t &el_i = elements[i];
            element_t &el_j = elements[j];

            // Exchange positions
            position[el_i.req] = j;
            position[el_j.req] = i;

            // Exchange elements
            element_t temp = el_i;
            el_i = el_j;
            el_j = temp;
        }


    private:

        // Number of elements in the heap.
        int n{};

        // Number of maximal elements.
        int max_n{};

        // An array of positions for all elements.
        vector<int> position;
    };

    template<int log_k, typename k_t, typename id_t>
    class heapSelectQ
    {

    public:

        // Expose types.
        typedef k_t key_t;
        typedef id_t node_t;

        // Some constants regarding the elements.
        //static const node_t NULLINDEX = 0xFFFFFFFF;
        static const node_t k = 1 << log_k;

        // A struct defining a heap element.
        struct element_t
        {
            key_t key;
            node_t element;

            element_t() : key(0), element(0) {}

            element_t(const key_t k, const node_t e) : key(k), element(e) {}
        };

        // Constructor of the heap.
        explicit heapSelectQ(node_t n) : n(0), max_n(n), elements(n), position(n, NULLINDEX) {}

        heapSelectQ() = default;

        // Size of the heap.
        inline node_t size() const
        {
            return n;
        }

        // Heap empty?
        [[nodiscard]] inline bool empty() const
        {
            return size() == 0;
        }

        // Extract max element.
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

        inline node_t top()
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


        // Clear the heap.
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


        // Test whether an element is contained in the heap.
        inline bool contains(const node_t element) const
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

        // Sift down an element.
        inline void shift_down(node_t i)
        {
            assert(i < n);

            while (true)
            {
                node_t min_ind = i;
                key_t min_key = elements[i].key;

                node_t child_ind_l = (i << log_k) + 1;
                node_t child_ind_u = std::min(child_ind_l + k, n);

                for (node_t j = child_ind_l; j < child_ind_u; ++j)
                {
                    if (elements[j].key > min_key)
                    {
                        min_ind = j;
                        min_key = elements[j].key;
                    }
                }

                // Exchange?
                if (min_ind != i)
                {
                    swap(i, min_ind);
                    i = min_ind;
                } else
                {
                    break;
                }
            }
        }

        // Swap two elements in the heap.
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
struct ET
{
    NodeId v1, v2;
    TimeIntIdx i;

    ET(NodeId v1, NodeId v2, TimeIntIdx i)
    {
        this->v1 = v1;
        this->v2 = v2;
        this->i = i;
    }
};

struct Request
{
    RequestId id;
    NodeId o, d;
    int stuckTime = 0, stuckO = -1, stuckD = -1, preStatyAt = -1;
    int sortBy = 0, departT = 0;

    Request(RequestId i, NodeId o, NodeId d, int departT)
    {
        this->id = i;
        this->o = o;
        this->d = d;
        this->departT = departT;
    }
};

struct CongInfo
{
    int id, t1 = -1, t2 = -1;
    NodeId v1, v2;
    unordered_set<RequestId> relatedReqs;
    double thresCnt;

    CongInfo(int id, NodeId v1, NodeId v2, double thresCnt, int t1, int t2 = -1)
    {
        this->id = id;
        this->v1 = v1;
        this->v2 = v2;
        this->thresCnt = thresCnt;
        this->t1 = t1;
        this->t2 = t2;
    }
};

class TrafficMaintain
{
public:
    vector<Request> requestODs; // request's s-t
    RoadNetwork &rN;
    double threshold;
    int reqNo = 1000, timeIntNum = 1000, timeReslo = 100, penalR = 10, threadNum = 50;

    vector<Label *> trajectories;
    vector<vector<Label *>> labelCollection;
    vector<unordered_map<NodeId, EdgeProfile>> trafficStat;
    unordered_map<Edge, list<CongInfo>, hash_edge> edgeCongList; //
    vector<unordered_set<CongInfo *>> reqCongCnt;
    unordered_map<Edge, unordered_set<TimeIntIdx>, hash_edge> traversedEdges;
    unordered_map<Edge, vector<vector<RequestId>>, hash_edge> stuckODs;

    TrafficMaintain(RoadNetwork &rN, vector<Request> &requestMap, int timeIntNum, int timeReslo, int penalR, int threadNum, double threshold);

    void allTempDij(vector<RequestId> &reqs);

    void writeSetting(const basic_string<char> &path, const basic_string<char> &rNName) const;

    void delTrajectoryInRN(RequestId request, vector<ET> &newUnderflowEdges);

    void clearEdgeProfile();

    [[nodiscard]] int costFunc(int baseCost, int edgeFlow, int capacity) const;

    void tempDij(RequestId requestId);

    void writeTrajectories(const basic_string<char> &path);

    void writeCollision(const basic_string<char> &path, vector<Edge> &overflowEdges);

    int simulateTraffic();

    void initialize(int i, int interval);

    void rangeTempDij(vector<RequestId> &reqs, int begin, int end);

    void rangeClearEdgeProfile(int begin, int end);

    void clearAnEdgeProfile(NodeId v1, NodeId v2);

    void updateETProfiles(vector<Edge> &edge);

    void updateRangeETProfiles(vector<Edge> &edge, int begin, int end);

    void updateHeuWeight(vector<Edge> &record);

    void updateTrafficLoading();

    void updateTraversedET();

    void updateWeight(vector<Edge> &record, int begin, int end);

    void rangeDeleteLabels(vector<RequestId> &reqs, int begin, int end);

    void deleteLabels(vector<RequestId> &reqs);

    bool detectCycles();

    void detectCycleRec(NodeId o, unordered_set<NodeId> recS, bool &breakOut, unordered_map<NodeId, bool> &visited,
                        unordered_map<NodeId, unordered_set<NodeId>> &adjList);

    void rangeClearReqCngs(int begin, int end);

    void clearReqCngs();
};

class PathSelection
{

public:
    PathSelection() = default;

    static vector<RequestId> selectRerouteReqsUnfixPaths(TrafficMaintain &traffic, double frac);

    static vector<RequestId> selectRerouteReqsCongET(TrafficMaintain &traffic, double frac);

    static vector<RequestId> selectRerouteReqsCongET2(TrafficMaintain &traffic, double frac);

    static vector<RequestId> selectRerouteReqsFixPaths(TrafficMaintain &traffic, vector<int> &reqs, double frac);
};

#endif //TRAFFIC_ASSIGNMENT_TRAFFIC_H
