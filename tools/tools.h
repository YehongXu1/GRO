//
// Created by yehong.xu on 26/12/21.
//

#ifndef TRAFFIC_ASSIGNMENT_TOOLS_H
#define TRAFFIC_ASSIGNMENT_TOOLS_H
//
// Created by yehong.xu on 11/12/21.
//

#include <queue>
#include <vector>
#include <algorithm>
#include <unordered_set>
#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <boost/random.hpp>
#include <boost/nondet_random.hpp>
#include <boost/shared_ptr.hpp>
#include "../model/graph.h"

using namespace std;

class Label
{
public:
    NodeId node_id;
    long long int length = 0;
    long long int heuLength = 0;
    int lowerBound;
    Label *previous;
    Label *nextL = nullptr;

    Label()
    {
        this->node_id = -1;
        this->length = -1;
        this->heuLength = -1;
        this->previous = nullptr;
        this->nextL = nullptr;
        this->lowerBound = -1;
    }

    explicit Label(Label *label)
    {
        // copy
        this->node_id = label->node_id;
        this->length = label->length;
        this->heuLength = label->heuLength;
        this->previous = label->previous;
        this->lowerBound = label->lowerBound;
        this->nextL = label->nextL;
    }

    Label(NodeId node_id, int length)
    {
        this->node_id = node_id;
        this->length = length;
        this->previous = nullptr;
        this->lowerBound = 0;
    };

    Label(NodeId node_id, int length, Label *previous)
    {
        this->node_id = node_id;
        this->length = length;
        this->previous = previous;
        this->lowerBound = 0;
    };

    Label(NodeId node_id, int length, int lowerBound)
    {
        this->node_id = node_id;
        this->length = length;
        this->previous = nullptr;
        this->lowerBound = lowerBound;
    };

    Label(NodeId node_id, int length, int lowerBound, Label *previous)
    {
        this->node_id = node_id;
        this->length = length;
        this->previous = previous;
        this->lowerBound = lowerBound;
    };

    void copy(Label *label)
    {
        this->node_id = label->node_id;
        this->length = label->length;
        this->heuLength = label->heuLength;
        this->nextL = label->nextL;
        this->previous = label->previous;
        this->lowerBound = label->lowerBound;
    }
};

class MyComparator
{
    bool reverse;
public:
    explicit MyComparator(const bool &revparam = false)
    {
        reverse = revparam;
    }

    bool operator()(const Label *lhs, const Label *rhs) const
    {
        return (lhs->length > rhs->length);
    }
};

class AstarComparator
{
    bool reverse;
public:
    explicit AstarComparator(const bool &revparam = false)
    {
        reverse = revparam;
    }

    bool operator()(const Label *lhs, const Label *rhs) const
    {
        return (lhs->lowerBound > rhs->lowerBound);
    }
};

namespace benchmark
{

#define NULLINDEX 0xFFFFFFFF

    template<int log_k>
    class heapDij
    {

    public:

        // Expose types.

        // Some constants regarding the elements.
        //static const node_t NULLINDEX = 0xFFFFFFFF;
        static const int k = 1 << log_k;

        // A struct defining a heap element.
        struct element_t
        {
            Label* label{};

            element_t() : label() {}

            explicit element_t(Label* label) : label(label){}
        };

        // Constructor of the heap.
        explicit heapDij(int n) : n(0), max_n(n), elements(n), position(n, NULLINDEX)
        {
        }

        heapDij() = default;

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
        inline void extract_min(Label * &inputLabel)
        {
            assert(!empty());

            element_t &front = elements[0];

            // Assign element and key.
            inputLabel = front.label;

            // Replace elements[0] by last element.
            position[front.label->node_id] = NULLINDEX;
            --n;
            if (!empty())
            {
                front = elements[n];
                position[front.label->node_id] = 0;
                shift_down(0);
            }
        }

        inline Label* top()
        {
            assert(!empty());

            element_t &front = elements[0];

            return front.label;

        }

        inline int top_value()
        {

            assert(!empty());

            element_t &front = elements[0];

            return front.label->heuLength;
        }

        // Update an element of the heap.
        inline void update(Label *label)
        {
            if (position[label->node_id] == NULLINDEX)
            {
                element_t &back = elements[n];
                back.label = label;
                position[label->node_id] = n;
                shift_up(n++);
            } else
            {
                int el_pos = position[label->node_id];
                element_t &el = elements[el_pos];
                if (label->heuLength > el.label->heuLength)
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


        // Clear the heap.
        inline void clear()
        {
            for (int i = 0; i < n; ++i)
            {
                position[elements[i].label->node_id] = NULLINDEX;
            }
            n = 0;
        }

        // Cheaper clear.
        inline void clear(NodeId v)
        {
            position[v] = NULLINDEX;
        }

        inline void clear_n()
        {
            n = 0;
        }

        // Test whether an element is contained in the heap.
        [[nodiscard]] inline bool contains(const NodeId v) const
        {
            return position[v] != NULLINDEX;
        }


    protected:

        // Sift up an element.
        inline void shift_up(int i)
        {
            assert(i < n);
            int cur_i = i;
            while (cur_i > 0)
            {
                int parent_i = (cur_i - 1) >> log_k;
                if (elements[parent_i].label->heuLength > elements[cur_i].label->heuLength)
                    swap(cur_i, parent_i);
                else
                    break;
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
                key_t min_key = elements[i].label->heuLength;

                unsigned int child_ind_l = (i << log_k) + 1;
                unsigned int child_ind_u = std::min(child_ind_l + k, n);

                for (int j = child_ind_l; j < child_ind_u; ++j)
                {
                    if (elements[j].label->heuLength < min_key)
                    {
                        min_ind = j;
                        min_key = elements[j].label->heuLength;
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
        inline void swap(const  int i, const int j)
        {
            element_t &el_i = elements[i];
            element_t &el_j = elements[j];

            // Exchange positions
            position[el_i.label->node_id] = j;
            position[el_j.label->node_id] = i;

            // Exchange elements
            element_t temp = el_i;
            el_i = el_j;
            el_j = temp;
        }


    private:

        // Number of elements in the heap.
        unsigned int n{};

        // Number of maximal elements.
        unsigned int max_n{};

        // Array of length heap_elements.
        vector<element_t> elements;

        // An array of positions for all elements.
        vector<int> position;
    };
}


typedef priority_queue<Label *, std::vector<Label *>, MyComparator> PriorityQueue;
typedef priority_queue<Label *, std::vector<Label *>, AstarComparator> PriorityQueueAS;

pair<Path, vector<int>> dijkstra_path_and_bounds(RoadNetwork *rN, NodeId source, NodeId target);

vector<Label *> dijkstra_label_timedep(
        vector<unordered_map<NodeId, EdgeFlowInfo>> &trafficStat, int timeReslo,
        int timeIntNum, NodeId source, NodeId target);

int dijkstra_dist_del(RoadNetwork *rN, NodeId source, NodeId target);

Path astar_limited(RoadNetwork *rN, NodeId source, NodeId target, vector<int> &bounds,
                   unordered_set<Edge, boost::hash<Edge>> &deletedEdges);

bool randomBool(int trialNum, double sucRateEachTrial);

#endif //TRAFFIC_ASSIGNMENT_TOOLS_H
