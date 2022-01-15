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
#include <boost/functional/hash.hpp>

#include <unordered_map>
#include <boost/random.hpp>
#include <boost/nondet_random.hpp>
#include "../model/graph.h"

using namespace std;

class Label
{
public:
    NodeId node_id;
    int length;
    int lowerBound;
    Label *previous;

    Label()
    {
        this->node_id = -1;
        this->length = -1;
        this->previous = nullptr;
        this->lowerBound = -1;
    }

    explicit Label(Label *label)
    {
        // copy
        this->node_id = label->node_id;
        this->length = label->length;
        this->previous = label->previous;
        this->lowerBound = label->lowerBound;
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
    class heapCust
    {

    public:
        // Some constants regarding the elements.
        //static const node_t NULLINDEX = 0xFFFFFFFF;
        static const int k = 1 << log_k;


    public:

        // Constructor of the heapCust.
        explicit heapCust(int n) : n(0), max_n(n), position(n, NULLINDEX), elements(n)
        {

        }

        heapCust() = default;

        // Size of the heapCust.
        [[nodiscard]] inline int size() const
        {
            return n;
        }

        // Heap empty?
        [[nodiscard]] inline bool empty() const
        {
            return size() == 0;
        }

        // Extract min node_id.
        inline void extract_min(Label *label)
        {
            assert(!empty());

            Label &front = elements[0];

            // Assign element and length.
            label->length = front.length;
            label->node_id = front.node_id;
            label->previous = front.previous;

            // Replace elements[0] by last element.
            position[front.node_id] = NULLINDEX;
            --n;
            if (!empty())
            {
                front = elements[n];
                position[front.node_id] = 0;
                shift_down(0);
            }
        }

        inline key_t top()
        {
            assert(!empty());

            Label &front = elements[0];

            return front.length;

        }

        inline NodeId top_value()
        {

            assert(!empty());

            Label &front = elements[0];

            return front.node_id;
        }

        // Update an element of the heapCust.
        inline void update(Label *label)
        {
            const NodeId element = label->node_id;
            const int key = label->length;
            if (position[element] == NULLINDEX)
            {
                Label &back = elements[n];
                back.length = key;
                back.node_id = element;
                back.previous = label->previous;
                position[element] = n;
                shift_up(n++);
            } else
            {
                int el_pos = position[element];
                Label &el = elements[el_pos];
                el.length = key;
                el.previous = label->previous;

                if (key > el.length)
                    shift_down(el_pos);
                else
                    shift_up(el_pos);
            }
        }


        // Clear the heapCust.
        inline void clear()
        {
            for (NodeId i = 0; i < n; ++i)
            {
                position[elements[i].node_id] = NULLINDEX;
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


        // Test whether an node_id is contained in the heapCust.
        [[nodiscard]] inline bool contains(const NodeId node_id) const
        {
            return position[node_id] != NULLINDEX;
        }


    protected:

        // Shift up an element.
        inline void shift_up(int i)
        {
            assert(i < n);
            int cur_i = i;
            while (cur_i > 0)
            {
                int parent_i = (cur_i - 1) >> log_k;
                if (elements[parent_i].length > elements[cur_i].length)
                    swap(cur_i, parent_i);
                else
                    break;
                cur_i = parent_i;
            }
        }

        // Shift down an element.
        inline void shift_down(int i)
        {
            assert(i < n);

            while (true)
            {
                int min_ind = i;
                int min_key = elements[i].length;

                int child_ind_l = (i << log_k) + 1;
                NodeId child_ind_u = std::min(child_ind_l + k, n);

                for (int j = child_ind_l; j < child_ind_u; ++j)
                {
                    if (elements[j].length < min_key)
                    {
                        min_ind = j;
                        min_key = elements[j].length;
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

        // Swap two elements in the heapCust.
        inline void swap(const int i, const int j)
        {
            Label &el_i = elements[i];
            Label &el_j = elements[j];

            // Exchange positions
            position[el_i.node_id] = j;
            position[el_j.node_id] = i;

            // Exchange elements
            Label temp = el_i;
            el_i = el_j;
            el_j = temp;
        }


    private:

        // Number of elements in the heapCust.
        int n{};

        // Number of maximal elements.
        int max_n{};

        // Array of element heap_elements.
        vector<Label> elements;

        // An array of positions for all elements.
        vector<int> position;
    };
}


typedef priority_queue<Label *, std::vector<Label *>, MyComparator> PriorityQueue;
typedef priority_queue<Label *, std::vector<Label *>, AstarComparator> PriorityQueueAS;

pair<Path, vector<int>> dijkstra_path_and_bounds(RoadNetwork *rN, NodeId source, NodeId target);

void dijkstra_label(RoadNetwork *rN, NodeId source, NodeId target, Label *curLabel);

void dijkstra_label_timedep(vector<unordered_map<NodeId, EdgeFlowInfo>> &trafficStat, int timeReslo, int timeIntNum,
                            NodeId source, NodeId target, Label *curLabel);

int dijkstra_dist_del(RoadNetwork *rN, NodeId source, NodeId target);

Path astar_limited(RoadNetwork *rN, NodeId source, NodeId target, vector<int> &bounds,
                   unordered_set<Edge, boost::hash<Edge>> &deletedEdges);

bool randomBool(int trialNum, double sucRateEachTrial);
#endif //TRAFFIC_ASSIGNMENT_TOOLS_H
