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
#include <random>
#include "../model/Graph.h"

using namespace std;

class Label {
public:
    NodeId node_id;
    int length;
    int lowerBound;
    Label* previous;

    Label(NodeId node_id, double length) {
        this->node_id = node_id;
        this->length = length;
        this->previous = nullptr;
        this->lowerBound = 0;
    };

    Label(NodeId node_id, double length, Label* previous) {
        this->node_id = node_id;
        this->length = length;
        this->previous = previous;
        this->lowerBound = 0;
    };

    Label(NodeId node_id, double length, int lowerBound) {
        this->node_id = node_id;
        this->length = length;
        this->previous = nullptr;
        this->lowerBound = lowerBound;
    };

    Label(NodeId node_id, double length, int lowerBound, Label* previous) {
        this->node_id = node_id;
        this->length = length;
        this->previous = previous;
        this->lowerBound = lowerBound;
    };
};

class MyComparator {
    bool reverse;
public:
    explicit MyComparator(const bool &revparam = false) {
        reverse = revparam;
    }

    bool operator()(const Label *lhs, const Label *rhs) const {
        return (lhs->length > rhs->length);
    }
};

class AstarComparator {
    bool reverse;
public:
    explicit AstarComparator(const bool &revparam = false) {
        reverse = revparam;
    }

    bool operator()(const Label *lhs, const Label *rhs) const {
        return (lhs->lowerBound > rhs->lowerBound);
    }
};

namespace benchmark {

#define NULLINDEX 0xFFFFFFFF

    template<int log_k, typename k_t, typename id_t>
    class heap {

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
        explicit heap(node_t n) : n(0), max_n(n), elements(n), position(n, NULLINDEX) {
        }

        heap()
        = default;

        // Size of the heap.
        inline node_t size() const {
            return n;
        }

        // Heap empty?
        inline bool empty() const {
            return size() == 0;
        }

        // Extract min element.
        inline void extract_min(node_t &label, key_t &key) {
            assert(!empty());

            element_t &front = elements[0];

            // Assign element and key.
            label = front.element;
            key = front.key;

            // Replace elements[0] by last element.
            position[label] = NULLINDEX;
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
                if (key > el.key) {
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
                if (elements[parent_i].key > elements[cur_i].key)
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
                    if (elements[j].key < min_key) {
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


typedef priority_queue<Label *, std::vector<Label *>, MyComparator> PriorityQueue;
typedef priority_queue<Label *, std::vector<Label *>, AstarComparator> PriorityQueueAS;

pair<Path, vector<int>> dijkstra_path_and_bounds(RoadNetwork *rN, NodeId source, NodeId target);

//void tempDij(vector<unordered_map<NodeId, EdgeProfile>> &trafficStat, vector<Label *> &labels,
//                            int timeReslo, int timeIntNum, NodeId source, NodeId target);
//
int dijkstra_dist_del(RoadNetwork *rN, NodeId source, NodeId target);

Path astar_limited(RoadNetwork *rN, NodeId source, NodeId target, vector<int> &bounds,
                   unordered_set<Edge, boost::hash<Edge>> &deletedEdges);

bool randomBool(int trialNum, double sucRateEachTrial);

#endif //TRAFFIC_ASSIGNMENT_TOOLS_H
