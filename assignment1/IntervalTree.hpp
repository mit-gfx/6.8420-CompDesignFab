#pragma once
#include <vector>

namespace data_structure {
    template <typename T>
    class IntervalEntry {
    public:
        IntervalEntry() {}
        IntervalEntry(T _l, T _r, int _id) 
            :l(_l), r(_r), id(_id) {}

        T l, r;
        int id;
    };

    template <typename T>
    class IntervalTree {
    
    public:
        IntervalTree() : eps(1e-5) {
            _lower_bound = 0.0;
            _upper_bound = 1.0;
            _left_child.clear();
            _right_child.clear();
            _num_nodes = 1;
            _tree_sort_left.clear();
            _tree_sort_right.clear();
        }

        IntervalTree(T lower_bound, T upper_bound) : eps(1e-5) {
            _lower_bound = lower_bound;
            _upper_bound = upper_bound;
            _left_child.clear();
            _right_child.clear();
            _num_nodes = 1;
            _tree_sort_left.clear();
            _tree_sort_right.clear();
        }

        T lower_bound() { return _lower_bound; }
        T upper_bound() { return _upper_bound; }

        // TODO: HW1
        // part 3.1: accelerated slicing algorithm
        // Hints:
        // this function should be called only once at the beginning of the algorithm.
        // It will build the interval tree for [bound_l, bound_r] with all given intervals.
        // Hint: 
        //     1. this build should do recursively.
        //     2. you can add any argument you need to this function and to the class
        void build(T bound_l, T bound_r, std::vector<IntervalEntry<T>> &intervals) {

        }

        // TODO: HW1
        // part 3.1: accelerated slicing algorithm
        // Hints:
        // this function returns a list of intervals that overlap a given point. 
        // Hint: 
        //     1. this query should do recursively.
        //     2. you can add any argument you need to this function and to the class
        void query(T query_point, std::vector<IntervalEntry<T>>& results) {
        }

    private:
        /* Given */
        T _lower_bound, _upper_bound;   // The interval represented by the whole tree
        std::vector<int> _left_child;   // each node in tree has a unique id from 0, _left_child[i] is the id for node i's left child, -1 if node i doesn't have left child.
        std::vector<int> _right_child;  // same as _left_child but for right child.
        int _num_nodes;                 // number of the nodes in interval tree
        T eps;                          // just for geometry boundary check, you can modify its value depending on your implementation

        std::vector<std::vector<IntervalEntry<T>>> _tree_sort_left, _tree_sort_right;
    };
}