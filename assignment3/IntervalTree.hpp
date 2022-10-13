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

        void build(int id, T bound_l, T bound_r, std::vector<IntervalEntry<T>> &intervals) {

            _tree_sort_left.push_back(std::vector<IntervalEntry<T>>());
            _tree_sort_right.push_back(std::vector<IntervalEntry<T>>());
            _left_child.push_back(-1);
            _right_child.push_back(-1);

            T mid_axis = (bound_l + bound_r) / 2.0;

            std::vector<IntervalEntry<T>> left_intervals;
            std::vector<IntervalEntry<T>> right_intervals;
            left_intervals.clear();
            right_intervals.clear();

            for (int i = 0;i < intervals.size();++i) {
                if (intervals[i].r < mid_axis - eps) {
                    left_intervals.push_back(intervals[i]);
                } else if (intervals[i].l > mid_axis + eps) {
                    right_intervals.push_back(intervals[i]);
                } else {
                    _tree_sort_left[id].push_back(intervals[i]);
                    _tree_sort_right[id].push_back(intervals[i]);
                }
            }

            std::sort(_tree_sort_left[id].begin(), _tree_sort_left[id].end(), 
                [](const IntervalEntry<T> &A, const IntervalEntry<T> &B) -> bool {
                    return A.l < B.l - 1e-5;
                });
            
            std::sort(_tree_sort_right[id].begin(), _tree_sort_right[id].end(), 
                [](const IntervalEntry<T> &A, const IntervalEntry<T> &B) -> bool {
                    return A.r > B.r + 1e-5;
                });

            if (left_intervals.size() > 0) {
                _left_child[id] = _num_nodes ++;
                build(_left_child[id], bound_l, mid_axis, left_intervals);
            }

            if (right_intervals.size() > 0) {
                _right_child[id] = _num_nodes ++;
                build(_right_child[id], mid_axis, bound_r, right_intervals);
            }
        }

        void query(int id, T bound_l, T bound_r, T query_point, std::vector<IntervalEntry<T>>& results) {
            T mid_axis = (bound_l + bound_r) / 2.0;
            if (query_point < mid_axis) {
                if (_left_child[id] != -1)
                    query(_left_child[id], bound_l, mid_axis, query_point, results);
                for (int i = 0;i < _tree_sort_left[id].size();++i) {
                    if (_tree_sort_left[id][i].l > query_point + eps)
                        break;
                    results.push_back(_tree_sort_left[id][i]);
                }
            } else {
                if (_right_child[id] != -1)
                    query(_right_child[id], mid_axis, bound_r, query_point, results);
                for (int i = 0;i < _tree_sort_right[id].size();++i) {
                    if (_tree_sort_right[id][i].r < query_point - eps)
                        break;
                    results.push_back(_tree_sort_right[id][i]);
                }
            }
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