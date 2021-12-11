#include <iostream>
#include <vector>

namespace graph {

class DSU {
public:
    DSU() : pres_(), rank_(), size_(0) {
    }

    explicit DSU(size_t init_size) : DSU() {
        pres_.reserve(init_size);
        rank_.reserve(init_size);
        for (size_t i = 0; i < init_size; ++i) {
            AddSingleElementSet();
        }
    }

    void AddSingleElementSet() {
        pres_.push_back(pres_.size());
        rank_.push_back(1);
        ++size_;
    }

    size_t FindSet(size_t item_id) {
        if (pres_[item_id] == item_id) {
            return item_id;
        }
        return pres_[item_id] = FindSet(pres_[item_id]);
    }

    bool IsInSameSet(size_t item1_id, size_t item2_id) {
        return FindSet(item1_id) == FindSet(item2_id);
    }

    void Unite(size_t item1_id, size_t item2_id) {
        auto[set1, set2] = std::make_pair(FindSet(item1_id), FindSet(item2_id));
        if (set1 == set2) {
            return;
        }
        if (rank_[set1] < rank_[set2]) {
            pres_[set1] = set2;
        } else {
            pres_[set2] = set1;
            if (rank_[set1] == rank_[set2]) {
                ++rank_[set1];
            }
        }
        --size_;
    }

    size_t NSets() {
        return size_;
    }

private:
    std::vector<size_t> pres_;
    std::vector<size_t> rank_;
    size_t size_;
};

}  // namespace graph

template <class Weight = size_t>
class WeightedForest {
public:
    explicit WeightedForest(size_t n_trees) : dsu_(n_trees), weights_(n_trees, 0) {
    }

    Weight GetTreeWeight(size_t item_id) {
        return weights_[dsu_.FindSet(item_id)];
    }

    void AddEdge(size_t src_id, size_t dst_id, Weight weight) {
        if (dsu_.IsInSameSet(src_id - 1, dst_id - 1)) {
            weights_[dsu_.FindSet(src_id - 1)] += weight;
        } else {
            auto new_weight = weights_[dsu_.FindSet(src_id - 1)] + weights_[dsu_.FindSet(dst_id - 1)] + weight;
            dsu_.Unite(src_id - 1, dst_id - 1);
            weights_[dsu_.FindSet(src_id - 1)] = new_weight;
        }
    }

private:
    graph::DSU dsu_;
    std::vector<Weight> weights_;
};

using std::cin;
using std::cout;

int main() {
    std::ios_base::sync_with_stdio(false);
    cin.tie(nullptr);

    size_t n_vertices = 0;
    size_t n_requests = 0;
    cin >> n_vertices >> n_requests;

    auto forest = WeightedForest(n_vertices);

    int cmd = 0;
    for (size_t i = 0; i < n_requests; ++i) {
        cin >> cmd;
        if (cmd == 2) {
            size_t item = 0;
            cin >> item;
            cout << forest.GetTreeWeight(item - 1) << '\n';
            continue;
        }
        size_t src_id = 0;
        size_t dst_id = 0;
        size_t weight = 0;
        cin >> src_id >> dst_id >> weight;
        forest.AddEdge(src_id, dst_id, weight);
    }
}
