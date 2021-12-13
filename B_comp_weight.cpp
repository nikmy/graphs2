#include <iostream>
#include <vector>

namespace graph {

class DisjointedSetForest {
public:
    DisjointedSetForest() : root_(), rank_(), size_(0) {
    }

    explicit DisjointedSetForest(size_t init_size) : DisjointedSetForest() {
        root_.reserve(init_size);
        rank_.reserve(init_size);
        for (size_t i = 0; i < init_size; ++i) {
            AddVertex();
        }
    }

    void AddVertex() {
        root_.push_back(root_.size());
        rank_.push_back(1);
        ++size_;
    }

    size_t GetRoot(size_t vertex_id) {
        if (root_[vertex_id] == vertex_id) {
            return vertex_id;
        }
        return root_[vertex_id] = GetRoot(root_[vertex_id]);
    }

    bool HaveSameRoot(size_t vertex1_id, size_t vertex2_id) {
        return GetRoot(vertex1_id) == GetRoot(vertex2_id);
    }

    void UniteTrees(size_t vertex1_id, size_t vertex2_id) {
        auto[set1, set2] = std::make_pair(GetRoot(vertex1_id), GetRoot(vertex2_id));
        if (set1 == set2) {
            return;
        }
        if (rank_[set1] < rank_[set2]) {
            root_[set1] = set2;
        } else {
            root_[set2] = set1;
            if (rank_[set1] == rank_[set2]) {
                ++rank_[set1];
            }
        }
        --size_;
    }

    size_t NTrees() {
        return size_;
    }

private:
    std::vector<size_t> root_;
    std::vector<size_t> rank_;
    size_t size_;
};

}  // namespace graph

template <class Weight = size_t>
class WeightedForest {
public:
    explicit WeightedForest(size_t n_trees) : forest_(n_trees), weight_(n_trees, 0) {
    }

    Weight GetTreeWeight(size_t item_id) {
        return weight_[forest_.GetRoot(item_id)];
    }

    void AddEdge(size_t src_id, size_t dst_id, Weight weight) {
        if (forest_.HaveSameRoot(src_id - 1, dst_id - 1)) {
            weight_[forest_.GetRoot(src_id - 1)] += weight;
        } else {
            auto new_weight = weight_[forest_.GetRoot(src_id - 1)] + weight_[forest_.GetRoot(dst_id - 1)] + weight;
            forest_.UniteTrees(src_id - 1, dst_id - 1);
            weight_[forest_.GetRoot(src_id - 1)] = new_weight;
        }
    }

private:
    graph::DisjointedSetForest forest_;
    std::vector<Weight> weight_;
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
