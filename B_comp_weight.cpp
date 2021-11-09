#include <iostream>
#include <vector>

namespace graph {

class DSU {
public:
    using Item = size_t;

    DSU() : pres_(), rank_(), size_(0) {
    }

    explicit DSU(size_t init_size) : DSU() {
        pres_.reserve(init_size);
        rank_.reserve(init_size);
        for (size_t i = 0; i < init_size; ++i) {
            MakeSet();
        }
    }

    void MakeSet() {
        pres_.push_back(pres_.size());
        rank_.push_back(1);
        ++size_;
    }

    Item FindSet(Item x) {
        if (pres_[x] == x) {
            return x;
        }
        return pres_[x] = FindSet(pres_[x]);
    }

    void Union(Item x, Item y) {
        auto[x_set, y_set] = std::make_pair(FindSet(x), FindSet(y));
        if (x_set == y_set) {
            return;
        }
        if (rank_[x_set] < rank_[y_set]) {
            pres_[x_set] = y_set;
        } else {
            pres_[y_set] = x_set;
            if (rank_[x_set] == rank_[y_set]) {
                ++rank_[x_set];
            }
        }
        --size_;
    }

    size_t NSets() {
        return size_;
    }

private:
    std::vector<Item> pres_;
    std::vector<size_t> rank_;
    size_t size_;
};

}  // namespace graph

using std::cin;
using std::cout;

int main() {
    std::ios_base::sync_with_stdio(false);
    cin.tie(nullptr);

    size_t n_vertices = 0;
    size_t n_requests = 0;
    cin >> n_vertices >> n_requests;

    auto dsu = graph::DSU(n_vertices);
    std::vector<size_t> comp_weights(n_vertices, 0);
    int cmd = 0;
    for (size_t i = 0; i < n_requests; ++i) {
        cin >> cmd;
        if (cmd == 2) {
            size_t x = 0;
            cin >> x;
            cout << comp_weights[dsu.FindSet(x - 1)] << '\n';
            continue;
        }
        size_t from = 0;
        size_t to = 0;
        size_t weight = 0;
        cin >> from >> to >> weight;
        if (dsu.FindSet(from - 1) == dsu.FindSet(to - 1)) {
            comp_weights[dsu.FindSet(from - 1)] += weight;
        } else {
            auto new_weight = comp_weights[dsu.FindSet(from - 1)] + comp_weights[dsu.FindSet(to - 1)] + weight;
            dsu.Union(from - 1, to - 1);
            comp_weights[dsu.FindSet(from - 1)] = new_weight;
        }
    }
}
