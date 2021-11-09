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

    size_t n_islands = 0;
    size_t n_bridges = 0;
    cin >> n_islands >> n_bridges;

    auto dsu = graph::DSU(n_islands);
    size_t islands[2]{0};
    for (size_t i = 1; i <= n_bridges; ++i) {
        cin >> islands[0] >> islands[1];
        if (dsu.FindSet(islands[0]) != dsu.FindSet(islands[1])) {
            dsu.Union(islands[0], islands[1]);
        }
        if (dsu.NSets() == 1) {
            cout << i << '\n';
            break;
        }
    }
}
