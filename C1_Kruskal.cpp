#include <iostream>
#include <vector>
#include <unordered_set>
#include <stdexcept>
#include <cstring>

namespace graph {
using Vertex = int64_t;
using Edge = int64_t;

class NotImplementedError : public std::runtime_error {
public:
    explicit NotImplementedError(const char* method_name)
        : std::runtime_error((std::string("NotImplementedError: ") + method_name).c_str()) {
    }
};

class IGraph {
public:
    virtual void AddEdge(Vertex from, Vertex to) = 0;
    virtual Vertex AddVertex() = 0;

    virtual std::unordered_set<Vertex> Neighbors(Vertex v) const = 0;

    virtual const std::unordered_set<Vertex>& NeighborsByReference(Vertex) const {
        throw NotImplementedError("NeighborsByReference");
    }

    virtual std::vector<Edge> OutgoingEdges(Vertex) const = 0;

    virtual const std::vector<Edge>& OutgoingEdgesByReference(Vertex) const {
        throw NotImplementedError("NeighborsByReference");
    }

    size_t NVertices() const {
        return n_vertices_;
    }

    size_t NEdges() const {
        return n_edges_;
    }

protected:
    size_t n_vertices_;
    size_t n_edges_;
    bool is_directed_;

    IGraph(size_t n_vertices, bool is_directed) : n_vertices_(n_vertices), n_edges_(0), is_directed_(is_directed) {
    }
};

class Graph : public IGraph {
    using EdgeList = std::vector<Edge>;
    using AdjLists = std::vector<EdgeList>;

public:
    Graph(size_t n_vertices, bool is_directed)
        : IGraph(n_vertices, is_directed), adj_lists_(n_vertices), edge_begins_(), edge_ends_() {
    }

    void AddEdge(Vertex from, Vertex to) override {
        adj_lists_[from].push_back(static_cast<Edge>(n_edges_++));
        edge_begins_.emplace_back(from);
        edge_ends_.emplace_back(to);
        if (!is_directed_) {
            adj_lists_[to].push_back(static_cast<Edge>(n_edges_++));
            edge_begins_.emplace_back(to);
            edge_ends_.push_back(from);
        }
    }

    Vertex AddVertex() override {
        auto new_vertex = static_cast<Vertex>(n_vertices_++);
        adj_lists_.emplace_back();
        return new_vertex;
    }

    std::unordered_set<Vertex> Neighbors(Vertex v) const override {
        std::unordered_set<Vertex> neighbors;
        for (auto e : adj_lists_[v]) {
            neighbors.insert(edge_ends_[e]);
        }
        return neighbors;
    }

    std::vector<Edge> OutgoingEdges(Vertex v) const override {
        return adj_lists_[v];
    }

    const std::vector<Edge>& OutgoingEdgesByReference(Vertex v) const override {
        return adj_lists_[v];
    }

    Edge GetEdgeBegin(Edge e) const {
        return edge_begins_[e];
    }

    Edge GetEdgeEnd(Edge e) const {
        return edge_ends_[e];
    }

private:
    AdjLists adj_lists_;
    std::vector<Vertex> edge_begins_;
    std::vector<Vertex> edge_ends_;
};

using Weight = int64_t;

class WeightedGraph : public Graph {
public:
    WeightedGraph(size_t n_vertices, bool is_directed) : Graph(n_vertices, is_directed) {
    }

    void AddEdge(Vertex, Vertex) override {
        throw NotImplementedError("WeightedGraph::AddEdge");
    }

    void AddWeightedEdge(Vertex from, Vertex to, Weight weight) {
        Graph::AddEdge(from, to);
        edge_weights_.push_back(weight);
        if (!is_directed_) {
            edge_weights_.push_back(weight);
        }
    }

    Weight EdgeWeight(Edge e) const {
        return edge_weights_[e];
    }

private:
    std::vector<Weight> edge_weights_;
};

template <class T>
class DSU {
public:
    using Item = T;

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

// Edges must be sorted non-decreasing
Weight MSTWeightKruskal(const WeightedGraph& g) {
    Weight mst_weight = 0;

    auto dsu = DSU<Vertex>(g.NVertices());
    for (Edge e = 0; e < static_cast<Edge>(g.NEdges()); ++e) {
        auto[from, to] = std::make_pair(g.GetEdgeBegin(e), g.GetEdgeEnd(e));
        if (dsu.FindSet(from) != dsu.FindSet(to)) {
            dsu.Union(from, to);
            mst_weight += g.EdgeWeight(e);
        }
        if (dsu.NSets() == 1) {
            break;
        }
    }

    return mst_weight;
}

}  // namespace graph

using std::cin;
using std::cout;

int main() {
    std::ios_base::sync_with_stdio(false);
    cin.tie(nullptr);

    size_t n_vertices = 0;
    size_t n_edges = 0;
    cin >> n_vertices >> n_edges;

    auto g = graph::WeightedGraph(n_vertices, true);
    for (size_t i = 0; i < n_edges; ++i) {
        graph::Vertex from = 0;
        graph::Vertex to = 0;
        graph::Weight weight = 0;
        cin >> from >> to >> weight;
        g.AddWeightedEdge(from - 1, to - 1, weight);
    }

    cout << graph::MSTWeightKruskal(g) << '\n';
}