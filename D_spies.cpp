#include <iostream>
#include <vector>
#include <unordered_set>
#include <stdexcept>
#include <cstring>

#include <queue>

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
    Graph(size_t n_vertices, bool is_directed) : IGraph(n_vertices, is_directed), adj_lists_(n_vertices), edge_ends_() {
    }

    void AddEdge(Vertex from, Vertex to) override {
        adj_lists_[from].push_back(static_cast<Edge>(n_edges_++));
        edge_ends_.emplace_back(to);
        if (!is_directed_) {
            adj_lists_[to].push_back(static_cast<Edge>(n_edges_++));
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

    Edge GetEdgeEnd(Edge e) const {
        return edge_ends_[e];
    }

private:
    AdjLists adj_lists_;
    std::vector<Vertex> edge_ends_;
};

class FastNeighborsGraph : public Graph {
    using NeighborsList = std::unordered_set<Vertex>;
    using NeighborsData = std::vector<NeighborsList>;

public:
    FastNeighborsGraph(size_t n_vertices, bool is_directed)
        : Graph(n_vertices, is_directed), neighbors_data_(n_vertices) {
    }

    void AddEdge(Vertex from, Vertex to) override {
        Graph::AddEdge(from, to);
        neighbors_data_[from].insert(to);
        if (!is_directed_) {
            neighbors_data_[to].insert(from);
        }
    }

    Vertex AddVertex() override {
        auto new_vertex = Graph::AddVertex();
        neighbors_data_.emplace_back();
        return new_vertex;
    }

    std::unordered_set<Vertex> Neighbors(Vertex v) const override {
        return neighbors_data_[v];
    }

    const std::unordered_set<Vertex>& NeighborsByReference(Vertex v) const override {
        return neighbors_data_[v];
    }

private:
    NeighborsData neighbors_data_;
};

using Weight = int64_t;

class WeightedGraph : public FastNeighborsGraph {
public:
    WeightedGraph(size_t n_vertices, bool is_directed) : FastNeighborsGraph(n_vertices, is_directed) {
    }

    void AddEdge(Vertex, Vertex) override {
        throw NotImplementedError("WeightedGraph::AddEdge");
    }

    void AddWeightedEdge(Vertex from, Vertex to, Weight weight) {
        FastNeighborsGraph::AddEdge(from, to);
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

using WeightedEdgeData = std::pair<Weight, Vertex>;

Weight DirectedMSTWeightPrim(const WeightedGraph& g) {
    Weight mst_weight = 0;
    std::priority_queue<WeightedEdgeData, std::vector<WeightedEdgeData>, std::greater<WeightedEdgeData>> queue;
    std::vector<bool> used(g.NVertices(), false);
    queue.push({0, 0});

    while (!queue.empty()) {
        auto[weight, v] = queue.top();
        queue.pop();
        if (used[v]) {
            continue;
        }

        used[v] = true;
        mst_weight += weight;

        for (auto e : g.OutgoingEdgesByReference(v)) {
            auto u = g.GetEdgeEnd(e);
            auto vu_weight = g.EdgeWeight(e);
            if (!used[u]) {
                queue.push({vu_weight, u});
            }
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

    graph::Vertex n_spies = 0;
    cin >> n_spies;

    std::vector<std::vector<graph::Weight>> weights(n_spies);
    for (graph::Vertex from = 0; from < n_spies; ++from) {
        weights[from].resize(n_spies);
        for (graph::Vertex to = 0; to < n_spies; ++to) {
            cin >> weights[from][to];
        }
    }

    // One vertex for mission
    auto g = graph::WeightedGraph(n_spies + 1, false);
    for (graph::Vertex from = 0; from < n_spies; ++from) {
        for (graph::Vertex to = from + 1; to < n_spies; ++to) {
            g.AddWeightedEdge(from, to, weights[from][to]);
        }
        graph::Weight spy_price = 0;
        cin >> spy_price;
        g.AddWeightedEdge(from, n_spies, spy_price);
    }

    cout << graph::DirectedMSTWeightPrim(g) << '\n';
}