#include <iostream>
#include <vector>
#include <unordered_set>
#include <stdexcept>
#include <cstring>
#include <queue>

namespace graph {
using Vertex = int64_t;
using Edge = int64_t;

using EdgeProperty = int64_t;
static constexpr EdgeProperty kMaxEdgePropertyValue = INT64_MAX;

class NotImplementedError : public std::runtime_error {
public:
    explicit NotImplementedError(const char* method_name)
        : std::runtime_error((std::string("NotImplementedError: ") + method_name).c_str()) {
    }
};

class IGraph {
public:
    virtual void AddEdge(Vertex, Vertex, EdgeProperty = 0) = 0;

    virtual Vertex AddVertex() = 0;

    virtual std::unordered_set<Vertex> Neighbors(Vertex v) const = 0;

    virtual const std::unordered_set<Vertex>& NeighborsByReference(Vertex) const {
        throw NotImplementedError("NeighborsByReference");
    }

    virtual std::vector<Edge> OutgoingEdges(Vertex) const = 0;

    virtual const std::vector<Edge>& OutgoingEdgesByReference(Vertex) const {
        throw NotImplementedError("OutgoingEdgesByReference");
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

class AdjListsGraph : public IGraph {
    using EdgeList = std::vector<Edge>;
    using AdjLists = std::vector<EdgeList>;

public:
    AdjListsGraph(size_t n_vertices, bool is_directed)
        : IGraph(n_vertices, is_directed), adj_lists_(n_vertices), edge_begins_(), edge_ends_() {
    }

    void AddEdge(Vertex from, Vertex to, EdgeProperty = 0) override {
        adj_lists_[from].push_back(static_cast<Edge>(n_edges_++));
        edge_begins_.push_back(from);
        edge_ends_.push_back(to);
        if (!is_directed_) {
            adj_lists_[to].push_back(static_cast<Edge>(n_edges_++));
            edge_begins_.push_back(to);
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

    Edge GetEdge(Vertex from, Vertex to) const {
        return adj_lists_[from][to];
    }

    Vertex EdgeBegin(Edge e) const {
        return edge_begins_[e];
    }

    Vertex EdgeEnd(Edge e) const {
        return edge_ends_[e];
    }

private:
    AdjLists adj_lists_;
    std::vector<Vertex> edge_begins_;
    std::vector<Vertex> edge_ends_;
};

using Weight = EdgeProperty;

class WeightedGraph : public AdjListsGraph {
public:
    WeightedGraph(size_t n_vertices, bool is_directed) : AdjListsGraph(n_vertices, is_directed) {
    }

    void AddEdge(Vertex from, Vertex to, Weight weight) override {
        AdjListsGraph::AddEdge(from, to);
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

namespace impl {

using Distance = Weight;
static constexpr Distance kInfDistance = 2009000999;

decltype(auto) DijkstraMinDist(const WeightedGraph& g, Vertex source) {
    std::priority_queue<std::pair<int64_t, Vertex>> queue;
    std::vector<Distance> dist(g.NVertices(), kInfDistance);
    std::vector<bool> proc(g.NVertices(), false);

    dist[source] = 0;
    queue.emplace(0, source);
    while (!queue.empty()) {
        auto v = queue.top().second;
        queue.pop();
        if (proc[v]) {
            continue;
        }
        proc[v] = true;
        for (auto e : g.OutgoingEdgesByReference(v)) {
            auto u = g.EdgeEnd(e);
            auto w = g.EdgeWeight(e);
            if (dist[v] + w < dist[u]) {
                dist[u] = dist[v] + w;
                queue.emplace(-dist[u], u);
            }
        }
    }
    return dist;
}

}  // namespace impl

decltype(auto) FindShortestPaths(const WeightedGraph& g, Vertex source) {
    return impl::DijkstraMinDist(g, source);
}

}  // namespace graph

using std::cin;
using std::cout;

using graph::Vertex;
using graph::Weight;
using graph::WeightedGraph;

int main() {
    std::ios_base::sync_with_stdio(false);
    cin.tie(nullptr);

    size_t n_runs = 0;
    size_t n_vertices = 0;
    size_t n_edges = 0;
    Vertex source = 0;

    cin >> n_runs;
    for (size_t i = 0; i < n_runs; ++i) {

        cin >> n_vertices >> n_edges;

        auto g = WeightedGraph(n_vertices, false);
        for (size_t j = 0; j < n_edges; ++j) {
            Vertex from = 0;
            Vertex to = 0;
            Weight weight = 0;
            cin >> from >> to >> weight;
            g.AddEdge(from, to, weight);
        }
        cin >> source;

        auto dist = graph::FindShortestPaths(g, source);
        for (auto d : dist) {
            cout << d << ' ';
        }
    }
}