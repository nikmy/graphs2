#include <iostream>
#include <vector>
#include <unordered_set>
#include <stdexcept>
#include <cstring>

namespace graph {
using Vertex = int64_t;
using Edge = int64_t;

using EdgeProperty = int32_t;
static constexpr EdgeProperty kMaxEdgePropertyValue = INT32_MAX;

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

    decltype(auto) AsPair(Edge e) const {
        return std::make_pair(edge_begins_[e], edge_ends_[e]);
    }

    size_t Degree(Vertex v) const {
        return OutgoingEdgesByReference(v).size();
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

bool Relax(const WeightedGraph& g, Edge e, std::vector<Weight>& dist) {
    auto[u, v] = g.AsPair(e);
    if (dist[u] == kMaxEdgePropertyValue || dist[v] <= dist[u] + g.EdgeWeight(e)) {
        return false;
    }
    dist[v] = dist[u] + g.EdgeWeight(e);
    return true;
}

void FordBellman(const WeightedGraph& g, Vertex start, std::vector<Weight>& dist) {
    dist.assign(g.NVertices(), kMaxEdgePropertyValue);
    dist[start] = 0;
    for (size_t i = 0; i < g.NVertices() - 1; ++i) {
        for (Edge e = 0; e < static_cast<Edge>(g.NEdges()); ++e) {
            Relax(g, e, dist);
        }
    }
}

enum ShortestPathAlgorithm { FORD_BELLMAN };

}  // namespace impl

decltype(auto) FindShortestPath(const WeightedGraph& g, Vertex source,
                                impl::ShortestPathAlgorithm = impl::FORD_BELLMAN) {
    std::vector<Weight> dist;
    impl::FordBellman(g, source, dist);
    return dist;
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

    size_t n_vertices = 0;
    size_t n_edges = 0;
    cin >> n_vertices >> n_edges;

    auto g = WeightedGraph(n_vertices, true);
    for (size_t i = 0; i < n_edges; ++i) {
        Vertex from = 0;
        Vertex to = 0;
        Weight weight = 0;
        cin >> from >> to >> weight;
        g.AddEdge(from - 1, to - 1, weight);
    }

    auto dist = graph::FindShortestPath(g, 0);
    for (auto d : dist) {
        cout << (d == graph::kMaxEdgePropertyValue ? 30000 : d) << ' ';
    }
    cout << std::endl;
}