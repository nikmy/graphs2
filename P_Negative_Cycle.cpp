#include <algorithm>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <unordered_set>
#include <vector>

namespace graph {
using Vertex = size_t;
using Edge = size_t;

using EdgeProperty = int64_t;

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

    const Weight& EdgeWeight(Edge e) const {
        return edge_weights_[e];
    }

    Weight& EdgeWeight(Edge e) {
        return edge_weights_[e];
    }

private:
    std::vector<Weight> edge_weights_;
};

using Distance = Weight;
static constexpr Distance kInfDistance = 100000;

static constexpr Vertex kNoVertex = UINT64_MAX;

namespace impl {

bool RelaxNC(const WeightedGraph& g, Edge e, std::vector<Distance>& dist, std::vector<Vertex>& prev) {
    auto[u, v] = g.AsPair(e);
    if (dist[u] == kInfDistance || dist[v] <= dist[u] + g.EdgeWeight(e)) {
        return false;
    }
    dist[v] = dist[u] + g.EdgeWeight(e);
    prev[v] = u;
    return true;
}

Vertex FordBellmanNC(const WeightedGraph& g, Vertex start, std::vector<Weight>& dist, std::vector<Vertex>& prev) {
    dist.assign(g.NVertices(), kInfDistance);
    dist[start] = 0;
    for (size_t i = 0; i < g.NVertices() - 1; ++i) {
        bool any_relaxed = false;
        for (Edge e = 0; e < static_cast<Edge>(g.NEdges()); ++e) {
            any_relaxed |= RelaxNC(g, e, dist, prev);
        }
        if (!any_relaxed) {
            return kNoVertex;
        }
    }

    for (Edge e = 0; e < g.NEdges(); ++e) {
        if (RelaxNC(g, e, dist, prev)) {
            return g.EdgeEnd(e);
        }
    }
    return kNoVertex;
}

}  // namespace impl

void FindNegativeCycle(const WeightedGraph& g, std::vector<Vertex>& neg_cycle) {
    auto aux_g = g;
    auto aux_v = aux_g.AddVertex();
    for (Vertex v = 0; v < aux_v; ++v) {
        aux_g.AddEdge(aux_v, v, 0);
    }

    std::vector<Vertex> prev(aux_g.NVertices(), kNoVertex);
    std::vector<Distance> dist(aux_g.NVertices(), kInfDistance);
    auto cycle_end = impl::FordBellmanNC(aux_g, aux_v, dist, prev);

    neg_cycle.clear();
    if (cycle_end == kNoVertex) {
        return;
    }

    std::vector<bool> used(g.NVertices(), false);
    auto v = cycle_end;
    while (!used[v]) {
        neg_cycle.push_back(v);
        used[v] = true;
        v = prev[v];
    }
    neg_cycle.push_back(v);
    std::reverse(neg_cycle.begin(), neg_cycle.end());
    while (neg_cycle.back() != neg_cycle.front()) {
        neg_cycle.pop_back();
    }
}

}  // namespace graph

using std::cin;
using std::cout;

using graph::Distance;
using graph::Vertex;
using graph::Weight;
using graph::WeightedGraph;

int main() {
    std::ios_base::sync_with_stdio(false);
    cin.tie(nullptr);

    size_t n_vertices = 0;
    cin >> n_vertices;

    auto g = graph::WeightedGraph(n_vertices, true);
    for (size_t i = 0; i < n_vertices; ++i) {
        for (size_t j = 0; j < n_vertices; ++j) {
            Weight w = 0;
            cin >> w;
            if (w == graph::kInfDistance) {
                continue;
            }
            g.AddEdge(i, j, w);
        }
    }

    std::vector<Vertex> neg_cycle;
    graph::FindNegativeCycle(g, neg_cycle);
    if (neg_cycle.empty()) {
        cout << "NO\n";
    } else {
        cout << "YES\n" << neg_cycle.size() << '\n';
        for (auto v : neg_cycle) {
            cout << v + 1 << ' ';
        }
    }
}