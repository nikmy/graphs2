#include <iostream>
#include <memory>
#include <queue>
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
static constexpr Distance kInfDistance = INT32_MAX;
using DistanceMatrix = std::vector<std::vector<Distance>>;

namespace impl {

bool Relax(const WeightedGraph& g, Edge e, std::vector<Distance>& dist) {
    auto[u, v] = g.AsPair(e);
    if (dist[u] == kInfDistance || dist[v] <= dist[u] + g.EdgeWeight(e)) {
        return false;
    }
    dist[v] = dist[u] + g.EdgeWeight(e);
    return true;
}

void FordBellman(const WeightedGraph& g, Vertex start, std::vector<Weight>& dist) {
    dist.assign(g.NVertices(), kInfDistance);
    dist[start] = 0;
    for (size_t i = 0; i < g.NVertices() - 1; ++i) {
        bool any_relaxed = false;
        for (Edge e = 0; e < static_cast<Edge>(g.NEdges()); ++e) {
            any_relaxed |= Relax(g, e, dist);
        }
        if (!any_relaxed) {
            return;
        }
    }
}

void Dijkstra(const WeightedGraph& g, Vertex source, std::vector<Distance>& dist, DistanceMatrix* wm = nullptr) {
    dist.assign(g.NVertices(), kInfDistance);
    std::priority_queue<std::pair<int64_t, Vertex>> queue;
    std::vector<bool> proc(g.NVertices(), false);

    dist[source] = 0;
    queue.emplace(0, source);
    while (!queue.empty()) {
        auto u = queue.top().second;
        queue.pop();
        if (proc[u]) {
            continue;
        }
        proc[u] = true;
        for (auto e : g.OutgoingEdgesByReference(u)) {
            auto v = g.EdgeEnd(e);
            auto w = wm ? (*wm)[u][v] : g.EdgeWeight(e);
            if (dist[u] + w < dist[v]) {
                dist[v] = dist[u] + w;
                queue.emplace(-dist[v], v);
            }
        }
    }
}

enum SingleSourceAlgorithm { SS_FORD_BELLMAN, SS_DIJKSTRA };

}  // namespace impl

decltype(auto) FindShortestPaths(const WeightedGraph& g, Vertex source,
                                 impl::SingleSourceAlgorithm algorithm = impl::SS_DIJKSTRA) {
    std::vector<Distance> dist;
    switch (algorithm) {
        case impl::SS_FORD_BELLMAN:
            impl::FordBellman(g, source, dist);
            break;
        case impl::SS_DIJKSTRA:
            impl::Dijkstra(g, source, dist);
            break;
        default:
            throw NotImplementedError("FindShortestPaths: invalid algorithm");
    }
    return dist;
}

namespace impl {

void Johnson(const WeightedGraph& g, DistanceMatrix& distance_matrix) {
    auto aux_g = g;
    auto aux_v = aux_g.AddVertex();
    for (Vertex v = 0; v < aux_v; ++v) {
        aux_g.AddEdge(aux_v, v, 0);
    }

    auto p = FindShortestPaths(aux_g, aux_v, SS_FORD_BELLMAN);

    DistanceMatrix wm(g.NVertices(), std::vector<Distance>(g.NVertices(), kInfDistance));
    for (Edge e = 0; e < g.NEdges(); ++e) {
        auto[u, v] = g.AsPair(e);
        wm[u][v] = g.EdgeWeight(e) + p[u] - p[v];
    }

    std::vector<Distance> d;
    for (Vertex u = 0; u < aux_v; ++u) {
        Dijkstra(g, u, d, &wm);
        for (Vertex v = 0; v < aux_v; ++v) {
            if (d[v] == kInfDistance) {
                distance_matrix[u][v] = kInfDistance;
            } else {
                distance_matrix[u][v] = d[v] + p[v] - p[u];
            }
        }
    }
}

enum AllPairsAlgorithm { AP_JOHNSON };

}  // namespace impl

void BuildShortestDistancesMatrix(WeightedGraph& g, DistanceMatrix& m,
                                  impl::AllPairsAlgorithm algorithm = impl::AP_JOHNSON) {
    switch (algorithm) {
        case impl::AP_JOHNSON:
            impl::Johnson(g, m);
            break;
        default:
            throw NotImplementedError("BuildShortestDistancesMatrix: invalid algorithm");
    }
}

}  // namespace graph

using std::cin;
using std::cout;

using graph::Distance;
using graph::DistanceMatrix;
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
        g.AddEdge(from, to, weight);
    }

    DistanceMatrix m(n_vertices, std::vector<Distance>(n_vertices));
    graph::BuildShortestDistancesMatrix(g, m);
    Distance max_d = INT64_MIN;
    for (size_t i = 0; i < n_vertices; ++i) {
        for (size_t j = 0; j < n_vertices; ++j) {
            if (i != j && m[i][j] != graph::kInfDistance) {
                max_d = std::max(max_d, m[i][j]);
            }
        }
    }
    cout << max_d;
}