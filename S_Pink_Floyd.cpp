#include <iostream>
#include <memory>
#include <queue>
#include <stdexcept>
#include <unordered_set>
#include <map>
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
static constexpr Distance kInfDistance = INT64_MAX;
using DistanceMatrix = std::vector<std::vector<Distance>>;
static constexpr Vertex kUnreachable = INT64_MAX;

void FloydWarshallOptimization(DistanceMatrix& dist, std::vector<std::vector<Edge>>& next) {
    size_t n = dist.size();
    for (Vertex k = 0; k < n; ++k) {
        for (Vertex u = 0; u < n; ++u) {
            for (Vertex v = 0; v < n; ++v) {
                if (dist[u][k] == kInfDistance || dist[k][v] == kInfDistance) {
                    continue;
                }
                if (dist[u][k] + dist[k][v] < dist[u][v]) {
                    dist[u][v] = dist[u][k] + dist[k][v];
                    next[u][v] = next[u][k];
                }
            }
        }
    }
}

bool BuildShortestPathOrFindNegCycle(Vertex begin, Vertex end, std::vector<std::vector<Edge>>& next,
                                     std::vector<Vertex>& path, const DistanceMatrix& m) {
    Vertex v = begin;
    while (v != end) {
        if (v == kUnreachable || m[v][v] < 0) {
            return true;
        }
        path.push_back(v);
        v = next[v][end];
    }
    path.push_back(end);
    return false;
}

}  // namespace graph

using std::cin;
using std::cout;

using graph::Distance;
using graph::Edge;
using graph::Vertex;
using graph::Weight;
using graph::WeightedGraph;

int main() {
    std::ios_base::sync_with_stdio(false);
    cin.tie(nullptr);

    size_t n_cities = 0;
    size_t n_flights = 0;
    size_t n_concerts = 0;
    cin >> n_cities >> n_flights >> n_concerts;

    auto g = WeightedGraph(n_cities, true);

    std::map<std::pair<Vertex, Vertex>, Edge> flights;

    for (size_t i = 0; i < n_flights; ++i) {
        Vertex from = 0;
        Vertex to = 0;
        Weight weight = 0;
        cin >> from >> to >> weight;
        g.AddEdge(from - 1, to - 1, -weight);
        auto from_to = std::make_pair(from - 1, to - 1);
        if (flights.count(from_to)) {
            if (g.EdgeWeight(flights[from_to]) >= -weight) {
                flights[from_to] = i;
            }
        } else {
            flights[from_to] = i;
        }
    }

    std::vector<Vertex> concerts(n_concerts);
    for (auto& c : concerts) {
        cin >> c;
        --c;
    }

    graph::DistanceMatrix m(n_cities, std::vector<Distance>(n_cities, graph::kInfDistance));
    std::vector<std::vector<Vertex>> next(n_cities, std::vector<Vertex>(n_cities, graph::kUnreachable));
    for (Vertex v = 0; v < g.NVertices(); ++v) {
        m[v][v] = 0;
        next[v][v] = v;
    }
    for (Edge e = 0; e < g.NEdges(); ++e) {
        auto[u, v] = g.AsPair(e);
        if (g.EdgeWeight(e) < m[u][v]) {
            m[u][v] = g.EdgeWeight(e);
            next[u][v] = v;
        }
    }

    graph::FloydWarshallOptimization(m, next);

    std::vector<Vertex> path;
    for (size_t i = 1; i < n_concerts; ++i) {
        if (graph::BuildShortestPathOrFindNegCycle(concerts[i - 1], concerts[i], next, path, m)) {
            cout << "infinitely kind\n";
            return 0;
        }
        path.pop_back();
    }
    path.push_back(concerts.back());

    cout << path.size() - 1 << '\n';
    for (size_t i = 1; i < path.size(); ++i) {
        cout << flights[std::make_pair(path[i - 1], path[i])] + 1 << ' ';
    }
}