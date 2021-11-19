#include <algorithm>
#include <iostream>
#include <memory>
#include <queue>
#include <stdexcept>
#include <unordered_set>
#include <vector>

namespace graph {
using Vertex = size_t;
using Edge = size_t;

namespace except {

class NotImplementedError : public std::runtime_error {
public:
    explicit NotImplementedError(const char* method_name)
        : std::runtime_error((std::string("NotImplementedError: ") + method_name).c_str()) {
    }
};

class IncompatibleDataFormatError : public std::runtime_error {
public:
    IncompatibleDataFormatError() : std::runtime_error("IncompatibleDataFormatError") {
    }
};

}  // namespace except

struct EdgeData {
    virtual ~EdgeData() = default;
};

class IGraph {
public:
    virtual void AddEdge(Vertex, Vertex, const EdgeData* = nullptr) = 0;

    virtual Vertex AddVertex() = 0;

    virtual std::pair<Vertex, Vertex> AsPair(Edge) const = 0;

    virtual Vertex EdgeBegin(Edge e) const = 0;

    virtual Vertex EdgeEnd(Edge e) const = 0;

    virtual size_t Degree(Vertex v) const = 0;

    virtual std::unordered_set<Vertex> Neighbors(Vertex v) const = 0;

    virtual const std::unordered_set<Vertex>& NeighborsByReference(Vertex) const {
        throw except::NotImplementedError("NeighborsByReference");
    }

    virtual std::vector<Edge> OutgoingEdges(Vertex) const = 0;

    virtual const std::vector<Edge>& OutgoingEdgesByReference(Vertex) const {
        throw except::NotImplementedError("OutgoingEdgesByReference");
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

    void AddEdge(Vertex from, Vertex to, const EdgeData* = nullptr) override {
        adj_lists_[from].push_back(n_edges_++);
        edge_begins_.push_back(from);
        edge_ends_.push_back(to);
        if (!is_directed_) {
            adj_lists_[to].push_back(n_edges_++);
            edge_begins_.push_back(to);
            edge_ends_.push_back(from);
        }
    }

    Vertex AddVertex() override {
        auto new_vertex = n_vertices_++;
        adj_lists_.emplace_back();
        return new_vertex;
    }

    std::pair<Vertex, Vertex> AsPair(Edge e) const override {
        return std::make_pair(edge_begins_[e], edge_ends_[e]);
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

    Vertex EdgeBegin(Edge e) const override {
        return edge_begins_[e];
    }

    Vertex EdgeEnd(Edge e) const override {
        return edge_ends_[e];
    }

    size_t Degree(Vertex v) const override {
        return OutgoingEdgesByReference(v).size();
    }

private:
    AdjLists adj_lists_;
    std::vector<Vertex> edge_begins_;
    std::vector<Vertex> edge_ends_;
};

static constexpr int64_t kInfinity = INT64_MAX;

// namespace detail {
//
// namespace impl {
//
// template <typename Weight, typename WeightFunction>
// bool Relax(const IGraph& g, WeightFunction get_weight, Edge e, std::vector<Weight>& dist) {
//    auto[u, v] = g.AsPair(e);
//    if (dist[u] == kInfinity || dist[v] <= dist[u] + get_weight(e)) {
//        return false;
//    }
//    dist[v] = dist[u] + get_weight(e);
//    return true;
//}
//
//}  // namespace impl
//
// template <typename Weight, typename WeightFunction>
// void FordBellman(const IGraph& g, WeightFunction get_weight, Vertex start, std::vector<Weight>& dist) {
//    dist.assign(g.NVertices(), kInfinity);
//    dist[start] = 0;
//    for (size_t i = 0; i < g.NVertices() - 1; ++i) {
//        bool any_relaxed = false;
//        for (Edge e = 0; e < g.NEdges(); ++e) {
//            any_relaxed |= impl::Relax(g, get_weight, e, dist);
//        }
//        if (!any_relaxed) {
//            return;
//        }
//    }
//}
//
// template <typename Weight, typename WeightFunction>
// void Dijkstra(const IGraph& g, WeightFunction get_weight, Vertex source, std::vector<Weight>& dist,
//              std::vector<Vertex>* prev = nullptr) {
//    std::priority_queue<std::pair<Weight, Vertex>> queue;
//    std::vector<bool> proc(g.NVertices(), false);
//
//    dist[source] = 0;
//    queue.emplace(0, source);
//    while (!queue.empty()) {
//        auto u = queue.top().second;
//        queue.pop();
//        if (proc[u]) {
//            continue;
//        }
//        proc[u] = true;
//        for (auto e : g.OutgoingEdgesByReference(u)) {
//            auto v = g.EdgeEnd(e);
//            auto w = get_weight(e);
//            if (dist[u] + w < dist[v]) {
//                dist[v] = dist[u] + w;
//                if (prev) {
//                    (*prev)[v] = u;
//                }
//                queue.emplace(-dist[v], v);
//            }
//        }
//    }
//}
//
// enum SingleSourceAlgorithm { SS_FORD_BELLMAN, SS_DIJKSTRA };
//
//}  // namespace detail
//
// template <typename Weight, typename WeightFunction>
// decltype(auto) FindShortestPaths(const IGraph& g, WeightFunction get_weight, Vertex source,
//                                 detail::SingleSourceAlgorithm algorithm = detail::SS_DIJKSTRA) {
//    std::vector<Weight> dist(g.NVertices(), kInfinity);
//    switch (algorithm) {
//        case detail::SS_FORD_BELLMAN:
//            detail::FordBellman(g, get_weight, source, dist);
//            break;
//        case detail::SS_DIJKSTRA:
//            detail::Dijkstra(g, get_weight, source, dist);
//            break;
//        default:
//            throw except::NotImplementedError("FindShortestPaths: invalid algorithm");
//    }
//    return dist;
//}

using Cost = int64_t;
using Time = int64_t;

struct CostTimeData : public EdgeData {
    CostTimeData(Cost c, Time t) : cost(c), time(t) {
    }

    Cost cost;
    Time time;
};

class CostTimeGraph : public AdjListsGraph {
public:
    CostTimeGraph(size_t n_vertices, bool is_directed) : AdjListsGraph(n_vertices, is_directed) {
    }

    void AddEdge(Vertex from, Vertex to, const EdgeData* data) override {
        AdjListsGraph::AddEdge(from, to, data);
        auto* pt_data = dynamic_cast<const CostTimeData*>(data);
        if (!pt_data) {
            throw except::IncompatibleDataFormatError();
        }
        edge_costs_.push_back(pt_data->cost);
        edge_times_.push_back(pt_data->time);
        if (!is_directed_) {
            edge_costs_.push_back(pt_data->cost);
            edge_times_.push_back(pt_data->time);
        }
    }

    Cost GetCost(Edge e) const {
        return edge_costs_[e];
    }

    Time GetTime(Edge e) const {
        return edge_times_[e];
    }

private:
    std::vector<Cost> edge_costs_;
    std::vector<Time> edge_times_;
};

template <typename CostType>
struct PathData {
    CostType cost;
    std::vector<Vertex> path;
};

namespace impl {

template <typename Weight, typename WeightFunction, typename CostFunction>
void ZeroWeightedEdgesOptimization(const IGraph& g, WeightFunction get_weight, CostFunction get_cost,
                                   std::vector<Weight>& dist, std::vector<Edge>& prev) {
    std::priority_queue<std::pair<Weight, Vertex>> queue;
    std::vector<bool> proc(g.NVertices(), false);

    for (Vertex v = 0; v < g.NVertices(); ++v) {
        if (dist[v] != kInfinity) {
            queue.emplace(0, v);
        }
    }

    while (!queue.empty()) {
        auto u = queue.top().second;
        queue.pop();
        if (proc[u]) {
            continue;
        }
        proc[u] = true;
        for (auto e : g.OutgoingEdgesByReference(u)) {
            auto v = g.EdgeEnd(e);
            if (get_weight(e) == 0 && dist[u] != kInfinity && dist[u] + get_cost(e) < dist[v]) {
                dist[v] = dist[u] + get_cost(e);
                prev[v] = e;
                queue.emplace(-dist[v], v);
            }
        }
    }
}

}  // namespace impl

template <typename CostType, typename CostFunction, typename Time, typename TimeFunc>
decltype(auto) BuildCheapestWayWithTimeLimit(IGraph& g, CostFunction get_cost, Vertex start, Vertex finish,
                                             TimeFunc get_time, Time limit) {
    std::vector<std::vector<CostType>> dp(limit + 1, std::vector<CostType>(g.NVertices(), kInfinity));
    std::vector<std::vector<Edge>> prev(limit + 1, std::vector<Edge>(g.NVertices(), kInfinity));

    dp[0][start] = 0;
    Time best_t = 0;
    for (Time t = 0; t <= limit; ++t) {
        impl::ZeroWeightedEdgesOptimization(g, get_time, get_cost, dp[t], prev[t]);
        for (Edge e = 0; e < g.NEdges(); ++e) {
            auto[u, v] = g.AsPair(e);
            auto time = get_time(e);
            auto cost = get_cost(e);
            if (time == 0) {
                continue;
            }
            if (t + time <= limit && dp[t][u] != kInfinity) {
                if (dp[t][u] + cost < dp[t + time][v]) {
                    dp[t + time][v] = dp[t][u] + cost;
                    prev[t + time][v] = e;
                }
            }
        }
        if (dp[t][finish] < dp[best_t][finish]) {
            best_t = t;
        }
    }

    std::vector<Vertex> path;
    if (dp[best_t][finish] == kInfinity) {
        return PathData<CostType>{kInfinity, path};
    }
    auto v = finish;
    auto t = best_t;
    while (v != start) {
        path.push_back(v);
        auto e = prev[t][v];
        v = g.EdgeBegin(e);
        t -= get_time(e);
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return PathData<CostType>{dp[best_t][finish], path};
}

}  // namespace graph

using std::cin;
using std::cout;

using graph::Cost;
using graph::Edge;
using graph::Time;
using graph::Vertex;
using EData = graph::CostTimeData;
using Graph = graph::CostTimeGraph;

int main() {
    std::ios_base::sync_with_stdio(false);
    cin.tie(nullptr);

    size_t n_vertices = 0;
    size_t n_edges = 0;
    Time time_limit = 0;
    cin >> n_vertices >> n_edges >> time_limit;

    auto g = Graph(n_vertices, false);
    for (size_t i = 0; i < n_edges; ++i) {
        EData data(0, 0);
        Vertex from = 0;
        Vertex to = 0;
        cin >> from >> to >> data.cost >> data.time;
        if (data.time <= time_limit) {
            g.AddEdge(from - 1, to - 1, &data);
        }
    }

    auto get_time = [&](Edge e) { return g.GetTime(e); };
    auto get_cost = [&](Edge e) { return g.GetCost(e); };

    auto[c, p] = graph::BuildCheapestWayWithTimeLimit<Cost>(g, get_cost, 0, g.NVertices() - 1, get_time, time_limit);

    if (c == graph::kInfinity) {
        cout << -1;
    } else {
        cout << c << '\n' << p.size() << '\n';
        for (auto v : p) {
            cout << v + 1 << ' ';
        }
    }
}