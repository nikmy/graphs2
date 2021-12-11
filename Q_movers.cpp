#include <iostream>
#include <memory>
#include <queue>
#include <set>
#include <stdexcept>
#include <unordered_set>
#include <vector>

namespace graph {
using Vertex = size_t;
using EdgeID = size_t;
using Weight = int64_t;

struct EdgeData {
    Vertex src;
    Vertex dst;
};

class NotImplementedError : public std::runtime_error {
public:
    explicit NotImplementedError(const char* method_name)
        : std::runtime_error((std::string("NotImplementedError: ") + method_name).c_str()) {
    }
};

class IGraph {
public:
    virtual void AddEdge(Vertex, Vertex, Weight = 0) = 0;

    virtual Vertex AddVertex() = 0;

    virtual std::unordered_set<Vertex> Neighbors(Vertex v) const = 0;

    virtual const std::unordered_set<Vertex>& NeighborsByReference(Vertex) const {
        throw NotImplementedError("NeighborsByReference");
    }

    virtual std::vector<EdgeID> OutgoingEdges(Vertex) const = 0;

    virtual const std::vector<EdgeID>& OutgoingEdgesByReference(Vertex) const {
        throw NotImplementedError("OutgoingEdgesByReference");
    }

    virtual Vertex EdgeEnd(EdgeID) const = 0;
    virtual Weight EdgeWeight(EdgeID) const {
        throw NotImplementedError("EdgeWeight");
    }

    virtual EdgeData GetEdgeData(EdgeID) const {
        throw NotImplementedError("GetEdgeData");
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
    using EdgeList = std::vector<EdgeID>;
    using AdjLists = std::vector<EdgeList>;

public:
    AdjListsGraph(size_t n_vertices, bool is_directed)
        : IGraph(n_vertices, is_directed), adj_lists_(n_vertices), edge_begins_(), edge_ends_() {
    }

    void AddEdge(Vertex from, Vertex to, Weight = 0) override {
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

    std::unordered_set<Vertex> Neighbors(Vertex v) const override {
        std::unordered_set<Vertex> neighbors;
        for (auto e : adj_lists_[v]) {
            neighbors.insert(edge_ends_[e]);
        }
        return neighbors;
    }

    std::vector<EdgeID> OutgoingEdges(Vertex v) const override {
        return adj_lists_[v];
    }

    const std::vector<EdgeID>& OutgoingEdgesByReference(Vertex v) const override {
        return adj_lists_[v];
    }

    EdgeID GetEdge(Vertex from, Vertex to) const {
        return adj_lists_[from][to];
    }

    Vertex EdgeBegin(EdgeID e) const {
        return edge_begins_[e];
    }

    Vertex EdgeEnd(EdgeID e) const override {
        return edge_ends_[e];
    }

    EdgeData GetEdgeData(EdgeID edge_id) const override {
        return EdgeData{edge_begins_[edge_id], edge_ends_[edge_id]};
    }

    size_t Degree(Vertex v) const {
        return OutgoingEdgesByReference(v).size();
    }

private:
    AdjLists adj_lists_;
    std::vector<Vertex> edge_begins_;
    std::vector<Vertex> edge_ends_;
};

using Weight = Weight;

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

    Weight EdgeWeight(EdgeID e) const override {
        return edge_weights_[e];
    }

    Weight& EdgeWeight(EdgeID e) {
        return edge_weights_[e];
    }

private:
    std::vector<Weight> edge_weights_;
};

using Distance = Weight;
static constexpr Distance kInfDistance = INT32_MAX;

namespace impl {

bool Relax(const IGraph& g, EdgeID e, std::vector<Distance>& dist) {
    auto[src, dst] = g.GetEdgeData(e);
    if (dist[src] == kInfDistance || dist[dst] <= dist[src] + g.EdgeWeight(e)) {
        return false;
    }
    dist[dst] = dist[src] + g.EdgeWeight(e);
    return true;
}

}  // namespace impl

namespace detail {

void FordBellman(const IGraph& g, Vertex start, std::vector<Weight>& dist) {
    dist.assign(g.NVertices(), kInfDistance);
    dist[start] = 0;
    for (size_t i = 0; i < g.NVertices() - 1; ++i) {
        bool any_relaxed = false;
        for (EdgeID e = 0; e < g.NEdges(); ++e) {
            any_relaxed |= impl::Relax(g, e, dist);
        }
        if (!any_relaxed) {
            return;
        }
    }
}

struct WeightedEdgeData {
    Weight weight;
    Vertex dst;
    Vertex src;

    bool operator>(const WeightedEdgeData& rhs) const {
        return std::make_pair(weight, dst) > std::make_pair(rhs.weight, rhs.dst);
    }
};

void Dijkstra(const IGraph& g, Vertex source, std::vector<Distance>& dist) {
    dist.assign(g.NVertices(), kInfDistance);
    std::priority_queue<WeightedEdgeData, std::vector<WeightedEdgeData>, std::greater<WeightedEdgeData>> queue;
    std::vector<bool> proc(g.NVertices(), false);

    dist[source] = 0;
    queue.push({0, source, kInfDistance});
    while (!queue.empty()) {
        auto curr = queue.top().dst;
        queue.pop();
        if (proc[curr]) {
            continue;
        }
        proc[curr] = true;
        for (auto edge_id : g.OutgoingEdgesByReference(curr)) {
            auto next = g.EdgeEnd(edge_id);
            auto weight = g.EdgeWeight(edge_id);
            if (dist[curr] + weight < dist[next]) {
                dist[next] = dist[curr] + weight;
                queue.push({dist[next], next, curr});
            }
        }
    }
}

enum SingleSourceAlgorithm { SS_FORD_BELLMAN, SS_DIJKSTRA };

}  // namespace detail

decltype(auto) FindShortestPaths(const IGraph& g, Vertex source,
                                 detail::SingleSourceAlgorithm algorithm = detail::SS_DIJKSTRA) {
    std::vector<Distance> dist;
    switch (algorithm) {
        case detail::SS_FORD_BELLMAN:
            detail::FordBellman(g, source, dist);
            break;
        case detail::SS_DIJKSTRA:
            detail::Dijkstra(g, source, dist);
            break;
        default:
            throw NotImplementedError("FindShortestPaths: invalid algorithm");
    }
    return dist;
}

}  // namespace graph

using std::cin;
using std::cout;

using Price = graph::Weight;
using graph::WeightedGraph;

static const size_t kNFloors = 1000000;

int main() {
    std::ios_base::sync_with_stdio(false);
    cin.tie(nullptr);

    size_t target_floor = 0;
    size_t n_elevators = 0;
    Price upstairs_price = 0;
    Price downstairs_price = 0;
    Price into_elev_price = 0;
    Price out_of_elev_price = 0;

    cin >> target_floor >> upstairs_price >> downstairs_price >> into_elev_price >> out_of_elev_price;
    cin >> n_elevators;
    --target_floor;

    auto g = WeightedGraph(kNFloors + n_elevators, true);
    std::set<size_t> used_floors;

    for (size_t elev = 0; elev < n_elevators; ++elev) {
        size_t n_stops = 0;
        cin >> n_stops;
        for (size_t j = 0; j < n_stops; ++j) {
            size_t floor = 0;
            cin >> floor;
            --floor;
            used_floors.insert(floor);
            g.AddEdge(floor, kNFloors + elev, into_elev_price);
            g.AddEdge(kNFloors + elev, floor, out_of_elev_price);
        }
    }

    used_floors.insert(target_floor);
    used_floors.insert(0);

    auto curr = used_floors.begin();
    auto prev = curr;
    while (++curr != used_floors.end()) {
        auto diff = static_cast<Price>(*curr - *prev);
        g.AddEdge(*prev, *curr, diff * upstairs_price);
        g.AddEdge(*curr, *prev, diff * downstairs_price);
        ++prev;
    }

    cout << graph::FindShortestPaths(g, 0)[target_floor];
}