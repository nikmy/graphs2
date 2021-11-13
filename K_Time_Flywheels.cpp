#include <iostream>
#include <stdexcept>
#include <unordered_set>
#include <vector>

namespace graph {

using Vertex = int64_t;
using Edge = int64_t;
struct EdgeData {};

class NotImplementedError : public std::runtime_error {
public:
    explicit NotImplementedError(const char* method_name)
        : std::runtime_error((std::string("NotImplementedError: ") + method_name).c_str()) {
    }
};

class IGraph {
public:
    virtual void AddEdge(Vertex, Vertex, const EdgeData* data = nullptr) = 0;
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

    void AddEdge(Vertex from, Vertex to, const EdgeData* = nullptr) override {
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

private:
    AdjLists adj_lists_;
    std::vector<Vertex> edge_begins_;
    std::vector<Vertex> edge_ends_;
};

}  // namespace graph

namespace magic {

using Time = int64_t;
using Teleport = graph::Edge;
using Station = graph::Vertex;

static constexpr Time kUnreachableFuture = INT64_MAX;
static constexpr Time kUnreachablePast = INT64_MIN;

struct TeleportData : public graph::EdgeData {
    Time time_arrive;
    Time time_depart;
};

class TeleportsNetwork : private graph::AdjListsGraph {
public:
    explicit TeleportsNetwork(size_t n_tunnels) : AdjListsGraph(n_tunnels, true) {
    }

    void CreateTunnel(Station from, Station to, const TeleportData& data) {
        graph::AdjListsGraph::AddEdge(from, to);
        arrival_times_.push_back(data.time_arrive);
        departs_times_.push_back(data.time_depart);
    }

    decltype(auto) InAndOut(Teleport t) const {
        return AsPair(t);
    }

    Time DepartureTime(Teleport t) const {
        return departs_times_[t];
    }

    Time ArrivalTime(Teleport t) const {
        return arrival_times_[t];
    }

    size_t NStations() const {
        return NVertices();
    }

    size_t NTeleports() const {
        return NEdges();
    }

private:
    std::vector<Time> departs_times_;
    std::vector<Time> arrival_times_;
};

namespace impl {
bool Optimize(const TeleportsNetwork& net, Teleport t, std::vector<Time>& min_time) {
    auto[dep, arr] = net.InAndOut(t);
    if (min_time[dep] > net.DepartureTime(t) || min_time[arr] <= net.ArrivalTime(t)) {
        return false;
    }
    min_time[arr] = net.ArrivalTime(t);
    return true;
}
}  // namespace impl

Time MinArrivalTime(const TeleportsNetwork& net, Station from, Station to, Time now = 0) {
    std::vector<Time> min_time(net.NStations(), kUnreachableFuture);
    min_time[from] = now;

    for (size_t i = 0; i < net.NTeleports(); ++i) {
        for (Teleport t = 0; t < static_cast<Teleport>(net.NTeleports()); ++t) {
            impl::Optimize(net, t, min_time);
        }
    }
    return min_time[to];
}

}  // namespace magic

using std::cin;
using std::cout;

using magic::Station;
using magic::TeleportData;
using magic::TeleportsNetwork;

int main() {
    std::ios_base::sync_with_stdio(false);
    cin.tie(nullptr);

    size_t n_stations = 0;
    size_t n_teleports = 0;
    Station start = 0;
    Station target = 0;
    cin >> n_stations >> start >> target >> n_teleports;

    auto net = TeleportsNetwork(n_stations);
    for (size_t i = 0; i < n_teleports; ++i) {
        TeleportData data = {};
        Station from = 0;
        Station to = 0;
        cin >> from >> data.time_depart >> to >> data.time_arrive;
        net.CreateTunnel(from - 1, to - 1, data);
    }
    cout << magic::MinArrivalTime(net, start - 1, target - 1);
}