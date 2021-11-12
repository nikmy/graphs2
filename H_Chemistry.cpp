#include <iostream>
#include <vector>
#include <unordered_set>
#include <unordered_map>
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

    size_t Degree(Vertex v) const {
        return OutgoingEdgesByReference(v).size();
    }

private:
    AdjLists adj_lists_;
    std::vector<Vertex> edge_begins_;
    std::vector<Vertex> edge_ends_;
};

using Flow = EdgeProperty;

class NetworkGraph : public AdjListsGraph {
public:
    explicit NetworkGraph(size_t n_vertices) : AdjListsGraph(n_vertices, false) {
    }

    void AddEdge(Vertex from, Vertex to, Flow cap) override {
        AdjListsGraph::AddEdge(from, to);
        edge_caps_.push_back(cap);
        edge_caps_.push_back(0);
        edge_flows_.push_back(0);
        edge_flows_.push_back(0);
    }

    Flow EdgeCap(Edge e) const {
        return edge_caps_[e];
    }

    Flow& EdgeFlow(Edge e) const {
        return edge_flows_[e];
    }

    Flow& BackFlow(Edge e) const {
        return edge_flows_[e ^ 1];
    }

private:
    std::vector<Flow> edge_caps_;
    mutable std::vector<Flow> edge_flows_;
};

namespace impl {

using Distance = int32_t;
static constexpr Distance kInfDistance = INT32_MAX;

bool MinPathCheck(const NetworkGraph& net, Vertex start, Vertex finish, std::vector<Distance>& dist) {
    dist.assign(net.NVertices(), kInfDistance);
    std::queue<Vertex> queue;

    dist[start] = 0;
    queue.emplace(start);
    while (!queue.empty()) {
        auto v = queue.front();
        queue.pop();
        for (auto e : net.OutgoingEdgesByReference(v)) {
            auto u = net.EdgeEnd(e);
            if (net.EdgeFlow(e) < net.EdgeCap(e) && dist[u] == kInfDistance) {
                dist[u] = dist[v] + 1;
                queue.push(net.EdgeEnd(e));
            }
        }
    }
    return dist[finish] < kInfDistance;
}

Flow FindBlockingFlowDFS(const NetworkGraph& net, Vertex v, Flow way_min, Vertex sink, std::vector<size_t>& next,
                         std::vector<Distance>& dist) {
    if (v == sink || way_min == 0) {
        return way_min;
    }
    for (size_t i = next[v]; i < net.Degree(v); ++i, ++next[v]) {
        auto e = net.OutgoingEdgesByReference(v)[i];
        auto u = net.EdgeEnd(e);
        if (dist[u] != dist[v] + 1) {
            continue;
        }
        Flow subway_min = std::min(way_min, net.EdgeCap(e) - net.EdgeFlow(e));
        auto delta = FindBlockingFlowDFS(net, u, subway_min, sink, next, dist);
        if (delta != 0) {
            net.EdgeFlow(e) += delta;
            net.BackFlow(e) -= delta;
            return delta;
        }
    }
    return 0;
}

Flow DinicMaxFlow(const NetworkGraph& net, Vertex source, Vertex sink) {
    std::vector<size_t> next(net.NVertices(), 0);
    std::vector<Distance> dist;
    Flow max_flow = 0;
    while (MinPathCheck(net, source, sink, dist)) {
        next.assign(net.NVertices(), 0);
        Flow flow = 0;
        do {
            flow = FindBlockingFlowDFS(net, source, kMaxEdgePropertyValue, sink, next, dist);
            max_flow += flow;
        } while (flow != 0);
    }
    return max_flow;
}

}  // namespace impl

Flow FindMaxFlow(const NetworkGraph& net, Vertex source, Vertex sink) {
    return impl::DinicMaxFlow(net, source, sink);
}

}  // namespace graph

template <class T>
class PairEncoder {
public:
    using Value = T;
    using Pair = std::pair<Value, Value>;

    explicit PairEncoder(Value key) : key_(key) {
    }

    Value Encode(Value first, Value second) const {
        return first * key_ + second;
    }

    Pair Decode(Value encoded) const {
        if (key_ == 0) {
            return std::make_pair(0, 0);
        }
        return std::make_pair(encoded / key_, encoded % key_);
    }

private:
    Value key_;
};

using std::cin;
using std::cout;

using graph::Flow;
using graph::NetworkGraph;
using graph::Vertex;

int main() {
    std::ios_base::sync_with_stdio(false);
    cin.tie(nullptr);

    std::unordered_map<char, Flow> valences = {{'.', 0}, {'H', 1}, {'O', 2}, {'N', 3}, {'C', 4}};

    Vertex n_lines = 0;
    Vertex n_rows = 0;
    cin >> n_lines >> n_rows;

    auto net = NetworkGraph(n_lines * n_rows + 2);
    auto enc = PairEncoder<Vertex>(n_rows);

    Vertex source = static_cast<Vertex>(net.NVertices()) - 2;
    Vertex sink = static_cast<Vertex>(net.NVertices()) - 1;
    Flow flows[3] = {0};

    for (Vertex i = 0; i < n_lines; ++i) {
        for (Vertex j = 0; j < n_rows; ++j) {
            char elem = '^';  // sad smile: he is disappointed with the Google code style
            cin >> elem;

            auto val = valences[elem];

            flows[(i + j) % 2] += val;
            auto cell = enc.Encode(i, j);

            if ((i + j) % 2 == 0) {
                net.AddEdge(source, cell, val);

                auto left = enc.Encode(i, j - 1);
                auto right = enc.Encode(i, j + 1);
                auto top = enc.Encode(i - 1, j);
                auto bottom = enc.Encode(i + 1, j);

                if (j > 0) {
                    net.AddEdge(cell, left, 1);
                }
                if (j < n_rows - 1) {
                    net.AddEdge(cell, right, 1);
                }
                if (i > 0) {
                    net.AddEdge(cell, top, 1);
                }
                if (i < n_lines - 1) {
                    net.AddEdge(cell, bottom, 1);
                }
            } else {
                net.AddEdge(cell, sink, val);
            }
        }
    }

    flows[2] = graph::FindMaxFlow(net, source, sink);
    if (flows[0] == flows[1] && flows[1] == flows[2] && flows[2]) {
        cout << "Valid\n";
    } else {
        cout << "Invalid\n";
    }
}