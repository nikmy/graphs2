#include <iostream>
#include <vector>
#include <unordered_set>
#include <stdexcept>
#include <cstring>

namespace graph {
using Vertex = int64_t;
using Edge = int64_t;

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
Flow FordFulkersonDFS(const NetworkGraph& net, Vertex v, Flow path_flow, Vertex sink, std::vector<bool>& visited) {
    if (v == sink) {
        return path_flow;
    }
    visited[v] = true;
    for (auto e : net.OutgoingEdgesByReference(v)) {
        if (visited[net.EdgeEnd(e)] || net.EdgeFlow(e) == net.EdgeCap(e)) {
            continue;
        }
        path_flow = std::min(path_flow, net.EdgeCap(e) - net.EdgeFlow(e));
        auto delta = FordFulkersonDFS(net, net.EdgeEnd(e), path_flow, sink, visited);
        if (delta > 0) {
            net.EdgeFlow(e) += delta;
            net.BackFlow(e) -= delta;
            return delta;
        }
    }
    return 0;
}
}  // namespace impl

Flow FindMaxFlow(const NetworkGraph& net, Vertex source, Vertex sink) {
    Flow iter_result = 0;
    do {
        std::vector<bool> visited(net.NVertices(), false);
        iter_result = impl::FordFulkersonDFS(net, source, INT64_MAX, sink, visited);
    } while (iter_result > 0);

    Flow max_flow = 0;
    for (auto e : net.OutgoingEdgesByReference(source)) {
        max_flow += net.EdgeFlow(e);
    }
    return max_flow;
}

}  // namespace graph

using std::cin;
using std::cout;

using graph::Flow;
using graph::NetworkGraph;
using graph::Vertex;

int main() {
    std::ios_base::sync_with_stdio(false);
    cin.tie(nullptr);

    size_t n_vertices = 0;
    size_t n_edges = 0;
    cin >> n_vertices >> n_edges;
    auto g = NetworkGraph(n_vertices);
    for (size_t i = 0; i < n_edges; ++i) {
        Vertex from = 0;
        Vertex to = 0;
        Flow cap = 0;
        cin >> from >> to >> cap;
        g.AddEdge(from - 1, to - 1, cap);
    }
    cout << graph::FindMaxFlow(g, 0, static_cast<Vertex>(n_vertices) - 1) << '\n';
}