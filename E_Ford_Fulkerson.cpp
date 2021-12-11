#include <iostream>
#include <vector>
#include <stdexcept>
#include <cstring>

namespace graph {
using Vertex = int64_t;
using EdgeID = int64_t;

using EdgeProperty = int64_t;

class NotImplementedError : public std::runtime_error {
public:
    explicit NotImplementedError(const char* method_name)
        : std::runtime_error((std::string("NotImplementedError: ") + method_name).c_str()) {
    }
};

class IGraph {
public:
    virtual void AddEdge(Vertex from, Vertex to, EdgeProperty = 0) = 0;
    virtual Vertex AddVertex() = 0;

    virtual std::vector<Vertex> Neighbors(Vertex v) const = 0;

    virtual const std::vector<Vertex>& NeighborsByReference(Vertex) const {
        throw NotImplementedError("NeighborsByReference");
    }

    virtual std::vector<EdgeID> OutgoingEdges(Vertex) const = 0;

    virtual const std::vector<EdgeID>& OutgoingEdgesByReference(Vertex) const {
        throw NotImplementedError("NeighborsByReference");
    }

    virtual size_t NVertices() const = 0;
    virtual size_t NEdges() const = 0;

    virtual Vertex EdgeEnd(EdgeID) const = 0;

    virtual EdgeProperty EdgeEdgeProperty(EdgeID) const {
        throw NotImplementedError("EdgeEdgeProperty");
    };
};

class AdjListsGraph : public IGraph {
    using EdgeList = std::vector<EdgeID>;
    using AdjLists = std::vector<EdgeList>;

public:
    AdjListsGraph(size_t n_vertices, bool is_directed)
        : n_vertices_(n_vertices), n_edges_(0), is_directed_(is_directed), adj_lists_(n_vertices), edge_ends_() {
    }

    size_t NVertices() const override {
        return n_vertices_;
    }

    size_t NEdges() const override {
        return n_edges_;
    }

    void AddEdge(Vertex from, Vertex to, EdgeProperty = 0) override {
        adj_lists_[from].push_back(static_cast<EdgeID>(n_edges_++));
        edge_ends_.emplace_back(to);
        if (!is_directed_) {
            adj_lists_[to].push_back(static_cast<EdgeID>(n_edges_++));
            edge_ends_.push_back(from);
        }
    }

    Vertex AddVertex() override {
        auto new_vertex = static_cast<Vertex>(n_vertices_++);
        adj_lists_.emplace_back();
        return new_vertex;
    }

    std::vector<Vertex> Neighbors(Vertex vertex) const override {
        std::vector<Vertex> neighbors;
        for (auto edge_id : adj_lists_[vertex]) {
            neighbors.push_back(edge_ends_[edge_id]);
        }
        return neighbors;
    }

    std::vector<EdgeID> OutgoingEdges(Vertex vertex) const override {
        return adj_lists_[vertex];
    }

    const std::vector<EdgeID>& OutgoingEdgesByReference(Vertex vertex) const override {
        return adj_lists_[vertex];
    }

    EdgeID EdgeEnd(EdgeID edge_id) const override {
        return edge_ends_[edge_id];
    }

protected:
    size_t n_vertices_;
    size_t n_edges_;
    bool is_directed_;

private:
    AdjLists adj_lists_;
    std::vector<Vertex> edge_ends_;
};

using Flow = EdgeProperty;

class INetwork {
public:
    virtual EdgeID EdgeEnd(EdgeID) const = 0;
    virtual Flow EdgeCap(EdgeID) const = 0;
    virtual Flow& EdgeFlow(EdgeID) const = 0;
    virtual Flow& BackFlow(EdgeID) const = 0;
    virtual const std::vector<EdgeID>& OutgoingEdgesByReference(Vertex vertex) const = 0;
    virtual size_t NVertices() const = 0;
};

class NetworkGraph : public AdjListsGraph, public INetwork {
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

    Flow EdgeCap(EdgeID edge_id) const override {
        return edge_caps_[edge_id];
    }

    Flow& EdgeFlow(EdgeID edge_id) const override {
        return edge_flows_[edge_id];
    }

    Flow& BackFlow(EdgeID edge_id) const override {
        return edge_flows_[edge_id ^ 1];
    }

    const std::vector<EdgeID>& OutgoingEdgesByReference(Vertex vertex) const override {
        return AdjListsGraph::OutgoingEdgesByReference(vertex);
    }

    EdgeID EdgeEnd(EdgeID edge_id) const override {
        return AdjListsGraph::EdgeEnd(edge_id);
    }

    size_t NVertices() const override {
        return AdjListsGraph::NVertices();
    }

private:
    std::vector<Flow> edge_caps_;
    mutable std::vector<Flow> edge_flows_;
};

namespace impl {
Flow FordFulkersonDFS(const INetwork& net, Vertex v, Flow path_flow, Vertex sink, std::vector<bool>& visited) {
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

Flow FindMaxFlow(const INetwork& net, Vertex source, Vertex sink) {
    Flow delta_flow = 0;
    do {
        std::vector<bool> visited(net.NVertices(), false);
        delta_flow = impl::FordFulkersonDFS(net, source, INT64_MAX, sink, visited);
    } while (delta_flow > 0);

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