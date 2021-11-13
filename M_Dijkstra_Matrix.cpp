#include <iostream>
#include <vector>
#include <unordered_set>
#include <stdexcept>

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

private:
    AdjLists adj_lists_;
    std::vector<Vertex> edge_begins_;
    std::vector<Vertex> edge_ends_;
};

using Weight = EdgeProperty;
using Distance = Weight;
static constexpr Distance kInfDistance = INT64_MAX;

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

Distance DijkstraMinDist(const WeightedGraph& g, Vertex from, Vertex to) {
    std::vector<Distance> dist(g.NVertices(), kInfDistance);
    std::vector<bool> proc(g.NVertices(), false);

    dist[from] = 0;

    for (size_t k = 0; k < g.NVertices(); ++k) {
        Vertex u = g.NVertices();
        Distance min_d = kInfDistance;
        for (Vertex i = 0; i < g.NVertices(); ++i) {
            if (!proc[i] && dist[i] < min_d) {
                u = i;
                min_d = dist[i];
            }
        }
        if (min_d == kInfDistance) {
            break;
        }
        proc[u] = true;
        for (auto e : g.OutgoingEdgesByReference(u)) {
            auto v = g.EdgeEnd(e);
            auto w = g.EdgeWeight(e);
            if (dist[u] + w < dist[v]) {
                dist[v] = dist[u] + w;
            }
        }
    }
    return dist[to];
}

}  // namespace impl

Distance FindShortestPath(const WeightedGraph& g, Vertex from, Vertex to) {
    return impl::DijkstraMinDist(g, from, to);
}

}  // namespace graph

using std::cin;
using std::cout;

using graph::Distance;
using graph::Vertex;
using graph::Weight;

int main() {
    std::ios_base::sync_with_stdio(false);
    cin.tie(nullptr);

    size_t n_vertices = 0;
    Vertex from = 0;
    Vertex to = 0;
    cin >> n_vertices >> from >> to;

    auto g = graph::WeightedGraph(n_vertices, true);
    for (size_t i = 0; i < n_vertices; ++i) {
        for (size_t j = 0; j < n_vertices; ++j) {
            Weight w = 0;
            cin >> w;
            if (i == j || w == -1) {
                continue;
            }
            g.AddEdge(i, j, w);
        }
    }
    auto d = graph::FindShortestPath(g, from - 1, to - 1);
    cout << (d == graph::kInfDistance ? -1 : d);
}