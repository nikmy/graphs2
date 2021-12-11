#include <iostream>
#include <vector>
#include <unordered_set>
#include <stdexcept>
#include <cstring>

#include <queue>

namespace graph {
using Vertex = int64_t;
using EdgeID = int64_t;

static const Vertex kUnreachable = INT64_MIN;

class NotImplementedError : public std::runtime_error {
public:
    explicit NotImplementedError(const char* method_name)
        : std::runtime_error((std::string("NotImplementedError: ") + method_name).c_str()) {
    }
};

using Weight = int64_t;

class IGraph {
public:
    virtual void AddEdge(Vertex from, Vertex to, Weight = 0) = 0;
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

    virtual Weight EdgeWeight(EdgeID) const {
        throw NotImplementedError("EdgeWeight");
    };
};

class Graph : public IGraph {
    using EdgeList = std::vector<EdgeID>;
    using AdjLists = std::vector<EdgeList>;

public:
    Graph(size_t n_vertices, bool is_directed)
        : n_vertices_(n_vertices), n_edges_(0), is_directed_(is_directed), adj_lists_(n_vertices), edge_ends_() {
    }

    size_t NVertices() const override {
        return n_vertices_;
    }

    size_t NEdges() const override {
        return n_edges_;
    }

    void AddEdge(Vertex from, Vertex to, Weight = 0) override {
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

class FastNeighborsGraph : public Graph {
    using NeighborsList = std::vector<Vertex>;
    using NeighborsData = std::vector<NeighborsList>;

public:
    FastNeighborsGraph(size_t n_vertices, bool is_directed)
        : Graph(n_vertices, is_directed), neighbors_data_(n_vertices) {
    }

    void AddEdge(Vertex from, Vertex to, Weight = 0) override {
        Graph::AddEdge(from, to);
        neighbors_data_[from].push_back(to);
        if (!is_directed_) {
            neighbors_data_[to].push_back(from);
        }
    }

    Vertex AddVertex() override {
        auto new_vertex = Graph::AddVertex();
        neighbors_data_.emplace_back();
        return new_vertex;
    }

    std::vector<Vertex> Neighbors(Vertex vertex) const override {
        return neighbors_data_[vertex];
    }

    const std::vector<Vertex>& NeighborsByReference(Vertex vertex) const override {
        return neighbors_data_[vertex];
    }

private:
    NeighborsData neighbors_data_;
};

class WeightedGraph : public FastNeighborsGraph {
public:
    WeightedGraph(size_t n_vertices, bool is_directed) : FastNeighborsGraph(n_vertices, is_directed) {
    }

    void AddEdge(Vertex from, Vertex to, Weight weight) override {
        FastNeighborsGraph::AddEdge(from, to);
        edge_weights_.push_back(weight);
        if (!is_directed_) {
            edge_weights_.push_back(weight);
        }
    }

    Weight EdgeWeight(EdgeID edge_id) const override {
        return edge_weights_[edge_id];
    }

private:
    std::vector<Weight> edge_weights_;
};

struct WeightedEdgeData {
    Weight weight;
    Vertex dst;
    Vertex src;

    bool operator>(const WeightedEdgeData& rhs) const {
        return std::make_pair(weight, dst) > std::make_pair(rhs.weight, rhs.dst);
    }
};

struct MSTData {
    std::vector<WeightedEdgeData> edges;
    Weight weight;
};

decltype(auto) DirectedMSTPrim(const IGraph& g) {
    Weight mst_weight = 0;
    std::priority_queue<WeightedEdgeData, std::vector<WeightedEdgeData>, std::greater<WeightedEdgeData>> queue;
    std::vector<bool> used(g.NVertices(), false);
    queue.push({0, 0, kUnreachable});

    std::vector<WeightedEdgeData> mst;

    while (!queue.empty()) {
        auto[weight, current, prev] = queue.top();
        queue.pop();
        if (used[current]) {
            continue;
        }

        used[current] = true;
        if (prev != kUnreachable) {
            mst.push_back({weight, current, prev});
            mst_weight += weight;
        }

        for (auto edge_id : g.OutgoingEdgesByReference(current)) {
            auto next = g.EdgeEnd(edge_id);
            if (!used[next]) {
                queue.push({g.EdgeWeight(edge_id), next, current});
            }
        }
    }

    return MSTData{mst, mst_weight};
}

Weight DirectedMSTWeightPrim(const IGraph& g) {
    return DirectedMSTPrim(g).weight;
}

}  // namespace graph

using std::cin;
using std::cout;

int main() {
    std::ios_base::sync_with_stdio(false);
    cin.tie(nullptr);

    graph::Vertex n_spies = 0;
    cin >> n_spies;

    std::vector<std::vector<graph::Weight>> weights(n_spies);
    for (graph::Vertex from = 0; from < n_spies; ++from) {
        weights[from].resize(n_spies);
        for (graph::Vertex to = 0; to < n_spies; ++to) {
            cin >> weights[from][to];
        }
    }

    // One vertex for mission
    auto g = graph::WeightedGraph(n_spies + 1, false);
    for (graph::Vertex from = 0; from < n_spies; ++from) {
        for (graph::Vertex to = from + 1; to < n_spies; ++to) {
            g.AddEdge(from, to, weights[from][to]);
        }
        graph::Weight spy_price = 0;
        cin >> spy_price;
        g.AddEdge(from, n_spies, spy_price);
    }

    cout << graph::DirectedMSTWeightPrim(g) << '\n';
}