#include <iostream>
#include <vector>

namespace graph {

using Vertex = size_t;
using Distance = int64_t;
using AdjMatrix = std::vector<std::vector<Distance>>;

static const Distance kInfDistance = INT64_MAX;

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
    virtual const AdjMatrix& GetMatrix() const {
        throw NotImplementedError("GetMatrixByReference");
    }
};

class AdjMatrixGraph : IGraph {
public:
    AdjMatrixGraph(size_t n_vertices, bool is_oriented)
        : matrix_(n_vertices, std::vector<Distance>(n_vertices, kInfDistance)), is_oriented_(is_oriented) {
        for (size_t i = 0; i < n_vertices; ++i) {
            matrix_[i][i] = 0;
        }
    }

    void AddEdge(Vertex from, Vertex to, Weight weight) override {
        matrix_[from][to] = weight;
        if (!is_oriented_) {
            matrix_[to][from] = weight;
        }
    }

    const AdjMatrix& GetMatrix() const override {
        return matrix_;
    }

private:
    AdjMatrix matrix_;
    bool is_oriented_;
};

namespace impl {

void FloydWarshallOptimization(AdjMatrix& dist) {
    size_t n = dist.size();
    for (Vertex k = 0; k < n; ++k) {
        for (Vertex u = 0; u < n; ++u) {
            for (Vertex v = 0; v < n; ++v) {
                dist[u][v] = std::min(dist[u][v], dist[u][k] + dist[k][v]);
            }
        }
    }
}

}  // namespace impl

AdjMatrix GetShortestDistancesMatrix(const AdjMatrixGraph& g) {
    AdjMatrix dist = g.GetMatrix();
    impl::FloydWarshallOptimization(dist);
    return dist;
}

}  // namespace graph

using std::cin;
using std::cout;

using graph::AdjMatrix;
using graph::Distance;
using graph::Vertex;

int main() {
    std::ios_base::sync_with_stdio(false);
    cin.tie(nullptr);

    size_t n_vertices = 0;
    cin >> n_vertices;

    graph::AdjMatrixGraph g(n_vertices, true);

    for (Vertex from = 0; from < n_vertices; ++from) {
        for (Vertex to = 0; to < n_vertices; ++to) {
            graph::Weight weight = 0;
            cin >> weight;
            g.AddEdge(from, to, weight);
        }
    }

    auto dist = graph::GetShortestDistancesMatrix(g);
    for (Vertex u = 0; u < n_vertices; ++u) {
        for (Vertex v = 0; v < n_vertices; ++v) {
            cout << dist[u][v] << ' ';
        }
        cout << '\n';
    }
}