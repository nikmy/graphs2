#include <iostream>
#include <vector>

namespace graph {

using Vertex = size_t;
using Distance = int64_t;
using AdjMatrix = std::vector<std::vector<Distance>>;

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

    AdjMatrix dist(n_vertices, std::vector<Distance>(n_vertices));
    for (Vertex u = 0; u < n_vertices; ++u) {
        for (Vertex v = 0; v < n_vertices; ++v) {
            cin >> dist[u][v];
        }
    }

    graph::impl::FloydWarshallOptimization(dist);

    for (Vertex u = 0; u < n_vertices; ++u) {
        for (Vertex v = 0; v < n_vertices; ++v) {
            cout << dist[u][v] << ' ';
        }
        cout << '\n';
    }
}