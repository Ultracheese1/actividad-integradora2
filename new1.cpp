#include <iostream>
#include <vector>
#include <fstream>
#include <algorithm>
#include <limits>
#include <cmath>

using namespace std;

/* =====================  ESTRUCTURAS  ===================== */

struct Edge {
    int u, v;
    int weight;
};

struct Result {
    int cost;
    vector<int> path;
};

/* =====================  UNION-FIND  ===================== */

struct UnionFind {
    vector<int> parent;

    UnionFind(int n) {
        parent.resize(n);
        for (int i = 0; i < n; i++)
            parent[i] = i;
    }

    int find(int x) {
        if (parent[x] != x)
            parent[x] = find(parent[x]);
        return parent[x];
    }

    bool unite(int x, int y) {
        int rx = find(x);
        int ry = find(y);
        if (rx == ry) return false;
        parent[ry] = rx;
        return true;
    }
};

/* =====================  PASO 1: MST  ===================== */

vector<Edge> resolverMST(const vector<vector<int>>& mat) {
    int N = mat.size();
    vector<Edge> edges;

    for (int i = 0; i < N; i++)
        for (int j = i + 1; j < N; j++)
            if (mat[i][j] > 0)
                edges.push_back({i, j, mat[i][j]});

    sort(edges.begin(), edges.end(),
         [](const Edge& a, const Edge& b) {
             return a.weight < b.weight;
         });

    UnionFind uf(N);
    vector<Edge> mst;

    for (const Edge& e : edges) {
        if (uf.unite(e.u, e.v)) {
            mst.push_back(e);
            if (mst.size() == N - 1)
                break;
        }
    }

    return mst;
}

/* =====================  PASO 2: TSP  ===================== */

const int INF = 1e9;

int calcularCota(const vector<vector<int>>& mat,
                 const vector<bool>& visited,
                 int currentCity) {
    int n = mat.size();
    int bound = 0;

    int minEdge = INF;
    for (int j = 0; j < n; j++)
        if (mat[currentCity][j] > 0 && mat[currentCity][j] < minEdge)
            minEdge = mat[currentCity][j];
    bound += minEdge;

    for (int i = 0; i < n; i++) {
        if (!visited[i]) {
            int minOut = INF;
            for (int j = 0; j < n; j++)
                if (i != j && mat[i][j] > 0 && mat[i][j] < minOut)
                    minOut = mat[i][j];
            bound += minOut;
        }
    }

    return bound;
}

void branchAndBound(const vector<vector<int>>& mat,
                    vector<bool>& visited,
                    vector<int>& currentPath,
                    int currentCost,
                    int currentCity,
                    int level,
                    Result& best) {
    int n = mat.size();

    if (level == n) {
        int totalCost = currentCost + mat[currentCity][0];
        if (totalCost < best.cost) {
            best.cost = totalCost;
            best.path = currentPath;
        }
        return;
    }

    for (int next = 0; next < n; next++) {
        if (!visited[next] && mat[currentCity][next] > 0) {

            int newCost = currentCost + mat[currentCity][next];

            visited[next] = true;
            int bound = newCost + calcularCota(mat, visited, next);
            visited[next] = false;

            if (bound < best.cost) {
                visited[next] = true;
                currentPath[level] = next;

                branchAndBound(mat, visited, currentPath,
                               newCost, next, level + 1, best);

                visited[next] = false;
            }
        }
    }
}

Result resolverTSP(const vector<vector<int>>& mat) {
    int n = mat.size();
    Result best;
    best.cost = INF;
    best.path.resize(n);

    vector<bool> visited(n, false);
    vector<int> path(n);

    path[0] = 0;
    visited[0] = true;

    branchAndBound(mat, visited, path, 0, 0, 1, best);
    return best;
}

/* =====================  PASO 3: FLUJO MÁXIMO  ===================== */

bool dfs(int u, int t, vector<int>& parent,
         vector<vector<int>>& residual,
         vector<bool>& visited) {
    visited[u] = true;
    if (u == t) return true;

    for (int v = 0; v < residual.size(); v++) {
        if (!visited[v] && residual[u][v] > 0) {
            parent[v] = u;
            if (dfs(v, t, parent, residual, visited))
                return true;
        }
    }
    return false;
}

int fordFulkerson(vector<vector<int>> cap, int s, int t) {
    int n = cap.size();
    vector<vector<int>> residual = cap;
    vector<int> parent(n);
    int maxFlow = 0;

    while (true) {
        vector<bool> visited(n, false);
        fill(parent.begin(), parent.end(), -1);

        if (!dfs(s, t, parent, residual, visited))
            break;

        int pathFlow = INF;
        for (int v = t; v != s; v = parent[v]) {
            int u = parent[v];
            pathFlow = min(pathFlow, residual[u][v]);
        }

        for (int v = t; v != s; v = parent[v]) {
            int u = parent[v];
            residual[u][v] -= pathFlow;
            residual[v][u] += pathFlow;
        }

        maxFlow += pathFlow;
    }

    return maxFlow;
}

/* =====================  PASO 4: CENTRALES / POLÍGONOS  ===================== */

double distEuclidiana(double x1, double y1, double x2, double y2) {
    return sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
}

double distanciaCentralMasCercana(
    int idx,
    const vector<pair<double,double>>& centrales
) {
    double minDist = numeric_limits<double>::max();

    for (int j = 0; j < centrales.size(); j++) {
        if (j == idx) continue;

        double d = distEuclidiana(
            centrales[idx].first, centrales[idx].second,
            centrales[j].first, centrales[j].second
        );
        minDist = min(minDist, d);
    }

    return minDist;
}

vector<pair<double,double>> construirPoligono(
    int idx,
    const vector<pair<double,double>>& centrales
) {
    double cx = centrales[idx].first;
    double cy = centrales[idx].second;

    double radio = distanciaCentralMasCercana(idx, centrales) / 2.0;

    vector<pair<double,double>> poligono;
    poligono.push_back({cx - radio, cy - radio});
    poligono.push_back({cx + radio, cy - radio});
    poligono.push_back({cx + radio, cy + radio});
    poligono.push_back({cx - radio, cy + radio});

    return poligono;
}

/* =====================  MAIN  ===================== */

int main() {
    ifstream input("entrada.txt");
    if (!input) {
        cerr << "Error al abrir archivo.\n";
        return 1;
    }

    int N;
    input >> N;

    vector<vector<int>> dist(N, vector<int>(N));
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            input >> dist[i][j];

    vector<vector<int>> cap(N, vector<int>(N));
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            input >> cap[i][j];

    vector<pair<double,double>> centrales(N);
    for (int i = 0; i < N; i++) {
        char c;
        double x, y;
        input >> c >> x >> c >> y >> c;
        centrales[i] = {x, y};
    }

    input.close();

    cout << "Forma de cablear las colonias con fibra:\n";
    for (auto e : resolverMST(dist))
        cout << "(" << char('A'+e.u) << "," << char('A'+e.v) << ")\n";

    cout << "\nRuta del repartidor:\n";
    Result tsp = resolverTSP(dist);
    for (int i : tsp.path) cout << char('A'+i) << " -> ";
    cout << "A\nDistancia total: " << tsp.cost << "\n";

    cout << "\nFlujo máximo:\n";
    cout << fordFulkerson(cap, 0, N-1) << "\n";

    cout << "\nLista de polígonos (regiones de influencia):\n";
    for (int i = 0; i < centrales.size(); i++) {
        vector<pair<double,double>> poligono =
            construirPoligono(i, centrales);

        cout << "Poligono " << i+1 << ": ";
        for (auto& p : poligono) {
            cout << "(" << p.first << "," << p.second << ") ";
        }
        cout << "\n";
    }

    return 0;
}