#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <climits>
#include <cmath>

using namespace std;

/* ================= ESTRUCTURAS ================= */

struct Point {
    double x, y;
};

/* ================= LECTURA ================= */

vector<vector<int>> readDistanceMatrix(ifstream& input, int& N) {
    input >> N;
    vector<vector<int>> dist(N, vector<int>(N));
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            input >> dist[i][j];
    return dist;
}

vector<vector<int>> readCapacityMatrix(ifstream& input, int N) {
    vector<vector<int>> cap(N, vector<int>(N));
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            input >> cap[i][j];
    return cap;
}

vector<Point> readCentralCoordinates(ifstream& input, int N) {
    vector<Point> centers(N);
    char ch;
    for (int i = 0; i < N; i++) {
        input >> ch; // (
        input >> centers[i].x;
        input >> ch; // ,
        input >> centers[i].y;
        input >> ch; // )
    }
    return centers;
}

/* ================= MST (PRIM) ================= */

vector<int> computeMST(const vector<vector<int>>& dist) {
    int N = dist.size();
    vector<int> key(N, INT_MAX), parent(N, -1);
    vector<bool> inMST(N, false);

    key[0] = 0;

    for (int i = 0; i < N - 1; i++) {
        int u = -1, minKey = INT_MAX;
        for (int v = 0; v < N; v++)
            if (!inMST[v] && key[v] < minKey)
                minKey = key[v], u = v;

        inMST[u] = true;

        for (int v = 0; v < N; v++)
            if (!inMST[v] && dist[u][v] > 0 && dist[u][v] < key[v])
                key[v] = dist[u][v], parent[v] = u;
    }
    return parent;
}

void printMST(const vector<int>& parent) {
    cout << "Forma de cablear las colonias con fibra:\n";
    for (int i = 1; i < parent.size(); i++)
        cout << "(" << char('A' + parent[i]) << ", "
             << char('A' + i) << ")\n";
}

/* ================= TSP ================= */

vector<int> computeTSPRoute(const vector<vector<int>>& dist) {
    int N = dist.size();
    vector<bool> visited(N, false);
    vector<int> route;

    int current = 0;
    visited[current] = true;
    route.push_back(current);

    for (int i = 1; i < N; i++) {
        int next = -1, best = INT_MAX;
        for (int v = 0; v < N; v++)
            if (!visited[v] && dist[current][v] < best)
                best = dist[current][v], next = v;

        visited[next] = true;
        route.push_back(next);
        current = next;
    }

    route.push_back(0);
    return route;
}

void printTSPRoute(const vector<int>& route) {
    cout << "\nRuta de reparto:\n";
    for (int i = 0; i < route.size(); i++) {
        cout << char('A' + route[i]);
        if (i + 1 < route.size()) cout << " -> ";
    }
    cout << "\n";
}

/* ================= FLUJO MÁXIMO (DINIC) ================= */

struct Edge {
    int to, cap, rev;
};

class Dinic {
    vector<vector<Edge>> g;
    vector<int> level, it;

public:
    Dinic(int n) : g(n), level(n), it(n) {}

    void addEdge(int u, int v, int c) {
        g[u].push_back({v, c, (int)g[v].size()});
        g[v].push_back({u, 0, (int)g[u].size() - 1});
    }

    bool bfs(int s, int t) {
        fill(level.begin(), level.end(), -1);
        queue<int> q;
        level[s] = 0;
        q.push(s);

        while (!q.empty()) {
            int u = q.front(); q.pop();
            for (auto& e : g[u])
                if (level[e.to] < 0 && e.cap > 0)
                    level[e.to] = level[u] + 1, q.push(e.to);
        }
        return level[t] >= 0;
    }

    int dfs(int u, int t, int f) {
        if (!f || u == t) return f;
        for (int& i = it[u]; i < g[u].size(); i++) {
            Edge& e = g[u][i];
            if (level[e.to] == level[u] + 1 && e.cap > 0) {
                int pushed = dfs(e.to, t, min(f, e.cap));
                if (pushed) {
                    e.cap -= pushed;
                    g[e.to][e.rev].cap += pushed;
                    return pushed;
                }
            }
        }
        return 0;
    }

    int maxFlow(int s, int t) {
        int flow = 0;
        while (bfs(s, t)) {
            fill(it.begin(), it.end(), 0);
            while (int pushed = dfs(s, t, INT_MAX))
                flow += pushed;
        }
        return flow;
    }
};

int computeMaxFlow(const vector<vector<int>>& cap) {
    int N = cap.size();
    Dinic d(N);
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            if (cap[i][j] > 0)
                d.addEdge(i, j, cap[i][j]);
    return d.maxFlow(0, N - 1);
}

/* ================= CENTRALES / REGIONES ================= */

double distance(const Point& a, const Point& b) {
    return hypot(a.x - b.x, a.y - b.y);
}

vector<vector<Point>> computeRegions(const vector<Point>& centers) {
    int N = centers.size();
    vector<vector<Point>> regions(N);

    // 1. Bounding box
    double minX = centers[0].x, maxX = centers[0].x;
    double minY = centers[0].y, maxY = centers[0].y;

    for (const auto& p : centers) {
        minX = min(minX, p.x);
        maxX = max(maxX, p.x);
        minY = min(minY, p.y);
        maxY = max(maxY, p.y);
    }

    // 2. Paso del grid (ajustable)
    double step = 50.0;

    // 3. Muestreo del plano
    for (double x = minX; x <= maxX; x += step) {
        for (double y = minY; y <= maxY; y += step) {

            Point sample{x, y};

            double best = 1e18;
            int closest = -1;

            for (int i = 0; i < N; i++) {
                double d = distance(sample, centers[i]);
                if (d < best) {
                    best = d;
                    closest = i;
                }
            }

            regions[closest].push_back(sample);
        }
    }

    return regions;
}

void printRegions(const vector<vector<Point>>& regions) {
    cout << "\nLista de polígonos:\n";
    for (const auto& region : regions) {
        cout << "[";
        for (int i = 0; i < region.size(); i++) {
            cout << "(" << region[i].x << ", " << region[i].y << ")";
            if (i + 1 < region.size()) cout << ", ";
        }
        cout << "]\n";
    }
}

/* ================= MAIN ================= */

int main() {
    ifstream input("entrada.txt");
    if (!input.is_open()) {
        cerr << "Error al abrir entrada.txt\n";
        return 1;
    }

    int N;
    auto dist = readDistanceMatrix(input, N);
    auto cap = readCapacityMatrix(input, N);
    auto centers = readCentralCoordinates(input, N);

    printMST(computeMST(dist));
    printTSPRoute(computeTSPRoute(dist));

    cout << "\nValor de flujo máximo de información del nodo inicial al nodo final:\n";
    cout << computeMaxFlow(cap) << "\n";

    printRegions(computeRegions(centers));

    input.close();
    return 0;
}
