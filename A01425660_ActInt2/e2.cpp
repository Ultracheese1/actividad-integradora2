#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <algorithm>
#include <climits>
#include <cmath>

using namespace std;

/* ================= ESTRUCTURAS ================= */

struct Point {
    double x, y;
};

struct Edge {
    int u, v, w;
};

struct Solution {
    int totalDist;
    int maxEdge;
};

struct TSPSolution {
    vector<int> route;
    int totalDist;
    int maxEdge;
};

struct NetworkSolution {
    int totalDist;
    int maxEdge;
    int flow;
};

struct RegionSolution {
    double avgDist;
    double maxDist;
    vector<int> assignment; // colonia -> central
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
        input >> ch;
        input >> centers[i].x;
        input >> ch;
        input >> centers[i].y;
        input >> ch;
    }
    return centers;
}

/* ================= 1. MST (PRIM) ================= */

vector<int> computeMST(const vector<vector<int>>& dist) {
    int N = dist.size();
    vector<int> key(N, INT_MAX), parent(N, -1);
    vector<bool> inMST(N, false);
    key[0] = 0;

    for (int i = 0; i < N - 1; i++) {
        int u = -1;
        for (int v = 0; v < N; v++)
            if (!inMST[v] && (u == -1 || key[v] < key[u]))
                u = v;

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

/* ================= 2. TSP ================= */

vector<int> computeTSPRoute(const vector<vector<int>>& dist) {
    int N = dist.size();
    vector<bool> visited(N, false);
    vector<int> route;
    int cur = 0;
    visited[cur] = true;
    route.push_back(cur);

    for (int i = 1; i < N; i++) {
        int nxt = -1;
        for (int v = 0; v < N; v++)
            if (!visited[v] && (nxt == -1 || dist[cur][v] < dist[cur][nxt]))
                nxt = v;
        visited[nxt] = true;
        route.push_back(nxt);
        cur = nxt;
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

/* ================= 3. FLUJO MÁXIMO (DINIC) ================= */

struct FlowEdge {
    int to, cap, rev;
};

class Dinic {
    vector<vector<FlowEdge>> g;
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
            auto& e = g[u][i];
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

/* ================= 4. REGIONES ================= */

double distance(const Point& a, const Point& b) {
    return hypot(a.x - b.x, a.y - b.y);
}

vector<vector<Point>> computeRegions(const vector<Point>& centers) {
    int N = centers.size();
    vector<vector<Point>> regions(N);
    double step = 50.0;

    double minX = centers[0].x, maxX = centers[0].x;
    double minY = centers[0].y, maxY = centers[0].y;
    for (auto& p : centers) {
        minX = min(minX, p.x);
        maxX = max(maxX, p.x);
        minY = min(minY, p.y);
        maxY = max(maxY, p.y);
    }

    for (double x = minX; x <= maxX; x += step)
        for (double y = minY; y <= maxY; y += step) {
            int best = 0;
            double bestD = distance({x,y}, centers[0]);
            for (int i = 1; i < N; i++) {
                double d = distance({x,y}, centers[i]);
                if (d < bestD)
                    bestD = d, best = i;
            }
            regions[best].push_back({x,y});
        }
    return regions;
}

void printRegions(const vector<vector<Point>>& regions) {
    cout << "\nLista de polígonos:\n";
    for (auto& r : regions) {
        cout << "[";
        for (int i = 0; i < r.size(); i++) {
            cout << "(" << r[i].x << ", " << r[i].y << ")";
            if (i + 1 < r.size()) cout << ", ";
        }
        cout << "]\n";
    }
}
/* ================= PARTE 2 ================= */
/* ================= 1. Cableado ================= */

vector<Solution> computeParetoCableado(const vector<vector<int>>& dist) {
    int N = dist.size();
    vector<Edge> edges;

    for (int i = 0; i < N; i++)
        for (int j = i + 1; j < N; j++)
            edges.push_back({i, j, dist[i][j]});

    sort(edges.begin(), edges.end(),
         [](auto& a, auto& b){ return a.w < b.w; });

    vector<Solution> sols;

    for (int i = 0; i < edges.size(); i++) {
        int limit = edges[i].w;

        vector<vector<int>> g(N);
        for (auto& e : edges)
            if (e.w <= limit) {
                g[e.u].push_back(e.v);
                g[e.v].push_back(e.u);
            }

        vector<bool> vis(N,false);
        queue<int> q;
        q.push(0);
        vis[0] = true;
        int cnt = 1;

        while (!q.empty()) {
            int u = q.front(); q.pop();
            for (int v : g[u])
                if (!vis[v]) {
                    vis[v] = true;
                    q.push(v);
                    cnt++;
                }
        }
        if (cnt != N) continue;

        auto parent = computeMST(dist);
        int total = 0, mx = 0;
        for (int v = 1; v < N; v++) {
            int w = dist[v][parent[v]];
            total += w;
            mx = max(mx, w);
        }
        sols.push_back({total, mx});
    }

    vector<Solution> pareto;
    for (auto& s : sols) {
        bool dom = false;
        for (auto& t : sols)
            if (t.totalDist <= s.totalDist &&
                t.maxEdge <= s.maxEdge &&
                (t.totalDist < s.totalDist ||
                 t.maxEdge < s.maxEdge))
                dom = true;
        if (!dom) pareto.push_back(s);
    }

    return pareto;
}

vector<Solution> removeDuplicateSolutions(vector<Solution>& sols) {
    sort(sols.begin(), sols.end(),
         [](const Solution& a, const Solution& b) {
             if (a.totalDist != b.totalDist)
                 return a.totalDist < b.totalDist;
             return a.maxEdge < b.maxEdge;
         });

    vector<Solution> unique;
    for (const auto& s : sols) {
        if (unique.empty() ||
            unique.back().totalDist != s.totalDist ||
            unique.back().maxEdge != s.maxEdge) {
            unique.push_back(s);
        }
    }
    return unique;
}

void printPareto(const vector<Solution>& p) {
    cout << "\nFrente de Pareto (Cableado):\n";
    cout << "DistanciaTotal  TramoMaximo\n";
    for (auto& s : p)
        cout << s.totalDist << "              " << s.maxEdge << "\n";
}

/* ================= 2.Ruta de reparto físico ================= */

TSPSolution buildTSPSolution(
    const vector<vector<int>>& dist,
    int start
) {
    int N = dist.size();
    vector<bool> visited(N, false);
    vector<int> route;

    int cur = start;
    visited[cur] = true;
    route.push_back(cur);

    int total = 0;
    int mx = 0;

    for (int step = 1; step < N; step++) {
        int nxt = -1;
        for (int v = 0; v < N; v++) {
            if (!visited[v] &&
               (nxt == -1 || dist[cur][v] < dist[cur][nxt])) {
                nxt = v;
            }
        }
        visited[nxt] = true;
        route.push_back(nxt);
        total += dist[cur][nxt];
        mx = max(mx, dist[cur][nxt]);
        cur = nxt;
    }

    // regresar al origen
    total += dist[cur][start];
    mx = max(mx, dist[cur][start]);
    route.push_back(start);

    return {route, total, mx};
}

vector<TSPSolution> computeParetoTSP(
    const vector<vector<int>>& dist
) {
    int N = dist.size();
    vector<TSPSolution> sols;

    // generar rutas iniciando desde cada colonia
    for (int s = 0; s < N; s++)
        sols.push_back(buildTSPSolution(dist, s));

    // eliminar dominadas
    vector<TSPSolution> pareto;
    for (int i = 0; i < sols.size(); i++) {
        bool dominated = false;
        for (int j = 0; j < sols.size(); j++) {
            if (j == i) continue;
            if (sols[j].totalDist <= sols[i].totalDist &&
                sols[j].maxEdge <= sols[i].maxEdge &&
               (sols[j].totalDist < sols[i].totalDist ||
                sols[j].maxEdge < sols[i].maxEdge)) {
                dominated = true;
                break;
            }
        }
        if (!dominated)
            pareto.push_back(sols[i]);
    }

    return pareto;
}

vector<TSPSolution> removeDuplicateTSP(
    vector<TSPSolution>& sols
) {
    sort(sols.begin(), sols.end(),
        [](const TSPSolution& a, const TSPSolution& b) {
            if (a.totalDist != b.totalDist)
                return a.totalDist < b.totalDist;
            return a.maxEdge < b.maxEdge;
        });

    vector<TSPSolution> unique;
    for (auto& s : sols) {
        if (unique.empty() ||
            unique.back().totalDist != s.totalDist ||
            unique.back().maxEdge != s.maxEdge) {
            unique.push_back(s);
        }
    }
    return unique;
}

void printParetoTSP(const vector<TSPSolution>& sols) {
    cout << "\nFrente de Pareto (Rutas de reparto):\n";
    cout << "Ruta | DistanciaTotal | TramoMaximo\n";

    for (const auto& s : sols) {
        for (int i = 0; i < s.route.size(); i++) {
            cout << char('A' + s.route[i]);
            if (i + 1 < s.route.size()) cout << " -> ";
        }
        cout << " | " << s.totalDist
             << " | " << s.maxEdge << "\n";
    }
}

/* ================= 3. Flujo de información ================= */

vector<NetworkSolution> computeParetoNetwork(
    const vector<vector<int>>& dist,
    const vector<vector<int>>& cap
) {
    auto cablePareto = computeParetoCableado(dist);
    cablePareto = removeDuplicateSolutions(cablePareto);

    vector<NetworkSolution> sols;

    for (auto& s : cablePareto) {
        int flow = computeMaxFlow(cap);
        sols.push_back({s.totalDist, s.maxEdge, flow});
    }

    // eliminar dominadas (flujo se maximiza)
    vector<NetworkSolution> pareto;

    for (int i = 0; i < sols.size(); i++) {
        bool dominated = false;
        for (int j = 0; j < sols.size(); j++) {
            if (i == j) continue;

            bool betterOrEqual =
                sols[j].totalDist <= sols[i].totalDist &&
                sols[j].maxEdge <= sols[i].maxEdge &&
                sols[j].flow >= sols[i].flow;

            bool strictlyBetter =
                sols[j].totalDist < sols[i].totalDist ||
                sols[j].maxEdge < sols[i].maxEdge ||
                sols[j].flow > sols[i].flow;

            if (betterOrEqual && strictlyBetter) {
                dominated = true;
                break;
            }
        }
        if (!dominated)
            pareto.push_back(sols[i]);
    }

    return pareto;
}

void printParetoNetwork(const vector<NetworkSolution>& sols) {
    cout << "\nFrente de Pareto (Cableado + Flujo):\n";
    cout << "DistanciaTotal  TramoMaximo  FlujoMaximo\n";

    for (auto& s : sols)
        cout << s.totalDist << "              "
             << s.maxEdge << "            "
             << s.flow << "\n";
}

/* ================= 4. Centrales y cercanía geográfica ================= */

RegionSolution evaluateAssignment(
    const vector<Point>& colonies,
    const vector<Point>& centers,
    const vector<int>& activeCenters
) {
    int N = colonies.size();
    vector<int> assign(N);

    double sum = 0.0;
    double mx = 0.0;

    for (int i = 0; i < N; i++) {
        int best = activeCenters[0];
        double bestD = distance(colonies[i], centers[best]);

        for (int c : activeCenters) {
            double d = distance(colonies[i], centers[c]);
            if (d < bestD) {
                bestD = d;
                best = c;
            }
        }

        assign[i] = best;
        sum += bestD;
        mx = max(mx, bestD);
    }

    return {sum / N, mx, assign};
}

vector<RegionSolution> computeRegionSolutions(
    const vector<Point>& colonies,
    const vector<Point>& centers
) {
    int M = centers.size();
    vector<RegionSolution> sols;

    // probar cada central individual
    for (int i = 0; i < M; i++) {
        sols.push_back(
            evaluateAssignment(colonies, centers, {i})
        );
    }

    // probar pares de centrales
    for (int i = 0; i < M; i++)
        for (int j = i + 1; j < M; j++)
            sols.push_back(
                evaluateAssignment(colonies, centers, {i, j})
            );

    return sols;
}

vector<RegionSolution> computeParetoRegions(
    const vector<RegionSolution>& sols
) {
    vector<RegionSolution> pareto;

    for (int i = 0; i < sols.size(); i++) {
        bool dominated = false;
        for (int j = 0; j < sols.size(); j++) {
            if (i == j) continue;

            if (sols[j].avgDist <= sols[i].avgDist &&
                sols[j].maxDist <= sols[i].maxDist &&
               (sols[j].avgDist < sols[i].avgDist ||
                sols[j].maxDist < sols[i].maxDist)) {
                dominated = true;
                break;
            }
        }
        if (!dominated)
            pareto.push_back(sols[i]);
    }

    return pareto;
}

void printParetoRegions(const vector<RegionSolution>& sols) {
    cout << "\nFrente de Pareto (Regiones de influencia):\n";
    cout << "DistanciaPromedio  DistanciaMaxima\n";
    for (auto& s : sols)
        cout << s.avgDist << "               " << s.maxDist << "\n";
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
    auto pareto = computeParetoCableado(dist);
    pareto = removeDuplicateSolutions(pareto);
    printPareto(pareto);

    auto paretoTSP = computeParetoTSP(dist);
    paretoTSP = removeDuplicateTSP(paretoTSP);
    printParetoTSP(paretoTSP);

    auto paretoNetwork = computeParetoNetwork(dist, cap);
    printParetoNetwork(paretoNetwork);

    auto regionSols = computeRegionSolutions(centers, centers);
    auto paretoRegions = computeParetoRegions(regionSols);
    printParetoRegions(paretoRegions);

    input.close();
    return 0;
}