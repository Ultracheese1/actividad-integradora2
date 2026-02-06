#include <iostream>
#include <vector>
#include <algorithm>
#include <fstream>
#include <queue>
#include <limits>
#include <cmath>

using namespace std;

const int INF = 1e9;

/* =========================================================
   =====================  ESTRUCTURAS  =====================
   ========================================================= */

struct Edge {
    int u, v, w;
};

struct SolucionMST {
    int distanciaTotal;
    int tramoMaximo;
    vector<Edge> edges;
};

struct SolucionTSP {
    int distanciaTotal;
    int saltoMaximo;
    vector<int> ruta;
};

struct Punto {
    double x, y;
};

struct SolucionCentrales {
    double distPromedio;
    double distMaxima;
    vector<int> centralesActivas;
};

/* =========================================================
   =====================  UNION FIND  ======================
   ========================================================= */

struct UnionFind {
    vector<int> parent;

    UnionFind(int n) : parent(n) {
        for (int i = 0; i < n; i++)
            parent[i] = i;
    }

    int find(int x) {
        if (parent[x] != x)
            parent[x] = find(parent[x]);
        return parent[x];
    }

    bool unite(int a, int b) {
        int ra = find(a);
        int rb = find(b);
        if (ra == rb) return false;
        parent[rb] = ra;
        return true;
    }
};

/* =========================================================
   =====================  PARTE 2.1 ========================
   ========================================================= */

SolucionMST kruskalConLimite(int N, const vector<Edge>& edges, int maxTramo) {
    vector<Edge> candidatas;
    for (const auto& e : edges)
        if (e.w <= maxTramo)
            candidatas.push_back(e);

    sort(candidatas.begin(), candidatas.end(),
         [](const Edge& a, const Edge& b) {
             return a.w < b.w;
         });

    UnionFind uf(N);
    SolucionMST sol{0, 0, {}};

    for (const auto& e : candidatas) {
        if (uf.unite(e.u, e.v)) {
            sol.edges.push_back(e);
            sol.distanciaTotal += e.w;
            sol.tramoMaximo = max(sol.tramoMaximo, e.w);
        }
    }

    if (sol.edges.size() != N - 1)
        sol.distanciaTotal = INF;

    return sol;
}

bool dominaMST(const SolucionMST& a, const SolucionMST& b) {
    return (a.distanciaTotal <= b.distanciaTotal &&
            a.tramoMaximo <= b.tramoMaximo &&
           (a.distanciaTotal < b.distanciaTotal ||
            a.tramoMaximo < b.tramoMaximo));
}

vector<SolucionMST> filtrarParetoMST(vector<SolucionMST>& sols) {
    vector<SolucionMST> pareto;
    for (auto& s : sols) {
        bool dominada = false;
        for (auto& p : pareto)
            if (dominaMST(p, s)) {
                dominada = true;
                break;
            }
        if (!dominada)
            pareto.push_back(s);
    }
    return pareto;
}

vector<SolucionMST> eliminarDuplicadosMST(const vector<SolucionMST>& sols) {
    vector<SolucionMST> unicas;
    for (const auto& s : sols) {
        bool repetida = false;
        for (const auto& u : unicas)
            if (s.distanciaTotal == u.distanciaTotal &&
                s.tramoMaximo == u.tramoMaximo)
                repetida = true;
        if (!repetida) unicas.push_back(s);
    }
    return unicas;
}

/* =========================================================
   =====================  PARTE 2.2 ========================
   ========================================================= */

void backtrackingTSP(
    const vector<vector<int>>& mat,
    int eps,
    vector<bool>& visitado,
    vector<int>& rutaActual,
    int nivel,
    int costoActual,
    int saltoMax,
    SolucionTSP& mejor
) {
    int n = mat.size();

    if (nivel == n) {
        int regreso = mat[rutaActual.back()][0];
        if (regreso > eps) return;

        int total = costoActual + regreso;
        int saltoFinal = max(saltoMax, regreso);

        if (total < mejor.distanciaTotal) {
            mejor.distanciaTotal = total;
            mejor.saltoMaximo = saltoFinal;
            mejor.ruta = rutaActual;
        }
        return;
    }

    for (int i = 1; i < n; i++) {
        if (!visitado[i]) {
            int w = mat[rutaActual.back()][i];
            if (w == 0 || w > eps) continue;

            int nuevoCosto = costoActual + w;
            if (nuevoCosto >= mejor.distanciaTotal) continue;

            visitado[i] = true;
            rutaActual.push_back(i);

            backtrackingTSP(mat, eps, visitado, rutaActual,
                            nivel + 1, nuevoCosto,
                            max(saltoMax, w), mejor);

            rutaActual.pop_back();
            visitado[i] = false;
        }
    }
}

bool dominaTSP(const SolucionTSP& a, const SolucionTSP& b) {
    return (a.distanciaTotal <= b.distanciaTotal &&
            a.saltoMaximo <= b.saltoMaximo &&
           (a.distanciaTotal < b.distanciaTotal ||
            a.saltoMaximo < b.saltoMaximo));
}

vector<SolucionTSP> filtrarParetoTSP(vector<SolucionTSP>& sols) {
    vector<SolucionTSP> pareto;
    for (auto& s : sols) {
        bool dominada = false;
        for (auto& p : pareto)
            if (dominaTSP(p, s)) {
                dominada = true;
                break;
            }
        if (!dominada)
            pareto.push_back(s);
    }
    return pareto;
}

/* =========================================================
   =====================  PARTE 2.3 ========================
   ========================================================= */

int maxFlowEdmondsKarp(int N, vector<vector<int>> cap, int s, int t) {
    int flujoMax = 0;

    while (true) {
        vector<int> parent(N, -1);
        queue<int> q;
        q.push(s);
        parent[s] = s;

        while (!q.empty() && parent[t] == -1) {
            int u = q.front(); q.pop();
            for (int v = 0; v < N; v++) {
                if (parent[v] == -1 && cap[u][v] > 0) {
                    parent[v] = u;
                    q.push(v);
                }
            }
        }

        if (parent[t] == -1) break;

        int aumento = INF;
        for (int v = t; v != s; v = parent[v])
            aumento = min(aumento, cap[parent[v]][v]);

        for (int v = t; v != s; v = parent[v]) {
            cap[parent[v]][v] -= aumento;
            cap[v][parent[v]] += aumento;
        }

        flujoMax += aumento;
    }

    return flujoMax;
}

/* =========================================================
   =====================  PARTE 2.4 ========================
   ====== CENTRALES Y CERCANÍA GEOGRÁFICA (PARETO) =========
   ========================================================= */

double distancia(const Punto& a, const Punto& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return sqrt(dx * dx + dy * dy);
}

SolucionCentrales evaluarCentrales(
    const vector<Punto>& colonias,
    const vector<Punto>& centrales,
    const vector<int>& activas
) {
    double suma = 0.0;
    double maxDist = 0.0;

    for (const auto& col : colonias) {
        double mejor = 1e18;
        for (int idx : activas)
            mejor = min(mejor, distancia(col, centrales[idx]));

        suma += mejor;
        maxDist = max(maxDist, mejor);
    }

    return {
        suma / colonias.size(),
        maxDist,
        activas
    };
}

bool dominaCentrales(
    const SolucionCentrales& a,
    const SolucionCentrales& b
) {
    return (a.distPromedio <= b.distPromedio &&
            a.distMaxima  <= b.distMaxima &&
           (a.distPromedio < b.distPromedio ||
            a.distMaxima  < b.distMaxima));
}

/* =========================================================
   ========================== MAIN =========================
   ========================================================= */

int main() {
    ifstream input("entrada.txt");

    int N;
    input >> N;

    vector<vector<int>> mat(N, vector<int>(N));
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            input >> mat[i][j];

    vector<vector<int>> cap(N, vector<int>(N));
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            input >> cap[i][j];

    vector<Punto> colonias(N);
    char c;
    for (int i = 0; i < N; i++)
        input >> c >> colonias[i].x >> c >> colonias[i].y >> c;

    int M;
    input >> M;

    vector<Punto> centrales(M);
    for (int i = 0; i < M; i++)
        input >> centrales[i].x >> centrales[i].y;

    input.close();

    /* =====================  2.4 OUTPUT ===================== */

    vector<SolucionCentrales> soluciones;

    for (int mask = 1; mask < (1 << M); mask++) {
        vector<int> activas;
        for (int i = 0; i < M; i++)
            if (mask & (1 << i))
                activas.push_back(i);

        soluciones.push_back(
            evaluarCentrales(colonias, centrales, activas)
        );
    }

    vector<SolucionCentrales> pareto;
    for (auto& s : soluciones) {
        bool dominada = false;
        for (auto& p : pareto)
            if (dominaCentrales(p, s))
                dominada = true;
        if (!dominada) pareto.push_back(s);
    }

    cout << "\nFrente de Pareto - Centrales (promedio maximo)\n";
    for (auto& s : pareto)
        cout << s.distPromedio << " " << s.distMaxima << "\n";

    return 0;
}