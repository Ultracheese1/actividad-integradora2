#include <iostream>
#include <vector>
#include <algorithm>
#include <fstream>
#include <queue>
#include <limits>

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
   ============  MST MULTIOBJETIVO (KRUSKAL) ===============
   ========================================================= */

SolucionMST kruskalConLimite(
    int N,
    const vector<Edge>& edges,
    int maxTramo
) {
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

vector<SolucionMST> eliminarDuplicadosMST(
    const vector<SolucionMST>& sols
) {
    vector<SolucionMST> unicas;

    for (const auto& s : sols) {
        bool repetida = false;
        for (const auto& u : unicas) {
            if (s.distanciaTotal == u.distanciaTotal &&
                s.tramoMaximo == u.tramoMaximo) {
                repetida = true;
                break;
            }
        }
        if (!repetida)
            unicas.push_back(s);
    }
    return unicas;
}

/* =========================================================
   =====================  PARTE 2.2 ========================
   ============  TSP MULTIOBJETIVO (BACKTRACK) =============
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

            backtrackingTSP(
                mat, eps, visitado, rutaActual,
                nivel + 1, nuevoCosto,
                max(saltoMax, w), mejor
            );

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

bool rutasIguales(const vector<int>& a, const vector<int>& b) {
    if (a.size() != b.size()) return false;
    for (int i = 0; i < a.size(); i++)
        if (a[i] != b[i]) return false;
    return true;
}

vector<SolucionTSP> eliminarDuplicadosTSP(
    const vector<SolucionTSP>& sols
) {
    vector<SolucionTSP> unicas;

    for (const auto& s : sols) {
        bool repetida = false;
        for (const auto& u : unicas) {
            if (s.distanciaTotal == u.distanciaTotal &&
                s.saltoMaximo == u.saltoMaximo &&
                rutasIguales(s.ruta, u.ruta)) {
                repetida = true;
                break;
            }
        }
        if (!repetida)
            unicas.push_back(s);
    }
    return unicas;
}

/* =========================================================
   =====================  PARTE 2.3 ========================
   ===============  FLUJO MAXIMO (EK) ======================
   ========================================================= */

int maxFlowEdmondsKarp(
    int N,
    vector<vector<int>> cap,
    int s,
    int t
) {
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
   ========================== MAIN =========================
   ========================================================= */

int main() {
    ifstream input("entrada.txt");
    if (!input) {
        cerr << "Error al abrir archivo\n";
        return 1;
    }

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

    char c; double x, y;
    for (int i = 0; i < N; i++)
        input >> c >> x >> c >> y >> c;

    input.close();

    /* =====================  2.1 MST ===================== */

    vector<Edge> edges;
    for (int i = 0; i < N; i++)
        for (int j = i + 1; j < N; j++)
            if (mat[i][j] > 0)
                edges.push_back({i, j, mat[i][j]});

    vector<SolucionMST> mstCandidatas;
    for (int eps = 10; eps <= 50; eps += 2) {
        auto sol = kruskalConLimite(N, edges, eps);
        if (sol.distanciaTotal < INF)
            mstCandidatas.push_back(sol);
    }

    auto mstUnicas = eliminarDuplicadosMST(mstCandidatas);
    auto paretoMST = filtrarParetoMST(mstUnicas);


    cout << "Frente de Pareto - Cableado de Fibra\n\n";
    int id = 1;
    for (auto& s : paretoMST) {
        cout << "Solucion " << id++ << ":\n";
        cout << "  Distancia total = " << s.distanciaTotal << "\n";
        cout << "  Tramo maximo    = " << s.tramoMaximo << "\n\n";
    }

    /* =====================  2.2 TSP ===================== */

    vector<SolucionTSP> tspCandidatas;

    for (int eps = 15; eps <= 40; eps += 2) {
        vector<bool> visitado(N, false);
        vector<int> ruta = {0};
        visitado[0] = true;

        SolucionTSP mejor{INF, 0, {}};
        backtrackingTSP(mat, eps, visitado, ruta, 1, 0, 0, mejor);

        if (mejor.distanciaTotal < INF)
            tspCandidatas.push_back(mejor);
    }

    auto tspUnicas = eliminarDuplicadosTSP(tspCandidatas);
    auto paretoTSP = filtrarParetoTSP(tspUnicas);


    cout << "Frente de Pareto - Ruta de Reparto\n\n";
    id = 1;
    for (auto& s : paretoTSP) {
        cout << "Solucion " << id++ << ":\n";
        cout << "  Distancia total = " << s.distanciaTotal << "\n";
        cout << "  Salto maximo    = " << s.saltoMaximo << "\n";
        cout << "  Ruta: ";
        for (int v : s.ruta)
            cout << char('A' + v) << " -> ";
        cout << "A\n\n";
    }

    /* =====================  2.3 MAX FLOW ===================== */

    int flujoMax = maxFlowEdmondsKarp(N, cap, 0, N - 1);

    cout << "Flujo maximo de informacion\n";
    cout << "  Nodo inicial: A\n";
    cout << "  Nodo final:   " << char('A' + N - 1) << "\n";
    cout << "  Flujo maximo: " << flujoMax << "\n";

    return 0;
}
