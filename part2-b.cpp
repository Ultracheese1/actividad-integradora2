#include <iostream>
#include <vector>
#include <algorithm>
#include <fstream>
#include <limits>

using namespace std;

const int INF = 1e9;

/* =====================  ESTRUCTURAS  ===================== */

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

/* =====================  UNION-FIND  ===================== */

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

/* =====================  KRUSKAL ε-CONSTRAINT (2.1) ===================== */

SolucionMST kruskalConLimite(int N, const vector<Edge>& edges, int maxTramo) {
    vector<Edge> candidatas;
    for (const auto& e : edges)
        if (e.w <= maxTramo)
            candidatas.push_back(e);

    sort(candidatas.begin(), candidatas.end(),
         [](auto& a, auto& b) { return a.w < b.w; });

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

/* =====================  TSP ε-CONSTRAINT (2.2) ===================== */

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

/* =====================  DOMINANCIA PARETO TSP ===================== */

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


/* =====================  MAIN ===================== */

int main() {
    ifstream input("entrada.txt");
    int N;
    input >> N;

    vector<vector<int>> mat(N, vector<int>(N));
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            input >> mat[i][j];

    vector<vector<int>> dummy(N, vector<int>(N));
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            input >> dummy[i][j];

    char c; double x, y;
    for (int i = 0; i < N; i++)
        input >> c >> x >> c >> y >> c;

    input.close();

    /* =====================  TSP MULTIOBJETIVO ===================== */

    vector<SolucionTSP> candidatas;

    for (int eps = 15; eps <= 40; eps += 2) {
        vector<bool> visitado(N, false);
        vector<int> ruta = {0};
        visitado[0] = true;

        SolucionTSP mejor;
        mejor.distanciaTotal = INF;
        mejor.saltoMaximo = 0;

        backtrackingTSP(mat, eps, visitado, ruta, 1, 0, 0, mejor);

        if (mejor.distanciaTotal < INF)
            candidatas.push_back(mejor);
    }

    auto unicas = eliminarDuplicadosTSP(candidatas);
    auto pareto = filtrarParetoTSP(unicas);

    cout << "\nFrente de Pareto - Ruta de Reparto\n\n";

    int id = 1;
    for (auto& s : pareto) {
        cout << "Solucion " << id++ << ":\n";
        cout << "  Distancia total = " << s.distanciaTotal << "\n";
        cout << "  Salto maximo    = " << s.saltoMaximo << "\n";
        cout << "  Ruta: ";
        for (int v : s.ruta)
            cout << char('A' + v) << " -> ";
        cout << "A\n\n";
    }

    return 0;
}
