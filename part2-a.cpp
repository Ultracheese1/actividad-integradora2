#include <iostream>
#include <vector>
#include <algorithm>
#include <fstream>
#include <limits>

using namespace std;

const int INF = 1e9;

/* =====================  ESTRUCTURAS  ===================== */

struct Edge {
    int u, v;
    int w;
};

struct SolucionMST {
    int distanciaTotal;
    int tramoMaximo;
    vector<Edge> edges;
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

/* =====================  KRUSKAL CON ε-CONSTRAINT  ===================== */

SolucionMST kruskalConLimite(
    int N,
    const vector<Edge>& edges,
    int maxTramo
) {
    vector<Edge> candidatas;
    for (const auto& e : edges) {
        if (e.w <= maxTramo)
            candidatas.push_back(e);
    }

    sort(candidatas.begin(), candidatas.end(),
         [](const Edge& a, const Edge& b) {
             return a.w < b.w;
         });

    UnionFind uf(N);
    SolucionMST sol;
    sol.distanciaTotal = 0;
    sol.tramoMaximo = 0;

    for (const auto& e : candidatas) {
        if (uf.unite(e.u, e.v)) {
            sol.edges.push_back(e);
            sol.distanciaTotal += e.w;
            sol.tramoMaximo = max(sol.tramoMaximo, e.w);
        }
    }

    if (sol.edges.size() != N - 1) {
        sol.distanciaTotal = INF; // solución inválida
    }

    return sol;
}

/* =====================  DOMINANCIA DE PARETO  ===================== */

bool domina(const SolucionMST& a, const SolucionMST& b) {
    return (a.distanciaTotal <= b.distanciaTotal &&
            a.tramoMaximo <= b.tramoMaximo &&
           (a.distanciaTotal < b.distanciaTotal ||
            a.tramoMaximo < b.tramoMaximo));
}

vector<SolucionMST> filtrarPareto(
    const vector<SolucionMST>& soluciones
) {
    vector<SolucionMST> pareto;

    for (const auto& s : soluciones) {
        bool dominada = false;
        for (const auto& p : pareto) {
            if (domina(p, s)) {
                dominada = true;
                break;
            }
        }
        if (!dominada)
            pareto.push_back(s);
    }

    return pareto;
}

vector<SolucionMST> eliminarDuplicados(
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

/* =====================  MAIN  ===================== */

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

    // La matriz de capacidades y las centrales NO se usan en este incremento
    vector<vector<int>> cap(N, vector<int>(N));
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            input >> cap[i][j];

    double dummyX, dummyY;
    char c;
    for (int i = 0; i < N; i++)
        input >> c >> dummyX >> c >> dummyY >> c;

    input.close();

    /* Construcción de aristas */
    vector<Edge> edges;
    for (int i = 0; i < N; i++) {
        for (int j = i + 1; j < N; j++) {
            if (mat[i][j] > 0)
                edges.push_back({i, j, mat[i][j]});
        }
    }

    /* Generación de soluciones */
    vector<SolucionMST> candidatas;
    for (int eps = 10; eps <= 50; eps += 2) {
        auto sol = kruskalConLimite(N, edges, eps);
        if (sol.distanciaTotal < INF)
            candidatas.push_back(sol);
    }

    auto unicas = eliminarDuplicados(candidatas);
    auto pareto = filtrarPareto(unicas);


    /* Salida */
    cout << "Frente de Pareto - Cableado de Fibra\n\n";

    int id = 1;
    for (const auto& sol : pareto) {
        cout << "Solucion " << id++ << ":\n";
        cout << "  Distancia total = " << sol.distanciaTotal << "\n";
        cout << "  Tramo maximo    = " << sol.tramoMaximo << "\n";
        cout << "  Aristas:\n";
        for (const auto& e : sol.edges) {
            cout << "    (" << char('A' + e.u)
                 << "," << char('A' + e.v) << ")\n";
        }
        cout << "\n";
    }

    return 0;
}