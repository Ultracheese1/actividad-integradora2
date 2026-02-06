#include <bits/stdc++.h>
using namespace std;

/*========================
  ESTRUCTURAS
========================*/

struct Solution {
    vector<double> objectives; // [distanciaTotal, tramoMaximo]
};

/*========================
  DOMINANCIA DE PARETO
========================*/

bool domina(const Solution& a, const Solution& b) {
    bool mejoraAlguno = false;
    for (size_t i = 0; i < a.objectives.size(); i++) {
        if (a.objectives[i] > b.objectives[i])
            return false;
        if (a.objectives[i] < b.objectives[i])
            mejoraAlguno = true;
    }
    return mejoraAlguno;
}

vector<Solution> obtenerFrentePareto(const vector<Solution>& sols) {
    vector<Solution> frente;
    for (size_t i = 0; i < sols.size(); i++) {
        bool dominada = false;
        for (size_t j = 0; j < sols.size(); j++) {
            if (i != j && domina(sols[j], sols[i])) {
                dominada = true;
                break;
            }
        }
        if (!dominada)
            frente.push_back(sols[i]);
    }
    return frente;
}

/*========================
  EVALUAR RUTA TSP
========================*/

bool evaluarRuta(const vector<int>& ruta,
                 const vector<vector<double>>& dist,
                 double& total,
                 double& maxTramo) {
    total = 0.0;
    maxTramo = 0.0;

    for (size_t i = 0; i < ruta.size() - 1; i++) {
        double d = dist[ruta[i]][ruta[i+1]];
        if (d == 0) return false; // ruta inválida
        total += d;
        maxTramo = max(maxTramo, d);
    }

    return true;
}

/*========================
  MAIN
========================*/

int main() {
    int N;
    cin >> N;

    vector<vector<double>> dist(N, vector<double>(N));
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            cin >> dist[i][j];

    vector<int> ciudades;
    for (int i = 1; i < N; i++)
        ciudades.push_back(i);

    vector<Solution> soluciones;

    // Permutaciones de ciudades intermedias
    do {
        vector<int> ruta;
        ruta.push_back(0); // A
        for (int c : ciudades)
            ruta.push_back(c);
        ruta.push_back(0); // regreso a A

        double total, maxTramo;
        if (evaluarRuta(ruta, dist, total, maxTramo)) {
            Solution s;
            s.objectives = { total, maxTramo };
            soluciones.push_back(s);
        }

    } while (next_permutation(ciudades.begin(), ciudades.end()));

    auto frente = obtenerFrentePareto(soluciones);

    // Salida
    cout << "Frente de Pareto:\n";
    cout << "# DistanciaTotal TramoMaximo\n";
    for (const auto& s : frente) {
        cout << s.objectives[0] << " " << s.objectives[1] << "\n";
    }

    return 0;
}