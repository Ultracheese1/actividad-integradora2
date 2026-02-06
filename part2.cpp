#include <iostream>
#include <vector>
using namespace std;

/* ================== SOLUCIÓN MULTIOBJETIVO ================== */

struct Solution {
    vector<double> objectives; // todos se minimizan
};

/* ================== DOMINANCIA DE PARETO ================== */

bool domina(const Solution& a, const Solution& b) {
    bool estrictamenteMejor = false;

    for (size_t i = 0; i < a.objectives.size(); i++) {
        if (a.objectives[i] > b.objectives[i])
            return false;
        if (a.objectives[i] < b.objectives[i])
            estrictamenteMejor = true;
    }
    return estrictamenteMejor;
}

vector<Solution> obtenerFrentePareto(const vector<Solution>& soluciones) {
    vector<Solution> frente;

    for (size_t i = 0; i < soluciones.size(); i++) {
        bool dominada = false;

        for (size_t j = 0; j < soluciones.size(); j++) {
            if (i != j && domina(soluciones[j], soluciones[i])) {
                dominada = true;
                break;
            }
        }

        if (!dominada)
            frente.push_back(soluciones[i]);
    }
    return frente;
}

/* ================== MAIN (TEMPORAL) ================== */

int main() {
    // Este main solo es de prueba por ahora
    // Se reemplazará en los siguientes pasos

    vector<Solution> soluciones = {
        {{120, 40}},
        {{130, 30}},
        {{140, 20}},
        {{150, 50}}
    };

    vector<Solution> frente = obtenerFrentePareto(soluciones);

    cout << "Frente de Pareto:\n";
    for (auto& s : frente) {
        cout << "(";
        for (double v : s.objectives)
            cout << v << " ";
        cout << ")\n";
    }

    return 0;
}
