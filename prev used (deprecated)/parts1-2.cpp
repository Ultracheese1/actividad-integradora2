#include <iostream>
#include <fstream>
#include <vector>
#include <climits>

using namespace std;

/* ================= LECTURA ================= */

vector<vector<int>> readDistanceMatrix(ifstream& input, int& N) {
    input >> N;
    vector<vector<int>> dist(N, vector<int>(N));

    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            input >> dist[i][j];
        }
    }
    return dist;
}

/* ================= 1. MST (PRIM) ================= */

vector<int> computeMST(const vector<vector<int>>& dist) {
    int N = dist.size();
    vector<int> key(N, INT_MAX);
    vector<int> parent(N, -1);
    vector<bool> inMST(N, false);

    key[0] = 0;

    for (int count = 0; count < N - 1; count++) {
        int u = -1;
        int minKey = INT_MAX;

        for (int v = 0; v < N; v++) {
            if (!inMST[v] && key[v] < minKey) {
                minKey = key[v];
                u = v;
            }
        }

        inMST[u] = true;

        for (int v = 0; v < N; v++) {
            if (!inMST[v] && dist[u][v] > 0 && dist[u][v] < key[v]) {
                key[v] = dist[u][v];
                parent[v] = u;
            }
        }
    }

    return parent;
}

void printMST(const vector<int>& parent) {
    cout << "Forma de cablear las colonias con fibra:\n";
    for (int i = 1; i < parent.size(); i++) {
        char from = 'A' + parent[i];
        char to   = 'A' + i;
        cout << "(" << from << ", " << to << ")\n";
    }
}

/* ================= 2. TSP (NEAREST NEIGHBOR) ================= */

vector<int> computeTSPRoute(const vector<vector<int>>& dist) {
    int N = dist.size();
    vector<bool> visited(N, false);
    vector<int> route;

    int current = 0;
    visited[current] = true;
    route.push_back(current);

    for (int step = 1; step < N; step++) {
        int next = -1;
        int minDist = INT_MAX;

        for (int v = 0; v < N; v++) {
            if (!visited[v] && dist[current][v] < minDist) {
                minDist = dist[current][v];
                next = v;
            }
        }

        visited[next] = true;
        route.push_back(next);
        current = next;
    }

    route.push_back(0); // regresar al origen
    return route;
}

void printTSPRoute(const vector<int>& route) {
    cout << "\nRuta de reparto:\n";
    for (int i = 0; i < route.size(); i++) {
        cout << char('A' + route[i]);
        if (i < route.size() - 1)
            cout << " -> ";
    }
    cout << "\n";
}

/* ================= MAIN ================= */

int main() {
    ifstream input("entrada.txt");
    if (!input.is_open()) {
        cerr << "Error: No se pudo abrir el archivo entrada.txt\n";
        return 1;
    }

    int N;
    vector<vector<int>> distanceMatrix = readDistanceMatrix(input, N);

    // Paso 1.1: MST
    vector<int> mstParent = computeMST(distanceMatrix);
    printMST(mstParent);

    // Paso 1.2: TSP
    vector<int> tspRoute = computeTSPRoute(distanceMatrix);
    printTSPRoute(tspRoute);

    input.close();
    return 0;
}
