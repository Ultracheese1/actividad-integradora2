import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ===============================
# Cableado
# ===============================
def plot_cableado():
    df = pd.read_csv("pareto_cableado.csv")
    plt.figure()
    plt.scatter(df["DistanciaTotal"], df["TramoMaximo"])
    plt.xlabel("Distancia Total")
    plt.ylabel("Tramo Máximo")
    plt.title("Frente de Pareto – Cableado")
    plt.grid(True)
    plt.show()

# ===============================
# Rutas de reparto (TSP)
# ===============================
def plot_tsp():
    df = pd.read_csv("pareto_tsp.csv")
    plt.figure()
    plt.scatter(df["DistanciaTotal"], df["TramoMaximo"])
    plt.xlabel("Distancia Total")
    plt.ylabel("Tramo Máximo")
    plt.title("Frente de Pareto – Rutas de Reparto")
    plt.grid(True)
    plt.show()

# ===============================
# Cableado + Flujo
# ===============================
def plot_network():
    df = pd.read_csv("pareto_network.csv")
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(
        df["DistanciaTotal"],
        df["TramoMaximo"],
        df["FlujoMaximo"]
    )
    ax.set_xlabel("Distancia Total")
    ax.set_ylabel("Tramo Máximo")
    ax.set_zlabel("Flujo Máximo")
    ax.set_title("Frente de Pareto – Cableado + Flujo")
    plt.show()

# ===============================
# Regiones de influencia
# ===============================
def plot_regions():
    df = pd.read_csv("pareto_regions.csv")
    plt.figure()
    plt.scatter(df["DistanciaPromedio"], df["DistanciaMaxima"])
    plt.xlabel("Distancia Promedio")
    plt.ylabel("Distancia Máxima")
    plt.title("Frente de Pareto – Regiones de Influencia")
    plt.grid(True)
    plt.show()

# ===============================
# MAIN
# ===============================
if __name__ == "__main__":
    plot_cableado()
    plot_tsp()
    plot_network()
    plot_regions()