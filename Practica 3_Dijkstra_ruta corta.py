"""Dafne Villanueva 21310176.
Evaluación de Vulnerabilidades:Este script proporcionará una representación visual clara de la red y la ruta más corta calculada entre
el nodo de origen y el nodo de destino usando el algoritmo de Dijkstra."""

import heapq  # Para implementar la cola de prioridad
import matplotlib.pyplot as plt  # Para la visualización gráfica
import networkx as nx  # Para crear y manipular el grafo

# Función para implementar el algoritmo de Dijkstra
def dijkstra(graph, start, end):
    priority_queue = [(0, start)]  # Cola de prioridad para almacenar las distancias y nodos
    shortest_paths = {vertex: (float('infinity'), []) for vertex in graph}  # Diccionario de distancias más cortas
    shortest_paths[start] = (0, [start])  # Distancia al nodo inicial es 0 y la ruta es solo el nodo inicial

    while priority_queue:
        current_distance, current_vertex = heapq.heappop(priority_queue)  # Nodo con la menor distancia

        if current_vertex == end:
            break  # Si hemos llegado al destino, terminamos

        if current_distance > shortest_paths[current_vertex][0]:
            continue  # Saltar si encontramos un camino más corto anteriormente

        for neighbor, weight in graph[current_vertex].items():  # Iterar sobre los vecinos
            distance = current_distance + weight  # Calcular la nueva distancia

            if distance < shortest_paths[neighbor][0]:  # Si la nueva distancia es menor
                shortest_paths[neighbor] = (distance, shortest_paths[current_vertex][1] + [neighbor])  # Actualizar la distancia y la ruta
                heapq.heappush(priority_queue, (distance, neighbor))  # Añadir a la cola de prioridad

    return shortest_paths[end]

# Grafo representando una red con nodos y pesos
graph = {
    'A': {'B': 1, 'C': 4},
    'B': {'A': 1, 'C': 2, 'D': 6},
    'C': {'A': 4, 'B': 2, 'D': 3},
    'D': {'B': 6, 'C': 3}
}

# Calcular la ruta más corta desde el nodo 'A' hasta el nodo 'D'
shortest_distance, shortest_path = dijkstra(graph, 'A', 'D')
print(f"Distancia más corta desde A hasta D: {shortest_distance}")
print(f"Ruta más corta desde A hasta D: {shortest_path}")

# Visualizar la red usando matplotlib y networkx
G = nx.Graph()  # Crear un grafo vacío

# Añadir nodos y arcos al grafo
for node in graph:
    for neighbor, weight in graph[node].items():
        G.add_edge(node, neighbor, weight=weight)

# Posiciones de los nodos para la visualización
pos = nx.spring_layout(G)

# Dibujar el grafo
nx.draw(G, pos, with_labels=True, node_color='lightblue', node_size=500, font_size=10)
labels = nx.get_edge_attributes(G, 'weight')
nx.draw_networkx_edge_labels(G, pos, edge_labels=labels)

# Resaltar la ruta más corta desde el nodo 'A' hasta 'D'
path_edges = list(zip(shortest_path, shortest_path[1:]))
nx.draw_networkx_edges(G, pos, edgelist=path_edges, edge_color='r', width=2)

# Mostrar el gráfico
plt.title("Ruta Más Corta desde el Nodo A hasta el Nodo D")
plt.show()
