"""Dafne Villanueva 21310176
Evaluación de Vulnerabilidades: Este script proporcionará una representación visual clara de la red y las rutas más cortas calculadas 
hacia el nodo crítico, lo cual es útil para evaluar las vulnerabilidades y fortalecer la seguridad en esos puntos."""

import heapq  # Para implementar la cola de prioridad
import matplotlib.pyplot as plt  # Para la visualización gráfica
import networkx as nx  # Para crear y manipular el grafo

# Función para implementar el algoritmo de Dijkstra
def dijkstra(graph, start):
    # Inicializar la cola de prioridad con el nodo de inicio y distancia 0
    priority_queue = [(0, start)]  # Cola de prioridad para almacenar las distancias y nodos
    # Inicializar las distancias más cortas a infinito para todos los nodos
    shortest_paths = {vertex: (float('infinity'), []) for vertex in graph}  # Diccionario de distancias más cortas
    shortest_paths[start] = (0, [start])  # Distancia al nodo inicial es 0 y la ruta es solo el nodo inicial

    while priority_queue:
        # Extraer el nodo con la menor distancia de la cola de prioridad
        current_distance, current_vertex = heapq.heappop(priority_queue)  # Nodo con la menor distancia

        # Si la distancia extraída es mayor que la almacenada, continuar
        if current_distance > shortest_paths[current_vertex][0]:
            continue  # Saltar si encontramos un camino más corto anteriormente

        # Iterar sobre los vecinos del nodo actual
        for neighbor, weight in graph[current_vertex].items():  # Iterar sobre los vecinos
            # Calcular la nueva distancia
            distance = current_distance + weight  # Calcular la nueva distancia

            # Si la nueva distancia es menor que la distancia almacenada
            if distance < shortest_paths[neighbor][0]:  # Si la nueva distancia es menor
                # Actualizar la distancia y la ruta más corta
                shortest_paths[neighbor] = (distance, shortest_paths[current_vertex][1] + [neighbor])  # Actualizar la distancia y la ruta
                # Añadir el vecino a la cola de prioridad
                heapq.heappush(priority_queue, (distance, neighbor))  # Añadir a la cola de prioridad

    return shortest_paths

# Grafo representando una red con nodos y pesos
graph = {
    'A': {'B': 1, 'C': 4, 'E': 7},
    'B': {'A': 1, 'C': 2, 'D': 6},
    'C': {'A': 4, 'B': 2, 'D': 3, 'E': 5},
    'D': {'B': 6, 'C': 3, 'E': 1},
    'E': {'A': 7, 'C': 5, 'D': 1}
}

# Nodo crítico al cual evaluamos la vulnerabilidad
critical_node = 'E'

# Calcular las rutas más cortas desde todos los nodos al nodo crítico
vulnerability_paths = {node: dijkstra(graph, node)[critical_node] for node in graph if node != critical_node}

# Imprimir las rutas más cortas y distancias al nodo crítico
print(f"Distancias y rutas más cortas al nodo crítico {critical_node}:")
for node, (distance, path) in vulnerability_paths.items():
    print(f"Desde {node} hasta {critical_node}: Distancia = {distance}, Ruta = {path}")

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

# Resaltar las rutas más cortas hacia el nodo crítico
for node, (distance, path) in vulnerability_paths.items():
    path_edges = list(zip(path, path[1:]))
    nx.draw_networkx_edges(G, pos, edgelist=path_edges, edge_color='r', width=2)

# Mostrar el gráfico
plt.title(f"Rutas Más Cortas hacia el Nodo Crítico {critical_node}")
plt.show()
 
#Este script proporciona una representación visual de las rutas más cortas en una red y destaca las rutas más
# vulnerables hacia un nodo crítico específico.