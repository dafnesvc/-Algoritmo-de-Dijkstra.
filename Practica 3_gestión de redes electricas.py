""""Dafne  Villanueva 21310176          
Algoritmo de Dijkstra en la gestión de redes eléctricas: Este script proporcionará una representación visual clara de la red eléctrica 
y las rutas más cortas calculadas desde un nodo específico usando el algoritmo de Dijkstra."""

import heapq  # Para implementar la cola de prioridad
import matplotlib.pyplot as plt  # Para la visualización gráfica de la red
import networkx as nx  # Para crear y manipular el grafo

# Función para implementar el algoritmo de Dijkstra
def dijkstra(graph, start):
    priority_queue = [(0, start)]  # Cola de prioridad para almacenar las distancias y nodos
    shortest_paths = {vertex: float('infinity') for vertex in graph}  # Diccionario de distancias más cortas
    shortest_paths[start] = 0  # Distancia al nodo inicial es 0

    while priority_queue:
        current_distance, current_vertex = heapq.heappop(priority_queue)  # Nodo con la menor distancia

        if current_distance > shortest_paths[current_vertex]:
            continue  # Saltar si encontramos un camino más corto anteriormente

        for neighbor, weight in graph[current_vertex].items():  # Iterar sobre los vecinos
            distance = current_distance + weight  # Calcular la nueva distancia

            if distance < shortest_paths[neighbor]:  # Si la nueva distancia es menor
                shortest_paths[neighbor] = distance  # Actualizar la distancia más corta
                heapq.heappush(priority_queue, (distance, neighbor))  # Añadir a la cola de prioridad

    return shortest_paths

# Grafo representando una red eléctrica con nodos y pesos
graph = {
    'A': {'B': 1, 'C': 4},
    'B': {'A': 1, 'C': 2, 'D': 6},
    'C': {'A': 4, 'B': 2, 'D': 3},
    'D': {'B': 6, 'C': 3}
}

# Calcular las distancias más cortas desde el nodo 'A'
shortest_paths = dijkstra(graph, 'A')
print("Distancias más cortas desde A:", shortest_paths)

# Visualizar la red eléctrica usando matplotlib y networkx
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

# Resaltar las rutas más cortas desde el nodo 'A'
for node, distance in shortest_paths.items():
    if node != 'A':
        path = nx.shortest_path(G, source='A', target=node, weight='weight')
        edges = list(zip(path, path[1:]))
        nx.draw_networkx_edges(G, pos, edgelist=edges, edge_color='r', width=2)

# Mostrar el gráfico
plt.title("Red Eléctrica y Rutas Más Cortas desde el Nodo A")
plt.show()
