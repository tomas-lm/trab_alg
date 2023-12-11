import networkx as nx
import time
import itertools
import math
import random
from utils import *

def find_minimum_weight_perfect_matching(G, odd_vertices):
    # Cria um novo grafo onde cada aresta é o custo da aresta original
    g = nx.Graph()
    for u, v in itertools.combinations(odd_vertices, 2):
        g.add_edge(u, v, weight=nx.shortest_path_length(G, u, v, weight='weight'))

    # Encontrar o emparelhamento mínimo perfeito
    # Como networkx não tem uma função integrada para isto, usaremos uma abordagem aproximada
    matching = nx.algorithms.max_weight_matching(g, maxcardinality=True)

    return matching

def christofides_algorithm(graph):
    # Encontrar a árvore geradora mínima
    mst = nx.minimum_spanning_tree(graph, weight='weight')

    # Encontrar todos os vértices de grau ímpar na árvore geradora mínima
    odd_degree_nodes = [v for v, d in mst.degree() if d % 2 == 1]

    # Encontrar um emparelhamento mínimo perfeito no subgrafo induzido pelos vértices de grau ímpar
    matching = find_minimum_weight_perfect_matching(graph, odd_degree_nodes)

    # Adicionar as arestas do emparelhamento à árvore geradora mínima
    mst.add_edges_from(matching)

    # Encontrar um ciclo Euleriano no multigrafo
    eulerian_circuit = list(nx.eulerian_circuit(mst))

    # Converter o ciclo Euleriano em um ciclo Hamiltoniano
    visited = set()
    hamiltonian_path = []
    for u, v in eulerian_circuit:
        if u not in visited:
            hamiltonian_path.append(u)
            visited.add(u)
    hamiltonian_path.append(hamiltonian_path[0])

    return hamiltonian_path

# Criar um grafo ponderado aleatório
G = nx.complete_graph(10)
for (u, v, w) in G.edges(data=True):
    w['weight'] = math.ceil(10 * random.uniform(0, 1))

# Medir o tempo de início
start_time = time.time()

# Aplicar o Algoritmo de Christofides
cycle = christofides_algorithm(G)

# Medir o tempo de fim
end_time = time.time()

# Calcular e imprimir o tempo total
total_time = end_time - start_time
print("Ciclo Hamiltoniano Aproximado:", cycle)
print("Tempo de Execução:", total_time, "segundos")

# Mostrar o grafo e o ciclo Hamiltoniano aproximado
pos = nx.spring_layout(G)
nx.draw(G, pos, with_labels=True, node_color='lightblue', edge_color='gray')
nx.draw_networkx_edges(G, pos, edgelist=[(cycle[i], cycle[i+1]) for i in range(len(cycle)-1)], width=2, edge_color='red')
plt.title("Christofides Algorithm")
plt.show()
