import networkx as nx
import matplotlib.pyplot as plt
import time

def twice_around_the_tree(graph):
    # Encontrar a árvore geradora mínima do grafo
    mst = nx.minimum_spanning_tree(graph)

    # Duplicar as arestas da árvore geradora mínima
    doubled_edges = list(mst.edges()) + list(mst.edges())

    # Criar um multigrafo com as arestas duplicadas
    multigraph = nx.MultiGraph()
    multigraph.add_edges_from(doubled_edges)

    # Encontrar um ciclo Euleriano no multigrafo (que percorre cada aresta exatamente uma vez)
    eulerian_cycle = list(nx.eulerian_circuit(multigraph))

    # Converter o ciclo Euleriano em um ciclo Hamiltoniano (visitando cada vértice uma vez)
    visited = set()
    hamiltonian_cycle = []
    for u, v in eulerian_cycle:
        if u not in visited:
            hamiltonian_cycle.append(u)
            visited.add(u)
    hamiltonian_cycle.append(hamiltonian_cycle[0])  # Retornar ao ponto inicial

    return hamiltonian_cycle

# Criar um grafo aleatório
G = nx.gnp_random_graph(10, 0.3, seed=42)

# Medir o tempo de início
start_time = time.time()

# Aplicar o algoritmo 'twice around the tree'
cycle = twice_around_the_tree(G)

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
plt.title("Twice Around the Tree Algorithm")
plt.show()
