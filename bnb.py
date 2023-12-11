import networkx as nx
import queue
import math as m
from utils import *
import random

# Usado pro branch and bound
class Node:
    def __init__(self, bound, level, cost, sol):
        self.bound = bound
        self.level = level
        self.cost  = cost
        self.sol   = sol

    def __lt__(self, other):
        return self.bound < other.bound

def precomputeEdges(graph: nx.graph) -> dict:
    edgesOrdered = {}

    for node in graph.nodes:
        # Pega os menores valores
        edgesOrdered[node] = sorted(graph.edges(node, data=True),
                                    key=lambda x: x[2]['weight'])[:2]

    return edgesOrdered

def bound(precomp: dict, graph: nx.Graph, node: Node, extra: int) -> int:
    bound = node.bound
    last = node.sol[-1]

    # Adiciona o custo do nó extra
    bound += 2 * graph[last][extra]['weight']

    # Remove o menor edge do nó extra
    if (last != 1):
        bound -= precomp[last][0][2]['weight']
    else:
        bound -= precomp[last][1][2]['weight']

    # Remove o segundo menor edge do nó extra
    if (extra != 1):
        bound -= precomp[extra][1][2]['weight']
    else:
        bound -= precomp[extra][0][2]['weight']

    return bound

def calculateBaseBound(graph: nx.Graph, precomp) -> int:
    base = 0

    for node in graph.nodes:
        base += precomp[node][0][2]['weight'] # Menor edge
        base += precomp[node][1][2]['weight'] # Segundo menor edge

    return base

def branchAndBound(graph: nx.Graph) -> int:
    # Pre computa os edges
    n    = graph.number_of_nodes()
    pre  = precomputeEdges(graph)
    best = float('inf')
    baseBound = calculateBaseBound(graph, pre)

    # Base bound
    # 0 custo, level 1, solução [0] (Ajustando para começar do nó 0)
    root = Node(baseBound, 1, 0, [0])

    # Solução trivial
    pq = queue.PriorityQueue()
    pq.put(root)

    while not pq.empty():
        node = pq.get()

        # Possivel solução
        if node.level > n:
            if node.cost < best:
                best = node.cost
        # Explorar outras possibilidades
        elif m.ceil(node.bound / 2) < best:
            if node.level < n:
                for k in range(1, n):  # Ajustando o range para começar de 1 (nó 0 já está na solução)
                    # Se k não está na solução
                    if k not in node.sol:
                        lowerBound = bound(pre, graph, node, k)

                        # Promissor, adiciona nó na fila
                        if lowerBound < best:
                            last    = node.sol[-1]
                            newNode = Node(lowerBound,
                                        node.level + 1,
                                        node.cost + graph[last][k]['weight'],
                                        node.sol + [k])
                            pq.put(newNode)
            # Todos os nós foram explorados
            else:
                lowerBound = bound(pre, graph, node, 0)  # Voltando ao nó 0

                if lowerBound < best:
                    last    = node.sol[-1]
                    newNode = Node(lowerBound,
                                node.level + 1,
                                node.cost + graph[last][0]['weight'],
                                node.sol + [0])
                    pq.put(newNode)

    return best


def main():
    # Número de nós no grafo
    num_nodes = 10

    # Gerar um grafo completo com pesos aleatórios nas arestas
    G = nx.complete_graph(num_nodes)
    for u, v in G.edges():
        G[u][v]['weight'] = m.ceil(10 * random.uniform(0, 1))

    # Iniciar a medição do tempo
    start_time = time.time()

    # Aplicar o algoritmo Branch and Bound
    best_cost = branchAndBound(G)

    # Parar a medição do tempo
    end_time = time.time()

    # Calcular e imprimir o tempo total de execução
    execution_time = end_time - start_time
    print("Melhor custo encontrado:", best_cost)
    print("Tempo de Execução:", execution_time, "segundos")

if __name__ == "__main__":
    main()