import networkx as nx
import matplotlib.pyplot as plt
import time
import numpy as np
import pandas as pd
import heapq
import itertools
import math

class point:
    def __init__(self,x,y):
        self.x = x
        self.y = y
        
    def euclidean_dist(self,n):
        return np.sqrt((self.x - n.x) **2 + (self.y - n.y) **2)
    
    def manhattan_dist(self,n):
        return np.abs(self.x - n.x) + np.abs(self.y - n.y)
    
    #para permitir fazer print de um ponto
    def __str__(self):
        return str((self.x, self.y))
    def __repr__(self):
        return str((self.x, self.y))

class node:
    def __init__(self, bound, level, cost, s):
        self.cost = cost
        self.s = s
        self.level = level
        self.bound = bound
        
    #para permitir fazer print de um nó na arvore do branch and bound
    def __str__(self):
        return str((self.bound,self.level,self.cost,self.s))  
    #função de comparação para uso no heap
    def __lt__(self, other):
        return self.cost < other.cost 
    
#função que cria a matriz de adjacencia dos pontos de interesse, em que cada A[I][J] é a distancia de i para j
def createAdjacency(instance,distance='euclidean'):
    adjacency = []
    if distance =='euclidean':
        for i in instance:
            aux = []
            for j in instance:
                if i != j:
                    aux.append(i.euclidean_dist(j))
                else:
                    aux.append(0)
            adjacency.append(aux)
    elif distance =='manhattan':
        for i in instance:
            aux = []
            for j in instance:
                if i != j:
                    aux.append(i.manhattan_dist(j))
                else:
                    aux.append(0)
            adjacency.append(aux)
    return adjacency

class vertice:
    def __init__(self, i, pi):
        self.pi = pi
        self.i = i
    
    #funções para impressão
    def __str__(self):
        return str((self.i,self.pi))  
    def __repr__(self):
        return str((self.i,self.pi))
    #função de comparação para uso no heap
    def __lt__(self, other):
        return self.pi < other.pi
    
def prim(A,n):
    solution = [] # Solution armazena as arestas incluídas
    for i in range(n):
        solution.append(0)
    #marca se o vertice i está na mst
    intree = []
    for i in range(n):
        intree.append(False)
        
    #o vertice 0 sempre começa na arvore
    intree[0] = True
    z = vertice(0, 0)
    
    #gerando todos os pis, exceto o de zero, como infinitos// pi é a distancia minima até o vertice i
    sol = []
    sol.append(z)
    for i in range(1, n):
        sol.append(vertice(i, np.inf))
        
    #gerando os pis dos vertices adjacentes ao inicial(0)
    for v in sol:
        v.pi = A[0][v.i]
    
    #gerando a fila de prioridade
    queue = []
    for v in sol:
        queue.append(v)
    heapq.heapify(queue)
    
    #começa o algoritmo de prim
    while len(queue) > 0 and queue[0] != np.inf:
        u = heapq.heappop(queue)
        if not intree[u.i]:
            intree[u.i]=True
            sol[u.i] = u
            if(min(intree)): #se todos vertices ja tiverem sido incluidos
                break
        for i in range(len(A[u.i])): # for each edge (u,v)
            if not intree[i]: #if v not in solution / nao é necessario checar se i é igual a u.n, visto que u.n sempre estará intree
                if sol[i].pi > A[u.i][i]: #se o pi atual for maior que o novo pi
                    sol[i].pi = A[u.i][i]
                    solution[i] = u.i
                    heapq.heappush(queue,sol[i])
    return sol,solution

def Cost(A,path):
    cost = 0
    for i in range(len(path)-1):
        cost += A[path[i]][path[i+1]]
    return cost

def getDegrees(n,s):
    degrees = np.ones(n)#em todo vertice chegou pelo menos 1 arco saindo de outro arco
    degrees[0] = 0 #com exceção do 0, que é a raiz
    
    for x in s[1:]:#como o número x indica a partir de qual vertice chegou no vertice i, onde i é a posição em s
        degrees[x]+=1 #saiu um arco a mais de x
    return degrees

def getOdd(degrees):
    aux = np.array(degrees)
    return list(np.where(aux%2==1)[0])

def getMatching(n,s,A,mst):
    degrees = getDegrees(n,s) #obtém grau dos vertices
    odd = getOdd(degrees) #odd possui a numeração dos vertices com grau impar
    G = nx.Graph()
    #adicionamos as arestas e vértices com grau impar ao grafo
    for i in odd:
        G.add_node(i)
    for i in odd:
        for j in odd:
            if j != i:
                G.add_edge(i, j, weight =- A[i][j])#o peso é negativo, pois assim o peso "maximo" será o minimo no grafo real
                
    #executamos o matching
    match = nx.algorithms.matching.max_weight_matching(G, maxcardinality = True)
    match = list(match)
    
    #formata o matching, adicionando os pesos à tupla
    for i in range(len(match)):
        match[i] += (A[match[i][0]][match[i][1]],)
    
    return match

#função para gerar o novo grafo
def getNewTree(match,s,A):
    graph = []
    for i in range(1,len(s)): #adiciona as arestas da mst
        graph.append((s[i], i, A[s[i]][i]))
    for j in match: # adiciona as arestas do matching
        graph.append(j)
    
    return graph

#função para obter o tour euleriano
def tour(graph,n):
    #cria multigrafo com todas as arestas do grafo(permitindo repetições)
    G = nx.MultiGraph()
    for i in range(n):   
        G.add_node(i)
    for i in graph:
        G.add_edge(i[0], i[1], weight = i[2])
    #encontra o ciclo euleriano
    cycle = list(nx.eulerian_circuit(G))
    return cycle

#função para remover duplicatas no caminho final
def removeDup(s):
    path = []
    current = s[0]
    path.append(current)
    #obtem resposta na forma de caminho 0-1-2-3-0 (podendo ter repetidos )
    for i in s:      
        if i != current:
            current = i
            path.append(current)
    #remove os duplicados
    nodup = list(dict.fromkeys(path))
    #fecha o circuito
    nodup.append(path[0])
    return nodup


def get_cost(graph, order):
    cost = 0
    for i in range(0, len(order) - 1):   
        cost += graph[order[i]][order[i+1]]['weight']
        
    return cost + graph[order[0]][order[len(order) - 1]]['weight']


def distance(v1, v2, metric):
  if metric == 'euclidean':
    return ((v1[0] - v2[0])**2 + (v1[1] - v2[1])**2)**(1/2)
  else:
    return abs(v1[0] - v2[0]) + abs(v1[1] - v2[1])


def generate_graph(metric, amount_of_nodes):

  graph = nx.soft_random_geometric_graph(n = amount_of_nodes, radius  = 10, dim = 2, p_dist = lambda dist: 1, seed = 5)
  for i in graph.nodes():
    for j in graph.nodes():
      if i != j:
        dist = distance(graph.nodes[i]['pos'], graph.nodes[j]['pos'], metric)
        graph.add_edge(i, j, weight = dist)

  return graph



#função que gera uma instancia de n pontos aleatórios de 0 até l
def instanceGenerator(size = 0, l = 1000, seed = np.random.randint(low = 100000000, size = 1)):
    points=[]
    np.random.seed(seed=seed)
    for i in range(size):
        a, b = np.random.randint(low = l, size = 2)
        p = point(a,b)
        points.append(p)
    return points


def buildGraph(edges, n):
    graph = nx.Graph()
    graph.add_nodes_from(range(n))
    graph.add_edges_from(edges)
    return graph



def find_minimum_weight_perfect_matching(G, odd_vertices):
    # Cria um novo grafo onde cada aresta é o custo da aresta original
    g = nx.Graph()
    for u, v in itertools.combinations(odd_vertices, 2):
        g.add_edge(u, v, weight=nx.shortest_path_length(G, u, v, weight='weight'))

    # Encontrar o emparelhamento mínimo perfeito
    # Como networkx não tem uma função integrada para isto, usaremos uma abordagem aproximada
    matching = nx.algorithms.max_weight_matching(g, maxcardinality=True)

    return matching