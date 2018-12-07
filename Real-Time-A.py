class Vertex:
    def __init__(self, vertex):
        self.name = vertex
        self.neighbors = []

    def add_neighbor(self, neighbor):
        if isinstance(neighbor, Vertex):
            if neighbor.name not in self.neighbors:
                self.neighbors.append(neighbor.name)
                neighbor.neighbors.append(self.name)
                self.neighbors = sorted(self.neighbors)
                neighbor.neighbors = sorted(neighbor.neighbors)
        else:
            return False

    def add_neighbors(self, neighbors):
        for neighbor in neighbors:
            if isinstance(neighbor, Vertex):
                if neighbor.name not in self.neighbors:
                    self.neighbors.append(neighbor.name)
                    neighbor.neighbors.append(self.name)
                    self.neighbors = sorted(self.neighbors)
                    neighbor.neighbors = sorted(neighbor.neighbors)
            else:
                return False

    def __repr__(self):
        return str(self.neighbors)


###################################################################################


class Graph:
    def __init__(self):
        self.vertices = {}

    def add_vertex(self, vertex):
        if isinstance(vertex, Vertex):
            self.vertices[vertex.name] = vertex.neighbors

    def add_vertices(self, vertices):
        for vertex in vertices:
            if isinstance(vertex, Vertex):
                self.vertices[vertex.name] = vertex.neighbors

    def add_edge(self, vertex_from, vertex_to):
        if isinstance(vertex_from, Vertex) and isinstance(vertex_to, Vertex):
            vertex_from.add_neighbor(vertex_to)
            if isinstance(vertex_from, Vertex) and isinstance(vertex_to, Vertex):
                self.vertices[vertex_from.name] = vertex_from.neighbors
                self.vertices[vertex_to.name] = vertex_to.neighbors

    def add_edges(self, edges):
        for edge in edges:
            self.add_edge(edge[0], edge[1])

    def adjacencyList(self):
        if len(self.vertices) >= 1:
            return [str(key) + ":" + str(self.vertices[key]) for key in self.vertices.keys()]
        else:
            return dict()

    def adjacencyMatrix(self):
        if len(self.vertices) >= 1:
            self.vertex_names = sorted(g.vertices.keys())
            self.vertex_indices = dict(zip(self.vertex_names, range(len(self.vertex_names))))
            import numpy as np
            self.adjacency_matrix = np.zeros(shape=(len(self.vertices), len(self.vertices)))
            for i in range(len(self.vertex_names)):
                for j in range(i, len(self.vertices)):
                    for el in g.vertices[self.vertex_names[i]]:
                        j = g.vertex_indices[el]
                        self.adjacency_matrix[i, j] = 1
            return self.adjacency_matrix
        else:
            return dict()

    def getMatrixNeighbours(self, i):
        self.neighbours = []
        for j in range(len(self.adjacencyMatrix()[0])):
            a = int(self.adjacencyMatrix()[i][int(j)])
            self.neighbours.append(a)
        return self.neighbours


def graph(g):
    """ Function to print a graph as adjacency list and adjacency matrix. """
    return str(g.adjacencyList()) + '\n' + '\n' + str(g.adjacencyMatrix())


###################################################################################
h = []
turns = 0
###################################################################################
def add_bonus(A3, own3):
    maximum = 0
    index111 = 0
    for i in range(len(A3)):
        if own3[i] == 1 and A3[i] > maximum:
            maximum = A3[i]
            index111 = i
    A3[index111] += 2
###################################################################################
def add_enemy_bonus(A4, own4):
    minimum = 1000000
    index222 = 0
    for i in range(len(A4)):
        if own4[i] == 0 and A4[i] < minimum:
            minimum = A4[i]
            index222 = i
    A4[index222] += 2
###################################################################################
def inHeap(myHeap, enemy , my):
    for i in range(len(myHeap)):
        if myHeap[i][1] == enemy and myHeap[i][2] == my:
            return 1
    return 0
###################################################################################
import heapq

def can_attack(graph2, index12, own2, A2, heuristics):
    neighbours = graph2.getMatrixNeighbours(index12)
    for i in range(len(neighbours)):
        if(own2[i] == 0 and neighbours[i] == 1):
            if inHeap(h, i) == 0:
                if A2[index12] - A2[i] > 1:
                    heapq.heappush(h, (heuristics - 1, i , index12))
                else:
                    heapq.heappush(h, (heuristics, i , index12))
            else:
                for j in range(len(h)):
                    if h[j][1] == i and h[j][2] == index12:
                        if A2[index12] - A2[i] > 1:
                            h[j][0] = min(h[j][0] , heuristics - 1)
                        else:
                            h[j][0] = min(h[j][0] , heuristics)
# ######################Greedy Method##########################
def real_A_star(graph1, A1, own1):
    # 1- search for your territories
    # 2- choose the one which have most armies
    # 3- search for its neighbours
    # 4- attack the neighbour with larger armies in

    add_bonus(A1 , own1)
    for k in range(len(A1)):
        can_attack(graph1, k, own1, A1, A_star_cost(own1))
    while len(h) != 0:
        attacked = heapq.heappop(h)[1]
        attacking = heapq.heappop(h)[2]
        own1[attacked] = 1
        A1[attacking] -= A1[attacked]
        A1[attacked] = A1[attacking] - 1
        A1[attacking] = 1
        if enemyTerritories(own1) == 0:
            return True
        add_enemy_bonus(A1 , own1)
        can_attack(graph1 , attacked , own1 , A1 , A_star_cost(own1))
        turns = turns + 1

    return False
##############################################################
def enemyTerritories(own):
    counter = 0;
    for i in range(len(own)):
        if own[i] == 0:
            counter = counter + 1
    return counter
##############################################################
def A_star_cost(own):
    return enemyTerritories(own) + turns
##############################################################


a = Vertex('A')
b = Vertex('B')
c = Vertex('C')
d = Vertex('D')
e = Vertex('E')

a.add_neighbors([b, c])
b.add_neighbors([a, c])
c.add_neighbors([b, d, a, e])
d.add_neighbor(c)
e.add_neighbors([c])

g = Graph()
g.add_vertices([a, b, c, d, e])
g.add_edge(b, d)
print(g.adjacencyMatrix())
print(g.getMatrixNeighbours(3))

A = [1, 2, 3, 4, 5]
own = [1, 0, 0, 0, 1]

for i in range(len(A)):
    print(A[i])

print()
real_A_star(g, A, own)

for i in range(len(A)):
    print(A[i])
