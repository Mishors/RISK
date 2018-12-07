import heapq


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
##################################################################################


def graph(g1):
    """ Function to print a graph as adjacency list and adjacency matrix. """
    return str(g1.adjacencyList()) + '\n' + '\n' + str(g1.adjacencyMatrix())


###################################################################################
h = []
turns = 0
###################################################################################


###################################################################################
def numNeighbours(graph6, index, own7):
    num = 0
    neighbours2 = graph6.getMatrixNeighbours(index)
    for ii in range(len(neighbours2)):
        if neighbours2[ii] == 1 and own7[ii] == 0:
            num = num + 1
    return num
###################################################################################


def add_bonus(graph7, A3, own3):
    maximum = -1
    index111 = 0
    for y in range(len(A3)):
        if own3[y] == 1:
            numNei = numNeighbours(graph7, y, own3)
            if numNei > maximum:
                maximum = numNei
                index111 = y
    A3[index111] += 2
###################################################################################


def add_enemy_bonus(A4, own4):
    minimum = 1000000
    index222 = 0
    for n in range(len(A4)):
        if own4[n] == 0 and A4[n] < minimum:
            minimum = A4[n]
            index222 = n
    A4[index222] += 2
###################################################################################


def inHeap(myHeap, enemy, my):
    for l in range(len(myHeap)):
        if myHeap[l][1] == enemy and myHeap[l][2] == my:
            return 1
    return 0
###################################################################################


def can_attack(graph2, index12, own2, A2, heuristics):
    # get enemy's territory with maximum armies
    # maximum2 = 0
    # index2 = -1
    global h
    neighbours = graph2.getMatrixNeighbours(index12)
    for i in range(len(neighbours)):
        if own2[i] == 0 and neighbours[i] == 1 and own2[index12] == 1:
            if inHeap(h, i, heuristics) == 0:
                if A2[index12] - A2[i] > 1:
                    heapq.heappush(h, (heuristics - 1, i, index12))
                # else:
                #     heapq.heappush(h, (heuristics, i, index12))
            else:
                for j in range(len(h)):
                    if h[j][1] == i and h[j][2] == index12:
                        if A2[index12] - A2[i] > 1:
                            # never accessed
                            te = h[j][0]
                            te = min(te, heuristics - 1)
                            h[j] = list(h[j])
                            h[j][0] = te
                            h[j] = tuple(h[j])
                        # else:
                            # te = h[j][0]
                            # te = min(te, heuristics)
                            # h[j] = list(h[j])
                            # h[j][0] = te
                            # h[j] = tuple(h[j])

# ######################A* Method##########################


def A_Star(graph1, A1, own1):
    global h
    for k in range(len(A1)):
        can_attack(graph1, k, own1, A1, A_star_cost(own1))
    while True:
        if len(h) == 0:
            add_enemy_bonus(A1, own1)
            add_bonus(graph1, A1, own1)
            for k in range(len(A1)):
                can_attack(graph1, k, own1, A1, A_star_cost(own1))
            continue
        else:
            element = heapq.heappop(h)
            attacked = element[1]
            attacking = element[2]
            numNeiAttacked = numNeighbours(graph1, attacked, own1)
            numNeiAttacking = numNeighbours(graph1, attacking, own1)
            own1[attacked] = 1
            if numNeiAttacked > numNeiAttacking:
                A1[attacking] -= A1[attacked]
                A1[attacked] = A1[attacking] - 1
                A1[attacking] = 1
            else:
                A1[attacking] -= A1[attacked]
                A1[attacking] = A1[attacking] - 1
                A1[attacked] = 1
            if enemy_territories(own1) == 0:
                return True
            add_enemy_bonus(A1, own1)
            add_bonus(graph1, A1, own1)
            can_attack(graph1, attacked, own1, A1, A_star_cost(own1))
            global turns
            turns = turns + 1

    # return False
###############################################################


def enemy_territories(own4):
    counter = 0
    for m in range(len(own4)):
        if own4[m] == 0:
            counter = counter + 1
    return counter
##############################################################


###################################################################################
def A_star_cost(own8):
    return enemy_territories(own8) + turns
###################################################################################


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
print()
# print(g.getMatrixNeighbours(4))

A = [1, 2, 3, 4, 5]
own = [1, 0, 0, 0, 1]

A_Star(g, A, own)

print("new A: ")
for i in range(len(A)):
    print(A[i])

print("new own: ")
for i in range(len(own)):
    print(own[i])
