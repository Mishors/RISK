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


def add_bonus(graph3, own3, A3):
    maximum3 = 0
    index3 = 0
    for i in range(len(A3)):
        if own[i] == 1 and A[i] > maximum3:
            maximum3 = A[i]
            index3 = i
    A3[index3] += 2
###################################################################################


def can_attack(graph2, index12, own2, A2):
    # get enemy's territory with maximum armies
    maximum2 = 0
    index2 = -1
    neighbours = graph2.getMatrixNeighbours(index12)
    for i in range(len(neighbours)):
        if neighbours[i] == 1 and A2[i] > maximum2 and A2[index12] - A2[i] > 1 and own2 != 1:
            maximum2 = A2[i]
            index2 = i
    return index2


# ##############################aggressive Method##################################
def aggressive(graph1, A1, own1):
    # 1- search for your territories
    # 2- choose the one which have most armies
    # 3- search for its neighbours
    # 4- attack the neighbour with larger armies in
    add_bonus(graph1, A1, own1)

    maximum1 = 0
    index1 = -1
    index2 = -1

    # get my territory with maximum armies
    for k in range(len(A1)):
        if own1[k] == 1:
            index2 = can_attack(graph1, k, own1, A1)
            if A1[k] > maximum1 and index2 != -1:
                maximum1 = A1[k]
                index1 = k

    # make attack
    A1[index1] -= A1[index2]
    A1[index2] = A1[index1] - 1
    A1[index1] = 1
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

# get from GUI
A = [1, 2, 3, 4, 5]
own = [1, 0, 0, 0, 1]

for i in range(len(A)):
    print(A[i])

print()
aggressive(g, A, own)

for i in range(len(A)):
    print(A[i])
