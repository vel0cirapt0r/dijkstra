import heapq
import os

from graph_parser import GraphParser
from filepaths import DATASET_PATH
from decorators import measure_time
from sample_graph_generator import generate_graph


class Dijkstra:
    def __init__(self, graph):
        self.graph = graph
        self.distances = {}
        self.previous_nodes = {}

    @measure_time
    def dijkstra(self, source):
        Q = []  # Priority queue

        # __init__ialize distances and previous nodes
        for vertex in self.graph:
            self.distances[vertex] = float('inf')
            self.previous_nodes[vertex] = None
            if vertex == source:
                self.distances[vertex] = 0
                heapq.heappush(Q, (self.distances[vertex], vertex))

        while Q:
            u_dist, u = heapq.heappop(Q)
            for v in self.graph[u]:
                alt = self.distances[u] + self.graph[u][v]
                if alt < self.distances[v]:
                    self.distances[v] = alt
                    self.previous_nodes[v] = u
                    heapq.heappush(Q, (self.distances[v], v))

        return self.distances, self.previous_nodes


if __name__ == '__main__':
    dataset_folder_path = input('Enter path to graph files folder: ') or DATASET_PATH
    files_path = input('Enter name of graph files: ') or 'bay'
    folder_path = os.path.join(dataset_folder_path, files_path)

    parser = GraphParser()
    parsed_graph = parser.parse(folder_path)
    graph = parsed_graph['distance_edges']

    total_edges = sum(len(adjacent_vertices) for adjacent_vertices in graph.values())
    print('number of nodes: ', len(graph), ', number of edges: ', total_edges)
    # print(type(graph))

    # graph = generate_graph(321270, 794830)
    # print("generated graph: ", graph)

    source = int(input('Enter source vertex: ') or 1)
    dijkstra_instance = Dijkstra(graph)
    distances, previous_nodes = dijkstra_instance.dijkstra(source)
    print("Distances:", distances)
    print("Previous nodes:", previous_nodes)
