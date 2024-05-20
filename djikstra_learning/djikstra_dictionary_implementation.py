import math

class Graph:
    def __init__(self, graph):
        self.graph = graph
    
    def show_graph(self):
        print(self.graph)
    
    def get_neighbours(self, node):
        if node in self.graph.keys():
            return self.graph[node]
        return -1
    
    def djikstra(self, src, dest):
        shortest_path_map = {key:{'distance' : math.inf, 'previous' : ''} for key in self.graph}
        if src not in self.graph.keys() or dest not in self.graph.keys() and src != dest:
            return -1

        shortest_path_map[src]['distance'] = 0
        
        current_node = src
        
        traversed_list = [current_node]
        
        while current_node != dest:
            # print(current_node)
            closest_neighbour_distance, closest_neighbour = math.inf, ''
            neighbours = self.get_neighbours(current_node)
            for node, distance in neighbours.items():
                if distance < closest_neighbour_distance and node not in traversed_list:
                    closest_neighbour_distance = distance
                    closest_neighbour = node
                    
            shortest_path_map[closest_neighbour]['distance'] = closest_neighbour_distance
            shortest_path_map[closest_neighbour]['previous'] = current_node
            current_node = closest_neighbour
            
            traversed_list.append(current_node)
                    
        return shortest_path_map
    
    def traverse_path(self, path_map, src, dest):
        current = dest
        path_list = [dest]
        distance = path_map[current]['distance']
        print(src, dest, path_map)
        # return path_map
        while True:
            current = path_map[current]['previous']
            distance = distance + path_map[current]['distance']
            if path_map[current]['previous'] == '':
                path_list.append(src)
                final_path = path_list[::-1]
                return final_path, distance
            
            path_list.append(current)
        
        

    
if __name__ == '__main__':

    graph = {
        'A': {'B': 3, 'C': 8},
        'B': {'A': 3, 'C': 2, 'E': 5},
        'C': {'A': 8, 'B': 2, 'D': 1, 'E': 6},
        'E': {'B': 5, 'C': 6, 'D': 2, 'F': 5},
        'D': {'C': 1, 'E': 2, 'F': 3},
        'F': {'D': 3, 'E': 5}
    }
    graph = Graph(graph)
    src, dest = 'F', 'A'
    # graph.show_graph()
    # print('Neighbours of A : ', graph.get_neighbours('A'))
    path = graph.djikstra(src, dest)
    print(graph.traverse_path(path, src, dest))