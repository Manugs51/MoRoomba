
from collections import defaultdict 


def transform_map_to_edges(mapa):
    edges = [] # [[]]
    '''
        Para cada casilla se comprueban los vecinos, solo el de la derecha y abajo de cada uno, asi se asegura el 
          que se cubre todo sin repeticion. Solo se comprueba si:
            1: la propia casilla está vacia (0, puede navegar por ahi el robot) y
            2: se añade si el respectivo vecino tambien es 0
          Las relaciones de vecindad son bidireccionales [(x,m),(y,n)] = [(y,n),(x,m)], aunque nunca se van a dar repeticiones
    '''
    for i in range(len(mapa) - 1):
        for j in range(len(mapa[0]) - 1):
            #print(i)
            #print(j)
            #print(mapa)
            if mapa[i][j] == 0:
                if mapa[i + 1][ j] == 0:
                    edges.append([(i, j),(i + 1, j)])
                if mapa[i][j + 1] == 0:
                    edges.append([(i, j),(i, j + 1)])
    return edges

def BFS_SP(graph, start, goal): 
    explored = [] 
      
    # Queue for traversing the  
    # graph in the BFS 
    queue = [[start]] 
      
    # If the desired node is  
    # reached 
    if start == goal: 
        print("Same Node") 
        return
      
    # Loop to traverse the graph  
    # with the help of the queue 
    while queue: 
        path = queue.pop(0) 
        node = path[-1] 
          
        # Codition to check if the 
        # current node is not visited 
        if node not in explored: 
            neighbours = graph[node] 
              
            # Loop to iterate over the  
            # neighbours of the node 
            for neighbour in neighbours: 
                new_path = list(path) 
                new_path.append(neighbour) 
                queue.append(new_path) 
                  
                # Condition to check if the  
                # neighbour node is the goal 
                if neighbour == goal: 
                    print("Shortest path = ", *new_path) 
                    return
            explored.append(node) 
  
    # Condition when the nodes  
    # are not connected 
    print("So sorry, but a connecting"\
                "path doesn't exist :(") 
    return

mapa1 = [[1,1,1,1,1,1],
         [1,0,1,0,0,1],
         [1,0,1,1,0,1],
         [1,0,0,0,0,1],
         [1,1,1,1,1,1]]

graph = defaultdict(list)
edges = transform_map_to_edges(mapa1)
for edge in edges:
    a, b = edge[0], edge[1]
    # Se crea el grafo como una lista de adyacencias
    graph[a].append(b)
    graph[b].append(a)

BFS_SP(graph, (1,1), (1,3))
