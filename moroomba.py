#!/usr/bin/python3

# --------------------------------------------------------------------------

print('### Script:', __file__)

# --------------------------------------------------------------------------

import math
import sys
import time

# import cv2 as cv
import numpy as np
import sim
from collections import defaultdict
from Information import Information

MAX_CHARGE = 300

estadosMoRoomba = {'mapeando': 'mapeando', 'limpiando': 'limpiando', 'recargando': 'recargando'}

orientaciones = {'derecha': 'derecha', 'arriba': 'arriba', 'izquierda': 'izquierda', 'abajo': 'abajo'}


# --------------------------------------------------------------------------

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
            if mapa[i][j] == 0 or mapa[i][j] >= 2:
                if mapa[i + 1][j] == 0 or mapa[i + 1][j] >= 2:
                    edges.append([(i, j),(i + 1, j)])
                if mapa[i][j + 1] == 0 or mapa[i][j + 1] >= 2:
                    edges.append([(i, j),(i, j + 1)])
    return edges


# --------------------------------------------------------------------------
# https://www.geeksforgeeks.org/building-an-undirected-graph-and-finding-shortest-path-using-dictionaries-in-python/
# Python implementation to find the  
# shortest path in the graph using  
# dictionaries  
  
# Function to find the shortest 
# path between two nodes of a graph 
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
                    return np.array(new_path)
            explored.append(node) 
  
    # Condition when the nodes  
    # are not connected 
    print("So sorry, but a connecting"\
                "path doesn't exist :(") 
    return


# --------------------------------------------------------------------------
def oritacion2posicion(orientacion):
    if orientacion == "derecha":
        return np.array([0, 1])
    if orientacion == "arriba":
        return np.array([-1, 0])
    if orientacion == "izquierda":
        return np.array([0, -1])
    if orientacion == "abajo":
        return np.array([1, 0])

# --------------------------------------------------------------------------

def getRobotHandles(clientID):
    # Motor handles
    _,lmh = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor',sim.simx_opmode_blocking)
    _,rmh = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor',sim.simx_opmode_blocking)

    # Sonar handles
    str = 'Pioneer_p3dx_ultrasonicSensor%d'
    sonar = [0] * 16
    for i in range(16):
        _,h = sim.simxGetObjectHandle(clientID, str % (i+1),
                                       sim.simx_opmode_blocking)
        sonar[i] = h
        sim.simxReadProximitySensor(clientID, h, sim.simx_opmode_streaming)

    # Camera handles
    _,cam = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_camera',
                                        sim.simx_opmode_oneshot_wait)
    sim.simxGetVisionSensorImage(clientID, cam, 0, sim.simx_opmode_streaming)
    sim.simxReadVisionSensor(clientID, cam, sim.simx_opmode_streaming)

    return [lmh, rmh], sonar, cam

# --------------------------------------------------------------------------

def setSpeed(clientID, hRobot, lspeed, rspeed):
    sim.simxSetJointTargetVelocity(clientID, hRobot[0][0], lspeed,
                                    sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID, hRobot[0][1], rspeed,
                                    sim.simx_opmode_oneshot)

# --------------------------------------------------------------------------

def getSonar(clientID, hRobot):
    r = [1.0] * 16
    for i in range(16):
        handle = hRobot[1][i]
        e,s,p,_,_ = sim.simxReadProximitySensor(clientID, handle,
                                                 sim.simx_opmode_buffer)
        if e == sim.simx_return_ok and s:
            r[i] = math.sqrt(p[0]*p[0]+p[1]*p[1]+p[2]*p[2])

    return r

#-------------------------------------------------------------------

def comprobar_laterales(sonar, orientacion):
    max_distance = 0.4
    max_distance_sides = 0.7
    matriz_descubierta = np.array([[-1, 0, -1], [0, 0, 0], [-1, 0, -1]])
    # Izquierda
    if sonar[0] < max_distance_sides or sonar[15] < max_distance_sides:
        matriz_descubierta[1][0] = 1

    # Delante
    if sonar[3] < max_distance or sonar[4] < max_distance:
        matriz_descubierta[0][1] = 1

    # Derecha
    if sonar[7] < max_distance_sides or sonar[8] < max_distance_sides:
        matriz_descubierta[1][2] = 1

    # Detras
    if sonar[11] < max_distance_sides or sonar[12] < max_distance_sides:
        matriz_descubierta[2][1] = 1

    return matriz_descubierta

#-------------------------------------------------------------------

def rotate_matrix(matriz, orientacion):
    aux = matriz
    if orientacion == orientaciones['abajo']:
        return np.rot90(aux, 2)
    if orientacion == orientaciones['izquierda']:
        return np.rot90(aux, 1)
    if orientacion == orientaciones['derecha']:
        return np.rot90(aux, -1)

    return aux

#-------------------------------------------------------------------

# dir_giro es o derecha o izquierda
def get_next_orientacion(orientacion, dir_giro):
    if dir_giro == "derecha":
        if orientacion == "arriba":
            orientacion = "derecha"
        elif orientacion == "derecha":
            orientacion = "abajo"
        elif orientacion == "abajo":
            orientacion = "izquierda"
        else:
            orientacion = "arriba"
    
    else:
        if orientacion == "arriba":
            orientacion = "izquierda"
        elif orientacion == "derecha":
            orientacion = "arriba"
        elif orientacion == "abajo":
            orientacion = "derecha"
        else:
            orientacion = "abajo"

    return orientacion


#-------------------------------------------------------------------

def rotate_dir(clientID, hRobot, orientacion, dir="derecha", speed=.1):
    if dir == "derecha":
        lspeed = speed
        rspeed = -speed
    else:
        lspeed = -speed
        rspeed = speed
    next_orientacion = get_next_orientacion(orientacion, dir)

    while(1):
        angles = sim.simxGetObjectOrientation(clientID, hRobot[-1], -1, sim.simx_opmode_blocking)
        epsilon = 0.005
        if orientacion == "arriba" or orientacion == "abajo":
            if angles[1][1] > -epsilon and angles[1][1] < epsilon:
                break
        else:
            abs_angle = abs(angles[1][1])
            if abs_angle > (np.pi / 2 - epsilon):
                break
        setSpeed(clientID, hRobot, lspeed, rspeed)
        angles = sim
    return next_orientacion

#-------------------------------------------------------------------


def rotate_robot(matriz_descubierta, orientacion, clientID, hRobot):
    # Voy a asumir que empezamos en una pared para simplificar de forma gorda
    rotation_speed = 0.1
    lspeed = -rotation_speed
    rspeed = -rotation_speed
    if matriz_descubierta[1, 0] == 1 and matriz_descubierta[1, 2] == 1:
        # Si me veo en esta situación prefiero morir a pensar que hacer :)
        pass
    elif matriz_descubierta[1, 2] == 0:
        lspeed = rotation_speed
        next_orientacion = get_next_orientacion(orientacion, "derecha")
    elif matriz_descubierta[1, 0] == 0:
        rspeed = rotation_speed
        next_orientacion = get_next_orientacion(orientacion, "izquierda")


    while(1):
        angles = sim.simxGetObjectOrientation(clientID, hRobot[-1], -1, sim.simx_opmode_blocking)
        epsilon = 0.005
        if orientacion == "arriba" or orientacion == "abajo":
            if angles[1][1] > -epsilon and angles[1][1] < epsilon:
                break
        else:
            abs_angle = abs(angles[1][1])
            if abs_angle > (np.pi / 2 - epsilon):
                break
        setSpeed(clientID, hRobot, lspeed, rspeed)
        #print(angles[1][1])
        angles = sim
    return next_orientacion

#-------------------------------------------------------------------

def mapear(sonar, orientacion, mapa, posicion, clientID, hRobot, verbose=1):
    matriz_descubierta = comprobar_laterales(sonar, orientacion)

    lspeed = 1
    rspeed = 1
    
    matriz_rotada = rotate_matrix(matriz_descubierta, orientacion)
    # Arriba
    if mapa[posicion[0] - 1, posicion[1]] == -2:
        mapa[posicion[0] - 1, posicion[1]] = matriz_rotada[0, 1]
    # Izquierda
    if mapa[posicion[0], posicion[1] - 1] == -2:
        mapa[posicion[0], posicion[1] - 1] = matriz_rotada[1, 0]
    # Derecha
    if mapa[posicion[0], posicion[1] + 1] == -2:
        mapa[posicion[0], posicion[1] + 1] = matriz_rotada[1, 2]
    # Abajo
    if mapa[posicion[0] + 1, posicion[1]] == -2:
        mapa[posicion[0] + 1, posicion[1]] = matriz_rotada[2, 1]


    if 2 in mapa[posicion[0]-1:posicion[0]+2, posicion[1]-1:posicion[1] + 2] and \
        mapa[posicion[0]+1, posicion[1]] != 2 and mapa[posicion[0], posicion[1]] != 2: 
        mapa = fill_reachable_map(mapa)
        return mapa, posicion, orientacion, True

    if matriz_descubierta[0, 1] == 1:
        if verbose > 0:
            print("ROTANDO")
        orientacion = rotate_robot(matriz_descubierta, orientacion, clientID, hRobot)
    else:
        posicion = posicion + oritacion2posicion(orientacion)


    if verbose > 0:
        print(orientacion)

    return mapa, posicion, orientacion, False

#-------------------------------------------------------------------

def limpiar(charge, posicion, path, orientacion, mapa, clientID, hRobot, graph):
    set_vacuuming(clientID, True)
    if len(path) < 2:
        dirty_spot = search_closest_dirty_spot(posicion, orientacion, mapa)
        if dirty_spot is None:
            # Hacer que vuelva a la base. CASA LIMPIA, EXITO SEÑORES
            pass
        else:
            path = BFS_SP(graph, posicion, dirty_spot)
            print("CAMINO A LIMPIAR", path)
    
    posicion, orientacion, mapa, path = follow_path(path, orientacion, mapa, clientID, hRobot)
    
    mapa[i, j] = 0

    charge = charge - 1

    return charge, path, orientacion, mapa, posicion, 1, 1

#-------------------------------------------------------------------

def cargar(charge, path, orientacion, mapa, clientID, hRobot):
    if len(path) > 1:
        lspeed = 1
        rspeed = 1
    else:
        charge = np.min([MAX_CHARGE, charge + 10])
        print("Charging... (", charge, "/", MAX_CHARGE, ")", sep="")

        lspeed = 0
        rspeed = 0
    
    posicion, orientacion, mapa, path = follow_path(path, orientacion, mapa, clientID, hRobot)
    
    return charge, path, orientacion, mapa, posicion, lspeed, rspeed

# --------------------------------------------------------------------------

def set_vacuuming(clientID, vacuuming):
    if vacuuming:
        sim.simxSetIntegerSignal(clientID, "vacuuming", 1, sim.simx_opmode_blocking)
    else:
        sim.simxSetIntegerSignal(clientID, "vacuuming", 0, sim.simx_opmode_blocking)

# --------------------------------------------------------------------------

def save_map_to_file(mapa, file_path):
    np.savetxt(file_path, mapa.astype(int), fmt='%i', delimiter=";")

# --------------------------------------------------------------------------

def fill_reachable_map(mapa):
    for i in range(1, mapa.shape[0]):
        for j in range(1, mapa.shape[1]):
            if (mapa[i - 1, j] == 0 or mapa[i - 1, j] == -1) \
                and (mapa[i, j - 1] == 0 or mapa[i, j - 1] == -1) \
                and mapa[i, j] == -2:
                mapa[i, j] = -1

    return mapa

# --------------------------------------------------------------------------

def follow_path(path, orientacion, mapa, clientID, hRobot):
    if len(path) < 2:
        return path, orientacion, mapa, path

    dest = path[1] - path[0]
    pos = oritacion2posicion(orientacion)
    dir_giro = "derecha"
    if np.all(dest == -pos[::-1]):
        dir_giro = "izquierda"

    print(dest, oritacion2posicion(orientacion))
    print(dest != oritacion2posicion(orientacion),np.any(dest != oritacion2posicion(orientacion)))
    while(np.any(dest != oritacion2posicion(orientacion))):
        orientacion = rotate_dir(clientID, hRobot, orientacion, dir_giro)
        print(dest, oritacion2posicion(orientacion))
    
    mapa[path[0,0], path[0,1]] = 0
    
    path = path[1:]
    return path[0], orientacion, mapa, path
# --------------------------------------------------------------------------

def dirty_floor(mapa):
    for i in range(mapa.shape[0]):
        for j in range(mapa.shape[1]):
            if mapa[i, j] == -1 or mapa[i, j] == 0:
                mapa[i, j] = 3
    return mapa

# --------------------------------------------------------------------------

def get_graph_from_map(mapa):
    graph = defaultdict(list)

    edges = transform_map_to_edges(mapa)
    for edge in edges:
        a, b = edge[0], edge[1]
        # Se crea el grafo como una lista de adyacencias
        graph[a].append(b)
        graph[b].append(a)

    return graph

# --------------------------------------------------------------------------

def search_closest_dirty_spot(posicion, orientacion, mapa):
    pos = oritacion2posicion(orientacion)
    # inmediatamente delante
    following = pos + posicion
    following = following[0]
    print("FOLLOWING", following)
    if mapa[following[0], following[1]] == 3:
        return following

    # alrededor
    alrededor = mapa[posicion[0]-1:posicion[0]+2, posicion[1]-1:posicion[1] + 2]
    pos = None
    for i in range(2):
        for j in range(2):
            if alrededor[i, j] == 3:
                pos = posicion + np.array([i - 1, j - 1])
                return pos

    # la que haya
    for i in range(mapa.shape[0]):
        for j in rage(mapa.shape[1]):
            if mapa[i, j] == 3:
                return np.array([i, j])        
    
    return None
    
    


def main():
    print('### Program started')

    print('### Number of arguments:', len(sys.argv), 'arguments.')
    print('### Argument List:', str(sys.argv))

    # AQUI SE CAMBIA DONDE SE GUARDAN LOS LOGS DE ERROR Y EL MAPA
    path = "D:\\Users\\Manuel Guerrero\\Desktop\\WorkingDirectoryCopelia\\MoRoomba\\MoRoomba\\"
    # path = "C:\\Users\\Miguel\\Documents\\MoRoomba\\"
    sys.stderr = open(path + "logerr.txt", "w")

    estado = estadosMoRoomba['mapeando']
    '''
        -2: No observado fuera de rango
        -1: No observado en rango
         0: Vacio
         1: Obstaculo
         2: Base
         3: Sucio
    '''
    map_size = 100
    mapa = np.ones((map_size, map_size)) * -2
    half_map = map_size // 2

    mapa[half_map][half_map] = 2
    initial_position = np.array([half_map, half_map])
    posicion = initial_position

    orientacion = orientaciones['arriba']
    
    graph = defaultdict(list)

    sim.simxFinish(-1) # just in case, close all opened connections

    port = int(sys.argv[1])
    clientID = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)

    charge = 200 #MAX_CHARGE

    if clientID == -1:
        print('### Failed connecting to remote API server')

    else:
        print('### Connected to remote API server')
        hRobot = getRobotHandles(clientID) ##hRobot contains '[lmh, rmh], sonar, cam'
        SECOND_INTERVAL = 1
        
        information = Information()

        while sim.simxGetConnectionId(clientID) != -1:
            # Perception
            sonar = getSonar(clientID, hRobot)

            start_time = time.time()

            if estado == estadosMoRoomba['mapeando']:
                lspeed = 1
                rspeed = 1
                set_vacuuming(clientID, False)
                mapa, posicion, orientacion, ended = mapear(sonar, orientacion, mapa, posicion, clientID, hRobot)                 
                save_map_to_file(mapa, path + "mapa.csv")
                if ended:
                    edges = transform_map_to_edges(mapa)
                    for edge in edges:
                        a, b = edge[0], edge[1]
                        # Se crea el grafo como una lista de adyacencias
                        graph[a].append(b)
                        graph[b].append(a)
                    estado = estadosMoRoomba['recargando']
                    path = BFS_SP(graph, (posicion[0], posicion[1]), (half_map, half_map))
                    lspeed = 0
                    rspeed = 0


            elif estado == estadosMoRoomba['limpiando']:
                charge, path, orientacion, mapa, posicion, lspeed, rspeed = limpiar(charge, path, posicion, orientacion, mapa, clientID, hRobot, graph)

            else:
                charge, path, orientacion, mapa, posicion, lspeed, rspeed = cargar(charge, path, orientacion, mapa, clientID, hRobot)
                if charge == MAX_CHARGE and len(path) < 2:
                    mapa = dirty_floor(mapa)
                    graph = get_graph_from_map(mapa)
                    estado = estadosMoRoomba['limpiando']

                set_vacuuming(clientID, False)

            #pos = sim.simxGetObjectPosition(clientID, hRobot[-1], -1, sim.simx_opmode_blocking)
            #if orientacion == "arriba" or orientacion == "abajo":

            # Action
            setSpeed(clientID, hRobot, lspeed, rspeed)

            time_interval = (start_time + SECOND_INTERVAL) - time.time()
            if time_interval > 0:
                time.sleep(time_interval)
            else:
                time.sleep(SECOND_INTERVAL)
            
            information.update(mapa, estado, posicion, charge)
            
        # End main while
        print('### Finishing...')
        sim.simxFinish(clientID)

    print('### Program ended')

# --------------------------------------------------------------------------

if __name__ == '__main__':
    main()
