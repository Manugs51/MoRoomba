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


estadosMoRoomba = {'mapeando': 'mapeando', 'limpiando': 'limpiando', 'recargando': 'recargando'}

orientaciones = {'derecha': 'derecha', 'arriba': 'arriba', 'izquierda': 'izquierda', 'abajo': 'abajo'}

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

def rotate_robot(matriz_descubierta, orientacion, clientID, hRobot):
    # Voy a asumir que empezamos en una pared para simplificar de forma gorda
    rotation_speed = 0.15
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
        #print(mapa[posicion[0]-1:posicion[0]+2, posicion[1]-1:posicion[1] + 2])
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
        #print(matriz_descubierta[0])
        #print(matriz_descubierta[1])
        #print(matriz_descubierta[2])
        #print(mapa[posicion[0]-1:posicion[0]+2, posicion[1]-1:posicion[1] + 2])

    return mapa, posicion, orientacion, False

#-------------------------------------------------------------------

def limpiar():
    return 0, 0

#-------------------------------------------------------------------

def cargar():
    return 0, 0

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
    for i in range(mapa.shape[0]):
        in_horizontal_limit = False
        for j in range(mapa.shape[1]):
            if mapa[i, j] == 1:
                in_horizontal_limit = not in_horizontal_limit
            elif mapa[i, j] == -2 and in_horizontal_limit:
                mapa[i, j] = -1

    return mapa

# --------------------------------------------------------------------------

def main():
    print('### Program started')

    print('### Number of arguments:', len(sys.argv), 'arguments.')
    print('### Argument List:', str(sys.argv))

    # AQUI SE CAMBIA DONDE SE GUARDAN LOS LOGS DE ERROR Y EL MAPA
    path = "C:\\Users\\Miguel\\Documents\\MoRoomba\\"
    sys.stderr = open(path + "logerr.txt", "w")

    estado = estadosMoRoomba['mapeando']
    '''
        -2: No observado fuera de rango
        -1: No observado en rango
         0: Vacio
         1: Obstaculo
         2: Base
    '''
    map_size = 100
    mapa = np.ones((map_size, map_size)) * -2
    half_map = map_size // 2

    mapa[half_map][half_map] = 2
    posicion = np.array([half_map, half_map])

    orientacion = orientaciones['arriba']

    sim.simxFinish(-1) # just in case, close all opened connections

    port = int(sys.argv[1])
    clientID = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)

    if clientID == -1:
        print('### Failed connecting to remote API server')

    else:
        print('### Connected to remote API server')
        hRobot = getRobotHandles(clientID) ##hRobot contains '[lmh, rmh], sonar, cam'

        while sim.simxGetConnectionId(clientID) != -1:
            # Perception
            sonar = getSonar(clientID, hRobot)

            if estado == estadosMoRoomba['mapeando']:
                set_vacuuming(clientID, False)
                mapa, posicion, orientacion, ended = mapear(sonar, orientacion, mapa, posicion, clientID, hRobot)                 
                save_map_to_file(mapa, path + "mapa.csv")
                if ended:
                    print("Ended")
                    time.sleep(5000)

            elif estado == estadosMoRoomba['limpiando']:
                set_vacuuming(clientID, True)
                lspeed, rspeed = limpiar()
            else:
                lspeed, rspeed = cargar()


            # Action
            setSpeed(clientID, hRobot, 1, 1)
            time.sleep(0.75)

        print('### Finishing...')
        sim.simxFinish(clientID)

    print('### Program ended')

# --------------------------------------------------------------------------

if __name__ == '__main__':
    main()
