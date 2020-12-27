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
    matriz_descubierta = [[-1, -1, -1], [-1, 0, -1], [-1, -1, -1]]
    # Izquierda
    if sonar[0] < 0.5 or sonar[15] < 0.5:
        matriz_descubierta[0][1] = 1
    else:
        matriz_descubierta[0][1] = 0
    # Delante
    if sonar[3] < 0.5 or sonar[4] < 0.5:
        matriz_descubierta[1][2] = 1
    else:
        matriz_descubierta[1][2] = 0
    # Derecha
    if sonar[7] < 0.5 or sonar[8] < 0.5:
        matriz_descubierta[2][1] = 1
    else:
        matriz_descubierta[2][1] = 0
    # Detras
    if sonar[11] < 0.5 or sonar[12] < 0.5:
        matriz_descubierta[1][0] = 1
    else:
        matriz_descubierta[1][0] = 0
    # Rotar
    aux = matriz_descubierta #TODO esto esta mal
    if orientacion == orientaciones['abajo']:
        for i in range(3):
            for j in range(3):
                aux[j, 2-i] = matriz_descubierta[i, j]
    elif orientacion == orientaciones['izquierda']:
        for i in range(3):
            for j in range(3):
                aux[2-i, 2-j] = matriz_descubierta[i, j]
    elif orientacion == orientaciones['arriba']:
        for i in range(3):
            for j in range(3):
                aux[j, 2-i] = matriz_descubierta[i, j]
    
    matriz_descubierta = aux
    
    return matriz_descubierta

def mapear(sonar, orientacion):
    matriz_descubierta = comprobar_laterales(sonar, orientacion)
    print("mapeando")
    return 1, 1
    
#-------------------------------------------------------------------

def limpiar():
    return 0, 0
    
#-------------------------------------------------------------------

def cargar():
    return 0, 0

# --------------------------------------------------------------------------

def main():
    print('### Program started')

    print('### Number of arguments:', len(sys.argv), 'arguments.')
    print('### Argument List:', str(sys.argv))
    
    estado = estadosMoRoomba['mapeando']
    '''
        -2: No observado fuera de rango
        -1: No observado en rango
         0: Vacio
         1: Obstaculo
    '''
    mapa = [[-2 for y in range(100)] for x in range(100)]
    mapa[1][1] = 0
    posicion = (1, 1)
    
    orientacion = orientaciones['derecha']

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
                lspeed, rspeed = mapear(sonar, orientacion)
            elif estado == estadosMoRoomba['limpiando']:
                lspeed, rspeed = limpiar()
            else:
                lspeed, rspeed = cargar()


            # Action
            setSpeed(clientID, hRobot, lspeed, rspeed)
            time.sleep(1)

        print('### Finishing...')
        sim.simxFinish(clientID)

    print('### Program ended')

# --------------------------------------------------------------------------

if __name__ == '__main__':
    main()