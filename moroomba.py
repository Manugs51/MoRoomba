#!/usr/bin/python3

# --------------------------------------------------------------------------

print('### Script:', __file__)

# --------------------------------------------------------------------------
try:
    import math
    import sys
    import time

    # import cv2 as cv
    # import numpy as np
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

    # --------------------------------------------------------------------------

    # def getImage(clientID, hRobot):
    #     img = []
    #     err,r,i = sim.simxGetVisionSensorImage(clientID, hRobot[2], 0,
    #                                             sim.simx_opmode_buffer)

    #     if err == sim.simx_return_ok:
    #         img = np.array(i, dtype=np.uint8)
    #         img.resize([r[1],r[0],3])
    #         img = np.flipud(img)
    #         img = cv.cvtColor(img, cv.COLOR_RGB2BGR)

    #     return err, img

    # --------------------------------------------------------------------------

    def getImageBlob(clientID, hRobot):
        rc,ds,pk = sim.simxReadVisionSensor(clientID, hRobot[2],
                                            sim.simx_opmode_buffer)
        blobs = 0
        coord = []
        if rc == sim.simx_return_ok and pk[1][0]:
            blobs = int(pk[1][0])
            offset = int(pk[1][1])
            for i in range(blobs):
                coord.append(pk[1][4+offset*i])
                coord.append(pk[1][5+offset*i])

        return blobs, coord


    #------------------------------------------

    def avoid(sonar, lastSeenBall):
        if (sonar[2] < 0.15) or (sonar[3] < 0.15) or (sonar[1] < 0.15) or (sonar[0] < 0.15):
            #OBJETO A IZQ CERCA
            return +1.0, -1.0, True
        
        elif (sonar[4] < 0.15) or (sonar[5] < 0.15) or (sonar[6] < 0.15) or (sonar[7] <0.15): 
            #OBJETO A DCHA CERCA
            return -1.0, +1.0, True
        
        elif (sonar[2] < 0.20) or (sonar[3] < 0.20) or (sonar[1] < 0.20) or (sonar[0] < 0.20):
            if lastSeenBall < 0.5:
                #OBJETO A IZQ LEJOS
                return +1.5, +1.5, True
        
        elif (sonar[4] < 0.20) or (sonar[5] < 0.20) or (sonar[6] < 0.20) or (sonar[7] <0.20): 
            if lastSeenBall > 0.5:
                #OBJETO A DCHA LEJOS
                return +1.5, +1.5, True
        
        return +1.5, +1.5, False

    #------------------------------------------------------------------

    def followBall(coord, lspeed, rspeed):

        # Cuando está muy cerca, decelero
        if coord[1]>0.7:
            lspeed -= 1.0
            rspeed -= 1.0

        #si está muy lejos, acelero
        elif coord[1]<0.625:
            lspeed += 1.0
            rspeed += 1.0
            #Centro la pelota
            if coord[0] < 0.4:
                rspeed += 0.6
            elif coord[0] > 0.6:
                lspeed += 0.6

        #Si estoy a una distancia prudente
        else: 
            #Centro la pelota
            if coord[0] < 0.4:
                rspeed += 0.6
            elif coord[0] > 0.6:
                lspeed += 0.6
            else:
                #Si la tengo centrada
                lspeed = 1.5
                rspeed = 1.5

        return lspeed, rspeed

    #------------------------------------------------------------------

    def searchBall(lspeed,rspeed,lastSeenBall):
        
        if lastSeenBall < 0.5:
            lspeed += -0.5
            rspeed += 1.0
        else:
            lspeed += 1.0
            rspeed += -0.5
            
        return lspeed, rspeed
        
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
        if orientacion == orientaciones['abajo']:
            pass
        elif orientacion == orientaciones['izquierda']:
            pass
        elif orientacion == orientaciones['arriba']:
            pass
        
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

    def set_vacuuming(clientID, vacuuming):
        print("Vacuuming:", vacuuming)
        if vacuuming:
            sim.simxSetIntegerSignal(clientID, "vacuuming", 1, sim.simx_opmode_blocking)
        else:
            sim.simxSetIntegerSignal(clientID, "vacuuming", 0, sim.simx_opmode_blocking)

    # --------------------------------------------------------------------------

    def main():
        print('### Program started')

        print('### Number of arguments:', len(sys.argv), 'arguments.')
        print('### Argument List:', str(sys.argv))
        
        estado = estadosMoRoomba['mapeando']
        
        mapa = [[-1, -1, -1], [-1, 0, -1], [-1, -1, -1]]
        posicion = (1, 1)
        
        orientacion = orientaciones['derecha']

        sim.simxFinish(-1) # just in case, close all opened connections
        
        port = int(sys.argv[1])
        clientID = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)

        if clientID == -1:
            print('### Failed connecting to remote API server')

        #vacuum=sim.simxGetObjectHandle(clientID, 'vacuum', sim.simx_opmode_blocking)

        else:
            set_vacuuming(clientID, False)
            print('### Connected to remote API server')
            hRobot = getRobotHandles(clientID) ##hRobot contains '[lmh, rmh], sonar, cam'
            # TODO borrar, solo es prueba
            ixxx = 0
            while sim.simxGetConnectionId(clientID) != -1:
                # Perception
                sonar = getSonar(clientID, hRobot)
                
                # TODO borrar, solo es prueba
                if ixxx == 0:
                    set_vacuuming(clientID, True)
                elif ixxx > 0:
                    set_vacuuming(clientID, False)
                    ixxx = -1
                ixxx += 1

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
except Exception as err:
    print(err)
    time.sleep(5000)