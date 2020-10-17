#!/usr/bin/python3

# --------------------------------------------------------------------------

print('### Script:', __file__)

# --------------------------------------------------------------------------

import math
import sys
import time

# import cv2 as cv
# import numpy as np
import sim

# --------------------------------------------------------------------------
#Requiere: pip install simple-pid
from simple_pid import PID

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def getRobotHandles(clientID):
    # Motor handles
    _,lmh = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor',
                                     sim.simx_opmode_blocking)
    _,rmh = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor',
                                     sim.simx_opmode_blocking)

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

# --------------------------------------------------------------------------



def avoid(sonar):
    if (sonar[3] < 0.5) or (sonar[4] < 0.5):
        lspeed, rspeed = +3.0, -0.5
    elif sonar[1] < 0.5:
        lspeed, rspeed = +1.0, +0.3
    elif sonar[5] < 0.5:
        lspeed, rspeed = +0.2, +0.7
    else:
        lspeed, rspeed = +3.0, +3.0

    return lspeed, rspeed

# Implementación PID
# Se implemento un PD

g1 = []
g2 = []
g3 = []
g4 = []
g5 = []
g6 = []

kp = 220
ki = 0
kd = 20

PID_I = PID(kp,ki,kd)
PID_I.setpoint = 0.5
PID_I.output_limits = (0, 100)
PID_I.proportional_on_measurement = True
PID_I.sample_time = 0.01

PID_D = PID(kp,ki,kd)
PID_D.setpoint = 0.5
PID_D.output_limits = (-100, 0)
PID_D.proportional_on_measurement = True
PID_D.sample_time = 0.01


PID_Dist = PID(kp,ki,kd)
PID_Dist.setpoint = 0.68
PID_Dist.output_limits = (0, 100)
PID_Dist.proportional_on_measurement = True
PID_Dist.sample_time = 0.01

# --------------------------------------------------------------------------

def main():
    print('### Program started')

    print('### Number of arguments:', len(sys.argv), 'arguments.')
    print('### Argument List:', str(sys.argv))

    sim.simxFinish(-1) # just in case, close all opened connections

    port = int(sys.argv[1])
    clientID = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)

    
    if clientID == -1:
        print('### Failed connecting to remote API server')

    else:
        print('### Connected to remote API server')
        hRobot = getRobotHandles(clientID)       

        while sim.simxGetConnectionId(clientID) != -1:
            # Perception
            sonar = getSonar(clientID, hRobot)
            #print ('### s', sonar)

            blobs, coord = getImageBlob(clientID, hRobot)


            if blobs == 1:
                

                if coord[0] >= 0.5:
                    pos_d = 1 - coord[0]
                    pos_i = 0.5
                else:
                    pos_d = 0.5
                    pos_i = coord[0]

                
                res_MD = PID_I(pos_i)
                res_MI = PID_D(pos_d)
                res_Dist = PID_Dist(coord[1])
                

                speed_p = 1.75
                fact = -3            
                pwm_I = (100+res_MI)/100
                pwm_D = res_MD/100

                breake = res_Dist/100
                                
                lspeed = speed_p + fact*pwm_D + breake
                rspeed = speed_p + fact*pwm_I + breake
               
                #lspeed = 0
                #rspeed = 0


                g1.append(pos_d)
                g2.append(pos_i)
                g3.append(pwm_I)
                g4.append(pwm_D)
                
                g5.append(coord[1])
                g6.append(res_Dist/100)
                

                plt.figure(1)                
                #plt.plot(g1)
                #plt.plot(g2)
                #plt.plot(g3)
                #plt.plot(g4)
                plt.plot(g5)
                plt.plot(g6)
                plt.pause(0.0005)
                plt.cla()
                
               
               
            else:
                lspeed, rspeed = avoid(sonar)
                #lspeed, rspeed = 2*nspeed, 0

            # Planning
            #lspeed, rspeed = avoid(sonar)

            # Action
            setSpeed(clientID, hRobot, lspeed, rspeed)
            #time.sleep(0.1)

           

        print('### Finishing...')
        sim.simxFinish(clientID)

    print('### Program ended')

# --------------------------------------------------------------------------

if __name__ == '__main__':
    main()
