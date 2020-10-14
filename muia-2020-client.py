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

def avoid(sonar, ROBOT_MAX_SPEED = 5.0, MIN_SPACE_THRESHOLD = 0.05):
	# decrease the speed by distance
	lspeed = +ROBOT_MAX_SPEED * (sonar[4] - MIN_SPACE_THRESHOLD) 
	rspeed = +ROBOT_MAX_SPEED * (sonar[3] - MIN_SPACE_THRESHOLD)
	# gyro by left and right distances
	lspeed = lspeed + 2.0 - sonar[7] - sonar[15]
	rspeed = rspeed + 2.0 - sonar[0] - sonar[8] 
	# when blocked
	if (sonar[3] < MIN_SPACE_THRESHOLD * 3 or sonar[4] < MIN_SPACE_THRESHOLD * 3):
		right = sonar[0] + sonar[1] + sonar[2] + sonar[3]
		left = sonar[4] + sonar[5] + sonar[6] + sonar[7]
		if (right < left):
			return lspeed, rspeed * -1
		elif (right > left):
			return lspeed * -1, rspeed
	return None
    
# --------------------------------------------------------------------------

def track(blobs, coord, nspeed = 3.8, res = 0,  raz = 2):
    if blobs == 1:
        if coord[0] > 0.5:
            pd = abs(0.5 - coord[0])/0.5
            pi = 0                    
        else:
            pi = (0.5 - coord[0])/0.5
            pd = 0

        if coord[1] <= 0.70:
            res = 3.3*coord[1]
        else:
            res = 3.95*coord[1]
            
        print ('pd= ',pd,'pi= ',pi,'Y= ',coord[1])
        return nspeed + (raz*pd) - res, nspeed + (raz*pi) - res
    return None

# --------------------------------------------------------------------------

def explore(sonar, mem, nspeed = 1.8):
    if (mem["blobs"] == 1):
        if mem["coord"][0] > 0.5:
            pd = abs(0.5 - mem["coord"][0])/0.5
            pi = 0                    
        else:
            pi = (0.5 - mem["coord"][0])/0.5
            pd = 0

        if mem["coord"][1] >= 0.6:
            res = 0.5
        else:
            res = 0
            
        print ('pd= ',pd,'pi= ',pi,'Y= ',mem["coord"][1])
        return nspeed+(1.5*pd)- res, nspeed+(1.5*pi)-res
    elif(mem["lspeed"]):
        return mem["lspeed"], mem["rspeed"]
    else:
        right = sonar[0] + sonar[1] + sonar[2] + sonar[3]
        left = sonar[4] + sonar[5] + sonar[6] + sonar[7]
        if (right > left):
            return .8, .5
        elif(right < left):
            return .5, .8
        else:
            return .8, .8
# --------------------------------------------------------------------------

def coordinator(track_value, avoid_value, explore_value):
    if (avoid_value):
        return avoid_value
    if (track_value):
        return track_value
    if (explore_value):
        return explore_value

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
        
        #Memory
        mem = {
            "sonar": [],
            "blobs": 0,
            "coord": [],
            "lspeed": 0, 
            "rspeed": 0
        }

        while sim.simxGetConnectionId(clientID) != -1:
            # Perception
            sonar = getSonar(clientID, hRobot)
            blobs, coord = getImageBlob(clientID, hRobot)

            # Planning
            avoid_value = avoid(sonar)
            track_value = track(blobs, coord)                 
            explore_value = explore(sonar, mem)

            lspeed, rspeed = coordinator(track_value, avoid_value, explore_value)

            # Action
            mem = {
                "sonar": sonar,
                "blobs": blobs,
                "coord": coord,
                "lspeed":lspeed, 
                "rspeed": rspeed
            }
            setSpeed(clientID, hRobot, lspeed, rspeed)
            time.sleep(0.01)

        print('### Finishing...')
        sim.simxFinish(clientID)

    print('### Program ended')

# --------------------------------------------------------------------------

if __name__ == '__main__':
    main()
