import sim
import sys
import time
import numpy as np
import matplotlib.pyplot as plt

def connect():
    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
    if clientID != -1:
        print('Connected to remote API server')
        return clientID
    else:
        sys.exit('Failed connecting to remote API server')

def getHandle(client, objectID):
    retVal, handle = sim.simxGetObjectHandle(client, objectID, sim.simx_opmode_oneshot_wait)
    if retVal == 0:
        print('OK - ' + objectID + ' handle assigned')
        return handle
    else:
        sys.exit('Failed to get ' + objectID + ' handle')

def disconnect(client):
    sim.simxFinish(client)

def move(client, leftMotor, leftSpeed, rightMotor, rightSpeed):
    retLeft = sim.simxSetJointTargetVelocity(client, leftMotor, leftSpeed, sim.simx_opmode_streaming)
    retRight = sim.simxSetJointTargetVelocity(client, rightMotor, rightSpeed, sim.simx_opmode_streaming)
    return retLeft | retRight

def moveForward(client, leftMotor, rightMotor, speed):
    retLeft = sim.simxSetJointTargetVelocity(client, leftMotor, speed, sim.simx_opmode_streaming)
    retRight = sim.simxSetJointTargetVelocity(client, rightMotor, speed, sim.simx_opmode_streaming)
    return retLeft | retRight

def moveBackwards(client, leftMotor, rightMotor, speed):
    retLeft = sim.simxSetJointTargetVelocity(client, leftMotor, -speed, sim.simx_opmode_streaming)
    retRight = sim.simxSetJointTargetVelocity(client, rightMotor, -speed, sim.simx_opmode_streaming)
    return retLeft | retRight

def turnLeft(client, leftMotor, rightMotor, speed):
    retLeft = sim.simxSetJointTargetVelocity(client, leftMotor, 0, sim.simx_opmode_streaming)
    retRight = sim.simxSetJointTargetVelocity(client, rightMotor, speed, sim.simx_opmode_streaming)
    return retLeft | retRight

def turnRight(client, leftMotor, rightMotor, speed):
    retLeft = sim.simxSetJointTargetVelocity(client, leftMotor, speed, sim.simx_opmode_streaming)
    retRight = sim.simxSetJointTargetVelocity(client, rightMotor, 0, sim.simx_opmode_streaming)
    return retLeft | retRight

def stop(client, leftMotor, rightMotor):
    retLeft = sim.simxSetJointTargetVelocity(client, leftMotor, 0, sim.simx_opmode_oneshot_wait)
    retRight = sim.simxSetJointTargetVelocity(client, rightMotor, 0, sim.simx_opmode_oneshot_wait)
    return retLeft | retRight

def getPosition(client, handle):
    retVal, pos = sim.simxGetObjectPosition(client, handle, -1, sim.simx_opmode_oneshot_wait)
    return pos

def getRotation(client, handle):
    retVal, rot = sim.simxGetObjectOrientation(client, handle, -1, sim.simx_opmode_oneshot_wait)
    return rot

def isApproximatePosition(source, dest, error):
    retVal = True;
    retVal = retVal & (abs(source[0] - dest[0]) < error)
    retVal = retVal & (abs(source[1] - dest[1]) < error)
    retVal = retVal & (abs(source[2] - dest[2]) < error)
    return retVal

def isApproximateRotation(source, dest, error):
    return abs(source - dest) < error

def rotateUntil(client, robot, leftMotor, rightMotor, angle):
    rot = getRotation(client, robot)
    print('Current rotation: ' + str(rot[2]) + '\nDesired rotation: ' + str(angle))
    if angle < rot[2]:
        while not isApproximateRotation(rot[2], angle, 0.01):
            turnLeft(client, leftMotor, rightMotor, 0.2)
            rot = getRotation(client, robot)
    elif angle > rot[2]:
        while not isApproximateRotation(rot[2], angle, 0.01):
            turnRight(client, leftMotor, rightMotor, 0.2)
            rot = getRotation(client, robot)
    stop(client, leftMotor, rightMotor)

def getDesiredRotation(source, target):
    return np.arctan2(target[1] - source[1], target[0] - source[1])

def bug0(client, robot, leftMotor, rightMotor, sensors):
    destHandle = getHandle(client, 'destination1')
    destPos = getPosition(client, destHandle)
    robPos = getPosition(client, robot)
    
    desAngle = getDesiredRotation(robPos, destPos)
    rotateUntil(client, robot, leftMotor, rightMotor, desAngle)

    while not isApproximatePosition(robPos, destPos, 0.2):
        moveForward(client, leftMotor, rightMotor, 1)
        robPos = getPosition(client, robot)
    stop(client, leftMotor, rightMotor)

def main():
    clientID = connect()
    robot = getHandle(clientID, 'Pioneer_p3dx')
    leftMotor = getHandle(clientID, 'Pioneer_p3dx_leftMotor')
    rightMotor = getHandle(clientID, 'Pioneer_p3dx_rightMotor')
    sensors = []
    for i in range(16):
        sensor = getHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(i + 1))
    
    bug0(clientID, robot, leftMotor, rightMotor, sensors)
    stop(clientID, leftMotor, rightMotor)
    disconnect(clientID)

if __name__ == "__main__":
    main()
