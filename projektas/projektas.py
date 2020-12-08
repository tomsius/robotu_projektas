import sim
import sys
import time
import numpy as np
import matplotlib.pyplot as plt


def connect():
    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
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
    retLeft = sim.simxSetJointTargetVelocity(client, leftMotor, -speed, sim.simx_opmode_streaming)
    retRight = sim.simxSetJointTargetVelocity(client, rightMotor, speed, sim.simx_opmode_streaming)
    return retLeft | retRight


def turnRight(client, leftMotor, rightMotor, speed):
    retLeft = sim.simxSetJointTargetVelocity(client, leftMotor, speed, sim.simx_opmode_streaming)
    retRight = sim.simxSetJointTargetVelocity(client, rightMotor, -speed, sim.simx_opmode_streaming)
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


def getDistanceFromSensor(client, sensor):
    return_code, detection_state, detected_point, detected_object_handle, detected_surface_normal_vector = sim.simxReadProximitySensor(client, sensor, sim.simx_opmode_oneshot_wait)
    if detection_state:
        dist = np.sqrt(np.power(detected_point[0], 2) + np.power(detected_point[1], 2))
    else:
        dist = np.inf
    return dist


def isApproximatePosition(source, dest, error):
    retVal = True;
    # We only care about XY surface position
    retVal = retVal & (abs(source[0] - dest[0]) < error)
    retVal = retVal & (abs(source[1] - dest[1]) < error)
    return retVal


def isApproximateRotation(source, dest, error):
    return abs(source - dest) < error


def normalizeAngle(angle):
    if (angle < 0):
        return angle + 2 * np.pi
    else:
        return angle


def rotateUntilAngle(client, robot, leftMotor, rightMotor, angle, speed = 0.2, error = 0.01):
    rot = getRotation(client, robot)
    # Decide which direction to turn
    willTurnLeft = True
    normAngle = normalizeAngle(angle)
    normRot = normalizeAngle(rot[2])
    if normAngle > normRot:
        diff = normAngle - normRot
        if diff > np.pi:
            willTurnLeft = False
    else:
        diff = normRot - normAngle
        if diff < np.pi:
            willTurnLeft = False
    if willTurnLeft:
        while not isApproximateRotation(rot[2], angle, error):
            turnLeft(client, leftMotor, rightMotor, speed)
            rot = getRotation(client, robot)
    else:
        while not isApproximateRotation(rot[2], angle, error):
            turnRight(client, leftMotor, rightMotor, speed)
            rot = getRotation(client, robot)
    stop(client, leftMotor, rightMotor)


def getDesiredRotation(source, target):
    return np.arctan2(target[1] - source[1], target[0] - source[0])


def rotateTowards(client, robot, leftMotor, rightMotor, destinationHandle):
    destPos = getPosition(client, destinationHandle)
    # Repeat rotation 3 times for better accuracy
    # Because the center of the robot changes when rotating
    for i in range(3):
        if (i == 0):
            speed = 1.5
            error = 0.1
        else:
            speed = 0.2
            error = 0.01
        robPos = getPosition(client, robot)
        desAngle = getDesiredRotation(robPos, destPos)
        rotateUntilAngle(client, robot, leftMotor, rightMotor, desAngle, speed, error)


def moveForwardFor(client, leftMotor, rightMotor, speed, moveFor):
    time_end = time.time() + moveFor;
    while time.time() < time_end:
        moveForward(client, leftMotor, rightMotor, speed)

def wallFollowRHS(client, robot, leftMotor, rightMotor, sensor, speed):
    dist = getDistanceFromSensor(client, sensor)
    rotCoords = getRotation(client, robot)
    rot = normalizeAngle(rotCoords[2])
    prevRot = 0
    sumRotDelta = 0
    deg90 = np.pi / 2
    halfSpeed = speed / 3
    leftSpeed = speed
    rightSpeed = halfSpeed
    delta = 0
    prevDist = dist
    while sumRotDelta > -deg90 and sumRotDelta < deg90:
        if (dist < 0.08):
            leftSpeed = halfSpeed
            rightSpeed = speed
        elif (dist > 0.1):
            leftSpeed = speed
            rightSpeed = halfSpeed
        move(client, leftMotor, leftSpeed, rightMotor, rightSpeed)
        dist = getDistanceFromSensor(client, sensor)
        prevDist = dist
        rotCoords = getRotation(client, robot)
        rot = normalizeAngle(rotCoords[2])
        deltaRot = rot - prevRot
        prevRot = rot
        if (deltaRot < 0.1):
            sumRotDelta = sumRotDelta + deltaRot
    stop(client, leftMotor, rightMotor)


def bug0(client, robot, leftMotor, rightMotor, sensors, minWallDist = 0.15):
    print('BUG0 - started.')
    destHandle = getHandle(client, 'destination1')
    criticalDist = minWallDist / 3
    destPos = getPosition(client, destHandle)
    robPos = getPosition(client, robot)
    rotateTowards(client, robot, leftMotor, rightMotor, destHandle)
    while not isApproximatePosition(robPos, destPos, 0.05):
        dist1 = getDistanceFromSensor(client, sensors[3])
        dist2 = getDistanceFromSensor(client, sensors[4])
        if (dist1 < minWallDist and dist2 < minWallDist) or dist1 < criticalDist or dist2 < criticalDist:
            # rotate left
            prevDist1 = np.inf
            prevDist2 = np.inf
            sensDist1 = getDistanceFromSensor(client, sensors[7])
            sensDist2 = getDistanceFromSensor(client, sensors[8])
            print('rotating left')
            diff = np.abs(sensDist1 - sensDist2)
            while sensDist2 == np.inf or sensDist1 < prevDist1 or sensDist2 < prevDist2:
                turnLeft(client, leftMotor, rightMotor, 1)
                prevDist1 = sensDist1
                sensDist1 = getDistanceFromSensor(client, sensors[7])
                prevDist2 = sensDist2
                sensDist2 = getDistanceFromSensor(client, sensors[8])
                diff = np.abs(sensDist1 - sensDist2)
            print('moving until no obstacle')
            wallFollowRHS(client, robot, leftMotor, rightMotor, sensors[7], 1.5)
            stop(client, leftMotor, rightMotor)
            # rotate towards target
            print('rotating towards target')
            rotateTowards(client, robot, leftMotor, rightMotor, destHandle)
        moveForward(client, leftMotor, rightMotor, 1.5)
        robPos = getPosition(client, robot)
    stop(client, leftMotor, rightMotor)
    print('BUG0 - destination reached!')


def correctAngle(angle):
    if (angle > np.pi):
        return angle - 2 * np.pi
    elif (angle < -np.pi):
        return angle + 2 * np.pi
    else:
        return angle


def isApproximate(val1, val2, error = 0.1):
    return (np.abs(val1 - val2)) < error


def turn90Degrees(client, robot, leftMotor, rightMotor, direction, speed = 0.2):
    stop(client, leftMotor, rightMotor)
    rot = getRotation(client, robot)
    deg90 = np.pi / 2
    if direction == 'right':
        if isApproximate(rot[2], 0):
            rotateUntilAngle(client, robot, leftMotor, rightMotor, -deg90)
        elif isApproximate(rot[2], deg90):
            rotateUntilAngle(client, robot, leftMotor, rightMotor, 0)
        elif isApproximate(rot[2], -deg90):
            rotateUntilAngle(client, robot, leftMotor, rightMotor, np.pi)
        else:
            rotateUntilAngle(client, robot, leftMotor, rightMotor, deg90)
    elif direction == 'left':
        if isApproximate(rot[2], 0):
            rotateUntilAngle(client, robot, leftMotor, rightMotor, deg90)
        elif isApproximate(rot[2], deg90):
            rotateUntilAngle(client, robot, leftMotor, rightMotor, np.pi)
        elif isApproximate(rot[2], -deg90):
            rotateUntilAngle(client, robot, leftMotor, rightMotor, 0)
        else:
            rotateUntilAngle(client, robot, leftMotor, rightMotor, -deg90)


def maze(client, robot, leftMotor, rightMotor, frontSensor, rightSensor, leftSensor):
    print('Maze by right hand rule - started')
    stop(client, leftMotor, rightMotor)
    destHandle = getHandle(client, 'destination2')
    robPos = getPosition(client, robot)
    destPos = getPosition(client, destHandle)
    while not isApproximatePosition(robPos, destPos, 0.2):
        distanceFront = getDistanceFromSensor(client, frontSensor)
        distanceRight = getDistanceFromSensor(client, rightSensor)
        distanceLeft = getDistanceFromSensor(client, leftSensor)
        moveForward(client, leftMotor, rightMotor, 2)
        if distanceFront < 0.06:
            print('Wall infront')
            stop(client, leftMotor, rightMotor)
            if distanceRight == np.inf:
                print('No wall on the right - turning 90 degrees right')
                turn90Degrees(client, robot, leftMotor, rightMotor, 'right')
            elif distanceLeft == np.inf:
                print('No wall on the left - turning 90 degrees left')
                turn90Degrees(client, robot, leftMotor, rightMotor, 'left')
            else:
                print('Cannot turn right or left - turning around')
                turn90Degrees(client, robot, leftMotor, rightMotor, 'right')
                turn90Degrees(client, robot, leftMotor, rightMotor, 'right')
        if distanceRight == np.inf:
            print('I can go right')
            stop(client, leftMotor, rightMotor)
            moveForwardFor(client, leftMotor, rightMotor, 2, 2)
            turn90Degrees(client, robot, leftMotor, rightMotor, 'right')
            moveForwardFor(client, leftMotor, rightMotor, 2, 2.5)
        robPos = getPosition(client, robot)
    print('Maze completed')


def main():
    clientID = connect()
    robot = getHandle(clientID, 'Pioneer_p3dx')
    leftMotor = getHandle(clientID, 'Pioneer_p3dx_leftMotor')
    rightMotor = getHandle(clientID, 'Pioneer_p3dx_rightMotor')
    sensors = []
    for i in range(16):
        sensor = getHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(i + 1))
        sensors.append(sensor)
    stop(clientID, leftMotor, rightMotor)
    bug0(clientID, robot, leftMotor, rightMotor, sensors)
    stop(clientID, leftMotor, rightMotor)
    rotateUntilAngle(clientID, robot, leftMotor, rightMotor, np.pi, 0.1, 0.005)
    destPos = [2.5, 8, 0]
    robPos = getPosition(clientID, robot)
    while not isApproximatePosition(robPos, destPos, 0.1):
        moveForward(clientID, leftMotor, rightMotor, 1)
        robPos = getPosition(clientID, robot)
    stop(clientID, leftMotor, rightMotor)
    maze(clientID, robot, leftMotor, rightMotor, sensors[4], sensors[7], sensors[0])
    stop(clientID, leftMotor, rightMotor)
    moveForwardFor(clientID, leftMotor, rightMotor, 2, 8)
    stop(clientID, leftMotor, rightMotor)
    disconnect(clientID)


if __name__ == "__main__":
    main()
