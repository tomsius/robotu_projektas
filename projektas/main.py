"""
DOKUMENTACIJA:
https://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm
"""

import sim
import sys
import time
import numpy as np
import matplotlib.pyplot as plt

# inicializacija
sim.simxFinish(-1)                                                  # just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)     # Connect to CoppeliaSim

if clientID != -1:
    print('Connected to remote API server')
else:
    sys.exit('Failed connecting to remote API server')

# kairiojo rato valdymas
return_code, left_motor_handle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_oneshot_wait)
if return_code == 0:
    print('OK - left motor handle assigned')
else:
    print('Failed to get left motor handle')

# desiniojo rato valdymas
return_code, right_motor_handle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_oneshot_wait)
if return_code == 0:
    print('OK - right motor handle assigned')
else:
    print('Failed to get right motor handle')

# komandos ratams
time_end = time.time() + 5;
while time.time() < time_end:
    return_code = sim.simxSetJointTargetVelocity(clientID, left_motor_handle, 0.2, sim.simx_opmode_streaming)
    return_code = sim.simxSetJointTargetVelocity(clientID, right_motor_handle, 0.2, sim.simx_opmode_streaming)

time_end = time.time() + 5;
while time.time() < time_end:
    return_code = sim.simxSetJointTargetVelocity(clientID, left_motor_handle, -0.2, sim.simx_opmode_streaming)
    return_code = sim.simxSetJointTargetVelocity(clientID, right_motor_handle, -0.2, sim.simx_opmode_streaming)

return_code = sim.simxSetJointTargetVelocity(clientID, left_motor_handle, 0, sim.simx_opmode_streaming)
return_code = sim.simxSetJointTargetVelocity(clientID, right_motor_handle, 0, sim.simx_opmode_streaming)

# vieno is sensoriaus paemimas
return_code, sensor1 = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor1', sim.simx_opmode_oneshot_wait)
if return_code == 0:
    print('OK - sensor1 handle assigned')
else:
    print('Failed to get sensor1 handle')

# sensoriaus duomenu nuskaitymas
# pirma karta duomenys skaitomi su sim.simx_opmode_streaming
return_code, detection_state, detected_point, detected_object_handle, detected_surface_normal_vector = sim.simxReadProximitySensor(clientID, sensor1, sim.simx_opmode_streaming)
if return_code == 0 or return_code == 1:
    print('OK - ultrasonic sensor initial read')
else:
    print('Failed initial ultrasonic sensor read')

# kitus kartus sensoriaus duomenys skaitomi su sim.simx_opmode_buffer
time_end = time.time() + 5;
while time.time() < time_end:
    return_code, detection_state, detected_point, detected_object_handle, detected_surface_normal_vector = sim.simxReadProximitySensor(clientID, sensor1, sim.simx_opmode_buffer)
    if return_code == 0 or return_code == 1:
        print('OK - ultrasonic sensor read')
        print(detection_state)
        print(detected_point)
        print(detected_object_handle)
        print(detected_surface_normal_vector)
    else:
        print('Failed to read ultrasonic sensor')
    time.sleep(1)

# image paemimas is kameros
return_code, cam_handle = sim.simxGetObjectHandle(clientID, 'cam1', sim.simx_opmode_oneshot_wait)
if return_code == 0:
    print('OK - cam1 handle assigned')
else:
    print('Failed to get cam1 handle')

return_code, resolution, image = sim.simxGetVisionSensorImage(clientID, cam_handle, 0, sim.simx_opmode_streaming)
if return_code == 0 or return_code == 1:
    print('OK - initial imgae read')
else:
    print('Failed initial image read')

time_end = time.time() + 5;
while time.time() < time_end:
    return_code, resolution, image = sim.simxGetVisionSensorImage(clientID, cam_handle, 0, sim.simx_opmode_streaming)
    if return_code == 0:
        print('OK - imgae read')
        print(resolution)
        print(image)
        im = np.array(image, dtype=np.uint8)
        im.resize([resolution[0], resolution[1], 3])
        print(im.shape)
        plt.imshow(im, origin='lower')
        plt.show(block=False)
        plt.pause(0.5)
        plt.close()
    else:
        print('Failed to read image')
        time.sleep(1)


sim.simxFinish(clientID)