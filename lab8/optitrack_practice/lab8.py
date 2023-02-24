import socket
import sys
import time
import socket
import math
import numpy as np
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1


positions = {}
rotations = {}
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
robot_id = 207
s_t = 0
oritentation_error = []
distance_error = []

def connect():
    IP_ADDRESS = '192.168.0.207'

    # Connect to the robot
    s.connect((IP_ADDRESS, 5000))
    print('Connected')
    
    clientAddress = "192.168.0.7"
    optitrackServerAddress = "192.168.0.4"

    # This will create a new NatNet client
    streaming_client = NatNetClient()
    streaming_client.set_client_address(clientAddress)
    streaming_client.set_server_address(optitrackServerAddress)
    streaming_client.set_use_multicast(True)
    # Configure the streaming client to call our rigid body handler on the emulator to send data out.
    streaming_client.rigid_body_listener = receive_rigid_body_frame
    streaming_client.run()
    
# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    # Position and rotation received
    positions[robot_id] = position
    # The rotation is in quaternion. We need to convert it to euler angles

    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)

    rotations[robot_id] = rotz

def stop():
     # Stop
    command = 'CMD_MOTOR#00#00#00#00\n'
    s.send(command.encode('utf-8'))
    is_running = False
    
def get_position():
    while True:
        if robot_id in positions:
            x = positions[robot_id][0]
            y = positions[robot_id][1]
            z = positions[robot_id][2]
            r = rotations[robot_id]
            return[x,y,z,r]
            # print('Last position', x, y, z, ' rotation', r)

def calc_velocity(p, k, end):
    x = p[0]
    y = p[1]
    xd = end[0]
    yd= end[1]
    r = math.sqrt(((yd - y)**2) + ((xd - x)**2))
    # r is the distance to the goal at each time t
    v = k * r
    distance_error.append([(time.time()-s_t), r])
    return v

def calc_omega(p, k, end):   
    theta = p[3]
    x = p[0]
    y = p[1]
    xd = end[0]
    yd= end[1]
    a = math.atan((yd - y) / (xd - x))
    w = k * (a - theta)
    #orientation error is the difference in angles
    err = a - theta * math.pi / 180
    oritentation_error.append([(time.time()-s_t), err])
    return w
    
def controller(v, omega):
    u = np.array([v - omega, v + omega])
    u[u > 1500] = 1500
    u[u < -1500] = -1500
    # Send control input to the motors
    command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u[0], u[0], u[1], u[1])
    s.send(command.encode('utf-8'))
    time.sleep(0.1)

def run(start, end, k1, k2):
    p = start
    while True:
        p = get_position()
        v = calc_velocity(p, k2, end)
        w = calc_omega(p, k1, end)
        print("Error: " , oritentation_error[-1][0], distance_error[-1][1], oritentation_error[-1][1])
        controller(v, w) 
        if distance_error[-1][1] < 0.3 or oritentation_error[-1][1] < 0.2:
            print("DONE")
            break
    stop()
    
try:
    connect()
    
    s_t = time.time()
    start_pos = get_position()
    print(start_pos)
    # x, y , theta
    end_pos = [-1.0,-2.0]
    turnk = 200
    movek = 1000
    run(start_pos, end_pos, turnk, movek)

    # Wait for 1 second
    time.sleep(1)


except KeyboardInterrupt:
    # STOP
    stop()

# Close the connection
s.shutdown(2)
s.close()