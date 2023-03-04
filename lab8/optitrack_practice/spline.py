import socket
import sys
import time
import socket
import math
import numpy as np
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1

sys.path.insert(0, "..")

import trajectory.piecewise as piecewise

positions = {}
rotations = {}
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
robot_id = 207
    
# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    # Position and rotation received
    positions[robot_id] = position
    # The rotation is in quaternion. We need to convert it to euler angles

    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)

    rotations[robot_id] = rotz

def connect():
    IP_ADDRESS = '192.168.0.207'

    # Connect to the robot
    s.connect((IP_ADDRESS, 5000))
    print('Connected')
    
    clientAddress = "192.168.0.15"
    optitrackServerAddress = "192.168.0.4"

    # This will create a new NatNet client
    streaming_client = NatNetClient()
    streaming_client.set_client_address(clientAddress)
    streaming_client.set_server_address(optitrackServerAddress)
    streaming_client.set_use_multicast(True)
    # Configure the streaming client to call our rigid body handler on the emulator to send data out.
    streaming_client.rigid_body_listener = receive_rigid_body_frame
    streaming_client.run()

def stop():
    print("STOPPING")
     # Stop
    command = 'CMD_MOTOR#00#00#00#00\n'
    s.send(command.encode('utf-8'))
    # Close the connection
    s.shutdown(2)
    s.close()
    
def get_position():
    while True:
        if robot_id in positions:
            x = positions[robot_id][0]
            y = positions[robot_id][1]
            z = positions[robot_id][2]
            r = rotations[robot_id]
            return[x,y,z,r]
            # print('Last position', x, y, z, ' rotation', r)

def rectangle(t, a):
    p1 = np.array([5.0, 3.5])
    p2 = np.array([-3.8, 3.5])
    p3 = np.array([-3.8, -2.5])
    p4 = np.array([5.0, -2.5])

    t = (t/a) % 4

    if t < 1:
        return (p2-p1)*(t) + p1
    elif t < 2:
        return (p3-p2)*(t-1) + p2
    elif t < 3:
        return (p4-p3)*(t-2) + p3
    else:
        return (p1-p4)*(t-3) + p4


try:
    connect()

    x_points = [5.0, -3.8, -3.8, 5.0, 5.0]
    y_points = [3.5, 3.5, -2.5, -2.5, 3.5]
    vx = [-1.0, 0.0, 1.0, 0.0, 0.0]
    vy = [0.0, -1.0, 0.0, 1.0, 0.0]
    t = [0.0, 1.0, 2.0, 3.0, 4.0]

    time_, trajectory_x, trajectory_y, trajectory_dx, trajectory_dy = piecewise.spline_2d(x_points, y_points, vx, vy, t)

    points = [np.array([x, y]) for x, y in zip(trajectory_x, trajectory_y) ]
   

    time_, trajectory_x, trajectory_y, trajectory_dx, trajectory_dy = piecewise.spline_2d(x_points, y_points, vx, vy, t, 10)

    points = [np.array([x, y]) for x, y in zip(trajectory_x, trajectory_y) ]

    # print(points)
   
    point_num = 0
    xd = points[0]

    # K values
    move_k = 1500
    turn_k = 3000

    s_t = time.time()

    while True:
        # get position
        p = get_position()

        x = p[0]
        y = p[1]
        theta = p[3]

        xd = rectangle(time.time() - s_t, 20)

        #convert theta to radians
        theta = theta * (np.pi / 180)

        x_t = np.array([x, y]) 

        dist_err = xd - x_t

        # distance to end goal
        r = np.linalg.norm(dist_err)

        # break when close to destination
        if (r < 0.3):
            point_num += 1
            # move to next point as goal
            xd = points[point_num % len(points)]
            dist_err = xd - x_t
            r = np.linalg.norm(dist_err)

        # desired angle
        angle = np.arctan2(dist_err[1], dist_err[0])
        
        # needed correction to reach desired angle
        angle_diff = np.arctan2(np.sin(angle - theta), np.cos(angle - theta))

        # Print results to be exported to csv
        print(time.time()-s_t, xd[0], x, xd[1], y, sep=",")
        
        # Caclulate velocity and angular velocity
        v = move_k * r 
        w = turn_k * angle_diff

        # Controller 
        u = np.array([v - w, v + w])
        u[u > 1500.] = 1500.
        u[u < -1500.] = -1500.

        # Send control input to the motors
        command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u[0], u[0], u[1], u[1])
        s.send(command.encode('utf-8'))

        # Sleep for a second before sensing again
        time.sleep(0.1)

except KeyboardInterrupt or Exception:
    # STOP
    stop()