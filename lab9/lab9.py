import numpy as np
# import Motor
import time 
from collections import defaultdict
import networkx as nx
from NatNetClient import NatNetClient
import matplotlib.pyplot as plt
import socket
import sys
import robot
from util import quaternion_to_euler_angle_vectorized1


# from util import quaternion_to_euler_angle_vectorized1

positions = {}
rotations = {}
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
robot_id = 307

PATH = {}
path_step = -1
LENGTH = -1

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
    
    clientAddress = "192.168.0.115"
    optitrackServerAddress = "192.168.0.172"

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


node_names = dict()
obstacles = [5, 12, 13]
G = nx.Graph()  # nodes are two-tuples (x,y)
PATH = []
path_step = 1

# Distance heuristic calculates distance between 2 points
def dist(a, b):
    return np.linalg.norm(G.nodes[a]["pos"] - G.nodes[b]["pos"])

def make_graph():
    global node_names
    global obstacles
    global G
    
    grid = np.array([[[4.5, -0.9], [4.5, -0.2], [4.5, 0.5], [4.5, 1.2], [4.5, 1.9]],
            [[5.2, -0.9], [5.2, -0.2], [5.2, 0.5], [5.2, 1.2], [5.2, 1.9]],
            [[5.9, -0.9], [5.9, -0.2], [5.9, 0.5], [5.9, 1.2], [5.9, 1.9]],
            [[6.6, -0.9], [6.6, -0.2], [6.6, 0.5], [6.6, 1.2], [6.6, 1.9]]])
    
    
    # print(grid.shape) # 4, 4, 2
    
    
    count = 0
    for i in range(0, grid.shape[0]):
        for j in range(0, grid.shape[1]):
            node_names[tuple(grid[i][j])] = count
            if node_names[tuple(grid[i][j])] not in obstacles:
                G.add_node(count, pos=grid[i][j])
            count += 1
    print(node_names)
    for i in range(0, grid.shape[0]):
        for j in range(0, grid.shape[1]):
            if node_names[tuple(grid[i][j])] not in obstacles:
                if j + 1 < grid.shape[1] and node_names[tuple(grid[i][j+1])] not in obstacles:
                    G.add_edge(node_names[tuple(grid[i][j])], node_names[tuple(grid[i][j+1])], weight=np.linalg.norm(grid[i][j+1] - grid[i][j]))
                if i + 1 < grid.shape[0] and node_names[tuple(grid[i+1][j])] not in obstacles:
                    G.add_edge(node_names[tuple(grid[i][j])], node_names[tuple(grid[i+1][j])], weight=np.linalg.norm(grid[i+1][j] - grid[i][j]))
        
    # print(G.edges())

def get_next_goal():
    global path_step
    if path_step < 0 or path_step >= len(PATH):
        path_step = len(PATH) - 1
    return G.nodes[PATH[path_step]]["pos"]
    
    
try:
    make_graph()
    
    # 0 -15
    #15 - 14
    #18 - 8
    
    
    source = 18
    target = 8
    
    PATH = nx.astar_path(G, source, target, heuristic=dist)
    
    connect()


    # pos = nx.spring_layout(G)
    # nx.draw(G, pos, with_labels=True, node_color="#f86e00")
    # edge_labels = nx.get_edge_attributes(G, "cost")
    # nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)
    # plt.show()

    start = get_position()
    print("STARTING POSITON: ", start)
    
    final = get_next_goal()
    
    # K values
    move_k = 1500
    turn_k = 2000

    s_t = time.time()
    
    m = (final - np.array([start[0], start[1]]))
    
    print("Path: ", PATH)
   
    while True:
        # get position
        p = get_position()
        
        # make t a unit of 2 seconds
        t = (time.time() - s_t) / 3
        
        x = p[0]
        y = p[1]
        theta = p[3]
        
        x_t = np.array([x, y]) 
        
        end = (t*m) + np.array([start[0], start[1]])
        
        # when one unit of time passes, move to next point
        if (t > 1):
            s_t = time.time()
            t = (time.time() - s_t) / 3
            # increment path step
            path_step += 1
            if path_step >= len(PATH):
                m = np.array([0.0,0.0])
                start = get_next_goal()
            else:
                start = x_t
                print(x_t)
                final = get_next_goal()
                m = (final - start)
            end = (t*m) + start 
        
        #convert theta to radians
        theta = theta * (np.pi / 180)

        dist_err = end - x_t

        # distance to end goal
        r = np.linalg.norm(dist_err)

        # desired angle
        angle = np.arctan2(dist_err[1], dist_err[0])
        
        # needed correction to reach desired angle
        angle_diff = np.arctan2(np.sin(angle - theta), np.cos(angle - theta))
        
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

        
    
    

except KeyboardInterrupt:
    stop()

