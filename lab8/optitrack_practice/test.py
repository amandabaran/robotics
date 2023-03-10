import sys
import time
import socket
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1

positions = {}
rotations = {}

def stop():
    command = 'CMD_MOTOR#00#00#00#00\n'
    s.send(command.encode('utf-8'))
    is_running = False

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    # Position and rotation received
    positions[robot_id] = position
    # The rotation is in quaternion. We need to convert it to euler angles

    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)

    rotations[robot_id] = rotz


if __name__ == "__main__":
    # hostname = socket.gethostname()
    # ip_addr = socket.gethostbyname(hostname)
    clientAddress = "192.168.0.7"
    optitrackServerAddress = "192.168.0.4"
    robot_id = 207

    # This will create a new NatNet client
    streaming_client = NatNetClient()
    streaming_client.set_client_address(clientAddress)
    streaming_client.set_server_address(optitrackServerAddress)
    streaming_client.set_use_multicast(True)
    # Configure the streaming client to call our rigid body handler on the emulator to send data out.
    streaming_client.rigid_body_listener = receive_rigid_body_frame

    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    is_running = streaming_client.run()
    IP_ADDRESS = '192.168.0.207'

    # Connect to the robot
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((IP_ADDRESS, 5000))
    print('Connected')

    try:
        while is_running:
            if robot_id in positions:
                # last position
                print('Last position', positions, ' rotation', rotations)
          
                # Send control input to the motors
                command = 'CMD_MOTOR#1500#1500#1500#1500\n'
                s.send(command.encode('utf-8'))
                # Wait for 1 second
                time.sleep(.1)


    except KeyboardInterrupt:
        # STOP
        stop()