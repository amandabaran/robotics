import cv2
import numpy as np
import Motor
import time 
import socket
import math
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1

# Small Duck Colors
H_low = 0
H_high = 179
S_low= 183
S_high = 255
V_low= 171
V_high = 255
    
class Robot:
    def __init__(self):
        self.PWM = Motor.Motor()

    def move(self, left_speed, right_speed):
        self.PWM.setMotorModel(int(left_speed), int(left_speed), int(right_speed), int(right_speed))

    def stop(self):
        # Stop
        self.PWM.setMotorModel(0,0,0,0)

positions = {}
rotations = {}
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
robot_id = 207
start_p = []
robot = Robot()
    
# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    # Position and rotation received
    positions[robot_id] = position
    # The rotation is in quaternion. We need to convert it to euler angles

    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)

    rotations[robot_id] = rotz
    
def connect():
    IP_ADDRESS = '192.168.0.207'
    
    clientAddress = "192.168.0.207"
    optitrackServerAddress = "192.168.0.4"

    # This will create a new NatNet client
    streaming_client = NatNetClient()
    streaming_client.set_client_address(clientAddress)
    streaming_client.set_server_address(optitrackServerAddress)
    streaming_client.set_use_multicast(True)
    # Configure the streaming client to call our rigid body handler on the emulator to send data out.
    streaming_client.rigid_body_listener = receive_rigid_body_frame
    streaming_client.run()
    
class Cam:
    
    def __init__(self, raspberrypi = False):
        
        self.raspberrypi = raspberrypi
       
        if raspberrypi:
            # Only works on raspberry pi
            from picamera2 import Picamera2
            self.cam = Picamera2()
            self.cam.configure(self.cam.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
            self.cam.start()
        else:
            self.cam = cv2.VideoCapture(0)
    
    def load_image(self):
        if self.raspberrypi:
            return self.cam.capture_array()
        else:
            _, frame = self.cam.read()
            return frame
        
    def stop(self):
        if not self.raspberrypi:
            self.cam.release()
        cv2.destroyAllWindows()
    
    def find_blobs(self,lower_val, upper_val):
        # get image from robot camera
        frame = self.load_image()     
        # converts the BGR color space of image to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  
        
        # preparing the mask to overlay
        mask = cv2.inRange(hsv, lower_val, upper_val)
        
        # The black region in the mask has the value of 0,
        # so when multiplied with original image removes all non-blue regions
        result = cv2.bitwise_and(frame, frame, mask = mask)
        
        result = cv2.bitwise_not(result)
        
        # covert to grayscale and apply blur to reduce noise
        gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 11)
        
        # Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()
        
        # Change thresholds
        params.minThreshold = 100;
        params.maxThreshold = 255;
        
        # Filter by Area.
        params.filterByArea = True
        params.minArea = 1000
        params.maxArea = 2**64
        
        # Filter by Circularity
        params.filterByCircularity = False
        params.minCircularity = 0.1
        
        # Filter by Convexity
        params.filterByConvexity = False
        params.minConvexity = 0.87
        
        # Filter by Inertia
        params.filterByInertia = False
        params.minInertiaRatio = 0.01
        
        detector = None
        
        # Create a detector with the parameters
        ver = (cv2.__version__).split('.')
        if int(ver[0]) < 3 :
            detector = cv2.SimpleBlobDetector(params)
        else : 
            detector = cv2.SimpleBlobDetector_create(params)
        
        # Detect blobs.
        keypoints = detector.detect(gray)
        
        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(gray, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        
        blobs = []
        for k in keypoints:
            blobs.append([k.pt[0], k.pt[1], k.size])
        
        if len(blobs) != 0:
            return np.array([blobs]), im_with_keypoints
        
        return None, im_with_keypoints
        
        
    def find_duck(self, lower_val, upper_val):    
        blobs, gray = self.find_blobs(lower_val, upper_val)
        
        if blobs is None:
            return (None, gray)
        else:
            blobs = np.uint16(np.around(blobs))
        
            radius = -10000
            ball = None
            
            for b in blobs[0, :]:
                r = b[2]
                if r > radius:
                    radius = r
                    ball = b
                    
            return ball, gray
    
def calc_distance(s):
    oos = 1/s
    b = 1.285967919
    m = 1822.493965
    return m * oos + b

def calc_theta(u):
    b = 0.4044249454
    m = -0.001396264737
    return m * u + b

def get_position():
    while True:
        if robot_id in positions:
            x = positions[robot_id][0]
            y = positions[robot_id][1]
            z = positions[robot_id][2]
            r = rotations[robot_id]
            return[x,y,z,r]
            # print('Last position', x, y, z, ' rotation', r)

def drive_in_circle():
    #start position
    start_x, start_y, _, _ = get_position()
    start_p = np.array([start_x, start_y])
    
    # K values
    move_k = 1500
    turn_k = 1500
    
    s_t = time.time()
    
    while True:
        # get position
        p = get_position()

        x = p[0]
        y = p[1]
        theta = p[3]
        
        t = time.time()-s_t

        xd = np.array([np.cos(t/3) - 1, np.sin(t/3)]) + start_p

        #convert theta to radians
        theta = theta * (np.pi / 180)

        x_t = np.array([x, y]) 

        dist_err = xd - x_t

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
        
        # Look for ducks
        
        blob, gray = cam.find_duck(np.array([H_low, S_low, V_low]), np.array([H_high, S_high, V_high]))
        
        if blob is not None:
            found_blob = True
            dist = calc_distance(blob[2]) / 100 #convert to m
            theta = calc_theta(blob[0])
            
            duck_x = dist * np.cos(theta) + .16 #add the distance from the optitrack balls to the camera
            duck_y = dist * np.sin(theta)
            
            p1 = x_t[0]
            p2 = x_t[1]
            
            t_matrix = np.array([ [np.cos(theta), -np.sin(theta), p1], 
                       [np.sin(theta), np.cos(theta), p2], 
                       [0, 0, 1] ])
            
            duck_p = np.array([duck_x, duck_y]) 
            duck = np.dot(t_matrix, duck_p)
        
            print(duck[0], duck[1], sep=",")

        robot.move(u[0], u[1])
    
        # Sleep for a second before sensing again
        time.sleep(0.1)

cam = Cam(True) #use raspberry pi   
               
try:
    
    # connect to optitrack
    connect()
    
    drive_in_circle()
    
except KeyboardInterrupt:
    robot.stop()
    cam.stop()

