import cv2
import numpy as np
import Motor
import time 

# Small Duck Colors
H_low = 0
H_high = 179
S_low= 183
S_high = 255
V_low= 171
V_high = 255

# https://blog.electroica.com/hsv-trackbar-opencv-python/
def callback(x):
    global H_low,H_high,S_low,S_high,V_low,V_high
    #assign trackbar position value to H,S,V High and low variable
    H_low = cv2.getTrackbarPos('low H','controls')
    H_high = cv2.getTrackbarPos('high H','controls')
    S_low = cv2.getTrackbarPos('low S','controls')
    S_high = cv2.getTrackbarPos('high S','controls')
    V_low = cv2.getTrackbarPos('low V','controls')
    V_high = cv2.getTrackbarPos('high V','controls')
    
class Robot:
    def __init__(self):
        self.PWM = Motor.Motor()

    def move(self, left_speed, right_speed):
        self.PWM.setMotorModel(left_speed, left_speed, right_speed, right_speed)

    def stop(self):
        # Stop
        self.PWM.setMotorModel(0,0,0,0)


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
        
        # cv2.imshow('frame', frame)
        # cv2.imshow('mask', mask)
        # cv2.imshow('result', result)
        
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
            # u, v, size
            # print("U: ", k.pt[0], "\nV: ", k.pt[1], "\nSize: ", k.size, "\n\n\n\n")
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
                    dist = calc_distance(b[2])
                    theta = calc_theta(b[0])
            return (ball, gray, dist, theta)
     
            
def calc_distance(s):
    oos = 1/s
    b = 1.285967919
    m = 1822.493965
    return m * oos + b

def calc_theta(u):
    b = 0.4044249454
    m = -0.001396264737
    return m * u + b
    
               
try:
    
    cam = Cam(True) #use raspberry pi
    robot = Robot()
    
    # use sliders to control color filter
    cv2.namedWindow('controls', cv2.WINDOW_NORMAL)
    cv2.createTrackbar('low H','controls',0,179,callback)
    cv2.createTrackbar('high H','controls',179,179,callback)
    cv2.createTrackbar('low S','controls',0,255,callback)
    cv2.createTrackbar('high S','controls',255,255,callback)
    cv2.createTrackbar('low V','controls',0,255,callback)
    cv2.createTrackbar('high V','controls',255,255,callback)
    
    found_ball = False

    while True:
    
        ball, gray, dist, theta = cam.find_duck(np.array([H_low, S_low, V_low]), np.array([H_high, S_high, V_high]))
          
        if ball is None:
            found_ball = False
        else :
            found_ball = True
            print("The duck is at distance ", dist, " and the angle is ", theta, "\n")
        
        print(found_ball, "\n\n")
        
        # if found_ball is False:
        #     robot.move(-1000, 1000)
            
        # else:
        #     # Below is helped from depaul due to my health abseences
        #     # move left if ball[0] > 320
        #     # move right if ball[0] < 320
        #     k1 = 2
        #     k2 = 900
        #     angle = 320 - ball[0]
        #     robot.move(k2-k1*angle, k2+k1*angle)
        
        cv2.imshow('frame', gray)   
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        # cv2.waitKey(0)
            
        
        time.sleep(0.1)
            
    
except KeyboardInterrupt:
    robot.stop()
    cam.stop()

