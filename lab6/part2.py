import cv2
import numpy as np
import Motor
import time 

class Robot:
    def __init__(self):
        self.PWM = Motor.Motor()
        self.ultra=Ultrasonic.Ultrasonic() 

    def move(self, speed):
        self.PWM.setMotorModel(-speed,-speed,speed,speed)

    def stop(self):
        # Stop
        self.PWM.setMotorModel(0,0,0,0)

    def sense(self):
        return self.ultra.get_distance()


class Cam:
    
    def __init__(self, raspberrypi = False):
        
        self.raspberrypi = raspberrypi
       
        if raspberrypi:
            # Only works on raspberry pi
            from picamera2 import Picamera2
            self.cam = Picamera2()
            self.cam.configure(Picamera2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
            self.cam.start()
        else:
            self.cam = cv2.VideoCapture(0)
    
    def load_image(self):
        if self.raspberrypi:
            return self.cam.capture_array()
        else:
            _, frame = self.cam.read()
            return frame
        
    def find_blob(self, lower, upper):
        # Read image
        im = self.load_image()
        
        # Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()
        
        # Change thresholds
        params.minThreshold = 10;
        params.maxThreshold = 200;
        
        # Filter by Area.
        params.filterByArea = True
        params.minArea = 1500
        
        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.1
        
        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.87
        
        # Filter by Inertia
        params.filterByInertia = True
        params.minInertiaRatio = 0.01
        
        # Create a detector with the parameters
        ver = (cv2.__version__).split('.')
        if int(ver[0]) < 3 :
            detector = cv2.SimpleBlobDetector(params)
        else : 
            detector = cv2.SimpleBlobDetector_create(params)
        
        # Detect blobs.
        keypoints = detector.detect(im)
        
        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        
        # Show keypoints
        cv2.imshow("Keypoints", im_with_keypoints)
        cv2.waitKey(0)
        
    def find_ball(self, lower_val, upper_val):
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
        
        # covert to grayscale and apply blur to reduce noise
        gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 5)
        
        
        # Hough cirlce transform
        rows = gray.shape[0]
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, rows / 8,
                                param1=100, param2=30,
                                minRadius=1, maxRadius=100)
        
        if (circles is None) or (len(circles) < 1):
            return None

        size = 0
        index = 0
        circles = circles[0]
        
        # Find biggest circle and store index
        for j in range(len(circles)):
            if circles[j][2] > size:
                index = j
                size = circles[j][2]
                  
        return circles[index]
        
    def stop(self):
        if not self.raspberrypi:
            self.cam.release()
        cv2.destroyAllWindows()
    
try:
    
    cam = Cam()
    robot = Robot()
    
    while True:
      
        # Threshold of blue in HSV space
        lower_blue = np.array([60, 35, 140])
        upper_blue = np.array([180, 255, 255])
        
        # May need to increase range for detection of ball in lab
        lower_green = np.array([36, 0 , 0])
        upper_green = np.array([86, 255, 255])
        
        ball = cam.find_ball(lower_green, upper_green)
        
        if ball is not None:
            found = True
            print("Found ball\n")
            robot.stop()
        
        else:
            robot.move(1200)
            
        time.sleep(0.1)
            
    
except KeyboardInterrupt:
    robot.stop()
    cam.stop()

