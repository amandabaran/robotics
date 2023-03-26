import cv2 as cv
import numpy as np
import Motor
import time  

class Robot:
    def __init__(self):
        self.PWM = Motor.Motor()
        self.ultra=Ultrasonic.Ultrasonic() 

    def move(self, speed):
        # move forward for 2 seconds
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
            self.cam.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
            self.cam.start()
        else:
            self.cam = cv.VideoCapture(0)
    
    def load_image(self):
        if self.raspberrypi:
            return self.cam.capture_array()
        else:
            _, frame = self.cam.read()
            return frame
        
    def find_ball(self, lower_val, upper_val):
        frame = self.load_image()
        
        # converts the BGR color space of image to HSV color space
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        
        # preparing the mask to overlay
        mask = cv.inRange(hsv, lower_val, upper_val)
        
        # The black region in the mask has the value of 0,
        # so when multiplied with original image removes all non-blue regions
        result = cv.bitwise_and(frame, frame, mask = mask)
    
        # cv.imshow('frame', frame)
        # cv.imshow('mask', mask)
        # cv.imshow('result', result)
        
        # covert to grayscale and apply blur to reduce noise
        gray = cv.cvtColor(result, cv.COLOR_BGR2GRAY)
        gray = cv.medianBlur(gray, 5)
        
        
        # Hough cirlce transform
        rows = gray.shape[0]
        circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, 1, rows / 8,
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
        cv.destroyAllWindows()
    
try:
    
    cam = Cam()
    robot = Robot(True)
    
    while True:
      
        # Threshold of blue in HSV space
        lower_blue = np.array([60, 35, 140])
        upper_blue = np.array([180, 255, 255])
        
        ball = cam.find_ball(lower_blue, upper_blue)
        
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

