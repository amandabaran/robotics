import sys
import cv2 as cv
import numpy as np
import Motor
import Ultrasonic
import time  

class Robot:
    def __init__(self):
        self.PWM = Motor.Motor()
        self.ultra=Ultrasonic.Ultrasonic() 

    def moveForward(self, speed, t):
        # move forward for 2 seconds
        self.PWM.setMotorModel(speed,speed,speed,speed)
        time.sleep(t)

    def moveBackward(self, speed, t):
        # move forward for 2 seconds
        self.PWM.setMotorModel(-speed,-speed,-speed,-speed)
        time.sleep(t)   

    def stop(self):
        # Stop
        self.PWM.setMotorModel(0,0,0,0)
        time.sleep(1)

    def sense(self):
        return self.ultra.get_distance()

def run(k): 
    r = Robot()
    start_t = time.time()
    x = r.sense()
    while x != 50:
        x = r.sense()
        u = k * (50 - x)
        print(u, x)
        r.moveForward(u, .1)
        time.sleep(0.001)
    r.stop()
    end_t = time.time()
    print("Time to finish: ", end_t - start_t);

#
# -250: 3.132249116897583
# -50: 0.96 did not adjust
#-1000: too fast, values sense around 50 but wouldnt stop

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
    
    def stop(self):
        if not self.raspberrypi:
            self.cam.release()
        cv.destroyAllWindows()

    def load_image(self):
        if self.raspberrypi:
            return self.cam.capture_array()
        else:
            _, frame = self.cam.read()
            return frame
        

    
try:
    
    cam = Cam()
    robot = Robot()
    
    while True:
        
        # Close frame window by typing q
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
        
        # src = load_image(argv)
        frame = cam.load_image()
        
        # It converts the BGR color space of image to HSV color space
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        
        # Threshold of blue in HSV space
        lower_blue = np.array([60, 35, 140])
        upper_blue = np.array([180, 255, 255])
        
        # preparing the mask to overlay
        mask = cv.inRange(hsv, lower_blue, upper_blue)
        
        # The black region in the mask has the value of 0,
        # so when multiplied with original image removes all non-blue regions
        result = cv.bitwise_and(frame, frame, mask = mask)
    
        # cv.imshow('frame', frame)
        # cv.imshow('mask', mask)
        cv.imshow('result', result)
        
                
        # covert to grayscale and apply blur to reduce noise
        gray = cv.cvtColor(result, cv.COLOR_BGR2GRAY)
        gray = cv.medianBlur(gray, 5)
        
        
        # Hough cirlce transform
        rows = gray.shape[0]
        circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, 1, rows / 8,
                                param1=100, param2=30,
                                minRadius=1, maxRadius=100)
        
        
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                center = (i[0], i[1])
                # circle center
                cv.circle(gray, center, 1, (0, 100, 100), 3)
                # circle outline
                radius = i[2]
                cv.circle(gray, center, radius, (255, 0, 255), 3)
        
        # print(circles)
        cv.imshow('detected circles', gray)
        
    # When everything done, release the capture
    cam.stop()
    
except KeyboardInterrupt:
    cam.stop()
    stop()
