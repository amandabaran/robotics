import cv2
import numpy as np
import Motor
import time 


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
        
        
def calc_distance(s):
    oos = 1/s
    b = 1.285967919
    m = 1822.493965
    return m * oos + b

def calc_theta(u):
    b = 0.4044249454
    m = -0.001396264737
    return m * u + b