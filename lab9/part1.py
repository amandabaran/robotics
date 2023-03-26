import sys
import cv2 as cv
import numpy as np
        
# def load_image(argv):
#     default_file = 'smarties.png'
#     filename = argv[0] if len(argv) > 0 else default_file
#     # Loads an image
#     src = cv.imread(cv.samples.findFile(filename), cv.IMREAD_COLOR)
#     # Check if image is loaded fine
#     if src is None:
#         print ('Error opening image!')
#         print ('Usage: hough_circle.py [image_name -- default ' + default_file + '] \n')
#         return -1
#     return src

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
        
    def stop(self):
        if not self.raspberrypi:
            self.cam.release()
        cv.destroyAllWindows()

    
        

    
try:
    
    cam = Cam()
    
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
        
        # May need to increase range for detection of ball in lab
        lower_green = np.array([36, 0 , 0])
        upper_green = np.array([86, 255, 255])
        
        # preparing the mask to overlay
        # mask = cv.inRange(hsv, lower_blue, upper_blue)
        mask = cv.inRange(hsv, lower_green, upper_green)
        
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
                                minRadius=1, maxRadius=400)
        
        
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

