import numpy as np
import apis
import time
import sys

factor = 3.0

def elipse(a, b, t, init):
    return np.array([init[0] + a - a * np.cos(t / factor), init[1] + b * np.sin(t / factor)])

if __name__ == "__main__":

    IP_ADDRESS = '192.168.0.207'

    robot = apis.Robot(IP_ADDRESS)
  
    duck = np.array([0.9794962406158447, 1.392021656036377])
    box_width = .65

    print("Starting")
    try:
        # hostname = socket.gethostname()
        # ip_addr = socket.gethostbyname(hostname)
        clientAddress = "192.168.0.25"
        optitrackServerAddress = "192.168.0.4"
        robot_id = 207
        
        # Start up the streaming client now that the callbacks are set up.
        # This will run perpetually, and operate on a separate thread.

        position = apis.Position(clientAddress, optitrackServerAddress, robot_id)

        K1 = 2000#100
        K2 = 2500
        
        start = time.time()

        init, angle = position.get()

        sign = -1

        if init[0] <= duck[0]:
            sign = 1

        a = sign * apis.dist(init, duck) / 2.0

        print(a)

        b = box_width
        
        while True:

            x_t, angle = position.get()

            if time.time() - start <= 2 * np.pi * factor:
                x_d = elipse(a, b, time.time() - start, init)
            else:
                x_d = init
            
            dist, desiredAngle = apis.dist_and_angle(x_d, x_t)

            v = K1 * dist
            angleDiff = apis.angle_diff(desiredAngle, angle)
            omega = K2 * angleDiff

            u = np.array([v - omega, v + omega])
            u[u > 1500.] = 1500.
            u[u < -1500.] = -1500.
            
            robot.set_motor(u[0], u[0], u[1], u[1])
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Shutting down")
        robot.stop_motor()
        time.sleep(1)
        robot.stop_motor()
        pass
    
    robot.stop_motor()
    robot.shutdown()
    print("Done")