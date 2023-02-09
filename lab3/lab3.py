import Motor
import Ultrasonic
import time

PWM = Motor.Motor()
ultra=Ultrasonic.Ultrasonic()   

def moveForward(speed, t):
    # move forward for 2 seconds
    PWM.setMotorModel(speed,speed,speed,speed)
    time.sleep(t)

def moveBackward(speed, t):
    # move forward for 2 seconds
    PWM.setMotorModel(-speed,-speed,-speed,-speed)
    time.sleep(t)   

def stop():
     # Stop
    PWM.setMotorModel(0,0,0,0)
    time.sleep(1)
    
def program1():
    for x in [500, 1000, 1500, 2000]:
        moveForward(x, 2)
        stop()
        moveBackward(x, 2)
        stop()

def sense():
    return ultra.get_distance()

def program2():
    while True:
        print(sense())
        time.sleep(2)

def program3():
    k = -100
    x = sense()
    while x != 50:
        x = sense()
        u = k * (50 - x)
        print(u, x)
        moveForward(u, .1)
        time.sleep(0.1)
    stop()

def program4(): 
    k = -100
    start_t = time.time()
    x = sense()
    while x != 50:
        x = sense()
        u = k * (50 - x)
        print(u, x)
        moveForward(u, .1)
        time.sleep(0.001)
    stop()
    end_t = time.time()
    print("Time to finish: ", end_t - start_t);

# Time from x cm back with k = -100
# 60: 0.76 (did not adjust)
# 70: 1.26  (backed up just a bit)
# 80: 2.25858473777771
# 90: 1.89 (adjusted even less)
# 100: 1.7934198379516602


def program5(k): 
    start_t = time.time()
    x = sense()
    while x != 50:
        x = sense()
        u = k * (50 - x)
        print(u, x)
        moveForward(u, .1)
        time.sleep(0.001)
    stop()
    end_t = time.time()
    print("Time to finish: ", end_t - start_t);

#
# -250: 3.132249116897583
# -50: 0.96 did not adjust
#-1000: too fast, values sense around 50 but wouldnt stop

if __name__=='__main__':
    try:
        # program5(-250) 
        # program5(-50)
        program5(-1000)
        
    except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program destroy() will be  executed.
        stop()