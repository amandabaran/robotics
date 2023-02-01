import Motor
import Led
import time
import Buzzer

PWM = Motor.Motor()
led = Led.Led()
buzz = Buzzer.Buzzer()


def turn90degrees():
    # turn 90 degrees
    PWM.setMotorModel(-1000,-1000,1000,1000)
    time.sleep(0.9)

def moveForward():
    # move forward for 2 seconds
    PWM.setMotorModel(1000,1000,1000,1000)
    time.sleep(2)   

def stop():
     # Stop
    PWM.setMotorModel(0,0,0,0)   

def clearLights():
    led.ledIndex(0x1, 0, 0, 0)
    led.ledIndex(0x2, 0, 0, 0)
    led.ledIndex(0x4, 0, 0, 0)
    led.ledIndex(0x8, 0, 0, 0)
    
if __name__=='__main__':
    try:
        clearLights()
        moveForward()
        stop()          
        turn90degrees()
        stop()
        # Set LED 0 to red
        led.ledIndex(0x01, 255, 0, 0)
        moveForward()
        stop()
        turn90degrees()
        stop()
        # Set Led 1 to blue
        led.ledIndex(0x2, 0, 0, 255)
        moveForward()
        stop()
        turn90degrees()
        stop()
        # Set Led 2 to green
        led.ledIndex(0x4, 0, 255, 0)
        moveForward()
        stop()
        turn90degrees()
        stop()
        # Set Led 3 to yellow
        led.ledIndex(0x8, 255, 255, 0)
        stop()            
        # Buzz for 1 second
        buzz.run('1')
        time.sleep(1)
        buzz.run('0')
        
    except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program stop() will be  executed.
        Motor.stop()