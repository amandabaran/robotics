class Robot:
    def __init__(self):
        self.PWM = Motor.Motor()

    def move(self, left_speed, right_speed):
        self.PWM.setMotorModel(left_speed, left_speed, right_speed, right_speed)

    def stop(self):
        # Stop
        self.PWM.setMotorModel(0,0,0,0)