# drive.py

from gpiozero import Motor, PWMOutputDevice, AngularServo


class Drive:

    def __init__(self):

        self.IN1 = 27
        self.IN2 = 22
        self.ENA = 17

        self.SERVO_PIN = 24

        self.motor = Motor(
            forward=self.IN1,
            backward=self.IN2
        )

        self.speed = PWMOutputDevice(
            self.ENA
        )

        self.servo = AngularServo(
            self.SERVO_PIN,
            min_angle=0,
            max_angle=180,
            min_pulse_width=0.0005,
            max_pulse_width=0.0025
        )

        self.center_angle = 90

        # wider steering range
        self.left_limit = 30
        self.right_limit = 150

        # increase steering strength
        self.steering_scale = 0.75

        self.reverse_steering = False

        self.stop()

    def set_steering(self, steering_value):

        steering_value = max(
            -100,
            min(100, steering_value)
        )

        if self.reverse_steering:
            steering_value = -steering_value

        angle = self.center_angle + (
            steering_value * self.steering_scale
        )

        angle = max(
            self.left_limit,
            min(self.right_limit, angle)
        )

        self.servo.angle = angle

        return angle

    def forward(self, speed_value=0.25):

        speed_value = max(0, min(1, speed_value))

        self.speed.value = speed_value
        self.motor.forward()

    def backward(self, speed_value=0.25):

        speed_value = max(0, min(1, speed_value))

        self.speed.value = speed_value
        self.motor.backward()

    def stop(self):

        self.motor.stop()
        self.speed.value = 0
        self.servo.angle = self.center_angle

    def cleanup(self):

        self.stop()
        self.servo.detach()