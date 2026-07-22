# drive.py

from gpiozero import Motor, PWMOutputDevice, AngularServo


class Drive:

    def __init__(self):

        self.IN1 = 27
        self.IN2 = 25
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

        self.center_angle = 80

        # servo range: 45 (left, 100%) - 80 (center) - 95 (right, 100%)
        self.left_limit = 45
        self.right_limit = 95

        # range is asymmetric (right: +15 deg, left: -35 deg), so
        # a single scale would clamp one side early. Two scales let
        # steering_value=100 reach exactly +15 (full right, 95deg) and
        # steering_value=-100 reach exactly -35 (full left, 45deg).
        self.steering_scale_right = 15 / 100   # 0.15
        self.steering_scale_left = 35 / 100    # 0.35

        self.reverse_steering = False

        self.stop()

    def set_steering(self, steering_value):

        steering_value = max(
            -100,
            min(100, steering_value)
        )

        if self.reverse_steering:
            steering_value = -steering_value

        if steering_value >= 0:
            angle = self.center_angle + (steering_value * self.steering_scale_right)
        else:
            angle = self.center_angle + (steering_value * self.steering_scale_left)

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

    def set_speed(self, speed_value):
        """
        Updates ONLY the PWM duty cycle (speed), without touching the
        direction pins. Use this inside a fast loop once forward()/
        backward() has already been called to set direction - calling
        forward()/backward() repeatedly at high frequency re-triggers
        the H-bridge direction pins every time, which can cause motor
        jitter and adds unnecessary overhead.
        """
        speed_value = max(0, min(1, speed_value))
        self.speed.value = speed_value

    def stop(self):

        self.motor.stop()
        self.speed.value = 0
        self.servo.angle = self.center_angle

    def cleanup(self):

        self.stop()
        self.servo.detach()
