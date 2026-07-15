from gpiozero import Motor, PWMOutputDevice
from time import sleep

# L298N control pins
IN1 = 27
IN2 = 22
ENA = 17

# Motor direction pins
motor = Motor(forward=IN1, backward=IN2)

# Speed control pin
speed = PWMOutputDevice(ENA)

try:
    # print("Forward")
    # speed.value = 0.4   # speed from 0.0 to 1.0
    # motor.forward()
    # sleep(3)

    # print("Stop")
    # motor.stop()
    # speed.value = 0
    # sleep(1)

    # print("Backward")
    # speed.value = 0.4
    # motor.backward()
    # sleep(3)

    # print("Stop")
    motor.stop()
    # speed.value = 0
    # sleep(1)

except KeyboardInterrupt:
    print("Stopping motor")
    motor.stop()
    speed.value = 0
