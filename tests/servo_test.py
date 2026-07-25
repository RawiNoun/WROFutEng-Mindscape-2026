	#!/usr/bin/env python3
"""
servo_test.py  --  the simplest possible servo check. No camera, no motor.

If the servo does NOT move when you run this, the problem is power or wiring,
not the driving code. If it DOES move here, the problem is in how open_drive
calls it.

    python3 servo_test.py
Ctrl+C to stop.

"""
import time
from gpiozero import AngularServo

SERVO_PIN = 24   # same pin as your Drive class

servo = AngularServo(
    SERVO_PIN,
    min_angle=0,
    max_angle=180,
    min_pulse_width=0.0005,
    max_pulse_width=0.0025,
)

print("Sweeping servo. Watch the front wheels. Ctrl+C to stop.")
try:
    while True:
            servo.angle =100
            time.sleep(1)
            servo.angle =80
            time.sleep(1)
            servo.angle =50
            time.sleep(1)
            servo.angle =80
            time.sleep(1)

except KeyboardInterrupt:
          print("\ndone")

