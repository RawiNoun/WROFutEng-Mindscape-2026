# Wiring Documentation

This document explains the electrical wiring of our WRO 2026 Future Engineers self-driving vehicle. It covers two diagrams stored in this folder:

- **`WRO26 wirings.png`** — the full electrical schematic of the vehicle (KiCad), showing power distribution, the Raspberry Pi 5 GPIO connections, the driving motor circuit, the steering servo, the ultrasonic distance sensors, the push button, and the status LED.
- **`camera wiring.png`** — the physical connection between the Raspberry Pi 5 and the Arducam 8MP camera module used for lane and obstacle detection.

The goal of this file is to make the wiring reproducible: another team should be able to look at the diagrams below and understand **what is connected to what, and why**, without needing to reverse-engineer our code.

---

## 1. System Overview

The vehicle's electronics are built around a **Raspberry Pi 5** as the single-board computer (SBC). It is responsible for:

- Reading an **8MP camera** feed, which is used to estimate what percentage of each wall (left/right) is visible in the frame. The robot uses this percentage as its main signal to stay centred between the two walls, and the camera is also used for traffic-sign (pillar) colour/side recognition in the Obstacle Challenge.
- Reading the two ultrasonic sensors, which are **not** used for continuous wall-distance measurement. They are used purely as an **emergency braking system**: if either sensor reports a distance below a safety threshold (i.e., the robot is about to hit a wall or pillar), the motor is cut/reversed regardless of what the camera-based control loop is doing.
- Driving the steering servo (SG90).
- Driving the rear axle DC motor through an L298N H-bridge motor driver.
- Reading the push button used as the mandatory **Start button** (see WRO rule 9.11).
- Driving a status LED that indicates the vehicle's power/ready state.

Two separate power sources are used, which is a deliberate design choice to protect the Raspberry Pi from motor electrical noise and voltage sag:

| Power source | Powers | Notes |
|---|---|---|
| Powerbank (22.5 W max output) | Raspberry Pi 5 (5V rail), logic side of the circuit | Clean, regulated 5V, isolated from motor current spikes. 22.5 W is the hard ceiling for everything drawing from this bank (Pi 5 + camera + sensors + servo + logic side of the L298N), so this is a real constraint we sized our components against, not just a convenient power source. |
| 12V Battery | L298N motor driver (Vs / motor supply) | Powers the DC drive motor directly through the H-bridge, completely separate from the powerbank |

The Raspberry Pi and the L298N share a **common ground (GND)** so that GPIO logic-level signals sent from the Pi to the L298N are read correctly. This is shown by the black ground wires connecting both power domains together in the schematic.

---

## 2. Main Circuit Diagram — `WRO26 wirings.png`

### 2.1 Raspberry Pi 5 GPIO Usage

The table below summarizes how each GPIO header connection is used in our circuit. Pin numbers refer to the physical Raspberry Pi 40-pin header (BCM/physical mapping is standard for the Pi 5).

| Subsystem | Signal | Raspberry Pi pin(s) / GPIO block | Wire colour on diagram |
|---|---|---|---|
| Steering servo (SG90) | PWM signal | GPIO24 / PWM0 (pin 32) | Line labelled "PWM1" next to the servo symbol |
| Status LED + resistor (R1/D1) / Push button (SW1) | Digital out / Digital in | GPIO22 (pin 33) area — the LED and the push button are both drawn on wires that group with the servo header at the top of the GPIO block | Red (through R1) / black-grey |
| Ultrasonic sensor 1 & 2 (Trig / Echo, ×2 sensors) | Digital out (Trig) / Digital in (Echo) | GPIO6, GPIO5, GPIO12, GPIO13 (pins 36, 11, 35, 38) — the SPI1 header block, repurposed as plain GPIO since we don't use SPI1 | Green |
| L298N motor driver | IN1, IN2, IN3, IN4, ENA, ENB (6 signals) | GPIO25–GPIO27 (pins 15, 16, 18, 22, 37, 13) — the SDIO header block, repurposed as plain GPIO since we don't use SDIO | Green |
| Powerbank | 5V / GND | Pin 2 (5V) / any GND pin | Red / black |

### 2.2 Power & Start-up Circuit

- The **Powerbank / power supply** feeds the Raspberry Pi's 5V rail directly.
- A separate **on/off switch** (labelled "Power supply" in the schematic) allows the whole vehicle to be switched fully off, satisfying WRO rule 9.6 (vehicle must be placed on the field **totally switched off**).
- The **push button (SW1)** is wired to a GPIO input and is used purely as the **Start button**: once the vehicle is switched on, our program waits in an idle state until this button is pressed, per WRO rule 9.10–9.11 (only one switch to power on, only one button to start).
- The **LED (D1) with current-limiting resistor (R1)** gives a visual indication of the robot's state (e.g., powered on / waiting for start / running), which is useful during judging and debugging.

### 2.3 Steering — SG90 Servo Motor

- The steering servo is connected to a single **PWM-capable GPIO pin** (GPIO12/PWM0).
- Power (5V) and ground are taken from the Raspberry Pi's 5V/GND pins, since the SG90 draws low enough current to be powered directly from the Pi during normal steering operation.
- Steering angle is controlled purely by varying the PWM duty cycle sent from this single GPIO pin — this satisfies the WRO requirement of a dedicated steering actuator separate from the driving motor(s) (rule 11.3).

### 2.4 Driving — DC Motor + L298N Motor Driver

- A single **DC motor with gearbox** drives the rear axle through the vehicle's mechanical drivetrain (physically connected via the gearbox, not one motor per wheel — satisfying rules 11.3/11.5/11.13, which prohibit independent per-wheel driving motors).
- The DC motor is connected to the **OUT1/OUT2** terminals of the L298N H-bridge driver.
- The L298N's logic inputs (**IN1–IN4** and enable pins **ENA/ENB**) are driven from Raspberry Pi GPIO pins, allowing the software to control both the direction (forward/backward) and speed (via PWM on the enable pins) of the drive motor.
- The L298N's **power input (Vs)** is connected to the separate **12V battery**, while its logic supply (**Vss**) and ground are shared with the Raspberry Pi's ground, keeping motor current away from the Pi's sensitive 5V logic rail.

### 2.5 Sensing — Ultrasonic Sensors (Emergency Brake Only)

- Two ultrasonic sensors (**Ultrasonic_1** and **Ultrasonic_2**) are mounted at two different positions on the vehicle. Unlike a typical setup, we do **not** use them for continuous wall-following or distance control — that job is done entirely by the camera (see section below). Instead, the ultrasonic sensors act as an independent **emergency braking safety layer**.
- Logic: each sensor is polled continuously in a lightweight loop that runs alongside the main camera-based control loop. If either sensor's measured distance drops below a fixed safety threshold (meaning a collision with a wall or pillar is imminent), the program immediately cuts or reverses the drive motor via the L298N, overriding whatever the camera-based steering/driving logic is currently doing.
- This gives us a hardware-adjacent safety net that is independent of camera lighting conditions or misdetections — if the vision pipeline fails to react in time, the ultrasonic emergency brake still protects the robot (and the field) from a hard collision.
- Each sensor has four connections: **VCC, GND, Trig (trigger output from Pi), and Echo (pulse-width input to Pi)** — a fifth **OUT** pin is also broken out on our sensor module but is currently unused/spare.
- Each sensor is wired to its own pair of GPIO pins (one for Trig, one for Echo) so both can be polled independently and quickly, since fast reaction time matters far more for an emergency brake than for a slower wall-following control loop.

---

## 3. Camera Wiring — `camera wiring.png`

- The vision sensor is an **8MP camera module**, connected to the Raspberry Pi 5's **CAM/DSP1** CSI camera port using the ribbon (FPC) cable supplied with the module.
- This is a direct CSI (Camera Serial Interface) connection, not a GPIO/wired serial connection — it carries high-speed image data straight to the Pi's image signal processor rather than through general-purpose pins.
- **Primary use — wall centering:** the camera image is processed to estimate what percentage of the frame is taken up by the left wall versus the right wall. The control loop uses the difference between these two percentages as its main steering error signal, correcting the servo to keep the robot centred between the two walls of the track. This is our main navigation method; it replaces distance-based wall following.
- **Secondary use — traffic signs:** the same camera is used to detect and classify the red and green traffic-sign pillars during the Obstacle Challenge, as required in section 5 of the WRO Game Rules, so the robot knows which side of each pillar it must pass on.
- No additional external wiring is required for the camera beyond correctly seating the ribbon cable (blue tab facing the Ethernet/USB side of the board, contacts facing the HDMI ports) into the CAM/DSP1 connector and locking the connector clip.

---

## 4. Why This Architecture

- **Two power domains (5V logic / 12V motor)** protect the Raspberry Pi from voltage dips and electrical noise caused by the DC motor's inrush current, improving reliability of the camera and GPIO sensor readings during driving.
- **22.5 W powerbank as a hard constraint:** we treated the powerbank's 22.5 W maximum output as a real design limit rather than an assumption of "plenty of power." The Raspberry Pi 5, the camera, both ultrasonic sensors, the servo, and the logic side of the L298N all draw from this single bank, so component selection (e.g., avoiding higher-draw peripherals) was done with this ceiling in mind. The 12V drive motor circuit is deliberately kept off this bank entirely.
- **Dedicated PWM pin for steering** keeps the servo control independent from the driving motor's control loop, simplifying the software's motion control logic.
- **Camera-based wall centering instead of ultrasonic distance control** was chosen because the camera can see both walls simultaneously and gives a continuous, proportional error signal (percentage of wall in frame), which is smoother for steering correction than discrete distance pings.
- **Two ultrasonic sensors used only as an emergency brake** give us a fast, lighting-independent safety layer that is decoupled from the main (camera-based) control loop — if the vision pipeline is ever wrong or too slow, the ultrasonic layer can still stop the vehicle before a hard collision, which also helps us avoid rule violations for moving/touching walls or pillars (WRO rule 9.18, 9.20).
- **A single push button and single power switch** were used specifically to comply with the WRO starting-procedure rules (9.6, 9.10, 9.11), which forbid any additional interactions beyond "one switch to power on, one button to start."

---

## 5. Files in this folder

| File | Description |
|---|---|
| `WRO26 wirings.png` | Full KiCad electrical schematic of the vehicle |
| `camera wiring.png` | Raspberry Pi 5 to Arducam camera module CSI connection |
| `wiring.md` | This file — written explanation of both diagrams |

For the full bill of materials and part numbers, see `../../components.md`.