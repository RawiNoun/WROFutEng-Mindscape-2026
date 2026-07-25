# Testing Log — Team Mindscape_1 ("Lightning McQueen")

This log tracks the real iteration path behind our Open Challenge code and design — including the strategies that didn't work — so the story in the code history makes sense to anyone reading it later.

---

## 1. Camera Colour Calibration

Before any driving strategy could work, we had to get the camera reliably detecting the right colours. This took a lot of back-and-forth adjusting threshold values by hand — an early sign of how sensitive the whole approach would be to lighting and calibration. This is why our final codebase keeps colour thresholds in dedicated configuration files (`camera_config.py`, `color_config.py`) rather than hard-coded in the driving logic.

## 2. Attempt 1 — Dynamic Line-Fitting

**Strategy:** Draw a dynamic line on the camera preview and steer to follow it, recalculating the line each frame as the robot moved.

**Result:**  Failed. Fitting an accurate equation for the line in real time turned out to be extremely difficult, and the result was never accurate enough to steer by reliably.

## 3. Attempt 2 — Early ROI Strategy + Camera FOV Investigation

**Strategy:** An early version of our current ROI-based approach, with far fewer regions/conditions on screen than the final version.

**Result:**  Failed. At the same time we were still dealing with the Version 1 weight-distribution and design problems (see the model README), which made isolating any single cause harder.

**Investigation:** Our first guess was that the camera's limited field of view meant it wasn't seeing enough of the wall.
- Looked for a replacement camera — nothing fit our budget and deadline.
- Tried improvising a fisheye attachment (similar to a door peephole viewer) to widen the FOV cheaply — didn't produce a usable image for our detection logic. Abandoned.

## 4. Attempt 3 — Dual-Ultrasonic PID Wall-Following

**Strategy:** Set the camera-based approach aside; use a PID controller on distance readings from the two ultrasonic sensors (both sides) to center the robot between the walls.

**Result:**  Failed. After spending significant time tuning it, this approach never stabilized, and our hardware/weight-distribution problems were still unresolved in parallel.

**Outcome:** This combination is what pushed us to commit to the physical redesign (sliding battery/powerbank plate, repositioned camera, new wheels).

## 5. After the Redesign — Returning to the ROI Strategy

Once the redesign was in place, overall behaviour improved significantly, but we kept trying to make the ultrasonic PID approach work for a while longer before eventually giving up on it.

We returned to the ROI/box-based camera strategy — by this point our strongest lead — helped by the camera's new, improved position on the chassis. From there we improved it incrementally, adding more regions and conditions one at a time:

- Wall-centering ROIs (left/right)
- Front-wall halves (collision check)
- Pre-corner strip (checked only before a turn)
- Stuck detection (frame-difference check)

...until the behaviour became reliable.

**Result:**  Working, with two remaining problems:

- **Lag.** The camera feed and control loop were laggy — traced to the Raspberry Pi 5 4GB board being underpowered for the processing load.
- **Fix:** Sourced a Raspberry Pi 5 8GB board (hard to find in stock and within budget) — lag was gone after the swap.
- **Recalibration.** With the new hardware in place, we went through another full pass of recalibrating colour thresholds and ROI parameters, arriving at the current working version (`race.py`).

---

## Summary Table

| # | Strategy | Result | Why |
|---|---|---|---|
| 1 | Dynamic line-fitting |  Failed | Line equation not accurate/stable in real time |
| 2 | Early ROI + camera/fisheye FOV fix |  Failed | Too few ROIs; FOV wasn't the real bottleneck; fisheye unusable |
| 3 | Dual-ultrasonic PID wall-following |  Failed | Never stabilized; hardware issues still unresolved |
| 4 | ROI/box strategy (post-redesign) |  Working | Incrementally added ROIs; fixed camera position; RPi5 8GB fixed lag |


---

## Obstacle Challenge Testing Log

This log documents the development process behind our Obstacle Challenge software, following the same philosophy as the Open Challenge. Rather than immediately arriving at the final solution, we evaluated several navigation strategies, analyzed their shortcomings, and gradually refined the system until it became reliable enough for competition.

---

## 1. Attempt 1 — Dynamic Curve-Fitting

**Strategy:** Extend the dynamic curve-fitting approach originally investigated during the Open Challenge by calculating a smooth path around each traffic-sign pillar and steering along that curve in real time.

**Result:** Failed. Although the idea appeared promising, generating an accurate curve every frame proved extremely difficult. Small variations in the detected position of the pillars caused large changes in the generated path, resulting in inconsistent steering and unreliable obstacle avoidance.

**Outcome:** Since we had already experienced similar limitations in the Open Challenge, we decided to abandon the curve-fitting approach entirely and investigate a more deterministic solution.

---

## 2. Attempt 2 — ROI-Based Detection

**Strategy:** Build upon the Region-of-Interest (ROI) concept developed for the Open Challenge by dividing the camera image into several dedicated regions responsible for detecting traffic signs, nearby walls, and other important field elements.

**Result:** Partially successful. The robot was now capable of detecting traffic signs and reacting to them, but the behaviour was still inconsistent. The initial implementation contained only a small number of ROIs and decision conditions, making the robot vulnerable to incorrect detections and poorly timed steering corrections.

---

## 3. Incremental Development

Rather than replacing the entire navigation algorithm after each unsuccessful test, we chose to improve the ROI strategy incrementally. Each new condition was introduced individually, tested thoroughly, and only then integrated into the complete obstacle challenge program.

Over multiple testing sessions we gradually added:

- Dedicated ROIs for traffic-sign detection.
- Additional wall-monitoring regions to maintain vehicle positioning while approaching obstacles.
- Separate navigation states for normal driving, obstacle avoidance, and recovery.
- Additional safety conditions to reject false detections and improve overall consistency.

This modular development process made debugging significantly easier, as every new behaviour could be evaluated independently before interacting with the rest of the software.

---

## 4. Steering Mechanism Issue

During testing we encountered a problem that was not caused by the software itself but by the steering mechanism. Under certain conditions, the steering assembly struggled to rotate sufficiently toward the left, causing inconsistent left turns despite the software producing the correct steering commands.

Completely redesigning the steering mechanism was not feasible within the available competition timeframe, so we developed a software workaround instead.

**Solution:** Whenever the robot needed to perform a significant left turn, it briefly reversed before steering left. This relieved the mechanical load on the steering linkage, allowing the wheels to reach the required steering angle before continuing forward.

**Result:** Although initially intended as a temporary workaround, repeated testing showed that this method consistently improved left-turn reliability and therefore became part of the final obstacle challenge program.

---

## 5. Final Validation

After integrating all navigation states, obstacle detection routines, steering corrections, and recovery behaviours, we performed repeated full-course tests using different randomized obstacle configurations.

The primary objective during this phase was to verify that improvements made to one behaviour did not negatively affect another. Through continuous testing, threshold tuning, and parameter adjustments, the obstacle challenge software gradually evolved into the final version included in our repository.

**Result:** Working. The final implementation combines ROI-based computer vision, state-machine navigation, and a software-assisted steering workaround to produce reliable obstacle avoidance under randomized competition conditions.

---

## Summary Table

| # | Strategy | Result | Why |
|---|---|---|---|
| 1 | Dynamic curve-fitting | Failed | Generated path was too unstable for reliable obstacle avoidance. |
| 2 | Initial ROI-based obstacle detection | Partially successful | Detection worked, but additional ROIs and decision conditions were required. |
| 3 | Incremental ROI development | Working | Added navigation states, ROIs, and safety checks progressively until behaviour became reliable. |
| 4 | Steering workaround | Working | Reverse-before-left manoeuvre compensated for the steering mechanism's left-turn limitation. |