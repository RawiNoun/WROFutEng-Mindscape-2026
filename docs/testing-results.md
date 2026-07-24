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

<!-- TODO: fill in once the Obstacle Challenge code and testing are done —
     strategies tried, what failed and why, and how we arrived at the final
     pillar-avoidance and parking logic, in the same format as above. -->