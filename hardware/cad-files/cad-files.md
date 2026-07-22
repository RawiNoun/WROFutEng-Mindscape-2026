# Robot Model — Cad files and switching between designs

This folder holds the CAD model for our first build and explains why we moved on from that version.

## Version 1 Layout

- Camera mounted on the grey front tower.
- Chamber on the main deck holding the breadboard and motor driver.
- Powerbank in a slot at the back of the robot, sitting on top of that chamber.
- Raspberry Pi 5 housed alone in its own chamber.
- Single DC motor driving a solid rear axle (a fixed design constraint, not a flaw), with front wheels steered by a micro servo through a plastic linkage.

## Problems With This Version

**Weight distribution.** This was the main issue. With the powerbank sitting high and at the very back of the chassis, the center of gravity shifted up and back. The robot leaned noticeably in turns and occasionally lifted a wheel off the ground, which cost us both speed and reliability on the track.

**Camera field of view.** The camera itself has a limited field of view, and mounting it at the front of the robot didn't help — the two problems stacked, leaving blind spots that caused obstacles to be missed. We tried repositioning and adjusting the camera to work around it, but couldn't get a clean fix this way.

**Wheel grip.** Our first wheels were standard PLA prints, printed at normal settings — solid and rigid, but with poor grip that wore down fast on turns. We found that printing the same PLA filament at a higher temperature changed how it came out of the nozzle, giving the wheels a softer, more rubber-like surface with noticeably better traction.

## Why We Redesigned Instead of Patching

The camera and grip issues could be worked around, but the weight distribution couldn't be solved with code or minor tweaks — it needed a physical fix.

## Final Design

The new build (pictured) addresses these issues directly:

- **Sliding battery/powerbank plate.** The batteries and powerbank now sit stacked on top of each other on a sliding plate, making them easy to pull out and swap. Centering and stacking them this way also fixes the weight distribution problem from Version 1.
- **Improved cooling.** The fan now sits on top of the Raspberry Pi 5 instead of underneath it, giving much better airflow and cooling.
- **Repositioned camera.** The camera was moved back slightly and mounted higher on the tower.
- **New wheels.** Reprinted using the high-temperature PLA technique for a more rubber-like, grippier surface.
