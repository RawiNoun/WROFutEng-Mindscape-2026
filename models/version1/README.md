This folder holds the CAD model for our first build (pictured alongside it) and explains why we moved on from that version.

## Version 1 Layout

- Camera mounted on the grey front tower.
- Chamber on the main deck holding the breadboard and motor driver.
- Powerbank in a slot at the back of the robot, sitting on top of that chamber.
- Raspberry Pi 5 housed alone in its own chamber.
- Single DC motor driving a solid rear axle, with front wheels steered by a micro servo through a plastic linkage.

## Problems With This Version

**Weight distribution.** This was the main issue. With the powerbank sitting high and at the very back of the chassis, right on top of the driver chamber, the center of gravity shifted up and back. The robot leaned noticeably in turns and occasionally lifted a wheel off the ground, which cost us both speed and reliability on the track.

**Camera field of view.** The camera itself has a limited field of view, and mounting it at the front of the robot didn't help — the two problems stacked, leaving blind spots that caused obstacles to be missed. We tried repositioning and adjusting the camera to work around it, but couldn't get a clean fix this way.

**Wheel grip.** Our first wheels were standard PLA prints, printed at normal settings — solid and rigid, but with poor grip that wore down fast on turns. We found that printing the same PLA filament at a higher temperature changed how it came out of the nozzle, giving the wheels a softer, more rubber-like surface with noticeably better traction.

## Why We Redesigned Instead of Patching

The camera and grip issues could be worked around, but the weight distribution couldn't be solved with code or minor tweaks — it needed a physical fix. So rather than keep patching around a chassis working against us, we rebuilt the model: the powerbank moved to a lower, more centered position, and the wheels were reprinted using the high-temperature PLA technique for better grip.

The new model in this repo replaces the design shown here, and our code was updated alongside it to match.
