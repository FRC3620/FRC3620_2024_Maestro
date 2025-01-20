# IO Assignments

## Digital IO
* DIO 0: Practice Robot Jumper
* DIO 1: Drive RF Absolute Encoder
* DIO 2: Drive LF Absolute Encoder
* DIO 3: Drive LR Absolute Encoder
* DIO 4: Drive RR Absolute Encoder
* ~~DIO 7: Shoulder CTRE Mag Encoder~~

## Analog IO

## PWM
* PWM 0: Blinky Lights
* PWM 1: Blinken
* PWM 2: Blinken

## Motor Controllers

# PDB assignments

# Vision

# Driver Controller

* L/R X/Y: drive

* A/SWA: reset NavX
* LB/SWF: turn off vision
* X/SWD: X mode
* LT/SWE: intake
* RT/SWH: shoot
* B/SWC: outtake barf

---

# Things to do different next year

* Angular stuff should use Rotation2d, eliminates this year's Mars Orbiter degrees/radians screwup.
* All results from Vision should be in one container (so all data comes from same data read).
* Use ErrorMessages.requireNonNullParam in constructors.

---

# Recalibrating shooter elevation

* Request 32.5 degrees from Dashboard (this is the angle that is good for a shot when the vision says the distance to target is 10.7 feet).
* Use level to check angle (lay it across the tops of the bottom churro). It should read 32.5 degrees.
* If the angle is significantly off, then:
    * Fiddle the dashboard until you have 32.5 degrees actual on the level.
    * Note what the dashboard says (in our example, was 28 degrees)
    * Subtract what the dashboard says from 32.5. This is the "adjustment amount". In our case, 32.5 - 28 = 4.5.
    * Find the shooterElevationHomePosition variable in ShooterSubsystem. Add the "adjustment amount" to it's value, and put the new value into the code. In our example, shooterElevationHomePosition was 64, we added 4.5 to it, and changed it to 69 (should have been 68.5, but we were doing math in our head).
    * Deploy code. Requesting 32.5 should give you 32.5 on the level.

---

# Adding swap space

2024 YAGSL is such a memory hog that it is necessary to add swap space to the roboRIO. Not an issue for 2025.

See [https://github.com/FRC3620/FRC3620_Programmers_Wiki/wiki/How-to-add-swap-space-to-a-roboRIO](https://github.com/FRC3620/FRC3620_Programmers_Wiki/wiki/How-to-add-swap-space-to-a-roboRIO).

