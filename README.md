# Robocon First Week Project
Our Problem statement is a DraftBot

## Drawing commands

Connect to the board at 115200 baud and send newline-terminated serial commands:

- `HOME` homes both stepper motors. Run this before issuing motion commands.
  Each motor has 30 seconds to trigger its limit switch; otherwise the command
  reports a timeout error. The left limit is configured active-low and the
  right limit active-high; adjust `RIGHT_LIMIT_POLARITY` in `src/main.cpp` to
  match the right switch wiring. The active-high configuration uses `INPUT`,
  so the right limit input needs an external pull-down when its switch is open.
- `SHAPE SQUARE`, `SHAPE TRIANGLE`, or `SHAPE CIRCLE` draws a shape.
- `MOVE x y` moves the pen tip to an `(x, y)` coordinate in millimetres.
- `STEP L +100` or `STEP R -100` moves the selected motor by a signed number
  of steps. This direct motor test does not require homing.
- `PEN UP` and `PEN DOWN` move the pen servo when no motion is active.

Shapes are centered at `(BASE_DISTANCE / 2, 100 mm)`. The square and triangle
have a 30 mm side; the circle has a 15 mm radius and is approximated with 36
segments. A shape is checked for IK reachability before the motors start.
Successful motion reports a `DONE` line when complete.

The firmware currently assumes 200 full steps per motor revolution, one
microstep, and pen-servo positions of 90 degrees up and 0 degrees down. Match
the step and microstep settings to the drivers, and calibrate the servo angles
and `LEFT_HOME_ANGLE_DEGREES` / `RIGHT_HOME_ANGLE_DEGREES` in `src/main.cpp`
to the hardware before drawing. The driver enable outputs are active-low: they
are held LOW while moving and driven HIGH when disabled. Shape motion waits
400 ms after servo commands for the pen to settle; adjust this delay if needed.
Adjust `STEPPER_DELAY_MICROSECONDS` in `src/stepper.cpp` to change the shared
HIGH and LOW pulse durations for normal moves, homing, and homing backoff.
Larger values produce longer pulses and move the motors more slowly.
