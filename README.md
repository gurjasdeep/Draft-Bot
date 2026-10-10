# Robocon First Week Project
Our Problem statement is a DraftBot

## Drawing commands

Connect to the board at 115200 baud and send newline-terminated serial commands:

- `HOME` homes both stepper motors. Run this before issuing motion commands.
  Each motor has 30 seconds to trigger its limit switch; otherwise the command
  reports a timeout error. Both limit switches are currently configured
  active-low and use `INPUT_PULLUP`; adjust `RIGHT_LIMIT_POLARITY` in
  `DraftBot.ino` if the right switch wiring differs.
- `SHAPE SQUARE`, `SHAPE TRIANGLE`, or `SHAPE CIRCLE` draws a shape.
- `MOVE x y` moves the pen tip to an `(x, y)` coordinate in millimetres.
- `STEP L +100` or `STEP R +100` moves the selected motor away from its home
  switch; a negative step count moves it toward the switch. This direct motor
  test does not require homing.
- `PEN UP` and `PEN DOWN` move the pen servo when no motion is active.

The first shape is centered at `(21.2 mm, 200 mm)`. The square and triangle
have a 50 mm side; the circle has a 25 mm radius and is approximated with 36
segments. Each successfully completed shape advances the next shape 55 mm in
the positive X direction, leaving a 5 mm gap while keeping the same Y
position. Shape points are checked for IK reachability before the motors
start, so a later shape is rejected if its position is outside the robot's
reachable workspace. Successful motion reports a `DONE` line when complete.
Moves and shape edges are interpolated as straight-line paths for the pen tip,
with Cartesian waypoints spaced at most 1 mm apart.

The firmware currently assumes 200 full steps per motor revolution, four
microsteps, and pen-servo positions of 90 degrees up and 0 degrees down. Match
the step and microstep settings to the drivers, and calibrate the servo angles
in `DraftBot.ino` to the hardware before drawing. The current home-angle
references are 180 degrees for the left link and 0 degrees for the right link,
measured from +X. Update `LEFT_HOME_ANGLE_DEGREES` /
`RIGHT_HOME_ANGLE_DEGREES` if the arm positions at the home switches differ.
The right motor direction is inverted in software so positive motion moves
away from either home switch. The driver
enable outputs are active-low: they
are held LOW while moving and driven HIGH when disabled. Shape motion waits
400 ms after servo commands for the pen to settle; adjust this delay if needed.
Adjust `STEPPER_DELAY_MICROSECONDS` in `src/stepper.cpp` to change the shared
HIGH and LOW pulse durations for normal moves, homing, and homing backoff.
Larger values produce longer pulses and move the motors more slowly.
