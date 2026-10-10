# Robocon First Week Project
Our Problem statement is a DraftBot

## Drawing commands

Connect to the board at 115200 baud and send newline-terminated serial commands:

The basic Tkinter laptop controller is `gui/main.py`. Install its serial
dependency with `python -m pip install -r requirements.txt`, then start it with
`python gui/main.py`. Select the Arduino serial port and connect before sending
commands. Tkinter is included with many Python installations; on Linux it may
need to be installed separately as the `python3-tk` system package.

- `HOME` homes both stepper motors. Run this before issuing motion commands.
  Each motor has 30 seconds to trigger its limit switch; otherwise the command
  reports a timeout error. Both limit switches are currently configured
  active-low and use `INPUT_PULLUP`; adjust `RIGHT_LIMIT_POLARITY` in
  `DraftBot.ino` if the right switch wiring differs.
- `SHAPE <name> <centerX> <centerY> <size>` draws a shape centered at the
  supplied pen-tip coordinates, in millimetres. For example,
  `SHAPE SQUARE 90 130 40`, `SHAPE TRIANGLE 90 130 40`, or
  `SHAPE CIRCLE 90 130 40`.
- `MOVE x y` moves the pen tip to an `(x, y)` coordinate in millimetres.
- `STEP L +100` or `STEP R +100` moves the selected motor away from its home
  switch; a negative step count moves it toward the switch. This direct motor
  test does not require homing.
- `PEN UP` and `PEN DOWN` move the pen servo when no motion is active.

The shape `size` is the square side length, equilateral triangle height, or
circle diameter, in millimetres. Circles are approximated with 36 segments.
A shape's outline is checked for IK reachability at intervals of at most 1 mm
before the motors start.
Successful motion reports a `DONE` line when complete.
Moves and shape edges are interpolated as straight-line paths for the pen tip,
with Cartesian waypoints spaced at most 1 mm apart.

The firmware currently assumes 200 full steps per motor revolution, four
microsteps, and pen-servo positions of 90 degrees up and 0 degrees down. Match
the step and microstep settings to the drivers, and calibrate the servo angles
in `DraftBot.ino` to the hardware before drawing. The current home-angle
references are 180 degrees for the left link and 0 degrees for the right link,
measured from +X. Update `LEFT_HOME_ANGLE_DEGREES` /
`RIGHT_HOME_ANGLE_DEGREES` if the arm positions at the home switches differ.
Inverse kinematics also limits the left link between
`atan2(LINK_WIDTH, BASE_DISTANCE)` and `LEFT_HOME_ANGLE_DEGREES`; `LINK_WIDTH`
is currently 20 mm. The right link is limited between
`RIGHT_HOME_ANGLE_DEGREES` and `180 degrees - atan2(LINK_WIDTH,
BASE_DISTANCE)`. These constraints are based on the current link-clearance
geometry.
The right motor direction is inverted in software so positive motion moves
away from either home switch. The driver
enable outputs are active-low: they
are held LOW while moving and driven HIGH when disabled. Shape motion waits
400 ms after servo commands for the pen to settle; adjust this delay if needed.
Adjust `STEPPER_DELAY_PEN_UP_MICROSECONDS` and
`STEPPER_DELAY_PEN_DOWN_MICROSECONDS` in `stepper.cpp` to change the HIGH and
LOW pulse durations used while the pen is raised or lowered. The pen-up delay
defaults to 2,000 microseconds for faster travel; the pen-down delay defaults
to 8,000 microseconds for smoother drawing. Larger values move the motors more
slowly.
