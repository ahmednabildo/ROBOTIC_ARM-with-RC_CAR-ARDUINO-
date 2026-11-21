# 4-Axis Stepper Robot Arm with Mecanum Mobile Base

Arduino Mega firmware that controls a 4-axis stepper-driven robot arm with a servo gripper, mounted on (or paired with) a 4-wheel mecanum RC car. The arm and the car are commanded over serial. Positions can be jogged live, **recorded** as a sequence of steps, and **replayed** automatically.

## Features

- Automatic **homing** of all four arm axes using limit switches
- **Live control** of the arm joints, gripper and speed over serial
- **Record and replay** up to 50 steps (arm pose + gripper + car movement + drive time)
- **8-direction mecanum drive**: forward, backward, strafe left/right and four diagonals
- Smooth, acceleration-limited motion using [AccelStepper](https://www.airspayce.com/mikem/arduino/AccelStepper/)

## Hardware

| Part | Quantity | Notes |
|------|----------|-------|
| Arduino Mega 2560 | 1 | Required: the wheel pins (47-54) do not exist on an Uno |
| Stepper motors + STEP/DIR drivers (A4988 / DRV8825 / similar) | 4 | Arm axes |
| Stepper motors + STEP/DIR drivers | 4 | Mecanum wheels |
| Limit switches | 4 | One per arm axis |
| Hobby servo | 1 | Gripper |
| Mecanum wheels | 4 | Standard X layout |

### Pin map

| Function | STEP | DIR | Other |
|----------|------|-----|-------|
| Stepper 1 (theta1) | 2 | 5 | Limit switch: 11 |
| Stepper 2 (theta2) | 3 | 6 | Limit switch: 10 |
| Stepper 3 (phi) | 4 | 7 | Limit switch: 9 |
| Stepper 4 (z) | 12 | 13 | Limit switch: A3 |
| Left back wheel | 47 | 48 | |
| Left front wheel | 49 | 50 | |
| Right back wheel | 51 | 52 | |
| Right front wheel | 53 | 54 | |
| Gripper servo | | | Signal: A0 (600-2500 us pulse range) |

Limit switches use `INPUT_PULLUP`. Steppers 4 and 3 home when their pin reads `0`; steppers 2 and 1 home when their pin reads `1`. Wire your switches accordingly, or change the conditions in `homing()`.

## Software requirements

- [Arduino IDE](https://www.arduino.cc/en/software) or `arduino-cli`
- Libraries (install through the Library Manager):
  - `AccelStepper`
  - `Servo` (bundled with the Arduino IDE)

## Getting started

1. Wire the hardware according to the pin map above.
2. Open `robot_arm_car_fixed.ino` in the Arduino IDE.
3. Select **Tools > Board > Arduino Mega or Mega 2560** and the correct port.
4. Upload the sketch.
5. Keep the arm clear of obstacles: on power-up it **homes automatically** (stepper 4, then 3, then 2, then 1).
6. Open a serial connection at **9600 baud** and start sending commands.

## Serial protocol

Each command is a single line of **11 comma-separated integers**, ending with a trailing comma:

```
d0,d1,d2,d3,d4,d5,d6,d7,d8,d9,d10,
```

| Index | Name | Meaning |
|-------|------|---------|
| `d0` | Mode | `1` = save this pose as a new step, `2` = clear all saved steps, `0` = nothing |
| `d1` | Run | `1` = replay saved steps, `0` = live control (stops a running replay) |
| `d2` | theta1 | Joint 1 angle in degrees (44.44 steps/deg) |
| `d3` | theta2 | Joint 2 angle in degrees (35.56 steps/deg) |
| `d4` | phi | Wrist / rotation axis (10 steps/unit) |
| `d5` | z | Vertical axis (100 steps/unit) |
| `d6` | gripper | Servo angle, 0-180 |
| `d7` | speed | Max speed in steps/s for the arm and wheels (1-4000) |
| `d8` | time | Car drive time in units of 3 s (stored with the step) |
| `d9` | direction | Car direction code (see below) |
| `d10` | unused | Reserved |

### Car direction codes

| Code | Motion |
|------|--------|
| `0` | Stop |
| `1` | Forward |
| `2` | Backward |
| `3` | Strafe left |
| `4` | Strafe right |
| `5` | Diagonal left-forward |
| `6` | Diagonal right-forward |
| `7` | Diagonal left-backward |
| `8` | Diagonal right-backward |

### Example session

```
0,0,90,45,0,10,40,1000,0,0,0,     # move arm live: theta1=90, theta2=45, z=10, gripper=40, speed=1000
1,0,90,45,0,10,40,1000,1,1,0,     # save this pose; car drives forward for 1 x 3 s on replay
1,0,0,0,0,0,90,1000,0,0,0,        # save a second pose (gripper to 90)
0,1,0,0,0,0,0,1000,0,0,0,         # replay the saved sequence (loops until stopped)
0,0,0,0,0,0,0,1000,0,0,0,         # stop replay, back to live control
2,0,0,0,0,0,0,0,0,0,0,            # clear all saved steps
```

> The `#` comments are for readability only. Do not send them to the board.

## How replay works

For every stored step, the firmware:

1. Moves all four arm axes to the saved pose
2. Updates the gripper (waits 800 ms if it changed)
3. Drives the car in the saved direction for the saved time
4. Waits 500 ms and checks the serial port for a new command

The sequence repeats until a command with `d1 = 0` arrives.

## Configuration

Tunable values at the top of the sketch:

| Constant | Default | Description |
|----------|---------|-------------|
| `theta1AngleToSteps` | 44.44 | Steps per degree, joint 1 |
| `theta2AngleToSteps` | 35.56 | Steps per degree, joint 2 |
| `phiAngleToSteps` | 10 | Steps per unit, phi axis |
| `zDistanceToSteps` | 100 | Steps per unit, z axis |
| `timeSlice` | 3000 | Milliseconds per car drive time unit |
| `maxPositions` | 50 | Maximum number of recorded steps |

If a wheel spins in the wrong direction, invert it in `initializeSteppers()`:

```cpp
rightBackWheel.setPinsInverted(true, false, false);
```

## Known limitations

- Serial commands are only read between moves, so a running motion cannot be interrupted mid-step.
- Recorded steps are stored in RAM and are lost on reset.
- The parser expects exactly 11 fields; a shorter or corrupted line will give wrong values.
- The mecanum wheel signs assume a standard X-roller layout.

## Project structure

```
.
├── robot_arm_car_fixed.ino   # Main firmware
└── README.md
```

## License

Add a license of your choice (for example MIT) and place it in a `LICENSE` file.

## Author

[Your Name](https://github.com/your-username)
