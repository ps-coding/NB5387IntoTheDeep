# NB5387: Into The Deep
Competition codebase for Team 5387 TecHounds (Northern Burlington) for the INTO THE DEEP season.

## Project Structure
- `TeamCode/` — Team-owned robot code (main focus of this repository)
  - `src/main/java/org/firstinspires/ftc/teamcode/ITDRobot.java`
    - Central robot abstraction: hardware mapping, drive/arm/slide/intake control, telemetry, and autonomous motion helpers
  - `src/main/java/org/firstinspires/ftc/teamcode/teleop/ITDTeleOp.java`
    - TeleOp entrypoint that routes gamepad input through the shared robot layer
  - `src/main/java/org/firstinspires/ftc/teamcode/autonomous/place/Left.java`
    - Left-side autonomous routine for scoring and cycling
  - `src/main/java/org/firstinspires/ftc/teamcode/autonomous/place/Right.java`
    - Right-side autonomous routine for specimen placement and parking
- `FtcRobotController/` — FTC SDK controller module (core, shared by all teams)

## TeamCode Highlights
- Built a shared `ITDRobot` control layer used by both TeleOp and Autonomous for consistent robot behavior.
- Implemented mecanum drive with two operator modes:
  - standard robot-centric drive
  - gyro-assisted field-centric drive with yaw reset support
- Added encoder-guided autonomous helpers (`driveTo`, `strafeTo`, `driveAndStrafe`, `turnTo`, `turnToFromHere`, `armToFromHere`) for reusable motion primitives.
- Integrated multiple manipulators with state-based control:
  - dual linear slides
  - horizontal slide
  - arm
  - intake wheel
  - intake turn servo
  - claw + claw rotation servos
- Added safety/driver usability features including debounce handling, mechanism limits, lock mode, and an automated return sequence.
- Built two substantial autonomous paths (`Left` and `Right`) that coordinate scoring, intake, heading correction, and endgame parking.

## Current OpModes
- `ITDTeleOp`
- `Left`
- `Right`
