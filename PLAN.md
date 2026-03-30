# Plan: Migrate TeamCode to SolversLib Command-Based Architecture

## TL;DR
Refactor the monolithic `RobotCommon` class and procedural OpModes into SolversLib's command-based architecture (maintained fork of FTCLib) with dedicated Subsystem classes and Commands, using SolversLib's built-in Pedro Pathing commands.

## Identified Subsystems

### 1. Drive
- **Hardware:** None directly (Pedro Follower manages drivetrain motors internally via Constants)
- **Current code:** `RobotCommon.runDrive()` is no longer used for anything, Pedro Follower in `DriverControlAssist` and all autonomous OpModes
- **SolversLib role:** Wraps Pedro `Follower`. `periodic()` calls `follower.update()`.
- **Notes:** All drive modes (teleop and autonomous) go through Follower. Field-centric with headingOffset.
- **Commands (factory methods):**
  - `followPathCommand(PathChain path)` — follows a Pedro path, finishes when path is complete
  - `followPathCommand(PathChain path, boolean holdEnd, double maxPower)` — follows with holdEnd and power options
  - `holdPointCommand(Pose pose)` — holds robot at a specific pose indefinitely
  - `teleopDriveCommand(GamepadEx gamepad, double speed)` — continuous teleop drive from gamepad joysticks (default command)
  - `goToPoseCommand(Supplier<Pose> target)` — builds BezierLine from current pose to supplier-provided target at schedule time, follows it. Uses `Supplier<Pose>` so the same command definition can go to a pose determined at runtime.

### 2. Shooter
- **Hardware:** shooter (DcMotorEx)
- **Current code:** `RobotCommon.runShooter()`, PIDF velocity controller, target velocity, fixJam logic
- **State:** target velocity (0 = off), fixJam boolean
- **periodic():** Runs PIDF loop each cycle
- **Commands (factory methods):**
  - `shootCommand(double velocity)` — sets target velocity (instant)
  - `stopCommand()` — sets velocity to 0 (instant)
  - `setFixJamCommand(boolean fix)` — enables/disables jam fix mode (instant)
  - `waitUntilAtTargetCommand()` — waits until shooter velocity is within tolerance of target

### 3. Intake
- **Hardware:** intake (DcMotorEx), agitator (CRServo)
- **Current code:** `RobotCommon.runIntake()`, ShaftDirection enum (IN/OUT/STOP)
- **State:** direction enum
- **Commands (factory methods):**
  - `inCommand()` — sets direction to IN (instant)
  - `outCommand()` — sets direction to OUT (instant)
  - `stopCommand()` — sets direction to STOP (instant)

### 4. Feeder
- **Hardware:** leftFeeder, rightFeeder (CRServo)
- **Current code:** `RobotCommon.runFeeder()`, ShaftDirection enum, fixJam logic
- **State:** direction enum, fixJam boolean
- **Commands (factory methods):**
  - `inCommand()` — sets feeder to IN (instant)
  - `outCommand()` — sets feeder to OUT (instant)
  - `stopCommand()` — sets feeder to STOP (instant)
  - `setFixJamCommand(boolean fix)` — enables/disables jam fix mode (instant)
  - `feedOneShotCommand()` — SequentialCommandGroup: IN → wait 700ms → STOP → wait 400ms
  - `shootSequenceCommand(int shots)` — repeats `feedOneShotCommand()` N times

### 5. Lift
- **Hardware:** leftLift, rightLift (DcMotorEx, RUN_TO_POSITION)
- **Current code:** `RobotCommon.runLift()`, target position, LIFT_VELOCITY
- **State:** target position (int)
- **Commands (factory methods):**
  - `setPositionCommand(int position)` — sets target position (instant)
  - `adjustCommand(int delta)` — adjusts position by delta, clamped to [0, LIFT_MAX] (instant)

### 6. LEDs
- **Hardware:** redLed, blueLed, yellowLed, greenLed (LED)
- **Current code:** `RobotCommon.runLeds()`, shooter velocity status, ball distance
- **Dependencies:** Reads Shooter velocity + ColorSensor distance
- **periodic():** Updates LED state based on shooter/color status
- **Commands:** None — runs entirely through `periodic()`

### 7. ColorSensor
- **Hardware:** color (NormalizedColorSensor / DistanceSensor)
- **Current code:** `RobotCommon.runColor()`, reads distance for ball detection
- **periodic():** Reads distance each cycle, exposes `getBallDistance()`, `hasBall()`
- **Commands:** None — runs entirely through `periodic()`

## Dependencies (build.dependencies.gradle)

Add SolversLib core + Pedro module:
```
implementation 'org.solverslib:core:0.3.4'
implementation 'org.solverslib:pedroPathing:0.3.4'
```

Add repository (in TeamCode `build.gradle` or `build.dependencies.gradle`):
```
repositories {
    maven { url 'https://repo.dairy.foundation/releases' }
}
```

Keep existing `com.pedropathing:ftc:2.0.3` as-is. SolversLib 0.3.3+ targets Pedro Pathing 2.0.0+, so it's compatible.

SolversLib includes built-in Pedro commands (`FollowPathCommand`, `HoldPointCommand`, `TurnCommand`, `TurnToCommand`) in `com.seattlesolvers.solverslib.pedroCommand`. Only `TeleopMovementCommand` needs to be vendored (copied from Pedro-FTCLib and adapted).

Package: `com.seattlesolvers.solverslib` (replaces `com.arcrobotics.ftclib`).

## Steps

### Phase 1: Build Setup
1. Update `build.dependencies.gradle` — add `implementation 'org.solverslib:core:0.3.4'` and `implementation 'org.solverslib:pedroPathing:0.3.4'`. Add `maven { url 'https://repo.dairy.foundation/releases' }` to repositories. Keep existing `com.pedropathing:ftc:2.0.3` unchanged.

### Phase 2: Create Subsystem Classes
All subsystems extend `com.seattlesolvers.solverslib.command.SubsystemBase`.

2. **Shooter** — Extract from `RobotCommon`: shooter motor init, PIDF controller, `periodic()` runs PIDF loop. Factory methods: `setVelocityCommand()`, `stopCommand()`, `setFixJamCommand()`, `waitUntilAtTargetCommand()`.
3. **Intake** — Extract: intake motor + agitator servo init, `periodic()` applies power based on direction. Factory methods: `inCommand()`, `outCommand()`, `stopCommand()`.
4. **Feeder** — Extract: leftFeeder + rightFeeder servo init, `periodic()` applies power, jam fix support. Factory methods: `inCommand()`, `outCommand()`, `stopCommand()`, `setFixJamCommand()`, `feedOneShotCommand()`, `shootSequenceCommand()`.
5. **Lift** — Extract: leftLift + rightLift motor init (RUN_TO_POSITION), `periodic()` sends target to motors. Factory methods: `setPositionCommand()`, `adjustCommand()`.
6. **ColorSensor** — Extract: color sensor init, `periodic()` reads distance. No commands.
7. **LEDs** — Extract: 4 LED init, takes Shooter + ColorSensor refs, `periodic()` updates LED state. No commands.
8. **Drive** — Wraps Pedro Follower. `periodic()` calls `follower.update()`. Factory methods: `followPathCommand()`, `holdPointCommand()`, `teleopDriveCommand()`, `goToPoseCommand()`.

### Phase 3: Create TeleopMovement Command
9. **Vendor TeleopMovementCommand only** — `commands/TeleopMovementCommand.java`. Adapted from [Pedro-FTCLib TeleopMovement](https://github.com/Pedro-Pathing/FTCLib-Pedro/blob/main/src/main/java/com/pedropathing/commands/TeleopMovement.java), using existing Pedro v2.0.3 API and SolversLib's `CommandBase`. All other Pedro commands (`FollowPathCommand`, `HoldPointCommand`, `TurnCommand`, `TurnToCommand`) are provided by SolversLib's `pedroPathing` module in `com.seattlesolvers.solverslib.pedroCommand`.

### Phase 4: Refactor TeleOp OpModes
10. **DriverControlAssist** → Extend `CommandOpMode`. Create subsystems, register them, bind gamepad buttons to command factory methods using `GamepadEx`. Default command = `drive.teleopDriveCommand(gamepad, speed)`. Assist poses use `drive.followPathCommand()`/`drive.holdPointCommand()` triggered by button presses.

### Phase 5: Refactor Autonomous OpModes
11. **BlueGate** (and all autonomous variants) → Extend `CommandOpMode`. Replace the integer state machine with a `SequentialCommandGroup` composed of: `drive.followPathCommand()`, `feeder.shootSequenceCommand()`, `intake.inCommand()`, `ParallelCommandGroup` for intake+movement, etc.
12. **Red variants** — Continue using `mirror()` on poses, feed mirrored poses to the same command structure.

### Phase 6: Cleanup
13. Delete `RobotCommon.java` once all logic has been extracted into subsystems.
14. Verify all autonomous OpModes and the TeleOp OpMode compile and function correctly.

## Relevant Files

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/RobotCommon.java` — Decompose into 7 subsystems. Reference: `initialize()` for hardware mapping, `run*()` methods for subsystem logic
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/DriverControlAssist.java` — Refactor to CommandOpMode; default command = `drive.teleopDriveCommand()`, assist poses via `drive.followPathCommand()`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BlueGate.java` — Template for all autonomous refactors; replace state machine with SequentialCommandGroup
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BlueGate9.java` through all Red/Blue variants — Same pattern as BlueGate
- `build.dependencies.gradle` — Add SolversLib core + pedroPathing dependencies and dairy.foundation repo
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/Constants.java` — Keep; used for follower creation
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing/RobotDrawing.java` — Keep as-is for dashboard visualization
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/dashboard/DashboardTelemetry.java` — Keep as-is

## New Files to Create
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Shooter.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Intake.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Feeder.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Lift.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/ColorSensor.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/LEDs.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Drive.java`
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/commands/TeleopMovementCommand.java`

## Verification
1. Build compiles: `./gradlew :TeamCode:assembleDebug` after each phase
2. Verify each subsystem's `periodic()` runs the same logic as the original `RobotCommon.run*()` method by comparing motor/servo outputs
3. Test TeleOp controls: all gamepad bindings produce the same behavior (intake, shooter velocity presets, feeder, lift, drive speed)
4. Test autonomous: BlueGate runs the same path sequence and shooting timing as the original state machine
5. Verify vendored FollowPathCommand finishes correctly (completion threshold, heading error check) — may need adjustments for Pedro v2.0.3 API differences
6. Confirm LEDs still reflect shooter state and ball detection

## Decisions
- `periodic()` methods handle continuous control loops (PIDF, LED updates, sensor reads). Commands set targets; subsystems execute them.
- Drive wraps Pedro Follower rather than raw motor access, since all drive modes go through Follower.
- SolversLib's built-in Pedro commands (`FollowPathCommand`, `HoldPointCommand`, `TurnCommand`, `TurnToCommand`) from `com.seattlesolvers.solverslib.pedroCommand` are used directly. Only `TeleopMovementCommand` is vendored (not included in SolversLib).
- All other commands are factory methods on their respective subsystems, returning `InstantCommand` or `SequentialCommandGroup` — no separate command class files needed.
- Red autonomous variants continue to mirror Blue poses and reuse the same command structure.
- The `blackboard` pattern for passing follower/heading between OpModes is preserved.

## Further Considerations
1. **Pedro v2.0.3 API compatibility:** SolversLib 0.3.4 targets Pedro 2.0.0+, so its built-in commands are compatible. Only the vendored `TeleopMovementCommand` (from Pedro-FTCLib 1.0.7) may need Follower API adjustments for v2.0.3.
2. **LEDs cross-dependency:** LEDs reads from Shooter and ColorSensor. Plan: pass subsystem references to LEDs constructor.
3. **Feeder timing in autonomous:** Current state machine uses `ElapsedTime` for 700ms/400ms feed cycles. In command-based, `WaitCommand` durations should match. Verify no drift from command scheduler overhead.

---

## Detailed Checklist

### Phase 1: Build Setup
- [x] Add `maven { url 'https://repo.dairy.foundation/releases' }` to `repositories` block in `build.dependencies.gradle`
- [x] Add `implementation 'org.solverslib:core:0.3.4'` to `dependencies` block in `build.dependencies.gradle`
- [x] Add `implementation 'org.solverslib:pedroPathing:0.3.4'` to `dependencies` block in `build.dependencies.gradle`
- [x] Keep existing `com.pedropathing:ftc:2.0.3` dependency unchanged
- [x] Run `./gradlew :TeamCode:assembleDebug` — verify build succeeds with new dependencies

### Phase 2: Shooter Subsystem
- [x] Create `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Shooter.java`
- [x] Class extends `SubsystemBase`
- [x] Constructor takes `HardwareMap`, gets `shooter` DcMotorEx from hardware map
- [x] Set shooter direction to `REVERSE`
- [x] Move `shooterCoefficients` and `PIDFController` from `RobotCommon`
- [x] Add `shooterTarget` field (double, default 0)
- [x] Add `fixJam` field (boolean, default false)
- [x] `periodic()` — run PIDF loop: if `fixJam` set power -1; else if target==0 set power 0; else run PIDF controller (setTargetPosition, updatePosition from `shooter.getVelocity()`, updateFeedForwardInput, run, setPower)
- [x] Add `getVelocity()` method — returns `shooter.getVelocity()`
- [x] Add `getTarget()` method — returns `shooterTarget`
- [x] Factory method `shootCommand(double velocity)` — returns `InstantCommand` setting `shooterTarget`
- [x] Factory method `stopCommand()` — returns `InstantCommand` setting `shooterTarget = 0`
- [x] Factory method `setFixJamCommand(boolean fix)` — returns `InstantCommand` setting `fixJam`
- [x] Factory method `waitUntilAtTargetCommand()` — returns `WaitUntilCommand` checking `Math.abs(shooter.getVelocity() - shooterTarget) < 100`

### Phase 2: Intake Subsystem
- [x] Create `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Intake.java`
- [x] Class extends `SubsystemBase`
- [x] Constructor takes `HardwareMap`, gets `intake` DcMotorEx and `agitator` CRServo
- [x] Add `direction` field (ShaftDirection enum: STOP/IN/OUT, default STOP)
- [x] Define `ShaftDirection` enum inside Intake (or as shared enum)
- [x] `periodic()` — if IN: intake power 0.7, agitator power -1; if OUT: intake power -1, agitator power 0; if STOP: both power 0
- [x] Factory method `inCommand()` — returns `InstantCommand` setting direction to IN
- [x] Factory method `outCommand()` — returns `InstantCommand` setting direction to OUT
- [x] Factory method `stopCommand()` — returns `InstantCommand` setting direction to STOP

### Phase 2: Feeder Subsystem
- [x] Create `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Feeder.java`
- [x] Class extends `SubsystemBase`
- [x] Constructor takes `HardwareMap`, gets `leftFeeder` and `rightFeeder` CRServos
- [x] Set `leftFeeder` direction to `REVERSE`
- [x] Add `direction` field (ShaftDirection, default STOP)
- [x] Add `fixJam` field (boolean, default false)
- [x] `periodic()` — if `fixJam`: both power -1; else if IN: both power 1; if OUT: both power -1; if STOP: both power 0
- [x] Factory method `inCommand()` — returns `InstantCommand` setting direction to IN
- [x] Factory method `outCommand()` — returns `InstantCommand` setting direction to OUT
- [x] Factory method `stopCommand()` — returns `InstantCommand` setting direction to STOP
- [x] Factory method `setFixJamCommand(boolean fix)` — returns `InstantCommand` setting `fixJam`
- [x] Factory method `feedOneShotCommand()` — returns `SequentialCommandGroup`: `inCommand()` → `WaitCommand(0.7)` → `stopCommand()` → `WaitCommand(0.4)` (700ms feed + 400ms shooting pause)
- [x] Factory method `shootSequenceCommand(int shots)` — returns `SequentialCommandGroup` repeating `feedOneShotCommand()` N times

### Phase 2: Lift Subsystem
- [x] Create `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Lift.java`
- [x] Class extends `SubsystemBase`
- [x] Constructor takes `HardwareMap`, gets `leftLift` and `rightLift` DcMotorEx
- [x] Initialize both motors to `RUN_TO_POSITION` mode
- [x] Read initial position from `rightLift.getCurrentPosition()` as starting target
- [x] Add `targetPosition` field (int)
- [x] Move `LIFT_VELOCITY = 5000` constant from `RobotCommon`
- [x] `periodic()` — set both motors' target position and velocity each cycle
- [x] Add `getTargetPosition()` method
- [x] Factory method `setPositionCommand(int position)` — returns `InstantCommand` setting `targetPosition`
- [x] Factory method `adjustCommand(int delta)` — returns `InstantCommand` adjusting `targetPosition` by delta, clamped to [0, LIFT_MAX]
- [x] Move `LIFT_MAX = 2800` constant (currently in `DriverControlAssist`) into `Lift` or keep as parameter

### Phase 2: ColorSensor Subsystem
- [x] Create `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/ColorSensor.java`
- [x] Class extends `SubsystemBase`
- [x] Constructor takes `HardwareMap`, gets `color` NormalizedColorSensor
- [x] Set gain to `COLOR_GAIN = 2`
- [x] Add `ballDistance` field (double)
- [x] `periodic()` — reads distance from `((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM)`
- [x] Add `getBallDistance()` method — returns `ballDistance`
- [x] Add `hasBall()` method — returns `ballDistance < 8`

### Phase 2: LEDs Subsystem
- [x] Create `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/LEDs.java`
- [x] Class extends `SubsystemBase`
- [x] Constructor takes `HardwareMap`, `Shooter` reference, `ColorSensor` reference
- [x] Gets `redLed`, `blueLed`, `yellowLed`, `greenLed` from hardware map
- [x] Initialize all LEDs to off
- [x] Add `ledTimer` ElapsedTime field
- [x] `periodic()` — replicate `RobotCommon.runLeds()` logic:
  - [x] If shooter velocity < 100 and shooter target == 0: reset timer, red = true
  - [x] Else if `|shooter.getVelocity() - shooterTarget| < 100`: if timer > 200ms yellow else red
  - [x] Else: reset timer, yellow = true
  - [x] green = colorSensor.hasBall()
  - [x] Apply LED states with `.enable()`

### Phase 2: Drive Subsystem
- [x] Create `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/Drive.java`
- [x] Class extends `SubsystemBase`
- [x] Constructor takes `Follower` instance (created externally, passed in)
- [x] Store `follower` reference
- [x] `periodic()` — calls `follower.update()`
- [x] Add `getFollower()` accessor for path building
- [x] Add `getPose()` convenience method
- [x] Factory method `followPathCommand(PathChain path)` — returns SolversLib `FollowPathCommand(follower, path)`
- [x] Factory method `followPathCommand(PathChain path, boolean holdEnd, double maxPower)` — returns `FollowPathCommand` with options
- [x] Factory method `holdPointCommand(Pose pose)` — returns SolversLib `HoldPointCommand(follower, pose)`
- [x] Factory method `teleopDriveCommand(GamepadEx gamepad, double speed)` — returns vendored `TeleopMovementCommand` configured for field-centric drive
- [x] Factory method `goToPoseCommand(Supplier<Pose> target)` — builds BezierLine from current pose to supplier-provided target, returns `FollowPathCommand`
- [x] Factory method `startTeleOpDrive()` — returns `InstantCommand` calling `follower.startTeleOpDrive(true)`

### Phase 2: Build Verification
- [x] Run `./gradlew :TeamCode:assembleDebug` — all 7 subsystem files compile

### Phase 3: TeleopMovementCommand
- [x] Create `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/commands/TeleopMovementCommand.java`
- [x] Class extends `com.seattlesolvers.solverslib.command.CommandBase`
- [x] Adapt from [Pedro-FTCLib TeleopMovement.java](https://github.com/Pedro-Pathing/FTCLib-Pedro/blob/main/src/main/java/com/pedropathing/commands/TeleopMovement.java)
- [x] Constructor takes `Follower`, `GamepadEx`, speed multiplier, rotation multiplier, heading offset
- [x] `execute()` — reads gamepad joystick values, applies square scaling, calls `follower.setTeleOpDrive(x, -y, -rot, false, headingOffset)`
- [x] `isFinished()` — returns `false` (runs indefinitely as default command)
- [x] Add `addRequirements(drive)` for the Drive subsystem
- [x] Verify Pedro v2.0.3 `Follower.setTeleOpDrive()` signature matches
- [x] Run `./gradlew :TeamCode:assembleDebug` — vendored command compiles

### Phase 4: Refactor DriverControlAssist
- [x] Change `DriverControlAssist` to extend `CommandOpMode` instead of `LinearOpMode`
- [x] Replace `@TeleOp` annotation (keep same name)
- [x] Override `initialize()` instead of `runOpMode()`
- [x] Create all 7 subsystems in `initialize()`
- [x] Create `Follower` from `Constants.createFollower(hardwareMap)` or restore from `blackboard`
- [x] Create `Drive` subsystem with follower
- [x] Create `Shooter` subsystem
- [x] Create `Intake` subsystem
- [x] Create `Feeder` subsystem
- [x] Create `Lift` subsystem
- [x] Create `ColorSensor` subsystem
- [x] Create `LEDs` subsystem with Shooter + ColorSensor refs
- [x] Register all subsystems with `register()`
- [x] Create `GamepadEx` for gamepad1 and gamepad2
- [x] Set default command on Drive: `drive.teleopDriveCommand(gamepadEx1, ROBOT_FAST)` (with heading offset support)
- [x] Bind gamepad2.a → `intake.inCommand()`
- [x] Bind gamepad2.b → `intake.outCommand()`
- [x] Bind gamepad2.a/b release → `intake.stopCommand()`
- [x] Bind gamepad2.y → `shooter.shootCommand(SHOOTER_Y)`
- [x] Bind gamepad2.x → `shooter.shootCommand(SHOOTER_X)`
- [x] Bind gamepad2.back → `shooter.shootCommand(SHOOTER_BACK)`
- [x] Bind gamepad2.guide → `shooter.stopCommand()`
- [x] Bind gamepad2.dpad_down (held) → `shooter.setFixJamCommand(true)` + `feeder.setFixJamCommand(true)`
- [x] Bind gamepad2.dpad_down (released) → `shooter.setFixJamCommand(false)` + `feeder.setFixJamCommand(false)`
- [x] Bind gamepad2.right_bumper → `feeder.inCommand()`
- [x] Bind gamepad2.left_bumper → `feeder.outCommand()`
- [x] Bind gamepad2.right_bumper/left_bumper release → `feeder.stopCommand()`
- [x] Bind gamepad1.dpad_up → `lift.adjustCommand(LIFT_CHANGE)`
- [x] Bind gamepad1.dpad_down → `lift.adjustCommand(-LIFT_CHANGE)`
- [x] Bind gamepad1.x → `drive.goToPoseCommand(() -> halfShotPose)` (assist pose)
- [x] Bind gamepad1.y → `drive.goToPoseCommand(() -> midShotPose)` (assist pose)
- [x] Bind gamepad1.back → `drive.goToPoseCommand(() -> farShotPose)` (assist pose)
- [x] Bind gamepad1.right_bumper/left_bumper → `drive.goToPoseCommand(() -> closeShotPose)` (assist pose)
- [x] Bind gamepad1.b → `drive.goToPoseCommand(() -> parkingPose)`
- [x] Handle assist pose release → `drive.startTeleOpDrive()` to resume teleop
- [x] Bind gamepad1.a (held) → slow speed mode (ROBOT_SLOW / ROT_SLOW)
- [x] Handle gamepad1.guide → set `headingOffset` to current heading, save to blackboard
- [x] Bind gamepad1/2.dpad_left → `updatePose(+1)` (adjust last assist pose heading)
- [x] Bind gamepad1/2.dpad_right → `updatePose(-1)` (adjust last assist pose heading)
- [x] Preserve `lastAssist` enum tracking for pose adjustment
- [x] Preserve `mirror(target)` logic when `headingOffset < 0`
- [x] Remove `common` (RobotCommon) reference — all logic now in subsystems
- [x] Add `sendTelemetry()` — add telemetry data from subsystems, Pedro pathing, RobotDrawing
- [x] Preserve `blackboard.put("follower", follower)` and `blackboard.put("headingOffset", headingOffset)` for auton↔teleop handoff
- [x] Remove `common.runAuton()`, `common.runAprilTags()`, `common.correctPose()` calls (handled by subsystem periodic or removed)
- [x] Run `./gradlew :TeamCode:assembleDebug` — TeleOp compiles

### Phase 5: Refactor Autonomous OpModes (7 Blue base classes)
Each Blue autonomous follows the same pattern:

#### BlueGate.java
- [x] Change `BlueGate` to extend `CommandOpMode` instead of `LinearOpMode`
- [x] Override `initialize()` — create all subsystems, create Follower, set starting pose
- [x] Build all `PathChain` objects in `initialize()` (shootingPath, ballAlignPath, ballPickupPath, shootingPath2, endPath)
- [x] Replace integer state machine (states 0–15) with `SequentialCommandGroup`:
  - [x] `intake.inCommand()` + `shooter.shootCommand(SHOOTER_AUTON)` (parallel)
  - [x] `drive.followPathCommand(shootingPath)` — drive to half-shot pose
  - [x] `feeder.shootSequenceCommand(3)` — fire 3 balls (700ms feed + 400ms pause each)
  - [x] `drive.followPathCommand(ballAlignPath)` — drive to ball align pose
  - [x] `drive.followPathCommand(ballPickupPath)` — drive to pick up balls
  - [x] `drive.followPathCommand(shootingPath2)` — drive back to half-shot pose
  - [x] `feeder.shootSequenceCommand(3)` — fire 3 more balls
  - [x] `shooter.stopCommand()` + `intake.stopCommand()` (parallel)
  - [x] `drive.followPathCommand(endPath)` — drive to end pose
- [x] Schedule the SequentialCommandGroup in `initialize()`
- [x] Preserve `blackboard.put("follower", follower)` and `setBlackboard()` (headingOffset = 90)
- [x] Remove `common` RobotCommon reference
- [x] Remove `ElapsedTime stateTime`, `state`, `shots` fields (replaced by command sequencing)
- [x] Keep telemetry reporting — adapt `sendTelemetry()` for subsystem data

#### BlueGate9.java
- [x] Refactor to extend `CommandOpMode`
- [x] Replace state machine with `SequentialCommandGroup` (same pattern as BlueGate, adapted for 9-ball strategy)
- [x] Preserve unique poses and path definitions

#### BluePickle.java
- [x] Refactor to extend `CommandOpMode`
- [x] Replace state machine with `SequentialCommandGroup`
- [x] Preserve unique poses and path definitions

#### BlueWallFar.java
- [x] Refactor to extend `CommandOpMode`
- [x] Replace state machine with `SequentialCommandGroup`
- [x] Preserve unique poses and path definitions

#### BlueWallFar9.java
- [x] Refactor to extend `CommandOpMode`
- [x] Replace state machine with `SequentialCommandGroup`
- [x] Preserve unique poses and path definitions

#### BlueWallLoading.java
- [x] Refactor to extend `CommandOpMode`
- [x] Replace state machine with `SequentialCommandGroup`
- [x] Preserve unique poses and path definitions

#### BlueWallMid.java
- [x] Refactor to extend `CommandOpMode`
- [x] Replace state machine with `SequentialCommandGroup`
- [x] Preserve unique poses and path definitions

### Phase 5: Refactor Red Autonomous Variants (7 classes)
Each Red variant extends its Blue counterpart:

- [x] `RedGate` — inherits from `BlueGate`, overrides poses with `mirror()`, overrides `setBlackboard()` with headingOffset = -90
- [x] `RedGate9` — inherits from `BlueGate9`, overrides poses with `mirror()`
- [x] `RedPickle` — inherits from `BluePickle`, overrides poses with `mirror()`
- [x] `RedWallFar` — inherits from `BlueWallFar`, overrides poses with `mirror()`
- [x] `RedWallFar9` — inherits from `BlueWallFar9`, overrides poses with `mirror()`
- [x] `RedWallLoading` — inherits from `BlueWallLoading`, overrides poses with `mirror()`
- [x] `RedWallMid` — inherits from `BlueWallMid`, overrides poses with `mirror()`
- [x] Verify Red variants still work with inherited `CommandOpMode.initialize()` and mirrored poses
- [ ] Run `./gradlew :TeamCode:assembleDebug` — all 14 autonomous OpModes compile

### Phase 6: Cleanup
- [ ] Delete `RobotCommon.java`
- [ ] Move `mirror()` static method from `RobotCommon` to a utility class or into `Drive` subsystem
- [ ] Move `ShaftDirection` enum from `RobotCommon` to shared location (or into each subsystem)
- [ ] Move `addPedroPathingTelemetry()` from `RobotCommon` to a utility or `Drive` subsystem
- [ ] Move `sendTelemetry()` data from `RobotCommon` into individual subsystem telemetry methods
- [ ] Remove AprilTag code (`initAprilTag()`, `runAprilTags()`, `correctPose()`) — not migrated to subsystem per plan
- [ ] Remove unused `runDrive()` method (mecanum math) — Pedro Follower handles all drive
- [ ] Remove unused drivetrain motor references (`frontLeft`, `backLeft`, `frontRight`, `backRight`) from any remaining code
- [ ] Verify no remaining imports of `RobotCommon` in any file
- [ ] Run `./gradlew :TeamCode:assembleDebug` — final build verification
- [ ] Manually test TeleOp: intake in/out/stop, shooter velocity presets, feeder in/out, lift up/down, drive fast/slow, assist poses, heading offset reset, jam fix
- [ ] Manually test at least one Blue autonomous (BlueGate): full path sequence, shooting timing, ball pickup, end pose
- [ ] Manually test at least one Red autonomous (RedGate): mirrored paths work correctly
- [ ] Verify LEDs reflect shooter state (red/yellow) and ball detection (green)
- [ ] Verify blackboard handoff: autonomous sets follower + headingOffset, TeleOp reads them
