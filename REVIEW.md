# Review of State Machine to Subsystems/Commands Refactor

## Summary

This refactor is a **net improvement**. It replaces a monolithic `RobotCommon` class and integer-based state machines with 7 focused subsystems and a command-based architecture built on SolversLib. The new code is easier to read, compose, and maintain.

**Behavior-preserving:** Mostly yes. The same paths, poses, shooter speeds, and feeder timings are used. A few subtle differences exist (documented below) that should be verified on the robot.

**Loop performance:** Improved in several areas. Color sensor reads are throttled (every 10 cycles vs. every cycle). LEDs diff before writing. Intake, feeder, and lift only write to hardware on state transitions instead of every cycle. The shooter PIDF loop moved from periodic to command execute, which is an improvement but has a subtle correctness concern.

**Biggest strengths:**
- Autonomous routines expressed as composable command groups instead of fragile integer state machines
- Clear subsystem ownership of hardware with reduced coupling
- Hardware writes reduced across most subsystems

**Biggest risks:**
- Intake runs continuously as a default command in teleop (new behavior)

## What's Good

### Red alliance inheritance preserved cleanly
**Files:** `RedGate.java`, `RedGate9.java`, etc.

The old code already used inheritance for Red variants (Red extends Blue, mirrors poses, calls `super.runOpMode()`). The refactor preserves this pattern, now overriding `initialize()` instead of `runOpMode()`. The inheritance works just as well with the command-based structure — the parent class builds all paths and command groups after the child has mirrored the poses. No regression here.

### Autonomous routines are declarative and composable
**Files:** `BlueGate.java`, `BlueWallLoading.java`, etc.

The old state machines used integer state variables with `switch/case` blocks spanning 15+ states. The new code expresses the same sequences as nested `SequentialCommandGroup` and `ParallelCommandGroup` compositions. The `BlueWallLoading` autonomous, which coordinates driving, shooting, intake, and loading zone visits, reads as a linear sequence of actions rather than a web of state transitions. Adding or removing a step no longer requires renumbering all subsequent states.

### Clear hardware ownership by subsystem
**Files:** All files in `subsystems/`

Each subsystem owns its hardware exclusively. `Shooter` owns the shooter motor. `Feeder` owns both feeder servos. `LEDs` owns all four LEDs. There is no shared mutable state between subsystems. The only cross-subsystem dependency is `LEDs` reading from `Shooter` and `ColorSensor` via injected references — a clean, read-only dependency.

### LED output diffing avoids redundant I2C writes
**File:** `LEDs.java`

The old `runLeds()` in `RobotCommon` called `enable()` on all four LEDs every cycle regardless of whether state changed. The new `LEDs.periodic()` tracks `oldRed`, `oldBlue`, `oldYellow`, `oldGreen` and only calls `enable()` when the value differs. This is a concrete I2C bus optimization.

### Color sensor read throttling
**File:** `ColorSensor.java`

The old `runColor()` read the distance sensor every cycle. Distance sensor reads over I2C are expensive (~30ms on some sensors). The new `ColorSensor.periodic()` reads every 10th cycle, caching the result. For a non-critical measurement like ball detection, this is an appropriate tradeoff that directly improves loop time.

### Intake and feeder write only on transition
**Files:** `Intake.java`, `Feeder.java`

These subsystems have no `periodic()` override. Hardware writes (`setPower()`) happen only in the `start` and `end` lambdas of `startEnd()` commands. In the old code, `runIntake()` and `runFeeder()` set motor/servo power every cycle even when the direction hadn't changed. CRServos and motors accept redundant writes without harm, but eliminating them reduces I2C bus traffic.

### Lift writes only on command
**File:** `Lift.java`

The old `runLift()` called `setTargetPosition()` and `setVelocity()` on both lift motors every cycle. The new Lift has no `periodic()` — it writes to hardware only when `setMotors()` is called from a command. Since the lift uses `RUN_TO_POSITION` mode, the motor controller maintains position between writes, so this is safe and reduces bus traffic.

### PoseUtil extraction
**File:** `PoseUtil.java`

`mirror()` and `facingPoint()` were static methods on `RobotCommon`. Extracting them to a utility class makes them usable without a `RobotCommon` dependency, which is necessary now that `RobotCommon` is being phased out.

## What's Problematic

### Shooter PIDF one-cycle lag is not a practical concern
**File:** `Shooter.java`
**Severity:** Low

The `periodic()` method caches velocity, then the `shootCommand` execute lambda uses that cached value to run the PIDF. The velocity reading is from the same scheduler tick (periodic runs before execute), so there is no additional lag beyond what any discrete-time control loop has. The P+F controller (no I or D terms) converges within a couple of ticks. This is not a meaningful issue.

One minor concern: the PIDF only runs while `shootCommand` is active. If the command is interrupted without its `end()` handler firing (which should not happen under normal command scheduler operation, but could in edge cases), the motor could be left at its last power.

### Assist pose end() relies on default command to resume teleop drive
**File:** `DriverControlAssist.java`, `assistPoseCommand()` method
**Severity:** Low

The `assistPoseCommand` creates a `StartEndCommand` where `end()` is `() -> {}` (empty lambda). The old code explicitly called `follower.startTeleOpDrive(true)` on button release. The new code relies on the default command (`TeleopMovementCommand`) being re-scheduled by the command scheduler when the assist command ends. `TeleopMovementCommand.initialize()` calls `follower.startTeleOpDrive(true)`, which interrupts any active path following. This happens within the same scheduler tick, so at most one extra `follower.update()` cycle runs with the old path before the mode switch — negligible in practice. The behavior is correct.

### Intake default command means intake always runs in teleop
**File:** `DriverControlAssist.java`
**Severity:** Medium

The line `intake.setDefaultCommand(intake.inCommand())` means the intake motor runs at 0.7 power and the agitator at -1 at all times during teleop, unless explicitly overridden by holding A (stop) or B (reverse). In the old code, the intake only ran when `gamepad2.a` was held:
```java
if (gamepad2.a) {
    common.setIntakeDirection(RobotCommon.ShaftDirection.IN);
} else if (gamepad2.b) {
    common.setIntakeDirection(RobotCommon.ShaftDirection.OUT);
} else {
    common.setIntakeDirection(RobotCommon.ShaftDirection.STOP);
}
```

This is a deliberate behavior change — the intake is now always-on by default. If this is intentional (the robot should always be collecting balls), it's fine. But it's a significant change from the old control scheme where the operator had explicit control. Verify this matches the team's intent.

### Shooter commands use whenPressed — no way to stop except PS button
**File:** `DriverControlAssist.java`
**Severity:** Low

Shooter commands are bound with `whenPressed`, meaning a single press of Y/X/Back starts the shooter and it stays running until the operator presses PS (guide). In the old code, the shooter target was set each cycle based on which button was held, and stopped immediately when no button was held (via the `else if` chain). The new behavior is arguably better for match flow (set-and-forget shooter speed), but it's different. Verify the operator expects this.

### goToPoseCommand builds path at schedule time, not declaration time
**File:** `Drive.java`
**Severity:** Low

`goToPoseCommand` wraps path building in an `InstantCommand` that evaluates the `Supplier<Pose>` and builds a `BezierLine` from the current pose when the command is *scheduled*, not when it's declared. This is actually correct behavior for dynamic pose targeting, but it means the path is a simple straight line from wherever the robot currently is. If the robot is not where expected, the path could be suboptimal. This matches the old `goToPose()` behavior exactly.

### RobotCommon.java still exists
**Files:** `RobotCommon.java`
**Severity:** Low

The old monolithic class is still in the codebase. While no new code references it, its presence could confuse contributors. It should be deleted once the refactor is verified on the robot.

### No telemetry in autonomous OpModes
**Files:** `BlueGate.java`, `BlueWallLoading.java`, etc.
**Severity:** Low

The old autonomous OpModes output state number, shot count, and detailed Pedro Pathing telemetry every cycle. The new autonomous OpModes have no `run()` override and no telemetry output. The command scheduler handles execution, but there's no visibility into which command is currently running, how many shots have been fired, or path following status. This makes field debugging harder.

**Recommended fix:** Override `run()` in autonomous OpModes to call `super.run()` plus telemetry output showing current command state and follower position.

### AprilTag vision not migrated
**Files:** N/A (missing from new code)
**Severity:** Low

The old code had `common.initAprilTag()`, `common.runAprilTags()`, and `common.correctPose()`. These are not migrated. The pose correction was commented out in the old code (`correctPose` had its body commented out), so this is not a functional regression. But if the team plans to re-enable vision correction, it will need a new subsystem.

## Behavior Changes to Verify

1. **Intake always-on in teleop:** The intake now runs continuously as the default command. Old behavior was operator-controlled (A to run, release to stop). Test whether this is desired match behavior and whether it creates heat or mechanical issues.

2. **Shooter set-and-forget vs. hold-to-run:** Shooter speed is now set with a single press and persists until PS is pressed. Old behavior required holding the button. Test operator comfort with this change.

3. **Assist pose release behavior:** When the driver releases an assist pose button (X, Y, Back, bumpers), the old code explicitly called `follower.startTeleOpDrive(true)`. The new code relies on the default command resuming. Test that the robot smoothly returns to manual driving after releasing an assist button, with no delay or stuck-following behavior.

4. **Feeder shot timing:** Old code used `ElapsedTime` with `FEEDER_TIME = 700ms` feed + `SHOOTING_TIME = 400ms` pause. New code uses `inCommand().withTimeout(700)` + `WaitCommand(400)`. The timeout mechanism may have slightly different timing characteristics (command scheduler tick alignment). Test that 3-shot sequences complete reliably and balls actually fire.

5. **Autonomous end state:** Autonomous OpModes now use `ParallelRaceGroup`, which ends when the drive/feeder sequence completes and interrupts intake and shooter via their `end()` handlers. Verify the shooter actually stops and intake actually stops at the end of autonomous.

6. **Lift velocity not set in periodic:** Old code set lift velocity every cycle. New code sets it only in `setMotors()`. If `RUN_TO_POSITION` mode clears velocity on its own (some motor controllers do), the lift might stop holding position. Test that the lift holds its position indefinitely after being adjusted.

## Loop Performance Observations

### Redundant hardware writes reduced
The old `RobotCommon.run()` made unconditional hardware writes every cycle for every subsystem:
- `runDrive()`: 4x `setVelocity()` — still happens via Follower
- `runIntake()`: `setPower()` on motor + servo every cycle — **now only on state change**
- `runLift()`: 2x `setTargetPosition()` + 2x `setVelocity()` every cycle — **now only on command**
- `runShooter()`: `setPower()` + `getVelocity()` every cycle — PIDF runs only in active command; velocity still cached each cycle
- `runFeeder()`: 2x `setPower()` every cycle — **now only on state change**
- `runColor()`: `getDistance()` every cycle — **now every 10th cycle**
- `runLeds()`: 4x `enable()` every cycle — **now only on change**

Conservative estimate: the old code made ~18 hardware calls per cycle. The new code makes ~3-5 calls per cycle during steady state (follower update, velocity cache, throttled color read, occasional LED update). This is a meaningful reduction in I2C bus time.

### Color sensor throttling
Reading a distance sensor over I2C can take 10-30ms depending on the sensor. Throttling from every cycle to every 10th cycle could save 9-27ms per 10 cycles on average, or roughly 1-3ms per cycle amortized. For a target loop time of ~20ms, this is significant.

### LED diffing
LED I2C writes are fast (~1ms each), but eliminating 4 unnecessary writes per cycle saves ~4ms in the common case where LED state hasn't changed.

### No new caching risks
The subsystems that cache (ColorSensor, LEDs) cache read values or output state, not control inputs. There is no risk of stale control values causing the robot to ignore operator input. The only risk is the color sensor's 10-cycle lag for ball detection, which is acceptable for LED feedback.

### Command scheduler overhead
The SolversLib command scheduler adds overhead per tick: iterating registered subsystems, checking command requirements, calling periodic/execute. For 7 subsystems and a handful of active commands, this is likely <1ms per cycle — negligible compared to the I2C savings.

### Net assessment
Loop time is likely improved. The biggest win is color sensor throttling. The I2C savings from eliminating redundant writes to intake, feeder, lift, and LEDs are smaller individually but add up. The command scheduler overhead is minimal and does not negate the savings.

## Resolved

### Autonomous ParallelCommandGroup never completes — robot never parks
**Files:** `BlueGate.java`, `BlueGate9.java`, `BluePickle.java`, `BlueWallFar.java`, `BlueWallFar9.java`, `BlueWallLoading.java`, `BlueWallMid.java` (and all Red variants via inheritance)
**Original severity:** High

All autonomous OpModes used `ParallelCommandGroup` containing `intake.inCommand()` and `shooter.shootCommand()`, which never finish (`isFinished()` returns false). `ParallelCommandGroup` requires all children to end, so the end path never executed — the robot would never park.

**Fix applied:** Replaced `ParallelCommandGroup` with `ParallelRaceGroup` in all 7 files. `ParallelRaceGroup` ends when the **first** child finishes. Since intake and shooter never finish on their own, the race group ends when the inner `SequentialCommandGroup` completes, interrupting intake and shooter (triggering their `end()` handlers that set power to 0). The outer sequential then proceeds to `endPath`.

## Final Verdict

**Accept with changes**

The refactor is a genuine architectural improvement that delivers real benefits: declarative autonomous composition replacing fragile state machines, clear subsystem ownership of hardware, and measurably reduced hardware bus traffic. The SolversLib framework is appropriate for FTC and well-integrated.

Two items should be addressed before merging:

1. **Verify intake always-on is intentional** — the default command change is significant and should be confirmed with the drive team.
2. **Add basic autonomous telemetry** — the loss of all debug output in autonomous makes field tuning unnecessarily difficult.

The remaining issues (shooter safety in periodic, shot timing verification, lift hold behavior) are worth testing on the robot but are low risk. The old `RobotCommon.java` should be deleted once testing is complete.
