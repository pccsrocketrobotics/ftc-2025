# Getting Started with Subsystems and Commands

A guide for building robot code from scratch using SolversLib's command-based
architecture. This is written for the start of a new season, when you have a new
robot and the library is already added to the project — but you're staring at an
empty `subsystems/` folder wondering where to begin.

Last year's code (`subsystems/`, `DriverControlAssist.java`, `BlueGate.java`,
etc.) is still in the repo. Use it as a **reference for the patterns**, but don't
try to reuse it directly — new game, new robot, new mechanisms.

---

## 1. The mental model

Command-based programming splits your robot code into two kinds of things:

- **Subsystems** — one class per mechanism (the shooter, the intake, the drive).
  A subsystem owns its hardware (motors, servos, sensors) and nobody else touches
  that hardware. Think of it as "the thing that knows how to run the shooter."

- **Commands** — a unit of *behavior* with a beginning, middle, and end. "Spin the
  shooter up to 1400," "drive this path," "fire 3 balls." Commands are what you
  bind to buttons in TeleOp and chain together in Autonomous.

A **scheduler** (provided by the library) runs the whole show. Every loop it:
1. Calls `periodic()` on every registered subsystem (for continuous work like
   reading a sensor or running a PID loop).
2. Runs whatever commands are currently scheduled.
3. Makes sure two commands never fight over the same subsystem.

That last point is the big win. You never again write a giant `if/else` in your
main loop deciding who gets to control the shooter. You schedule commands, and the
scheduler guarantees only one command uses a subsystem at a time.

### Why this beats a state machine

Last year's autonomous was an integer state machine — `switch (state)` with 15+
numbered cases, and every time you added a step you had to renumber everything
after it. With commands, an autonomous routine reads like a list of instructions:

```java
new SequentialCommandGroup(
    drive.followPathCommand(shootingPath),
    feeder.shootSequenceCommand(3),
    drive.followPathCommand(pickupPath),
    feeder.shootSequenceCommand(3)
)
```

Add a step by inserting a line. That's the whole point.

---

## 2. The pieces you'll use

All of these live in `com.seattlesolvers.solverslib.command` (and `.gamepad`):

| Piece | What it is |
|---|---|
| `SubsystemBase` | Base class you extend for each mechanism |
| `CommandOpMode` | Base class for your OpModes (instead of `LinearOpMode`) |
| `Command` | The return type of your factory methods |
| `InstantCommand` | Does one thing, then finishes immediately |
| `SequentialCommandGroup` | Runs commands one after another |
| `ParallelCommandGroup` | Runs commands at the same time; ends when **all** finish |
| `ParallelRaceGroup` | Runs commands at the same time; ends when the **first** finishes |
| `WaitCommand` | Waits N milliseconds |
| `GamepadEx` | Wraps a gamepad so you can bind buttons to commands |

---

## 3. Write your first subsystem

Start with the simplest mechanism you have — usually an intake or a single motor.
Here's the minimal shape. Compare it against `subsystems/Intake.java`.

```java
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final DcMotorEx motor;

    // Constructor: grab your hardware here.
    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "intake");
    }

    // A factory method that returns a Command.
    public Command inCommand() {
        return startEnd(
            () -> motor.setPower(0.7),   // runs once when the command starts
            () -> motor.setPower(0)      // runs once when the command ends
        );
    }
}
```

Three things to notice:

1. **The constructor grabs hardware from the `HardwareMap`.** The string
   (`"intake"`) must match the name in your Robot Configuration on the Driver Hub.
2. **Behaviors are *factory methods* that return `Command`.** You don't write
   separate command classes for simple things — `SubsystemBase` gives you helper
   builders (see below).
3. **The command is automatically bound to this subsystem.** When you call
   `startEnd(...)` from inside the subsystem, the returned command "requires" this
   subsystem, so the scheduler won't let anything else run the intake at the same
   time.

### The command-builder helpers on `SubsystemBase`

You almost never write `new SomeCommand()`. Instead use these helpers inside your
factory methods:

| Helper | Behavior | Use for |
|---|---|---|
| `runOnce(action)` | Do it once, finish immediately | Set a target, adjust a position |
| `run(action)` | Do it every loop, never finish on its own | Continuous control (a default command) |
| `startEnd(start, end)` | `start` once, then hold; `end` once when interrupted | Hold-to-run mechanisms (intake, feeder) |
| `runEnd(run, end)` | `run` every loop; `end` once when interrupted | A control loop that must run continuously (see `Shooter.shootCommand`) |

Real examples from last year's code:

```java
// Lift.java — set a target once
public Command adjustCommand(int delta) {
    return runOnce(() -> setMotors(targetPosition + delta));
}

// Shooter.java — run a PIDF loop continuously while active, stop on end
public Command shootCommand(double velocity) {
    return runEnd(() -> {
        // ... compute PIDF power from target velocity ...
        shooter.setPower(power);
    }, () -> shooter.setPower(0));
}
```

### `periodic()` — for continuous, always-on work

Override `periodic()` when a subsystem needs to do something *every loop
regardless of what command is running* — reading a sensor, updating a follower,
running an LED animation. The scheduler calls it automatically once you register
the subsystem.

```java
// ColorSensor.java
@Override
public void periodic() {
    // read the sensor (throttled to every 10th loop to save I2C time)
    if (readCounter++ >= READ_INTERVAL) {
        ballDistance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
        readCounter = 0;
    }
}
```

**Rule of thumb:** commands *set targets* ("go to 1400 rpm"); `periodic()` and
control loops *chase* those targets. If a value must be updated no matter what,
it goes in `periodic()`.

---

## 4. Build a TeleOp OpMode

Your OpMode extends `CommandOpMode` and overrides `initialize()` (not
`runOpMode()`). See `DriverControlAssist.java` for the full example.

```java
@TeleOp
public class MyTeleOp extends CommandOpMode {
    @Override
    public void initialize() {
        // 1. Create subsystems
        Intake intake = new Intake(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);

        // 2. Register them so periodic() runs
        register(intake, shooter);

        // 3. Wrap the gamepads
        GamepadEx operator = new GamepadEx(gamepad2);

        // 4. Bind buttons to commands
        operator.getGamepadButton(GamepadKeys.Button.A).whenHeld(intake.inCommand());
        operator.getGamepadButton(GamepadKeys.Button.Y).whenPressed(shooter.shootCommand(1400));
    }
}
```

That's a complete, working TeleOp. `CommandOpMode` runs the scheduler for you.

### Button binding vocabulary

Called on `operator.getGamepadButton(GamepadKeys.Button.X)`:

| Binding | When the command runs |
|---|---|
| `whenPressed(cmd)` | Once, on the press. Runs until it finishes on its own. |
| `whenHeld(cmd)` / `whileHeld(cmd)` | Starts on press, **interrupted on release** |
| `toggleWhenPressed(cmd)` | Press to start, press again to stop |

Choosing between these is a design decision. `whenPressed` on a shooter is
"set-and-forget" — one tap spins it up and it stays. `whenHeld` on an intake means
"only while I hold the button." Pick per mechanism based on how the drivers want
to operate it.

### Default commands

A subsystem can have a **default command** — what it does when nothing else is
scheduled for it. This is how the drivetrain reads the joysticks:

```java
drive.setDefaultCommand(drive.teleopDriveCommand(
    () -> square(-gamepad1.left_stick_y),   // forward
    () -> -square(gamepad1.left_stick_x),   // strafe
    () -> -gamepad1.right_trigger,          // rotate
    () -> Math.toRadians(headingOffset)
));
```

The default command runs whenever no other command is using that subsystem, and
automatically resumes when a higher-priority command finishes. A default command
must never finish on its own — build it with `run(...)` or a command whose
`isFinished()` returns false.

> **Gotcha (learned last year):** a default command runs *constantly*. Setting the
> intake's default command to `intake.inCommand()` means the intake runs the entire
> match unless a button overrides it. That may be what you want — or a surprise.
> Decide deliberately.

---

## 5. Build an Autonomous OpMode

Same base class (`CommandOpMode`), but instead of binding buttons you `schedule()`
one big command group. See `BlueGate.java`.

```java
@Autonomous
public class MyAuto extends CommandOpMode {
    @Override
    public void initialize() {
        Drive drive = new Drive(follower);
        Shooter shooter = new Shooter(hardwareMap);
        Feeder feeder = new Feeder(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        register(drive, shooter, feeder, intake);

        // ... build your PathChains from poses ...

        schedule(new SequentialCommandGroup(
            new ParallelRaceGroup(
                intake.inCommand(),              // runs the whole time...
                shooter.shootCommand(1325),      // ...spinning the whole time...
                new SequentialCommandGroup(      // ...until THIS finishes:
                    drive.followPathCommand(shootingPath),
                    feeder.shootSequenceCommand(3),
                    drive.followPathCommand(pickupPath),
                    feeder.shootSequenceCommand(3)
                )
            ),
            drive.followPathCommand(endPath)     // then park
        ));
    }
}
```

### The `ParallelRaceGroup` trick (important!)

Notice the intake and shooter are wrapped in a `ParallelRaceGroup` with the driving
sequence. Here's why:

`intake.inCommand()` and `shooter.shootCommand()` **never finish on their own** —
they run until interrupted. If you put them in a `ParallelCommandGroup` (which
waits for *all* children), the group would wait forever and the robot would never
reach `endPath`.

A `ParallelRaceGroup` ends the moment the **first** child finishes. The driving
sequence *does* finish, so when it's done the race group ends and interrupts the
intake and shooter — which triggers their `end()` handlers and stops the motors.

**Rule:** when you want a mechanism to run "for the duration of" some other work,
put the never-ending command and the finite work in a `ParallelRaceGroup`, and let
the finite work be the one that ends the race.

### Poses and paths

Driving uses Pedro Pathing. You define `Pose` objects (x, y, heading) and build
`PathChain`s between them:

```java
Pose start   = new Pose(48, 50, Math.toRadians(0));
Pose shoot   = new Pose(28, 28, Math.toRadians(46));

PathChain toShoot = follower.pathBuilder()
    .addPath(new BezierLine(start, shoot))
    .setLinearHeadingInterpolation(start.getHeading(), shoot.getHeading())
    .build();
```

Then `drive.followPathCommand(toShoot)` returns a command that finishes when the
path is complete. Use `PoseTester.java` / the dashboard to find pose coordinates on
the field.

---

## 6. A recommended order for the new season

Don't try to build everything at once. Bring the robot up one mechanism at a time:

1. **Set up the Robot Configuration** on the Driver Hub first. Write down the exact
   name of every motor/servo/sensor — those strings go in your `hardwareMap.get(...)`
   calls.

2. **Get the drivetrain moving.** Copy `pedroPathing/Constants.java` and tune it for
   the new drivetrain (this is the biggest up-front task; follow the Pedro tuning
   OpModes in `pedroPathing/Tuning.java`). Write a `Drive` subsystem that wraps the
   `Follower` and a TeleOp with just a default drive command. Confirm you can drive.

3. **Add one mechanism subsystem.** Pick the simplest (an intake). Write the
   subsystem, add a button binding, deploy, test. Repeat for each mechanism —
   shooter, feeder, lift, etc. **One subsystem, test, commit. Then the next.**

4. **Compose behaviors.** Once individual subsystems work, build combo commands:
   a `shootSequenceCommand`, an auto-aim, whatever your game needs. These are just
   factory methods returning command groups.

5. **Write autonomous last.** By now you have all the subsystems and their commands.
   Autonomous is mostly arranging them in `SequentialCommandGroup` /
   `ParallelRaceGroup` and defining poses.

Deploy and test on the physical robot after *every* subsystem. Command-based code
compiles cleanly and still does the wrong thing if a motor is reversed or a button
is mapped wrong — the robot is the only real test.

---

## 7. Common gotchas

- **Two commands, one subsystem.** If a command seems to not run, another command
  probably "requires" the same subsystem and won the scheduler's arbitration. Only
  one command uses a subsystem at a time — by design.

- **`ParallelCommandGroup` that never ends.** As above — if any child never finishes
  and you used `ParallelCommandGroup` instead of `ParallelRaceGroup`, the group
  hangs forever. This bit last year's autonomous.

- **Forgetting to `register()`.** If `periodic()` isn't running, you probably didn't
  pass the subsystem to `register(...)`.

- **A "do nothing" command.** To make a button *stop* a mechanism that has an
  always-running default command, you schedule a command that requires the
  subsystem but does nothing — that interrupts the default. Last year's
  `stopCommand()` is literally `return run(() -> {});`.

- **Config-tunable constants.** Annotating a subsystem `@Config` and making a
  constant `public static` exposes it in FTC Dashboard for live tuning
  (see `Shooter.shooterCoefficients`). Great for PID and speed tuning without
  redeploying.

- **`WaitCommand` is milliseconds.** `new WaitCommand(400)` is 0.4 seconds.

---

## 8. Where to look in this repo

| File | What it demonstrates |
|---|---|
| `subsystems/Intake.java` | Simplest subsystem — `startEnd` factory methods |
| `subsystems/Shooter.java` | `periodic()` + a `runEnd` control loop + `@Config` |
| `subsystems/Lift.java` | `runOnce` factories, clamping a target |
| `subsystems/ColorSensor.java` | Read-only sensor subsystem, throttled reads |
| `subsystems/Drive.java` | Wrapping Pedro `Follower`, path commands |
| `DriverControlAssist.java` | Full TeleOp: default commands + button bindings |
| `BlueGate.java` | Full Autonomous: `SequentialCommandGroup` + `ParallelRaceGroup` |
| `commands/TeleopMovementCommand.java` | A standalone command class (when a factory method isn't enough) |
| `PLAN.md` / `REVIEW.md` | The design decisions and trade-offs behind this architecture |

Start by reading `Intake.java` and `DriverControlAssist.java` side by side — they
are the smallest complete example of the whole pattern.
