# NextFTC Migration Guide

## Overview

This guide explains how to migrate from traditional mechanism files to NextFTC's command-based architecture, and how to integrate with Pedro Pathing for autonomous control.

## Architecture Comparison

### Old Architecture (Mechanism Files)
```java
public class IntakeMotorSpinner {
    DcMotor motor;
    
    public void init(HardwareMap hwMap) { ... }
    public void Intake() { motor.setPower(0.7); }
    public void Outtake() { motor.setPower(-0.7); }
    public void Stop() { motor.setPower(0.0); }
}
```

**Problems:**
- No built-in support for parallel/sequential operations
- Difficult to compose complex autonomous routines
- Manual state management required
- Hard to interrupt or cancel operations

### New Architecture (NextFTC Subsystems + Commands)

**Subsystems** represent robot mechanisms:
```java
public class IntakeSubsystem extends Subsystem {
    @Override
    public void initialize(HardwareMap hardwareMap) { ... }
    public void intake() { ... }
    public void stop() { ... }
}
```

**Commands** perform actions on subsystems:
```java
public class IntakeCommand extends Command {
    public IntakeCommand(IntakeSubsystem subsystem) {
        addRequirements(subsystem);  // Prevents conflicts
    }
    
    @Override
    public void init() { subsystem.intake(); }
    
    @Override
    public boolean isFinished() { return false; }
    
    @Override
    public void end(boolean interrupted) { subsystem.stop(); }
}
```

**Benefits:**
- ✅ Built-in parallel/sequential command groups
- ✅ Automatic resource conflict management
- ✅ Easy to interrupt and cancel
- ✅ Composable for complex autonomous routines
- ✅ Works seamlessly with Pedro Pathing

## File Structure

```
teamcode/
├── subsystems/
│   ├── IntakeSubsystem.java       # Manages intake motor hardware
│   └── FlywheelSubsystem.java     # Manages flywheel with velocity control
├── commands/
│   ├── IntakeCommand.java         # Runs intake continuously
│   ├── OuttakeCommand.java        # Runs outtake continuously
│   ├── StopIntakeCommand.java     # Stops intake instantly
│   ├── SpinFlywheelCommand.java   # Spins flywheel at RPM
│   ├── SpinUpFlywheelCommand.java # Spins up and waits for speed
│   └── StopFlywheelCommand.java   # Stops flywheel instantly
└── examples/
    ├── ExampleNextFTCTeleOp.java      # TeleOp example with button bindings
    └── ExampleNextFTCAutonomous.java  # Autonomous with Pedro Pathing
```

## Key Concepts

### 1. Subsystems

Subsystems represent physical mechanisms on your robot. They:
- Extend `dev.nextftc.core.command.Subsystem`
- Initialize hardware in `initialize(HardwareMap)`
- Provide methods to control the mechanism
- Only one command can use a subsystem at a time (enforced by NextFTC)

### 2. Commands

Commands perform actions using subsystems. They:
- Extend `dev.nextftc.core.command.Command`
- Declare requirements using `addRequirements(subsystem)`
- Implement lifecycle methods:
  - `init()` - Called once when command starts
  - `execute()` - Called repeatedly while running
  - `isFinished()` - Return true to end the command
  - `end(boolean interrupted)` - Called when command ends

### 3. Command Types

**Continuous Commands** - Run until interrupted:
```java
@Override
public boolean isFinished() {
    return false;  // Runs forever until another command needs the subsystem
}
```

**Instant Commands** - Finish immediately:
```java
@Override
public void init() {
    subsystem.stop();
}

@Override
public boolean isFinished() {
    return true;  // Finishes instantly
}
```

**Timed Commands** - Run for a specific duration or until a condition:
```java
@Override
public boolean isFinished() {
    return subsystem.isAtTargetSpeed(50.0);  // Finish when ready
}
```

### 4. Command Groups

**Sequential** - Run commands one after another:
```java
new SequentialGroup(
    new SpinUpFlywheelCommand(flywheel, 3000),
    new WaitCommand(1.0),
    new StopFlywheelCommand(flywheel)
)
```

**Parallel** - Run commands at the same time:
```java
new ParallelGroup(
    new FollowPathCommand(follower, path),
    new IntakeCommand(intake)
)
```

**Combined** - Mix parallel and sequential:
```java
new SequentialGroup(
    // Drive to score while spinning up flywheel
    new ParallelGroup(
        new FollowPathCommand(follower, pathToScore),
        new SpinUpFlywheelCommand(flywheel, 3000)
    ),
    // Then shoot
    new WaitCommand(1.0),
    new StopFlywheelCommand(flywheel),
    // Then drive to pickup while running intake
    new ParallelGroup(
        new FollowPathCommand(follower, pathToPickup),
        new IntakeCommand(intake)
    )
)
```

## Usage Examples

### TeleOp with Button Bindings

```java
@TeleOp(name = "My TeleOp")
public class MyTeleOp extends NextFTCOpMode {
    private IntakeSubsystem intake;
    
    @Override
    public void init() {
        intake = new IntakeSubsystem();
        register(intake);
        
        // Bind buttons to commands
        gamepad1().leftBumper().onTrue(new IntakeCommand(intake));
        gamepad1().rightBumper().onTrue(new OuttakeCommand(intake));
        gamepad1().a().onTrue(new StopIntakeCommand(intake));
    }
}
```

### Autonomous with Pedro Pathing

```java
@Autonomous(name = "My Auto")
public class MyAuto extends PedroOpMode {
    private IntakeSubsystem intake;
    private FlywheelSubsystem flywheel;
    
    @Override
    public void init() {
        intake = new IntakeSubsystem();
        flywheel = new FlywheelSubsystem();
        register(intake, flywheel);
        
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }
    
    @Override
    public void start() {
        schedule(
            new ParallelGroup(
                new FollowPathCommand(follower, myPath),
                new IntakeCommand(intake)
            )
        );
    }
}
```

## Pedro Pathing Integration

NextFTC provides `FollowPathCommand` for seamless Pedro Pathing integration:

```java
import dev.nextftc.extensions.pedro.FollowPathCommand;
import dev.nextftc.extensions.pedro.PedroOpMode;

// Use PedroOpMode as base class
public class MyAuto extends PedroOpMode {
    @Override
    public void start() {
        // Create path using Pedro Pathing
        PathChain path = follower.pathBuilder()
            .addPath(new BezierLine(start, end))
            .setLinearHeadingInterpolation(startHeading, endHeading)
            .build();
        
        // Wrap in FollowPathCommand
        schedule(new FollowPathCommand(follower, path));
    }
}
```

The `PedroOpMode` base class:
- Automatically updates the follower
- Provides `follower` field for building paths
- Integrates with NextFTC's command scheduler

## Migration Steps

### Step 1: Convert Mechanism to Subsystem

**Before:**
```java
public class MyMechanism {
    DcMotor motor;
    public void init(HardwareMap hwMap) { ... }
    public void doAction() { ... }
}
```

**After:**
```java
public class MySubsystem extends Subsystem {
    private DcMotor motor;
    
    @Override
    public void initialize(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, "motor_name");
    }
    
    public void doAction() { ... }
    public void stop() { ... }
}
```

### Step 2: Create Commands

For each action, create a command:
```java
public class DoActionCommand extends Command {
    private final MySubsystem subsystem;
    
    public DoActionCommand(MySubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }
    
    @Override
    public void init() {
        subsystem.doAction();
    }
    
    @Override
    public boolean isFinished() {
        return false; // or true for instant command
    }
    
    @Override
    public void end(boolean interrupted) {
        subsystem.stop();
    }
}
```

### Step 3: Update TeleOp

**Before:**
```java
MyMechanism mechanism = new MyMechanism();

@Override
public void init() {
    mechanism.init(hardwareMap);
}

@Override
public void loop() {
    if (gamepad1.left_bumper) {
        mechanism.doAction();
    } else {
        mechanism.stop();
    }
}
```

**After:**
```java
public class MyTeleOp extends NextFTCOpMode {
    private MySubsystem subsystem;
    
    @Override
    public void init() {
        subsystem = new MySubsystem();
        register(subsystem);
        
        gamepad1().leftBumper().onTrue(new DoActionCommand(subsystem));
    }
}
```

### Step 4: Update Autonomous

**Before (with RoadRunner Actions):**
```java
Actions.runBlocking(
    new ParallelAction(
        followPath(path),
        mechanism.intake()
    )
);
```

**After (with NextFTC):**
```java
schedule(
    new ParallelGroup(
        new FollowPathCommand(follower, path),
        new IntakeCommand(intakeSubsystem)
    )
);
```

## Configuration

### Configurable Constants

Both subsystems support configurable constants for easy tuning:

**IntakeSubsystem:**
```java
IntakeSubsystem.INTAKE_POWER = 0.8;   // Adjust intake power
IntakeSubsystem.OUTTAKE_POWER = -0.8; // Adjust outtake power
```

**FlywheelSubsystem:**
```java
FlywheelSubsystem.HIGH_TARGET_RPM = 3500.0;  // High shooting speed
FlywheelSubsystem.LOW_TARGET_RPM = 2500.0;   // Low shooting speed
FlywheelSubsystem.GEAR_RATIO = 2.0;          // Adjust for your gearing
```

## Advanced Features

### Flywheel Velocity Control

The `FlywheelSubsystem` uses velocity control for consistent shooting:

```java
// Set specific RPM
flywheel.setRPM(3000);

// Use preset speeds
flywheel.setHighSpeed();
flywheel.setLowSpeed();

// Check if at target speed
if (flywheel.isAtTargetSpeed(50.0)) {
    // Ready to shoot!
}

// Get current RPM for telemetry
double currentRPM = flywheel.getCurrentRPM();
```

### Waiting for Flywheel

Use `SpinUpFlywheelCommand` to wait for the flywheel to reach speed:

```java
new SequentialGroup(
    // Spin up and wait
    new SpinUpFlywheelCommand(flywheel, 3000, 50, 2.0),
    // Now ready to shoot
    new ShootCommand(shooter)
)
```

Parameters:
- `targetRPM` - Desired speed
- `toleranceRPM` - How close is "good enough" (default: 50)
- `timeoutSeconds` - Max time to wait (default: 2.0)

### Custom Commands

Create custom commands for complex behaviors:

```java
public class AutoShootCommand extends Command {
    private final FlywheelSubsystem flywheel;
    private final ShooterSubsystem shooter;
    
    @Override
    public void init() {
        flywheel.setHighSpeed();
    }
    
    @Override
    public void execute() {
        if (flywheel.isAtTargetSpeed(50)) {
            shooter.shoot();
        }
    }
    
    @Override
    public boolean isFinished() {
        return shooter.hasShot();
    }
}
```

## Troubleshooting

### Command Not Running

**Problem:** Command doesn't execute when scheduled.

**Solution:** Make sure you've registered the subsystem:
```java
register(mySubsystem);
```

### Commands Canceling Each Other

**Problem:** Commands keep stopping unexpectedly.

**Solution:** This is normal! Only one command can use a subsystem at a time. This prevents conflicting commands (e.g., intake and outtake at the same time).

### Flywheel Not Reaching Speed

**Problem:** `SpinUpFlywheelCommand` times out.

**Solutions:**
- Increase the timeout parameter
- Check motor direction
- Verify `GEAR_RATIO` is correct
- Increase tolerance if close enough

### Path Not Following

**Problem:** Robot doesn't follow Pedro path in autonomous.

**Solutions:**
- Make sure you're using `PedroOpMode` as base class
- Verify `follower.setStartingPose()` is called
- Check that paths are built before `start()`
- Ensure `follower` is initialized with your constants

## Best Practices

1. **One Subsystem Per Mechanism** - Don't share motors between subsystems
2. **Always Stop in end()** - Clean up when commands finish
3. **Use addRequirements()** - Let NextFTC handle conflicts
4. **Test Commands Individually** - Verify each command works before combining
5. **Use Command Groups** - Don't try to manually sequence commands
6. **Leverage Parallel Commands** - Run intake during path following, etc.
7. **Add Telemetry** - Use `periodic()` in subsystems for debug info

## Resources

- **NextFTC Documentation:** https://nextftc.dev
- **NextFTC + Pedro Pathing:** https://nextftc.dev/extensions/pedro/
- **Pedro Pathing:** https://pedropathing.com
- **NextFTC GitHub:** https://github.com/NextFTC
- **Pedro Pathing GitHub:** https://github.com/Pedro-Pathing/PedroPathing

## Need Help?

If you encounter issues:
1. Check that all dependencies are installed (see `build.dependencies.gradle`)
2. Review the example OpModes in `examples/`
3. Test subsystems individually before combining
4. Add telemetry to see what's happening
5. Check NextFTC documentation for advanced features

