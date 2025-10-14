# NextFTC - CORRECT API Reference

**⚠️ IMPORTANT**: The NextFTC documentation may not be complete. This document shows the **actual** package structure and methods that work.

## Required Dependencies

```gradle
implementation 'dev.nextftc:ftc:1.0.0'           // Core NextFTC
implementation 'dev.nextftc:bindings:1.0.0'      // OPTIONAL: For gamepad1() button bindings
implementation 'dev.nextftc.extensions:pedro:1.0.0'  // For Pedro Pathing integration
```

**Note**: The `bindings` library is **optional**. Without it, use `gamepad1.a` (no parentheses) instead of `gamepad1().a()`. See `NEXTFTC_BINDINGS_VS_TRADITIONAL.md` for details.

## Correct Package Imports

```java
// OpMode base class
import dev.nextftc.ftc.NextFTCOpMode;

// Gamepad bindings (OPTIONAL - only if using button bindings)
import dev.nextftc.ftc.Gamepads;

// Subsystem (it's an interface, use implements!)
import dev.nextftc.core.subsystems.Subsystem;

// Command groups
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;

// Utility commands
import dev.nextftc.core.commands.utility.InstantCommand;

// Delays
import dev.nextftc.core.commands.delays.Delay;

// Pedro Pathing integration
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
```

## Correct Subsystem Structure

**IMPORTANT**: `Subsystem` is an **interface**, not a class!

```java
import dev.nextftc.core.subsystems.Subsystem;

public class MySubsystem implements Subsystem {  // implements, not extends!
    
    // Custom initialization method (not part of Subsystem interface)
    public void initialize(HardwareMap hardwareMap) {
        // Initialize hardware here
        // Call this from your OpMode's onInit()
    }
    
    @Override
    public void periodic() {
        // Called automatically by NextFTC every loop
        // This IS part of the Subsystem interface
    }
    
    // Your custom methods here
}
```

## Correct Lifecycle Methods

### TeleOp Methods
```java
@Override
public void onInit() {
    // Called when "INIT" is pressed
    // Initialize hardware and subsystems here
}

@Override
public void onUpdate() {
    // Called repeatedly during TeleOp
    // Put telemetry and manual control here
}
```

### Autonomous Methods
```java
@Override
public void onInit() {
    // Called when "INIT" is pressed
    // Initialize hardware, subsystems, and build paths here
}

@Override
public void onStartButtonPressed() {
    // Called when "START/PLAY" is pressed
    // Schedule autonomous command groups here
}

@Override
public void onUpdate() {
    // Called repeatedly during autonomous
    // Put telemetry here (commands run automatically)
}
```

## Correct Subsystem Initialization

**DON'T use** `register(subsystem)` - this method doesn't exist in NextFTC!

**DO use** `subsystem.initialize(hardwareMap)`:

```java
@Override
public void onInit() {
    intake = new IntakeSubsystem();
    shooter = new ShooterSubsystem();
    
    // Initialize each subsystem manually
    // (initialize() is YOUR custom method, not from the Subsystem interface)
    intake.initialize(hardwareMap);
    shooter.initialize(hardwareMap);
}
```

**Note**: The `initialize(HardwareMap)` method is a **custom method** you add to your subsystems - it's NOT part of the `Subsystem` interface! The only method required by the `Subsystem` interface is `periodic()`.

## Correct Bindings Syntax (OPTIONAL)

**Only if you have `implementation 'dev.nextftc:bindings:1.0.1'` in dependencies:**

```java
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.core.commands.utility.InstantCommand;

@Override
public void onInit() {
    IntakeSubsystem intake = new IntakeSubsystem();
    intake.initialize(hardwareMap);
    
    // ✅ CORRECT: Gamepads.gamepad1().button().whenBecomesTrue(...)
    Gamepads.gamepad1().leftBumper().whenBecomesTrue(
        new InstantCommand(() -> intake.intake())
    );
    
    Gamepads.gamepad1().a().whenBecomesTrue(
        new InstantCommand(() -> intake.stop())
    );
}
```

**Common trigger methods**:
- `whenBecomesTrue(command)` - When button is pressed
- `whenBecomesFalse(command)` - When button is released
- `whileTrue(command)` - Continuously while held
- `toggleWhenPressed(command)` - Toggle on each press

**See `NEXTFTC_BINDINGS_QUICK_REFERENCE.md` for complete bindings guide.**

## Correct Command Scheduling

**BEST PRACTICE** - Create command methods and use `.schedule()`:

```java
private Command autoRoutine() {
    return new SequentialGroup(
        new FollowPath(pathToScore),
        new Delay(1.0),
        new InstantCommand(() -> intake.stop())
    );
}

@Override
public void onStartButtonPressed() {
    autoRoutine().schedule();
}
```

**Alternative** - Use `runAsync()` for simple cases:
```java
@Override
public void onStartButtonPressed() {
    runAsync(
        new SequentialGroup(
            new FollowPath(pathToScore),
            new Delay(1.0),
            new InstantCommand(() -> intake.stop())
        )
    );
}
```

## Correct Pedro Pathing Integration

### 1. Add PedroComponent in Constructor

```java
public MyAutonomous() {
    addComponents(
        new PedroComponent(Constants::createFollower)
    );
}
```

### 2. Access Follower via PedroComponent

**DON'T use** `follower = ...` or `follower.pathBuilder()`

**DO use** `PedroComponent.follower()`:

```java
@Override
public void onInit() {
    // Set starting pose
    PedroComponent.follower().setStartingPose(startPose);
    
    // Build paths
    pathChain = PedroComponent.follower().pathBuilder()
        .addPath(new BezierLine(start, end))
        .build();
}
```

### 3. Use FollowPath Command

**DON'T use** `new FollowPathCommand(follower, path)`

**DO use** `new FollowPath(path)`:

```java
runAsync(
    new SequentialGroup(
        new FollowPath(pathToScore),  // ✅ Correct
        new InstantCommand(() -> intake.stop())
    )
);
```

## Correct OpMode Structure

### TeleOp Example
```java
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.core.commands.utility.InstantCommand;

@TeleOp(name = "My TeleOp")
public class MyTeleOp extends NextFTCOpMode {
    
    private IntakeSubsystem intake;
    
    @Override
    public void onInit() {
        intake = new IntakeSubsystem();
        intake.initialize(hardwareMap);
        
        // Button bindings
        gamepad1().leftBumper().onTrue(
            new InstantCommand(() -> intake.intake())
        );
    }
    
    @Override
    public void onLoop() {
        // Traditional gamepad access for driving
        double drive = -gamepad1.right_stick_y;
        double strafe = -gamepad1.right_stick_x;
        
        // Telemetry
        telemetry.addData("Power", intake.getPower());
        telemetry.update();
    }
}
```

### Autonomous Example
```java
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;

@Autonomous(name = "My Auto")
public class MyAutonomous extends NextFTCOpMode {
    
    public MyAutonomous() {
        addComponents(
            new PedroComponent(Constants::createFollower)
        );
    }
    
    private IntakeSubsystem intake;
    private PathChain pathToScore;
    
    @Override
    public void onInit() {
        intake = new IntakeSubsystem();
        intake.initialize(hardwareMap);
        
        PedroComponent.follower().setStartingPose(startPose);
        
        pathToScore = PedroComponent.follower().pathBuilder()
            .addPath(new BezierLine(start, end))
            .build();
    }
    
    // Create reusable command method
    private Command autoRoutine() {
        return new SequentialGroup(
            new ParallelGroup(
                new FollowPath(pathToScore),
                new InstantCommand(() -> intake.intake())
            ),
            new Delay(1.0),
            new InstantCommand(() -> intake.stop())
        );
    }
    
    @Override
    public void onStartButtonPressed() {
        autoRoutine().schedule();
    }
    
    @Override
    public void onUpdate() {
        telemetry.addData("X", PedroComponent.follower().getPose().getX());
        telemetry.update();
    }
}
```

## Common Mistakes and Fixes

| ❌ WRONG | ✅ CORRECT |
|----------|-----------|
| `extends OpMode` | `extends NextFTCOpMode` |
| `init()` | `onInit()` |
| `start()` | `onStartButtonPressed()` |
| `loop()` or `run()` | `onUpdate()` |
| `schedule(commandGroup)` in method | `command.schedule()` on Command object |
| `register(subsystem)` | `subsystem.initialize(hardwareMap)` |
| `new WaitCommand(1.0)` | `new Delay(1.0)` |
| `new FollowPathCommand(follower, path)` | `new FollowPath(path)` |
| `follower = Constants.createFollower(...)` | `PedroComponent.follower()` |
| `extends PedroOpMode` | `extends NextFTCOpMode` with constructor |
| `import dev.nextftc.core.command.*` | `import dev.nextftc.core.commands.*` |
| `import dev.nextftc.core.NextFTCOpMode` | `import dev.nextftc.ftc.NextFTCOpMode` |
| `extends Subsystem` | `implements Subsystem` |
| `import dev.nextftc.core.command.Subsystem` | `import dev.nextftc.core.subsystems.Subsystem` |

## Key Differences from Documentation

1. **Subsystem is an Interface**: Use `implements Subsystem`, NOT `extends Subsystem`
2. **Subsystem Import**: `import dev.nextftc.core.subsystems.Subsystem` (not `dev.nextftc.core.command.Subsystem`)
3. **Subsystem Registration**: Call `subsystem.initialize(hardwareMap)` directly (not `register()`)
4. **Command Scheduling**: Best practice is to create `Command` methods and call `.schedule()` on them
5. **Package Names**: Most imports are under `dev.nextftc.core.commands.*` (plural), not `dev.nextftc.core.command.*`
6. **OpMode Base Class**: Import from `dev.nextftc.ftc.NextFTCOpMode`, not `dev.nextftc.core.NextFTCOpMode`
7. **Pedro Integration**: Use component-based architecture with `PedroComponent.follower()`, not direct follower field
8. **Lifecycle Methods**: Use `onUpdate()` for the loop, not `onLoop()` or `run()`

## Summary

All of your example files have been updated to use the **correct** NextFTC API. Use them as templates when creating new OpModes!

