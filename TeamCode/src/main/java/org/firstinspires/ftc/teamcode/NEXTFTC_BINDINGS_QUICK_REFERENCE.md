# NextFTC Bindings - Quick Reference

## ✅ Correct Bindings Syntax (Discovered by Testing!)

### Required Import
```java
import dev.nextftc.ftc.Gamepads;
```

### Required Dependency
```gradle
implementation 'dev.nextftc:bindings:1.0.1'
```

## Basic Button Bindings

```java
@Override
public void onInit() {
    // Initialize your subsystem
    IntakeSubsystem intake = new IntakeSubsystem();
    intake.initialize(hardwareMap);
    
    // ✅ CORRECT: Use Gamepads.gamepad1()
    Gamepads.gamepad1().leftBumper().whenBecomesTrue(
        new InstantCommand(() -> intake.intake())
    );
    
    Gamepads.gamepad1().rightBumper().whenBecomesTrue(
        new InstantCommand(() -> intake.outtake())
    );
    
    Gamepads.gamepad1().a().whenBecomesTrue(
        new InstantCommand(() -> intake.stop())
    );
}
```

## Available Buttons

```java
// Face buttons
Gamepads.gamepad1().a()
Gamepads.gamepad1().b()
Gamepads.gamepad1().x()
Gamepads.gamepad1().y()

// Bumpers
Gamepads.gamepad1().leftBumper()
Gamepads.gamepad1().rightBumper()

// D-pad
Gamepads.gamepad1().dpadUp()
Gamepads.gamepad1().dpadDown()
Gamepads.gamepad1().dpadLeft()
Gamepads.gamepad1().dpadRight()

// Stick buttons
Gamepads.gamepad1().leftStickButton()
Gamepads.gamepad1().rightStickButton()

// Back/Start
Gamepads.gamepad1().back()
Gamepads.gamepad1().start()
```

## Available Trigger Methods

```java
// When button is pressed (edge detection)
.whenBecomesTrue(command)

// When button is released
.whenBecomesFalse(command)

// While button is held (continuous)
.whileTrue(command)
.whileFalse(command)

// Toggle on each press
.toggleWhenPressed(command)
```

## Common Patterns

### Simple Action on Press
```java
Gamepads.gamepad1().a().whenBecomesTrue(
    new InstantCommand(() -> subsystem.doSomething())
);
```

### Continuous Action While Held
```java
Gamepads.gamepad1().leftBumper().whileTrue(
    new InstantCommand(() -> intake.intake())
);
```

### Toggle On/Off
```java
Gamepads.gamepad1().x().toggleWhenPressed(
    new InstantCommand(() -> flywheel.toggle())
);
```

### Complex Sequence
```java
Gamepads.gamepad1().y().whenBecomesTrue(
    new SequentialGroup(
        new InstantCommand(() -> arm.raise()),
        new Delay(0.5),
        new InstantCommand(() -> claw.open())
    )
);
```

## Using Both Gamepads

```java
// Driver controls (gamepad1)
Gamepads.gamepad1().a().whenBecomesTrue(...);

// Operator controls (gamepad2)
Gamepads.gamepad2().leftBumper().whenBecomesTrue(...);
```

## Mixing with Traditional Access

You can use **both** bindings and traditional gamepad access:

```java
@Override
public void onInit() {
    // Bindings for button presses
    Gamepads.gamepad1().a().whenBecomesTrue(
        new InstantCommand(() -> intake.stop())
    );
}

@Override
public void onUpdate() {
    // Traditional access for joysticks
    double drive = -gamepad1.right_stick_y;
    double strafe = -gamepad1.right_stick_x;
    double rotate = -gamepad1.left_stick_x;
    
    // Use these for driving
    // driveSubsystem.drive(drive, strafe, rotate);
}
```

## Complete Example

```java
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.core.commands.utility.InstantCommand;

@TeleOp(name = "My TeleOp")
public class MyTeleOp extends NextFTCOpMode {
    
    private IntakeSubsystem intake;
    private FlywheelSubsystem flywheel;
    
    @Override
    public void onInit() {
        // Initialize subsystems
        intake = new IntakeSubsystem();
        flywheel = new FlywheelSubsystem();
        intake.initialize(hardwareMap);
        flywheel.initialize(hardwareMap);
        
        // Intake bindings
        Gamepads.gamepad1().leftBumper().whenBecomesTrue(
            new InstantCommand(() -> intake.intake())
        );
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(
            new InstantCommand(() -> intake.outtake())
        );
        Gamepads.gamepad1().a().whenBecomesTrue(
            new InstantCommand(() -> intake.stop())
        );
        
        // Flywheel bindings
        Gamepads.gamepad1().x().whenBecomesTrue(
            new InstantCommand(() -> flywheel.setHighSpeed())
        );
        Gamepads.gamepad1().b().whenBecomesTrue(
            new InstantCommand(() -> flywheel.stop())
        );
        
        telemetry.addLine("TeleOp Ready!");
        telemetry.update();
    }
    
    @Override
    public void onUpdate() {
        // Traditional joystick access for driving
        double drive = -gamepad1.right_stick_y;
        double strafe = -gamepad1.right_stick_x;
        
        // Your drive code here...
        
        telemetry.addData("Intake Power", intake.getPower());
        telemetry.addData("Flywheel RPM", flywheel.getCurrentRPM());
        telemetry.update();
    }
}
```

## Key Differences from What Docs Might Show

| ❌ WRONG (guessed) | ✅ CORRECT (tested) |
|-------------------|---------------------|
| `gamepad1()` | `Gamepads.gamepad1()` |
| `onTrue(...)` | `whenBecomesTrue(...)` |
| No import needed | `import dev.nextftc.ftc.Gamepads;` |

## Tips for Teaching Middle Schoolers

1. **Start Simple**: Show them the basic pattern first
   ```java
   Gamepads.gamepad1().a().whenBecomesTrue(
       new InstantCommand(() -> subsystem.method())
   );
   ```

2. **Explain the Parts**:
   - `Gamepads.gamepad1()` - Which controller?
   - `.a()` - Which button?
   - `.whenBecomesTrue()` - When to run?
   - `new InstantCommand(...)` - What to run?
   - `() -> subsystem.method()` - How to run it?

3. **Show Traditional Too**: Let them see both approaches and choose what makes sense

4. **Use Hybrid**: Bindings for buttons, traditional for joysticks (best of both worlds!)

