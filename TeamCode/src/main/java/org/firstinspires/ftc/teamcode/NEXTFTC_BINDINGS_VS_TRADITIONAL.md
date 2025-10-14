# NextFTC: Bindings vs. Traditional Gamepad Access

## Two Ways to Use NextFTC

NextFTC gives you **flexibility** - you can use button bindings for cleaner code, OR stick with traditional gamepad access. You can even **mix both**!

---

## Option 1: With Bindings (Cleaner, More Features)

### Required Dependency
Add to `build.dependencies.gradle`:
```gradle
implementation 'dev.nextftc:bindings:1.0.0'
```

### Example TeleOp with Bindings
```java
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.core.commands.utility.InstantCommand;

@TeleOp(name = "With Bindings")
public class WithBindingsTeleOp extends NextFTCOpMode {
    
    private IntakeSubsystem intake;
    
    @Override
    public void onInit() {
        intake = new IntakeSubsystem();
        intake.initialize(hardwareMap);
        
        // ✅ NextFTC Bindings - Gamepads.gamepad1()
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
    
    @Override
    public void onUpdate() {
        // Traditional access for driving (still works!)
        double drive = -gamepad1.right_stick_y;
        double strafe = -gamepad1.right_stick_x;
        
        telemetry.addData("Drive", drive);
        telemetry.update();
    }
}
```

### Pros:
- ✅ Clean, declarative button binding syntax
- ✅ Built-in edge detection (onTrue, onFalse, whileTrue, etc.)
- ✅ No manual button state tracking needed
- ✅ More features (toggles, double-press, etc.)
- ✅ Can still use traditional access for joysticks

### Cons:
- ⚠️ Additional dependency (very small)
- ⚠️ Slightly more to learn initially

---

## Option 2: Without Bindings (Simpler, Traditional)

### No Extra Dependency Needed
Works with just:
```gradle
implementation 'dev.nextftc:ftc:1.0.0'
```

### Example TeleOp without Bindings
```java
import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp(name = "Without Bindings")
public class WithoutBindingsTeleOp extends NextFTCOpMode {
    
    private IntakeSubsystem intake;
    
    @Override
    public void onInit() {
        intake = new IntakeSubsystem();
        intake.initialize(hardwareMap);
    }
    
    @Override
    public void onUpdate() {
        // ✅ Traditional gamepad access - gamepad1 without parentheses
        if (gamepad1.left_bumper) {
            intake.intake();
        } else if (gamepad1.right_bumper) {
            intake.outtake();
        } else if (gamepad1.a) {
            intake.stop();
        }
        
        // Traditional driving
        double drive = -gamepad1.right_stick_y;
        double strafe = -gamepad1.right_stick_x;
        
        telemetry.addData("Intake Power", intake.getPower());
        telemetry.update();
    }
}
```

### Pros:
- ✅ Familiar - works exactly like old FTC SDK
- ✅ One less dependency
- ✅ Simpler for beginners
- ✅ Full control in onUpdate()

### Cons:
- ⚠️ Manual button state tracking for edge detection
- ⚠️ More verbose for complex button logic
- ⚠️ All logic in onUpdate() can get messy

---

## Option 3: Hybrid Approach (Recommended!)

You can **mix both** approaches in the same OpMode:

```java
import dev.nextftc.ftc.Gamepads;

@TeleOp(name = "Hybrid Approach")
public class HybridTeleOp extends NextFTCOpMode {
    
    private IntakeSubsystem intake;
    
    @Override
    public void onInit() {
        intake = new IntakeSubsystem();
        intake.initialize(hardwareMap);
        
        // Use bindings for discrete button presses
        Gamepads.gamepad1().a().whenBecomesTrue(new InstantCommand(() -> intake.stop()));
    }
    
    @Override
    public void onUpdate() {
        // Use traditional access for continuous control
        double drive = -gamepad1.right_stick_y;
        double strafe = -gamepad1.right_stick_x;
        
        // Can also mix in traditional button access if needed
        if (gamepad1.dpad_up) {
            // Do something continuously while held
        }
        
        telemetry.update();
    }
}
```

---

## For Autonomous: Bindings Not Needed

In autonomous, you don't use gamepads at all, so the bindings library doesn't matter:

```java
@Autonomous(name = "My Auto")
public class MyAuto extends NextFTCOpMode {
    
    // No bindings needed - just use commands!
    private Command autoRoutine() {
        return new SequentialGroup(
            new FollowPath(pathToScore),
            new InstantCommand(() -> intake.intake()),
            new Delay(1.0)
        );
    }
    
    @Override
    public void onStartButtonPressed() {
        autoRoutine().schedule();
    }
}
```

---

## What About Commands?

**Commands work either way!** You can use NextFTC's command system for autonomous without needing bindings:

### Without Bindings:
```java
// In autonomous
autoRoutine().schedule();

// Or manually in TeleOp
new InstantCommand(() -> intake.intake()).schedule();
```

### With Bindings:
```java
import dev.nextftc.ftc.Gamepads;

// Bind to button
Gamepads.gamepad1().a().whenBecomesTrue(new InstantCommand(() -> intake.intake()));
```

---

## Our Recommendation for Your Team

Given that you're teaching middle schoolers and want to minimize confusion:

### **Start WITHOUT bindings**, then add it later if you want:

1. **Week 1-2**: Learn NextFTC basics
   - Subsystems
   - Commands in autonomous
   - Traditional gamepad in TeleOp
   
2. **Week 3+**: Optionally introduce bindings
   - Show the cleaner syntax
   - Let students choose which they prefer

### How to Switch:

**If you DON'T add bindings**:
- Change all `gamepad1()` to `gamepad1` (no parentheses)
- Move button logic to `onUpdate()`
- Everything else stays the same!

**If you DO add bindings**:
- Just sync Gradle (it's already added!)
- Your current examples will work as-is

---

## Summary Table

| Feature | With Bindings | Without Bindings |
|---------|---------------|------------------|
| Dependency | `dev.nextftc:bindings:1.0.1` | Not needed |
| Button syntax | `Gamepads.gamepad1().a().whenBecomesTrue(...)` | `if (gamepad1.a)` |
| Edge detection | Built-in | Manual tracking needed |
| Complexity | Slightly more | Simpler |
| Best for | Clean code, advanced users | Beginners, traditional feel |
| Commands | Works great | Works great |
| Autonomous | Not needed | Not needed |

---

## Quick Decision Guide

**Add bindings if:**
- ✅ You want cleaner TeleOp code
- ✅ You need edge detection (button press vs. hold)
- ✅ You're comfortable with a bit more abstraction

**Skip bindings if:**
- ✅ You want to keep it simple
- ✅ You're just learning NextFTC
- ✅ You only care about autonomous (bindings not used there)

**You can always add it later!** The decision isn't permanent.

