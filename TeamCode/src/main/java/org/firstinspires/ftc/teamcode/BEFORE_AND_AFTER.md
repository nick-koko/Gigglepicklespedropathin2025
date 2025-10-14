# Before and After: What Actually Changes

This shows exactly what changes when you switch from your old code to NextFTC.

---

## Mechanism File (IntakeMotorSpinner → IntakeSubsystem)

### ❌ BEFORE (Your Old IntakeMotorSpinner.java)

```java
package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeMotorSpinner {
    DcMotor motor;
    double power = 0;

    public void init(HardwareMap hwMap) {
        motor = hwMap.get(DcMotor.class, "intake_motor");
    }

    public void Intake() {
        power = 0.7;
        motor.setPower(power);
    }

    public void Outtake() {
        power = -0.7;
        motor.setPower(power);
    }

    public void Stop() {
        power = 0.0;
        motor.setPower(power);
    }
}
```

### ✅ AFTER (New IntakeSubsystem.java)

```java
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.core.subsystems.Subsystem;  // ← NEW: Import Subsystem

public class IntakeSubsystem implements Subsystem {  // ← NEW: implements Subsystem (it's an interface!)
    private DcMotor motor;  // ← NEW: private (good practice)
    
    // ← NEW: Custom initialize method (call from onInit)
    public void initialize(HardwareMap hardwareMap) {  // ← CHANGED: init → initialize
        motor = hardwareMap.get(DcMotor.class, "intake_motor");
    }

    public void intake() {  // ← CHANGED: Intake → intake (lowercase)
        motor.setPower(0.7);
    }

    public void outtake() {  // ← CHANGED: Outtake → outtake
        motor.setPower(-0.7);
    }

    public void stop() {  // ← CHANGED: Stop → stop
        motor.setPower(0.0);
    }
    
    // ← NEW: Add this helper method
    public double getPower() {
        return motor.getPower();
    }
}
```

### What Changed:
1. ✏️ Add `extends Subsystem`
2. ✏️ Add import for Subsystem
3. ✏️ `init()` → `initialize()`
4. ✏️ Method names lowercase (`Intake()` → `intake()`)
5. ✏️ Make motor `private` (optional but good practice)

**That's it!** The code inside the methods is exactly the same.

---

## TeleOp

### ❌ BEFORE (Your Old TeleOp)

```java
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeMotorSpinner;

@TeleOp(name = "My TeleOp")
public class MyOldTeleOp extends OpMode {
    
    IntakeMotorSpinner intake = new IntakeMotorSpinner();
    double drivePower = 1.0;

    @Override
    public void init() {
        intake.init(hardwareMap);
        
        telemetry.addLine("Ready!");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Driving
        double driving = (-gamepad1.right_stick_y) * drivePower;
        double strafe = (-gamepad1.right_stick_x) * drivePower;
        double rotate = (-gamepad1.left_stick_x) * 0.5;
        // ... use for drive motors ...
        
        // Intake control
        if (gamepad1.left_bumper) {
            intake.Intake();
        } else if (gamepad1.right_bumper) {
            intake.Outtake();
        } else {
            intake.Stop();
        }
        
        telemetry.addData("Drive Power", drivePower);
        telemetry.update();
    }
}
```

### ✅ AFTER (New TeleOp)

```java
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;  // ← CHANGED: import path

import dev.nextftc.ftc.NextFTCOpMode;  // ← NEW: Import NextFTCOpMode

@TeleOp(name = "My TeleOp")
public class MyNewTeleOp extends NextFTCOpMode {  // ← CHANGED: extends NextFTCOpMode
    
    IntakeSubsystem intake = new IntakeSubsystem();  // ← CHANGED: class name
    double drivePower = 1.0;

    @Override
    public void init() {
        register(intake);  // ← NEW: register the subsystem
        
        telemetry.addLine("Ready!");
        telemetry.update();
    }

    @Override
    public void run() {  // ← CHANGED: loop() → run()
        // Driving - EXACTLY THE SAME
        double driving = (-gamepad1.right_stick_y) * drivePower;
        double strafe = (-gamepad1.right_stick_x) * drivePower;
        double rotate = (-gamepad1.left_stick_x) * 0.5;
        // ... use for drive motors ...
        
        // Intake control - ALMOST THE SAME
        if (gamepad1.left_bumper) {
            intake.intake();  // ← CHANGED: lowercase
        } else if (gamepad1.right_bumper) {
            intake.outtake();  // ← CHANGED: lowercase
        } else {
            intake.stop();  // ← CHANGED: lowercase
        }
        
        telemetry.addData("Drive Power", drivePower);
        telemetry.update();
    }
}
```

### What Changed:
1. ✏️ `extends OpMode` → `extends NextFTCOpMode`
2. ✏️ Add `register(intake)` in init
3. ✏️ `loop()` → `run()`
4. ✏️ Method names lowercase
5. ✅ **Everything else is the same!**

---

## Autonomous - This is Where the Magic Happens!

### ❌ BEFORE (With RoadRunner Actions - Had Issues!)

```java
@Autonomous(name = "Old Auto")
public class OldAuto extends OpMode {
    IntakeMotorSpinner intake = new IntakeMotorSpinner();
    
    // Manual state machine - complicated!
    int state = 0;
    Timer timer = new Timer();
    
    @Override
    public void init() {
        intake.init(hardwareMap);
        // Set up Pedro Pathing...
    }
    
    @Override
    public void start() {
        timer.reset();
    }
    
    @Override
    public void loop() {
        // Manual state machine - gets messy fast!
        switch(state) {
            case 0:  // Drive to pickup
                if (reachedPosition()) {
                    intake.Intake();
                    state = 1;
                    timer.reset();
                }
                break;
            case 1:  // Wait
                if (timer.seconds() > 1.0) {
                    intake.Stop();
                    state = 2;
                }
                break;
            // ... more states ...
        }
    }
}
```

**Problems with old way:**
- 😞 Manual state machine (complex!)
- 😞 Hard to do multiple things at once
- 😞 RoadRunner Actions had bugs with Pedro Pathing
- 😞 Difficult to read and understand

### ✅ AFTER (With NextFTC - Much Better!)

```java
@Autonomous(name = "New Auto")
public class NewAuto extends PedroOpMode {  // ← NEW: extends PedroOpMode
    
    private IntakeSubsystem intake;
    
    @Override
    public void init() {
        intake = new IntakeSubsystem();
        register(intake);  // ← NEW: register subsystem
        
        // Set up Pedro Pathing (SAME AS BEFORE!)
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }
    
    @Override
    public void start() {
        // ← NEW: This is SO much easier to read!
        schedule(
            new SequentialGroup(
                // Drive to pickup WHILE running intake
                new ParallelGroup(
                    new FollowPathCommand(follower, pathToPickup),
                    new InstantCommand(() -> intake.intake())
                ),
                
                // Wait to grab
                new WaitCommand(1.0),
                
                // Stop intake
                new InstantCommand(() -> intake.stop()),
                
                // Drive back
                new FollowPathCommand(follower, pathBack)
            )
        );
    }
    
    @Override
    public void run() {
        // Just telemetry - NextFTC handles everything else!
        telemetry.addData("X", follower.getPose().getX());
        telemetry.update();
    }
}
```

**Benefits of new way:**
- 😊 No state machine! (NextFTC handles it)
- 😊 Easy to do multiple things at once (`ParallelGroup`)
- 😊 Works perfectly with Pedro Pathing
- 😊 Easy to read: "drive while intaking, then wait, then stop"

---

## Side-by-Side: Key Concepts

### Sequential (Do Things in Order)

**OLD WAY:**
```java
int state = 0;

void loop() {
    switch(state) {
        case 0: 
            doThing1();
            state = 1;
            break;
        case 1:
            doThing2();
            state = 2;
            break;
    }
}
```

**NEW WAY:**
```java
schedule(
    new SequentialGroup(
        doThing1(),
        doThing2()
    )
);
```

### Parallel (Do Things Together)

**OLD WAY:**
```java
void loop() {
    // This is HARD - how do you coordinate?
    driveToPosition();
    intake.Intake();  // But we want both!
}
```

**NEW WAY:**
```java
schedule(
    new ParallelGroup(
        driveToPosition(),
        intake()
    )
);
```

---

## What You Tell Your Students

### "How much are we changing?"

**TeleOp:** About 5 lines
- Change which class we extend
- Add one `register()` line
- Change `loop()` to `run()`
- Use lowercase method names

**Autonomous:** Complete rewrite, BUT:
- Much easier to understand
- No more state machines!
- Can do things in parallel
- Works with Pedro Pathing

### "Why are we doing this?"

1. **Old way had bugs** with Pedro Pathing
2. **New way is actually easier** for autonomous
3. **Lots of FTC teams use it** - it's proven
4. **TeleOp barely changes** - still works the same

### "What do we need to learn?"

**New concepts (only 4!):**
1. `Subsystem` - just means "robot mechanism"
2. `SequentialGroup` - do things in order
3. `ParallelGroup` - do things together
4. `InstantCommand(() -> method())` - do one action

That's it! Everything else you already know!

---

## Summary

| What | Before | After | How Hard? |
|------|--------|-------|-----------|
| **Mechanism File** | `IntakeMotorSpinner` | `IntakeSubsystem` | Easy: 5 changes |
| **TeleOp** | `extends OpMode` | `extends NextFTCOpMode` | Easy: Almost the same |
| **Autonomous** | State machine | Command groups | Different, but easier! |
| **Pedro Pathing** | Same | Same | No change! |
| **Gamepad** | Same | Same | No change! |

**Bottom line:** Small changes for TeleOp, better system for autonomous!

