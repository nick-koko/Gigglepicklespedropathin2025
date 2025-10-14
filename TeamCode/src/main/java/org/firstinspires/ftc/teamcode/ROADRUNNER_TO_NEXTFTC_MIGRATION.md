# RoadRunner Actions → NextFTC Migration Guide

## For Students Who Used RoadRunner Actions Last Year

**Good news!** You already understand the hard parts (Sequential and Parallel). NextFTC works almost the same way!

---

## What You Already Know

✅ **Sequential** - Do things one after another  
✅ **Parallel** - Do things at the same time  
✅ **Actions** - Wrappers that call mechanism methods  
✅ **return true/false** - Whether action is done  

**All of this transfers directly to NextFTC!**

---

## Direct Translation Guide

### Old Way: RoadRunner Actions

```java
// 1. Mechanism file
public class IntakeServoSpinner {
    CRServo servo;
    
    public void init(HardwareMap hwMap) {
        servo = hwMap.get(CRServo.class, "intake_servo");
    }
    
    public void Intake() { servo.setPower(1.0); }
    public void Stop() { servo.setPower(0.0); }
}

// 2. Actions wrapper (separate file!)
public class IntakeservoSpinnerActions extends IntakeServoSpinner {
    
    public class IntakePosition implements Action {
        private boolean initialized = false;
        
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                Intake();
                initialized = true;
                return true;  // Still has more to run
            }
            return false;  // Done
        }
    }
    
    public Action intakePosition() {
        return new IntakePosition();
    }
}

// 3. In autonomous
Actions.runBlocking(
    new SequentialAction(
        new ParallelAction(
            followPath,
            intakeSpinner.intakePosition()
        ),
        new SleepAction(1.0),
        intakeSpinner.stopPosition()
    )
);
```

### New Way: NextFTC Commands

```java
// 1. Subsystem (replaces mechanism file)
public class IntakeSubsystem extends Subsystem {
    private CRServo servo;
    
    @Override
    public void initialize(HardwareMap hardwareMap) {
        servo = hardwareMap.get(CRServo.class, "intake_servo");
    }
    
    public void intake() { servo.setPower(1.0); }
    public void stop() { servo.setPower(0.0); }
}

// 2. NO Actions wrapper needed! Use InstantCommand instead

// 3. In autonomous
schedule(
    new SequentialGroup(
        new ParallelGroup(
            new FollowPathCommand(follower, path),
            new InstantCommand(() -> intake.intake())
        ),
        new WaitCommand(1.0),
        new InstantCommand(() -> intake.stop())
    )
);
```

---

## Side-by-Side Translation

| RoadRunner Actions | NextFTC Commands |
|-------------------|------------------|
| `Mechanism.java` | `Subsystem.java` |
| `MechanismActions.java` (extends Mechanism) | **Not needed!** |
| `implements Action` | `extends Command` (only for complex) |
| `Action.run()` returns true/false | `Command.isFinished()` returns true/false |
| `Actions.runBlocking()` | `schedule()` |
| `SequentialAction` | `SequentialGroup` |
| `ParallelAction` | `ParallelGroup` |
| `SleepAction(1.0)` | `WaitCommand(1.0)` |
| `new MyAction()` | `new InstantCommand(() -> method())` |

---

## Your Actual Code: Before and After

### OLD: RoadRunner Actions

**IntakeservoSpinnerActions.java** (65 lines):
```java
public class IntakeservoSpinnerActions extends IntakeServoSpinner {

    public class IntakePosition implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                Intake();
                initialized = true;
                return true;
            }
            return false;
        }
    }

    public Action intakePosition() {
        return new IntakePosition();
    }

    // ... repeat for outtake, stop, etc.
}
```

**In Autonomous:**
```java
Actions.runBlocking(
    new SequentialAction(
        new ParallelAction(
            followStartToBucket1,
            outtakeSlide.high(),
            new SequentialAction(
                new SleepAction(1),
                outtakeDump.bucketPosition()
            )
        ),
        new SleepAction(0.2),
        intakeSpinner.intakePosition(),
        intakeSpinner.stopPosition()
    )
);
```

### NEW: NextFTC

**IntakeSubsystem.java** (Just the subsystem - no Actions file!):
```java
public class IntakeSubsystem extends Subsystem {
    private CRServo servo;
    
    @Override
    public void initialize(HardwareMap hardwareMap) {
        servo = hardwareMap.get(CRServo.class, "intake_servo");
    }
    
    public void intake() { servo.setPower(1.0); }
    public void stop() { servo.setPower(0.0); }
}
```

**In Autonomous:**
```java
schedule(
    new SequentialGroup(
        new ParallelGroup(
            new FollowPathCommand(follower, pathToScore),
            new InstantCommand(() -> outtakeSlide.setHigh()),
            new SequentialGroup(
                new WaitCommand(1.0),
                new InstantCommand(() -> outtakeDump.bucketPosition())
            )
        ),
        new WaitCommand(0.2),
        new InstantCommand(() -> intake.intake()),
        new InstantCommand(() -> intake.stop())
    )
);
```

---

## Key Differences

### 1. No More Actions Wrapper Files!

**OLD:**
- ❌ Create mechanism file
- ❌ Create separate Actions file that extends mechanism
- ❌ Write Action inner classes for each method
- ❌ 65+ lines of boilerplate per mechanism

**NEW:**
- ✅ Create subsystem file
- ✅ Use InstantCommand to call methods directly
- ✅ Done! No extra files needed

### 2. Simpler Syntax

**OLD:**
```java
intakeSpinner.intakePosition()  // Calls factory method that returns Action
```

**NEW:**
```java
new InstantCommand(() -> intake.intake())  // Directly calls method
```

### 3. Same Concepts, Different Names

**Students already know these concepts!**

| Concept | RoadRunner | NextFTC |
|---------|-----------|---------|
| Do things in order | SequentialAction | SequentialGroup |
| Do things together | ParallelAction | ParallelGroup |
| Wait for time | SleepAction | WaitCommand |
| Run the routine | Actions.runBlocking() | schedule() |

---

## Converting Your Autonomous

Here's your actual autonomous code converted section by section:

### Section 1: Score Preload

**OLD:**
```java
Actions.runBlocking(
    new SequentialAction(
        new ParallelAction(
            followStartToBucket1,
            outtakeSlide.high(),
            new SequentialAction(
                new SleepAction(1),
                outtakeDump.bucketPosition()
            )
        )
    )
);
```

**NEW:**
```java
schedule(
    new SequentialGroup(
        new ParallelGroup(
            new FollowPathCommand(follower, StartToBucket1),
            new InstantCommand(() -> outtakeSlide.setHigh()),
            new SequentialGroup(
                new WaitCommand(1.0),
                new InstantCommand(() -> outtakeDump.bucketPosition())
            )
        )
    )
);
```

**Changes:**
1. `Actions.runBlocking()` → `schedule()`
2. `SequentialAction` → `SequentialGroup`
3. `ParallelAction` → `ParallelGroup`
4. `SleepAction(1)` → `WaitCommand(1.0)`
5. `outtakeSlide.high()` → `new InstantCommand(() -> outtakeSlide.setHigh())`
6. Follow path wrapped in `FollowPathCommand`

### Section 2: Complex Nested Actions

**OLD:**
```java
new ParallelAction(
    followBucketToSample2,
    outtakeSlide.low(),
    new SequentialAction(
        new SleepAction(0.1),
        outtakeDump.downPosition()
    ),
    new SequentialAction(
        new SleepAction(0.5),
        new ParallelAction(
            intakeSlide.autonAction(),
            intakeWrist.wristIntakeAbyss(),
            intakeArm.armIntakeAbyss()
        )
    )
)
```

**NEW:**
```java
new ParallelGroup(
    new FollowPathCommand(follower, BucketToSample2),
    new InstantCommand(() -> outtakeSlide.setLow()),
    new SequentialGroup(
        new WaitCommand(0.1),
        new InstantCommand(() -> outtakeDump.downPosition())
    ),
    new SequentialGroup(
        new WaitCommand(0.5),
        new ParallelGroup(
            new InstantCommand(() -> intakeSlide.autonPosition()),
            new InstantCommand(() -> intakeWrist.intakeAbyss()),
            new InstantCommand(() -> intakeArm.intakeAbyss())
        )
    )
)
```

**The structure is identical!** Just different names.

---

## Step-by-Step Migration

### Step 1: Convert One Mechanism

**Pick one mechanism from last year** (like IntakeServoSpinner)

**Before:**
```java
public class IntakeServoSpinner {
    CRServo servo;
    
    public void init(HardwareMap hwMap) {
        servo = hwMap.get(CRServo.class, "intake_servo");
    }
    
    public void Intake() { servo.setPower(1.0); }
}
```

**After:**
```java
public class IntakeSubsystem extends Subsystem {
    private CRServo servo;
    
    @Override
    public void initialize(HardwareMap hardwareMap) {
        servo = hardwareMap.get(CRServo.class, "intake_servo");
    }
    
    public void intake() { servo.setPower(1.0); }
}
```

**Delete the Actions file!** You don't need `IntakeservoSpinnerActions.java` anymore!

### Step 2: Update Autonomous

**Find/Replace in your autonomous:**
- `Actions.runBlocking(` → `schedule(`
- `SequentialAction` → `SequentialGroup`
- `ParallelAction` → `ParallelGroup`
- `SleepAction` → `WaitCommand`

**Then wrap mechanism calls:**
- `intakeSpinner.intakePosition()` → `new InstantCommand(() -> intake.intake())`
- `outtakeSlide.high()` → `new InstantCommand(() -> outtakeSlide.setHigh())`

**And wrap paths:**
- `followStartToBucket1` → `new FollowPathCommand(follower, StartToBucket1)`

### Step 3: Update OpMode Base Class

**OLD:**
```java
public class FourOrMoreSampleAutoPedro extends OpMode {
    @Override
    public void loop() {
        follower.update();
    }
}
```

**NEW:**
```java
public class FourOrMoreSampleAutoPedro extends PedroOpMode {
    @Override
    public void run() {
        // follower.update() is automatic!
    }
}
```

---

## Complex Actions (When InstantCommand Isn't Enough)

**Sometimes you need actions that wait for something.** Like your old Actions that returned true/false.

### OLD: Custom Action

```java
public class ExtendUntilPositionAction implements Action {
    private boolean done = false;
    
    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (slide.getPosition() >= targetPosition) {
            done = true;
            return false;  // Done
        }
        slide.setPower(1.0);
        return true;  // Keep running
    }
}
```

### NEW: Custom Command

```java
public class ExtendUntilPositionCommand extends Command {
    
    public ExtendUntilPositionCommand(SlideSubsystem slide, int target) {
        this.slide = slide;
        this.target = target;
        addRequirements(slide);
    }
    
    @Override
    public void init() {
        slide.setPower(1.0);
    }
    
    @Override
    public void execute() {
        // Runs repeatedly
    }
    
    @Override
    public boolean isFinished() {
        return slide.getPosition() >= target;  // Done when reached
    }
    
    @Override
    public void end(boolean interrupted) {
        slide.stop();
    }
}
```

**Same concept!** Just `isFinished()` instead of `return false`.

---

## Benefits of NextFTC Over RoadRunner Actions

### 1. Less Code

**Your IntakeservoSpinnerActions.java:** 65 lines  
**New IntakeSubsystem.java:** 20 lines  
**Savings:** 45 lines per mechanism!

### 2. Fewer Files

**OLD:** 2 files per mechanism (Mechanism + Actions)  
**NEW:** 1 file per mechanism (Subsystem only)

### 3. Easier to Understand

**OLD:**
```java
public Action intakePosition() {
    return new IntakePosition();
}
public class IntakePosition implements Action {
    // 10 lines of boilerplate...
}
```

**NEW:**
```java
new InstantCommand(() -> intake.intake())
```

### 4. No Bugs with Pedro Pathing!

**OLD:** RoadRunner Actions + Pedro Pathing = bugs ❌  
**NEW:** NextFTC + Pedro Pathing = works great! ✅

---

## What Students Need to Learn

Since they already know Sequential/Parallel from RoadRunner Actions:

**Only 3 new things:**

1. **Subsystem** instead of Mechanism
   - "Same thing, new name"

2. **InstantCommand** instead of Action wrapper
   - "Call the method directly, no wrapper needed"

3. **schedule()** instead of Actions.runBlocking()
   - "Start the commands running"

**That's it!** Everything else they already know!

---

## Teaching Approach

### Lesson 1: "Actions Wrappers Are Gone!"

**Show them:**
```java
// OLD - needed wrapper
intakeSpinner.intakePosition()

// NEW - direct call
new InstantCommand(() -> intake.intake())
```

**Key message:** "We don't need those Actions files anymore! We call methods directly."

### Lesson 2: "Names Changed"

**Show the table:**
- SequentialAction → SequentialGroup
- ParallelAction → ParallelGroup  
- SleepAction → WaitCommand
- Actions.runBlocking() → schedule()

**Key message:** "Same concepts, just renamed."

### Lesson 3: "Convert One Mechanism"

**Do together:**
1. Take IntakeServoSpinner
2. Change to IntakeSubsystem
3. Delete IntakeservoSpinnerActions
4. Use InstantCommand in autonomous

**Key message:** "See? Less code, same result!"

### Lesson 4: "Convert One Autonomous Routine"

**Start with simple part:**
```java
// OLD
Actions.runBlocking(
    new SequentialAction(
        intakeSpinner.intakePosition(),
        new SleepAction(1.0),
        intakeSpinner.stopPosition()
    )
);

// NEW
schedule(
    new SequentialGroup(
        new InstantCommand(() -> intake.intake()),
        new WaitCommand(1.0),
        new InstantCommand(() -> intake.stop())
    )
);
```

**Key message:** "Structure is identical! Just different names."

---

## Summary for Students

**What's the same:**
- ✅ Sequential and Parallel concepts
- ✅ Nesting groups
- ✅ Path following
- ✅ The structure of your autonomous

**What's different:**
- 📝 Mechanism → Subsystem
- 📝 Actions wrapper → InstantCommand
- 📝 SequentialAction → SequentialGroup
- 📝 ParallelAction → ParallelGroup

**What's better:**
- 😊 Less code to write
- 😊 Fewer files to manage
- 😊 No bugs with Pedro Pathing
- 😊 Same concepts you already know!

---

## Quick Reference

```java
// INITIALIZATION
// OLD
IntakeservoSpinnerActions intake = new IntakeservoSpinnerActions();
intake.init(hardwareMap);

// NEW
IntakeSubsystem intake = new IntakeSubsystem();
register(intake);  // automatic initialization

// AUTONOMOUS STRUCTURE
// OLD
Actions.runBlocking(
    new SequentialAction(
        new ParallelAction(followPath, intake.intakePosition()),
        new SleepAction(1.0),
        intake.stopPosition()
    )
);

// NEW
schedule(
    new SequentialGroup(
        new ParallelGroup(
            new FollowPathCommand(follower, path),
            new InstantCommand(() -> intake.intake())
        ),
        new WaitCommand(1.0),
        new InstantCommand(() -> intake.stop())
    )
);
```

---

## Bottom Line

**Your students already understand the hard parts!** They just need to learn:
1. New names for things they know
2. InstantCommand syntax
3. Delete the Actions wrapper files

This is more of a **refactor** than learning something new!

