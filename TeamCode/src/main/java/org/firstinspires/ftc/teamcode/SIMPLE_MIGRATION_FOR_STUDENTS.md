# Simple NextFTC Migration for Students

## What We're Changing and Why

### The Problem
- ❌ **RoadRunner Actions** had issues when using Pedro Pathing
- ❌ Hard to run mechanisms while following paths in autonomous

### The Solution
- ✅ **NextFTC** works perfectly with Pedro Pathing
- ✅ Easy to run mechanisms while driving (parallel commands)
- ✅ Almost everything else stays the same!

---

## What DOESN'T Change

### ✅ TeleOp Stays Almost the Same!

**Your old TeleOp:**
```java
@Override
public void loop() {
    // Driving - STAYS THE SAME
    double driving = -gamepad1.right_stick_y;
    double strafe = -gamepad1.right_stick_x;
    double rotate = -gamepad1.left_stick_x;
    
    // Mechanisms - STAYS THE SAME
    if (gamepad1.left_bumper) {
        intake.Intake();
    } else if (gamepad1.right_bumper) {
        intake.Outtake();
    } else {
        intake.Stop();
    }
}
```

**New TeleOp - Almost identical:**
```java
@Override
public void loop() {
    // Driving - EXACTLY THE SAME
    double driving = -gamepad1.right_stick_y;
    double strafe = -gamepad1.right_stick_x;
    double rotate = -gamepad1.left_stick_x;
    
    // Mechanisms - EXACTLY THE SAME
    if (gamepad1.left_bumper) {
        intake.intake();  // Just lowercase method name
    } else if (gamepad1.right_bumper) {
        intake.outtake();
    } else {
        intake.stop();
    }
}
```

**Only tiny difference:** Method names are lowercase (Java convention)

---

## What DOES Change

### 1. Mechanism Files (Tiny Change)

**Old way (IntakeMotorSpinner.java):**
```java
public class IntakeMotorSpinner {
    DcMotor motor;
    
    public void init(HardwareMap hwMap) {
        motor = hwMap.get(DcMotor.class, "intake_motor");
    }
    
    public void Intake() { motor.setPower(0.7); }
    public void Stop() { motor.setPower(0.0); }
}
```

**New way (IntakeSubsystem.java):**
```java
public class IntakeSubsystem extends Subsystem {
    private DcMotor motor;
    
    @Override
    public void initialize(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, "intake_motor");
    }
    
    public void intake() { motor.setPower(0.7); }
    public void stop() { motor.setPower(0.0); }
}
```

**What changed:**
1. Add `extends Subsystem`
2. Change `init()` to `initialize()`
3. That's it! Everything else is the same!

### 2. Autonomous (This is Where NextFTC Helps!)

**Old way (with RoadRunner Actions - had problems):**
```java
Actions.runBlocking(
    new ParallelAction(
        followPath(path),
        intake.intakeAction()
    )
);
```

**New way (with NextFTC - works great!):**
```java
schedule(
    new ParallelGroup(
        new FollowPathCommand(follower, path),
        new InstantCommand(() -> intake.intake())
    )
);
```

**Why this is better:**
- ✅ Works with Pedro Pathing (no issues!)
- ✅ Easy to understand (follow path AND run intake)
- ✅ Can stop/start things easily

---

## Simple Step-by-Step Migration

### Step 1: Convert One Mechanism File

**Change this:**
```java
public class IntakeMotorSpinner {
    DcMotor motor;
    
    public void init(HardwareMap hwMap) {
        motor = hwMap.get(DcMotor.class, "intake_motor");
    }
}
```

**To this:**
```java
public class IntakeSubsystem extends Subsystem {
    private DcMotor motor;
    
    @Override
    public void initialize(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, "intake_motor");
    }
}
```

**Students need to know:**
- "Subsystem" is NextFTC's word for a robot mechanism
- `extends Subsystem` tells NextFTC this controls part of the robot
- Everything else works the same!

### Step 2: Update TeleOp (Tiny Changes)

**Old TeleOp:**
```java
public class MyTeleOp extends OpMode {
    IntakeMotorSpinner intake = new IntakeMotorSpinner();
    
    @Override
    public void init() {
        intake.init(hardwareMap);
    }
    
    @Override
    public void loop() {
        if (gamepad1.left_bumper) {
            intake.Intake();
        }
    }
}
```

**New TeleOp (Option 1 - Easiest, most familiar):**
```java
public class MyTeleOp extends NextFTCOpMode {
    IntakeSubsystem intake = new IntakeSubsystem();
    
    @Override
    public void init() {
        register(intake);  // Just add this line
    }
    
    @Override
    public void run() {  // Changed from loop()
        if (gamepad1.left_bumper) {
            intake.intake();  // Same as before!
        }
    }
}
```

**What changed:**
1. `extends NextFTCOpMode` instead of `OpMode`
2. Add `register(intake)` in init
3. `loop()` becomes `run()`
4. That's it!

### Step 3: Update Autonomous (This is the Main Benefit!)

**New Autonomous with NextFTC:**
```java
@Autonomous(name = "My Auto")
public class MyAuto extends PedroOpMode {
    
    private IntakeSubsystem intake;
    
    @Override
    public void init() {
        intake = new IntakeSubsystem();
        register(intake);
        
        // Set up Pedro Pathing (same as before!)
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }
    
    @Override
    public void start() {
        // This is the NEW part - and it's EASY!
        
        // Drive to pickup WHILE running intake
        schedule(
            new ParallelGroup(
                new FollowPathCommand(follower, pathToPickup),
                new InstantCommand(() -> intake.intake())
            )
        );
    }
    
    @Override
    public void run() {
        // Telemetry
        telemetry.addData("X", follower.getPose().getX());
        telemetry.update();
    }
}
```

**Students need to understand:**
- `ParallelGroup` = do things at the same time
- `FollowPathCommand` = follow a Pedro path
- `InstantCommand(() -> intake.intake())` = turn on intake
- Put them together = drive while intaking!

---

## Teaching Concepts to Students

### Concept 1: Parallel vs Sequential

**Sequential (one after another):**
```java
new SequentialGroup(
    new InstantCommand(() -> claw.close()),     // 1. Close claw
    new FollowPathCommand(follower, path),      // 2. Then drive
    new InstantCommand(() -> claw.open())       // 3. Then open claw
)
```

**Parallel (at the same time):**
```java
new ParallelGroup(
    new FollowPathCommand(follower, path),      // Drive
    new InstantCommand(() -> intake.intake())   // AND intake
)
```

**Analogy for students:**
- Sequential = "Do your homework, THEN play video games"
- Parallel = "Listen to music WHILE doing homework"

### Concept 2: InstantCommand

```java
new InstantCommand(() -> intake.intake())
```

**Tell students:**
- "This is like pressing a button"
- "It happens instantly and we move on"
- "The `() ->` is like saying 'when this runs, do this'"

### Concept 3: Subsystems

**Tell students:**
- "A subsystem is just our old mechanism file with a fancy name"
- "We add `extends Subsystem` so NextFTC knows about it"
- "It controls one part of the robot (intake, claw, etc.)"

---

## Minimal Example Files for Students

### Simple TeleOp (Keep It Familiar!)

```java
@TeleOp(name = "Our TeleOp")
public class OurTeleOp extends NextFTCOpMode {
    
    private IntakeSubsystem intake;
    
    @Override
    public void init() {
        intake = new IntakeSubsystem();
        register(intake);
    }
    
    @Override
    public void run() {
        // Drive - SAME AS ALWAYS
        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;
        // ... use for driving ...
        
        // Intake - SAME AS ALWAYS
        if (gamepad1.left_bumper) {
            intake.intake();
        } else if (gamepad1.right_bumper) {
            intake.outtake();
        } else {
            intake.stop();
        }
    }
}
```

### Simple Autonomous (The New Part!)

```java
@Autonomous(name = "Our Auto")
public class OurAuto extends PedroOpMode {
    
    private IntakeSubsystem intake;
    
    @Override
    public void init() {
        intake = new IntakeSubsystem();
        register(intake);
        
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }
    
    @Override
    public void start() {
        // Score preload
        schedule(
            new SequentialGroup(
                // 1. Drive to score
                new FollowPathCommand(follower, pathToScore),
                
                // 2. Wait a second
                new WaitCommand(1.0),
                
                // 3. Drive to pickup WHILE intaking
                new ParallelGroup(
                    new FollowPathCommand(follower, pathToPickup),
                    new InstantCommand(() -> intake.intake())
                ),
                
                // 4. Wait a second
                new WaitCommand(1.0),
                
                // 5. Stop intake
                new InstantCommand(() -> intake.stop())
            )
        );
    }
}
```

---

## What to Tell Your Students

### "Why are we changing?"

**Old way (RoadRunner Actions):**
- ❌ Had bugs with Pedro Pathing
- ❌ Hard to fix those bugs
- ❌ Made autonomous programming frustrating

**New way (NextFTC):**
- ✅ Works great with Pedro Pathing
- ✅ Makes it easy to do multiple things at once
- ✅ Used by lots of FTC teams

### "What's actually different?"

**TeleOp:**
- Almost nothing! Just a few name changes
- Everything works the same way

**Autonomous:**
- Much easier to run multiple things at once
- Can drive while intaking (used to be hard!)
- Clear to read: "do this, then do that, then do both together"

### "Do we have to learn a lot of new stuff?"

**No!** Here's what's new:
1. `extends Subsystem` (just means "this is a mechanism")
2. `register(subsystem)` (tell NextFTC about it)
3. `ParallelGroup` (do things together)
4. `SequentialGroup` (do things one at a time)
5. `InstantCommand(() -> subsystem.method())` (run one action)

That's it! Everything else you already know!

---

## Migration Checklist for Students

### TeleOp Migration
- [ ] Change mechanism class to `extends Subsystem`
- [ ] Change `init()` to `initialize()` in mechanism
- [ ] Change OpMode to `extends NextFTCOpMode`
- [ ] Add `register(subsystem)` in init
- [ ] Change `loop()` to `run()`
- [ ] Test - should work the same!

### Autonomous Migration
- [ ] Change OpMode to `extends PedroOpMode`
- [ ] Add `register(subsystem)` in init
- [ ] Use `FollowPathCommand(follower, path)` for paths
- [ ] Use `InstantCommand(() -> subsystem.method())` for mechanisms
- [ ] Use `ParallelGroup` to do things together
- [ ] Use `SequentialGroup` to do things in order
- [ ] Test each part separately first!

---

## Common Student Questions

**Q: "Why do we need `() ->`?"**
A: It's Java's way of saying "here's some code to run later." Think of it like writing instructions on a sticky note.

**Q: "What's the difference between ParallelGroup and SequentialGroup?"**
A: 
- Sequential = Do homework, THEN play games
- Parallel = Listen to music WHILE doing homework

**Q: "Do we have to change our TeleOp?"**
A: Not really! Just tiny name changes. It works almost exactly the same.

**Q: "Why is this better than RoadRunner Actions?"**
A: It works with Pedro Pathing! The old way had bugs that made autonomous frustrating.

**Q: "Can we still use our gamepad sticks the same way?"**
A: Yes! `gamepad1.left_stick_y` still works exactly the same.

**Q: "Is this harder?"**
A: Autonomous is actually EASIER because you can say "drive AND intake" instead of complex code!

---

## Summary for Teachers

**Keep it simple:**
1. TeleOp barely changes (same patterns, new names)
2. Mechanisms become "Subsystems" (almost identical)
3. Autonomous gets command groups (easier to understand!)
4. Focus on the concepts: parallel vs sequential

**Don't overcomplicate:**
- ❌ Don't mention separate command files
- ❌ Don't show button bindings (keep if statements)
- ❌ Don't explain advanced features
- ✅ Just show: subsystems, parallel, sequential, instant command

**Best teaching order:**
1. Convert one mechanism to subsystem together
2. Update TeleOp together (show it's almost the same)
3. Test TeleOp (nothing should break!)
4. Explain parallel vs sequential with real-world examples
5. Build one autonomous routine together
6. Let them add to it

**This is enough!** Students can be successful with just these basics.

