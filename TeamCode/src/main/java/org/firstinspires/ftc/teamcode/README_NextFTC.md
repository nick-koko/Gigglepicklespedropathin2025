# NextFTC Command-Based Architecture

## What's Been Created

This repository now includes a complete NextFTC command-based framework for your robot mechanisms, integrated with Pedro Pathing for autonomous control.

### 📁 File Structure

```
teamcode/
├── subsystems/              # Robot mechanism control
│   ├── IntakeSubsystem.java       ✅ Converted from IntakeMotorSpinner
│   └── FlywheelSubsystem.java     ✅ NEW: Flywheel with velocity control
│
├── commands/                # Actions for subsystems
│   ├── IntakeCommand.java         ✅ Run intake continuously
│   ├── OuttakeCommand.java        ✅ Run outtake continuously
│   ├── StopIntakeCommand.java     ✅ Stop intake instantly
│   ├── SpinFlywheelCommand.java   ✅ Spin flywheel at RPM
│   ├── SpinUpFlywheelCommand.java ✅ Wait for flywheel to reach speed
│   └── StopFlywheelCommand.java   ✅ Stop flywheel instantly
│
├── examples/                # Working examples
│   ├── ExampleNextFTCTeleOp.java        ✅ TeleOp with button bindings
│   └── ExampleNextFTCAutonomous.java    ✅ Autonomous with Pedro Pathing
│
├── templates/               # Copy-paste templates for new mechanisms
│   ├── SubsystemTemplate.java     ✅ Template for creating subsystems
│   └── CommandTemplate.java       ✅ Template for creating commands
│
└── Documentation/
    ├── NextFTC_Migration_Guide.md      ✅ Comprehensive guide
    ├── NextFTC_Quick_Reference.md      ✅ Quick lookup reference
    └── README_NextFTC.md (this file)   ✅ Getting started
```

## 🚀 Quick Start

### 1. Test the Examples

The easiest way to get started is to test the example OpModes:

**TeleOp Testing (Start with Simplified!):**
```
1. Deploy to Robot Controller
2. Select "Simplified TeleOp (No Command Files)" from TeleOp
   ⭐ This one uses inline commands - no separate command files needed!
3. Press Play
4. Test controls:
   - Left Bumper: Run intake
   - Right Bumper: Run outtake
   - A: Stop intake
   - X: Spin flywheel high speed
   - Y: Spin flywheel low speed
   - B: Stop flywheel
```

**Autonomous Testing:**
```
1. Deploy to Robot Controller
2. Select "NextFTC Pedro Auto Example" from Autonomous
3. Press Play
4. Watch the robot:
   - Drive to score while spinning flywheel
   - Wait at scoring position
   - Drive to pickup while running intake
```

### 2. Understand the Architecture

**Old Way (Mechanism Files):**
```java
// Manual control in loop
if (gamepad1.left_bumper) {
    intake.Intake();
} else {
    intake.Stop();
}
```

**New Way (NextFTC - Simplified with Inline Commands):**
```java
// Bind button to command once in init() - NO separate command files!
gamepad1().leftBumper().onTrue(new InstantCommand(() -> intake.intake()));
// NextFTC handles everything else automatically!
```

**Alternative (Separate Command Files):**
```java
// Can also use separate command files if you prefer
gamepad1().leftBumper().onTrue(new IntakeCommand(intake));
// See Command_Approaches_Comparison.md for when to use each!
```

**Key Benefits:**
- ✅ Parallel/Sequential commands for autonomous
- ✅ Automatic conflict resolution
- ✅ Easy to interrupt/cancel
- ✅ Composable for complex routines
- ✅ Works seamlessly with Pedro Pathing

### 3. Create Your First Custom Mechanism

**Step 1: Copy Template**
```
Copy: templates/SubsystemTemplate.java
To: subsystems/ClawSubsystem.java
```

**Step 2: Customize for Your Hardware**
```java
public class ClawSubsystem extends Subsystem {
    private Servo clawServo;
    
    public static double OPEN_POSITION = 0.8;
    public static double CLOSED_POSITION = 0.2;
    
    @Override
    public void initialize(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(Servo.class, "claw");
    }
    
    public void open() { clawServo.setPosition(OPEN_POSITION); }
    public void close() { clawServo.setPosition(CLOSED_POSITION); }
}
```

**Step 3: Create Commands**
```
Copy: templates/CommandTemplate.java
To: commands/OpenClawCommand.java
     commands/CloseClawCommand.java
```

**Step 4: Use in OpMode**
```java
@TeleOp(name = "My TeleOp")
public class MyTeleOp extends NextFTCOpMode {
    @Override
    public void init() {
        ClawSubsystem claw = new ClawSubsystem();
        register(claw);
        
        gamepad1().leftBumper().onTrue(new OpenClawCommand(claw));
        gamepad1().rightBumper().onTrue(new CloseClawCommand(claw));
    }
}
```

## 📚 Documentation Guide

### Start Here
1. **README_NextFTC.md** (this file) - Overview and quick start
2. **Command_Approaches_Comparison.md** ⭐ **READ THIS!** - Shows simple vs complex approaches
3. **examples/SimplifiedTeleOp.java** - Simplest approach (no command files!)
4. **examples/ExampleNextFTCAutonomous.java** - Working autonomous code

### Learning Resources
5. **NextFTC_Quick_Reference.md** - Quick lookup for common patterns
6. **NextFTC_Migration_Guide.md** - Detailed explanations and best practices

### Creating New Mechanisms
6. **templates/SubsystemTemplate.java** - Copy this to create subsystems
7. **templates/CommandTemplate.java** - Copy this to create commands

## 🎯 Common Use Cases

### TeleOp Button Binding

**Simple approach (inline - no command files!):**
```java
// Press to run action
gamepad1().a().onTrue(new InstantCommand(() -> subsystem.doSomething()));

// Hold to run continuously
gamepad1().a().whileTrue(new InstantCommand(() -> subsystem.start()));

// Release to stop
gamepad1().a().onFalse(new InstantCommand(() -> subsystem.stop()));
```

**Alternative (separate command files for complex logic):**
```java
// Use when command needs complex logic or state tracking
gamepad1().a().onTrue(new MyComplexCommand(subsystem));
```

### Sequential Autonomous Actions
```java
new SequentialGroup(
    new DriveToPosition(drive, position1),
    new WaitCommand(0.5),
    new GrabPiece(claw),
    new DriveToPosition(drive, position2),
    new ReleasePiece(claw)
)
```

### Parallel Actions
```java
new ParallelGroup(
    new FollowPathCommand(follower, path),
    new IntakeCommand(intake),
    new SpinUpFlywheelCommand(flywheel, 3000)
)
```

### Complex Autonomous Routine
```java
new SequentialGroup(
    // Score preload while spinning flywheel
    new ParallelGroup(
        new FollowPathCommand(follower, pathToScore),
        new SpinUpFlywheelCommand(flywheel, 3000)
    ),
    new WaitCommand(1.0),  // Shoot
    new StopFlywheelCommand(flywheel),
    
    // Pick up game pieces
    new ParallelGroup(
        new FollowPathCommand(follower, pathToPickup1),
        new IntakeCommand(intake)
    ),
    new WaitCommand(0.5),
    new StopIntakeCommand(intake),
    
    // Score again
    new ParallelGroup(
        new FollowPathCommand(follower, pathToScore),
        new SpinUpFlywheelCommand(flywheel, 3000)
    ),
    new WaitCommand(1.0),  // Shoot
    new StopFlywheelCommand(flywheel)
)
```

## 🔧 Configuration

### Hardware Names
Make sure your Robot Configuration matches the hardware names in the code:

**IntakeSubsystem:**
- Motor: `"intake_motor"`

**FlywheelSubsystem:**
- Motor: `"flywheel_motor"`

### Tuning Constants

**Intake Power:**
```java
IntakeSubsystem.INTAKE_POWER = 0.8;   // Default: 0.7
IntakeSubsystem.OUTTAKE_POWER = -0.8; // Default: -0.7
```

**Flywheel RPM:**
```java
FlywheelSubsystem.HIGH_TARGET_RPM = 3500.0;  // Default: 3000.0
FlywheelSubsystem.LOW_TARGET_RPM = 2500.0;   // Default: 2000.0
FlywheelSubsystem.GEAR_RATIO = 2.0;          // Default: 1.0
```

## 🐛 Troubleshooting

### "Command not running"
✅ Make sure you registered the subsystem: `register(subsystem)`

### "Commands keep canceling each other"
✅ This is normal! Only one command can use a subsystem at a time. This prevents conflicting commands.

### "Flywheel not reaching target speed"
✅ Check `FlywheelSubsystem.GEAR_RATIO` is correct for your gearing
✅ Increase timeout in `SpinUpFlywheelCommand`

### "Robot not following path"
✅ Use `PedroOpMode` as base class for autonomous (not `NextFTCOpMode`)
✅ Make sure `follower.setStartingPose()` is called
✅ Verify paths are built before `start()`

### "Hardware not found" error
✅ Check hardware names match your Robot Configuration
✅ Verify motors/servos are plugged in correctly

## 📖 Learning Path

### Beginner
1. ✅ Run the example TeleOp
2. ✅ Run the example Autonomous
3. ✅ Modify button bindings in TeleOp
4. ✅ Change speeds/powers in subsystems

### Intermediate
1. ✅ Copy template and create a simple subsystem (servo claw)
2. ✅ Create instant commands (open/close)
3. ✅ Use your new subsystem in TeleOp
4. ✅ Create a simple sequential autonomous

### Advanced
1. ✅ Create complex subsystem with sensors
2. ✅ Create conditional commands (wait for sensor)
3. ✅ Build parallel command groups
4. ✅ Integrate multiple mechanisms in autonomous
5. ✅ Add telemetry and debugging

## 🔗 Resources

- **NextFTC Documentation:** https://nextftc.dev
- **NextFTC + Pedro Pathing:** https://nextftc.dev/extensions/pedro/
- **Pedro Pathing:** https://pedropathing.com
- **NextFTC GitHub:** https://github.com/NextFTC
- **NextFTC LLM Helper:** https://nextftc.dev/llms.txt

## 💡 Tips

1. **Start Simple** - Test each subsystem individually before combining
2. **Use Templates** - Copy the templates when creating new mechanisms
3. **Add Telemetry** - Use `periodic()` in subsystems for debugging
4. **Test in Stages** - Test TeleOp first, then add to autonomous
5. **Read Examples** - The example OpModes are fully functional and well-commented
6. **Check Quick Reference** - Use `NextFTC_Quick_Reference.md` for quick lookups

## 🎉 What's Next?

Now that you have the framework set up:

1. **Test the Examples** - Make sure everything works
2. **Convert Your Mechanisms** - Use templates to convert your existing mechanism files
3. **Build Your TeleOp** - Create button bindings for all your controls
4. **Build Your Autonomous** - Use command groups with Pedro Pathing
5. **Iterate and Improve** - Add features and refine as you go

## ❓ Questions?

If you get stuck:
1. Check `NextFTC_Quick_Reference.md` for code snippets
2. Read `NextFTC_Migration_Guide.md` for detailed explanations
3. Look at the example OpModes for working code
4. Check NextFTC documentation at https://nextftc.dev

---

**Remember:** The command-based architecture might feel different at first, but it will make your autonomous routines much easier to build and maintain. Take it one step at a time!

Good luck with your robot! 🤖

