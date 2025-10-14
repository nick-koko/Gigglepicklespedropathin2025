# NextFTC Quick Reference

## Quick Comparison

| Old Way (Mechanism Files) | New Way (NextFTC) |
|---------------------------|-------------------|
| `mechanism.init(hardwareMap)` | `subsystem.initialize(hardwareMap)` (automatic) |
| Manual if/else in loop | Button bindings with commands |
| RoadRunner Actions | NextFTC Command Groups |
| Manual state tracking | Built-in by NextFTC |

## Basic Patterns

### Creating a Subsystem
```java
public class MySubsystem extends Subsystem {
    private DcMotor motor;
    
    @Override
    public void initialize(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, "motor_name");
    }
    
    public void doSomething() { motor.setPower(1.0); }
    public void stop() { motor.setPower(0.0); }
}
```

### Creating a Command
```java
public class MyCommand extends Command {
    private final MySubsystem subsystem;
    
    public MyCommand(MySubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }
    
    @Override
    public void init() {
        subsystem.doSomething();
    }
    
    @Override
    public boolean isFinished() {
        return false; // or true for instant
    }
    
    @Override
    public void end(boolean interrupted) {
        subsystem.stop();
    }
}
```

### TeleOp Setup
```java
@TeleOp(name = "My TeleOp")
public class MyTeleOp extends NextFTCOpMode {
    @Override
    public void init() {
        MySubsystem subsystem = new MySubsystem();
        register(subsystem);
        
        // Bind buttons
        gamepad1().a().onTrue(new MyCommand(subsystem));
    }
}
```

### Autonomous Setup
```java
@Autonomous(name = "My Auto")
public class MyAuto extends PedroOpMode {
    @Override
    public void init() {
        MySubsystem subsystem = new MySubsystem();
        register(subsystem);
        
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
    }
    
    @Override
    public void start() {
        schedule(new MyCommand(subsystem));
    }
}
```

## Command Groups Cheat Sheet

### Sequential (one after another)
```java
new SequentialGroup(
    command1,  // Wait for this to finish
    command2,  // Then run this
    command3   // Then run this
)
```

### Parallel (all at same time)
```java
new ParallelGroup(
    command1,  // Run together
    command2,  // Run together
    command3   // Run together
)
```

### Mixed
```java
new SequentialGroup(
    // First do A and B together
    new ParallelGroup(
        commandA,
        commandB
    ),
    // Then do C
    commandC,
    // Then do D and E together
    new ParallelGroup(
        commandD,
        commandE
    )
)
```

## Common Patterns

### Follow Path While Intaking
```java
new ParallelGroup(
    new FollowPathCommand(follower, path),
    new IntakeCommand(intake)
)
```

### Spin Up Flywheel, Then Shoot
```java
new SequentialGroup(
    new SpinUpFlywheelCommand(flywheel, 3000),
    new WaitCommand(0.5),
    new ShootCommand(shooter)
)
```

### Drive, Grab, Return
```java
new SequentialGroup(
    // Drive to pickup while starting intake
    new ParallelGroup(
        new FollowPathCommand(follower, pathToPickup),
        new IntakeCommand(intake)
    ),
    // Wait to grab
    new WaitCommand(1.0),
    // Stop intake
    new StopIntakeCommand(intake),
    // Drive back
    new FollowPathCommand(follower, pathBack)
)
```

## Button Bindings

```java
// Run command when button pressed
gamepad1().a().onTrue(new MyCommand(subsystem));

// Run command while button held
gamepad1().a().whileTrue(new MyCommand(subsystem));

// Run command when button released
gamepad1().a().onFalse(new MyCommand(subsystem));

// Toggle command on/off
gamepad1().a().toggleOnTrue(new MyCommand(subsystem));
```

## Pedro Pathing Integration

### Build a Path
```java
PathChain path = follower.pathBuilder()
    .addPath(new BezierLine(start, end))
    .setLinearHeadingInterpolation(startHeading, endHeading)
    .build();
```

### Follow the Path
```java
schedule(new FollowPathCommand(follower, path));
```

### Follow While Doing Something
```java
schedule(
    new ParallelGroup(
        new FollowPathCommand(follower, path),
        new MyCommand(subsystem)
    )
);
```

## Common Commands

### Wait
```java
new WaitCommand(1.5)  // Wait 1.5 seconds
```

### Instant Command (Lambda)
```java
new InstantCommand(() -> subsystem.doSomething())
```

### Run Until Condition
```java
new Command() {
    @Override
    public boolean isFinished() {
        return sensor.isTriggered();
    }
}
```

## Subsystem Features

### IntakeSubsystem
```java
intake.intake();           // Run intake
intake.outtake();          // Run outtake
intake.stop();             // Stop
intake.setPower(0.5);      // Custom power
double p = intake.getPower(); // Get current power
```

### FlywheelSubsystem
```java
flywheel.setRPM(3000);                    // Set specific RPM
flywheel.setHighSpeed();                  // Use preset high
flywheel.setLowSpeed();                   // Use preset low
flywheel.stop();                          // Stop
double rpm = flywheel.getCurrentRPM();    // Get current RPM
double target = flywheel.getTargetRPM();  // Get target RPM
boolean ready = flywheel.isAtTargetSpeed(50); // Check if ready
```

## Configuration

```java
// In your init() or before creating subsystems:
IntakeSubsystem.INTAKE_POWER = 0.8;
IntakeSubsystem.OUTTAKE_POWER = -0.8;

FlywheelSubsystem.HIGH_TARGET_RPM = 3500.0;
FlywheelSubsystem.LOW_TARGET_RPM = 2500.0;
FlywheelSubsystem.GEAR_RATIO = 2.0;
```

## Debugging Tips

### Add Telemetry in Subsystem
```java
@Override
public void periodic() {
    // This runs every loop automatically
    telemetry.addData("Motor Power", motor.getPower());
}
```

### Add Telemetry in OpMode
```java
@Override
public void run() {
    telemetry.addData("Subsystem State", subsystem.getSomething());
    telemetry.update();
}
```

### Check Command Status
```java
Command cmd = new MyCommand(subsystem);
schedule(cmd);

// Later
if (cmd.isScheduled()) {
    // Still running
}
```

## Migration Checklist

- [ ] Convert mechanism class to extend `Subsystem`
- [ ] Change `init(HardwareMap)` to `initialize(HardwareMap)`
- [ ] Make hardware fields private
- [ ] Add public control methods
- [ ] Create command classes for each action
- [ ] Update TeleOp to extend `NextFTCOpMode`
- [ ] Replace loop() button checks with bindings
- [ ] Update Autonomous to extend `PedroOpMode`
- [ ] Replace Actions with Command Groups
- [ ] Test each subsystem individually
- [ ] Test commands individually
- [ ] Test command groups
- [ ] Add telemetry for debugging

## Common Mistakes

❌ **Forgetting to register subsystem**
```java
MySubsystem subsystem = new MySubsystem();
// Missing: register(subsystem);
```

❌ **Not adding requirements**
```java
public MyCommand(MySubsystem subsystem) {
    this.subsystem = subsystem;
    // Missing: addRequirements(subsystem);
}
```

❌ **Using OpMode instead of NextFTCOpMode**
```java
public class MyTeleOp extends OpMode {  // Wrong!
```
Should be:
```java
public class MyTeleOp extends NextFTCOpMode {  // Correct!
```

❌ **Using OpMode instead of PedroOpMode for autonomous**
```java
public class MyAuto extends NextFTCOpMode {  // Wrong for Pedro!
```
Should be:
```java
public class MyAuto extends PedroOpMode {  // Correct!
```

❌ **Manually calling follower.update()**
```java
// Don't do this in PedroOpMode - it's automatic!
follower.update();
```

## Need More Help?

See `NextFTC_Migration_Guide.md` for detailed explanations.

