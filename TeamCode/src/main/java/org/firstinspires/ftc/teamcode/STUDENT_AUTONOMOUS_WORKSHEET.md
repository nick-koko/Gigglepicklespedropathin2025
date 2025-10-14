# Autonomous Conversion Worksheet

**Team Members:** _______________________________  
**Date:** _______________________________

---

## Your Mission

Convert `ExampleAuto.java` to use NextFTC's Sequential and Parallel commands!

Right now, the autonomous just drives one big path. You need to:
1. Break it into smaller path sections
2. Add parallel/sequential groups
3. Add comments where mechanisms would run
4. Add wait times for shooting/intaking

---

## Step 1: Understand the Current Code

**Current code:**
```java
@Override
public void start() {
    follower.followPath(curvedLineBlue);  // One giant path!
}
```

**Problem:** Can't do anything WHILE driving or BETWEEN sections!

---

## Step 2: Your Available Paths

Look at `buildPaths()` in ExampleAuto.java. You have these paths:

| Path Name | What it does |
|-----------|--------------|
| `scorePreload` | Drive from start to scoring position |
| `grabPickup1` | Drive from score to pickup #1 |
| `scorePickup1` | Drive from pickup #1 back to score |
| `grabPickup2` | Drive from score to pickup #2 |
| `scorePickup2` | Drive from pickup #2 back to score |
| `grabPickup3` | Drive from score to pickup #3 |
| `scorePickup3` | Drive from pickup #3 back to score |
| `curvedLineBlue` | Giant path with all sections |

---

## Step 3: Plan Your Autonomous Routine

**Fill this out as a team:**

### What should the robot do?

1. **At start:**
   - Action: _________________________________________________

2. **First scoring:**
   - Drive to: _________________________________________________
   - Do what: _________________________________________________
   - How long: _________________________________________________

3. **First pickup:**
   - Drive to: _________________________________________________
   - Do what: _________________________________________________
   - How long: _________________________________________________

4. **Second scoring:**
   - Drive to: _________________________________________________
   - Do what: _________________________________________________
   - How long: _________________________________________________

5. **Continue the pattern...**

---

## Step 4: NextFTC Command Structure

Remember from last year's RoadRunner Actions? Same idea, new names!

### Sequential (Do things in order)
```java
new SequentialGroup(
    thing1,  // Do this first
    thing2,  // Then do this
    thing3   // Then do this
)
```

### Parallel (Do things together)
```java
new ParallelGroup(
    thing1,  // Do both
    thing2   // at the same time!
)
```

### Follow a path
```java
new FollowPath(pathName)  // ✅ Just the path name!
```

### Wait for time
```java
new Delay(1.5)  // Wait 1.5 seconds
```

### Run a mechanism (when we have them!)
```java
// For now, just add a comment like this:
// TODO: Start intake
// TODO: Spin up shooter to 3000 RPM
// TODO: Stop intake

// Later, these become:
new InstantCommand(() -> intake.intake())
new InstantCommand(() -> shooter.spinUp())
```

---

## Step 5: File Setup - IMPORTANT!

### At the top of your file, you need:

```java
package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.*;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// ✅ CORRECT NextFTC imports
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "Your Auto Name", group = "Competition")
public class YourAuto extends NextFTCOpMode {
    
    // ✅ Add this constructor!
    public YourAuto() {
        addComponents(
            new PedroComponent(Constants::createFollower)
        );
    }
    
    // Your code here...
}
```

---

## Step 6: Building Your Autonomous - The Pattern

### Use a Command Method!

Instead of putting everything in `onStartButtonPressed()`, create a method that returns a `Command`:

```java
// ✅ Create a method that returns your autonomous routine
private Command autoRoutine() {
    return new SequentialGroup(
        // Your autonomous steps here!
    );
}

// ✅ Then schedule it when START is pressed
@Override
public void onStartButtonPressed() {
    autoRoutine().schedule();
}
```

**Why?** This makes your code reusable and easier to read!

---

## Step 7: Example Pattern

Here's an example of scoring preload while spinning up shooter:

```java
private Command scorePreload() {
    return new SequentialGroup(
        // Score preload
        new ParallelGroup(
            new FollowPath(scorePreload),
            // TODO: Spin up shooter to 3000 RPM while driving
        ),
        
        new Delay(1.0),  // Wait for shooter to be ready
        // TODO: Shoot the preload
        
        new Delay(0.5),  // Wait for shot to complete
        // TODO: Stop shooter
    );
}

@Override
public void onStartButtonPressed() {
    scorePreload().schedule();
}
```

---

## Step 8: Your Turn! Plan First Score + First Pickup

**Write your code here (on paper first!):**

```java
private Command autoRoutine() {
    return new SequentialGroup(
        
        // ============================================
        // SECTION 1: Score Preload
        // ============================================
        new ParallelGroup(
            new FollowPath(scorePreload),
            // TODO: What should happen while driving?
        ),
        
        new Delay(____),  // How long to wait?
        // TODO: What action happens here?
        
        // ============================================
        // SECTION 2: Go to First Pickup
        // ============================================
        new ParallelGroup(
            new FollowPath(grabPickup1),
            // TODO: What should happen while driving?
        ),
        
        new Delay(____),  // How long to wait?
        // TODO: What action happens here?
        
        // ============================================
        // SECTION 3: Score First Pickup
        // ============================================
        // Fill this in yourself!
        
        
    );
}

@Override
public void onStartButtonPressed() {
    autoRoutine().schedule();
}
```

---

## Step 9: Checklist

Before you ask a mentor to review:

- [ ] Changed `extends OpMode` to `extends NextFTCOpMode`
- [ ] Added constructor with `new PedroComponent(Constants::createFollower)`
- [ ] Created a `private Command autoRoutine()` method
- [ ] All paths wrapped in `new FollowPath(pathName)` (NOT FollowPathCommand!)
- [ ] Used `new SequentialGroup()` for things in order
- [ ] Used `new ParallelGroup()` for things together
- [ ] Added `new Delay()` where you need waits (NOT WaitCommand!)
- [ ] Added `// TODO:` comments for all mechanism actions
- [ ] Added `@Override public void onStartButtonPressed()` that calls `.schedule()`
- [ ] Tested that it compiles (no red squiggles!)
- [ ] Run it on robot to test paths (mechanisms won't work yet - that's OK!)

---

## Step 10: Common Patterns to Use

### Pattern 1: Drive while starting something
```java
new ParallelGroup(
    new FollowPath(somePath),
    // TODO: Start intake
)
```

### Pattern 2: Wait for something to finish
```java
new SequentialGroup(
    // TODO: Spin up shooter to 3000 RPM,
    new Delay(1.5),  // Wait for it to reach speed
    // TODO: Shoot
)
```

### Pattern 3: Do multiple things at the same time
```java
new ParallelGroup(
    // TODO: Lower arm,
    // TODO: Open claw,
    // TODO: Extend slide
)
```

### Pattern 4: Complex nested pattern
```java
new SequentialGroup(
    // First do A and B together
    new ParallelGroup(
        thingA,
        thingB
    ),
    // Then do C
    thingC,
    // Then do D and E together
    new ParallelGroup(
        thingD,
        thingE
    )
)
```

---

## Step 11: Think About Timing

**Important questions to discuss:**

1. How long does it take to spin up the shooter?
   - Answer: _____ seconds

2. How long does it take to intake a game piece?
   - Answer: _____ seconds

3. How long does it take to shoot?
   - Answer: _____ seconds

4. Should we start spinning up the shooter early (while driving)?
   - Answer: _____________________

5. Should we keep the intake running while driving back?
   - Answer: _____________________

---

## Step 12: Test Strategy

**When you're ready to test:**

1. **First:** Test just driving the paths (no mechanisms)
   - Does the robot follow the right path?
   - Does it stop at the right places?

2. **Then:** When mechanisms are ready, replace `// TODO:` with real commands
   - Test one mechanism at a time
   - Make sure timing is right

---

## Example: Complete Two-Cycle Autonomous

Here's what a complete autonomous might look like:

```java
package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.*;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "Two Cycle Auto", group = "Competition")
public class TwoCycleAuto extends NextFTCOpMode {
    
    // ✅ Constructor - Always needed!
    public TwoCycleAuto() {
        addComponents(
            new PedroComponent(Constants::createFollower)
        );
    }
    
    // Your path chains (same as before!)
    private PathChain scorePreload, grabPickup1, scorePickup1;
    
    @Override
    public void onInit() {
        // Set starting position
        PedroComponent.follower().setStartingPose(new Pose(0, 0, 0));
        
        // Build paths (same as you did before!)
        buildPaths();
        
        telemetry.addLine("Two Cycle Auto Ready!");
        telemetry.update();
    }
    
    private void buildPaths() {
        // Your path building code here (same as before!)
        // scorePreload = PedroComponent.follower().pathBuilder()...
        // grabPickup1 = PedroComponent.follower().pathBuilder()...
        // scorePickup1 = PedroComponent.follower().pathBuilder()...
    }
    
    // ✅ Create a Command method for your routine
    private Command autoRoutine() {
        return new SequentialGroup(
            // ============================================
            // CYCLE 1: Score Preload
            // ============================================
            new ParallelGroup(
                new FollowPath(scorePreload),
                // TODO: Spin up shooter to 3000 RPM
            ),
            new Delay(1.0),
            // TODO: Shoot preload
            new Delay(0.3),
            // TODO: Stop shooter
            
            // ============================================
            // CYCLE 1: Get First Pickup
            // ============================================
            new ParallelGroup(
                new FollowPath(grabPickup1),
                // TODO: Deploy intake
                // TODO: Start intake motor
            ),
            new Delay(0.5),  // Wait to grab piece
            // TODO: Retract intake
            
            // ============================================
            // CYCLE 1: Score First Pickup
            // ============================================
            new ParallelGroup(
                new FollowPath(scorePickup1),
                // TODO: Spin up shooter to 3000 RPM
                // TODO: Transfer piece from intake to shooter
            ),
            new Delay(1.0),
            // TODO: Shoot
            new Delay(0.3),
            // TODO: Stop shooter
            
            // ============================================
            // CYCLE 2: You can add more here!
            // ============================================
        );
    }
    
    // ✅ Schedule the routine when START is pressed
    @Override
    public void onStartButtonPressed() {
        autoRoutine().schedule();
    }
    
    // ✅ Add telemetry during the run
    @Override
    public void onUpdate() {
        telemetry.addData("X", PedroComponent.follower().getPose().getX());
        telemetry.addData("Y", PedroComponent.follower().getPose().getY());
        telemetry.update();
    }
}
```

---

## Helpful Tips

### Tip 1: Start Simple
Don't try to do everything at once! Start with:
1. Just score preload
2. Test it
3. Add first pickup
4. Test it
5. Keep adding...

### Tip 2: Use Comments
When you're not sure about timing, add comments:
```java
new Delay(1.0),  // TODO: Tune this - might need longer
```

### Tip 3: Think About What Happens While Driving
Most actions can happen WHILE the robot drives:
- ✅ Spin up shooter while driving
- ✅ Deploy intake while driving
- ✅ Raise arm while driving
- ❌ Shooting (robot must be stopped)
- ❌ Picking up (robot must be stopped)

### Tip 4: Copy-Paste Patterns
If you have a pattern that works (like score + pickup), copy it for each cycle!

### Tip 5: You Can Break It Into Multiple Command Methods!
```java
private Command scorePreload() {
    return new SequentialGroup(
        new ParallelGroup(
            new FollowPath(scorePreload),
            // TODO: Spin up shooter
        ),
        new Delay(1.0),
        // TODO: Shoot
    );
}

private Command getAndScorePickup(PathChain goGet, PathChain goScore) {
    return new SequentialGroup(
        new ParallelGroup(
            new FollowPath(goGet),
            // TODO: Start intake
        ),
        new Delay(0.5),
        new ParallelGroup(
            new FollowPath(goScore),
            // TODO: Spin up shooter
        ),
        new Delay(1.0),
        // TODO: Shoot
    );
}

private Command autoRoutine() {
    return new SequentialGroup(
        scorePreload(),
        getAndScorePickup(grabPickup1, scorePickup1),
        getAndScorePickup(grabPickup2, scorePickup2)
    );
}
```

---

## Questions to Ask Mentors

1. "Should we spin up the shooter early or wait until we're at the scoring position?"

2. "How long should we wait for the intake to grab a piece?"

3. "Can we drive while transferring a piece from intake to shooter?"

4. "Should we stop the intake motor between pickups?"

---

## Hand this in when done:

- [ ] Your planned autonomous routine (with comments)
- [ ] Working code that compiles
- [ ] Notes about what you tested
- [ ] Questions about timing or strategy

**Good luck! You got this! 🤖**

---

## Reference: Old vs New

**Old Way (RoadRunner Actions):**
```java
Actions.runBlocking(
    new SequentialAction(
        new ParallelAction(
            followPath, 
            intake.intakePosition()
        ),
        new SleepAction(1.0)
    )
);
```

**New Way (NextFTC):**
```java
// Create a command method
private Command routine() {
    return new SequentialGroup(
        new ParallelGroup(
            new FollowPath(path),
            // TODO: Start intake (or later: new InstantCommand(() -> intake.intake()))
        ),
        new Delay(1.0)
    );
}

// Schedule it
@Override
public void onStartButtonPressed() {
    routine().schedule();
}
```

**Main Differences:**
| Old (RoadRunner) | New (NextFTC) |
|------------------|---------------|
| `Actions.runBlocking()` | Create `Command` method, call `.schedule()` |
| `SequentialAction` | `SequentialGroup` |
| `ParallelAction` | `ParallelGroup` |
| `SleepAction` | `Delay` |
| `intake.intakePosition()` | `// TODO:` comment (later: `new InstantCommand(...)`) |
| `extends OpMode` | `extends NextFTCOpMode` + constructor |
| `start()` | `onStartButtonPressed()` |
| Direct follower use | `PedroComponent.follower()` |

**Almost the same! Just new names and a bit more structure!**

---

## Quick Command Reference

```java
// ✅ Follow a path
new FollowPath(pathName)

// ✅ Wait for time
new Delay(1.5)  // seconds

// ✅ Do things in order
new SequentialGroup(
    thing1,
    thing2,
    thing3
)

// ✅ Do things together
new ParallelGroup(
    thing1,
    thing2
)

// ✅ Quick action (when mechanisms are ready)
new InstantCommand(() -> subsystem.method())
```

---

## Need Help?

1. Look at `ConvertedFromRoadRunnerAuto.java` for a complete example
2. Look at `StudentFriendlyAutonomous.java` for a simpler example
3. Look at `NEXTFTC_CORRECT_API.md` for technical details
4. Ask a mentor!

**Remember: Start simple, test often, and have fun! 🚀**
