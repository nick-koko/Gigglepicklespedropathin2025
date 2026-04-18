# FTC Test Plan - Repo Specific v3 (2026-04-17)

## Core conclusion for today's work

The shooter is effectively at max effort after shot 1, so the main tuning knobs for burst performance are:

- feeder timing
- feeder power
- ball arrival spacing
- ball contact aggressiveness

Not:
- smarter burst detection
- more complicated decision logic after shot 1

That means burst tuning should be treated as a **plant-limited feed-shaping problem**.

---

## What this means physically

After the first shot, the flywheel does not have enough time/authority to fully recover before the next ball unless the next ball arrives later or hits with less load.

So:

- **Distance consistency** depends on how much RPM is left when ball 2 / ball 3 arrive.
- **Left-right consistency** may also depend on how the next ball is presented into the wheel.
- Therefore the main software knobs are the ones that change:
  - when the next ball reaches the wheel
  - how hard it is pushed there
  - how much ball-to-ball overlap exists in the feed path

---

## Repo-specific knobs that now matter most

### Intake / dumbshoot feed knobs

From `IntakeWithSensorsSubsystem.java`:

- `DUMBSHOOT_DELAY_M1_MS`
- `DUMBSHOOT_DELAY_M2_MS`
- `DUMBSHOOT_M1_POWER`
- `DUMBSHOOT_M2_POWER`
- `DUMBSHOOT_M3_POWER`

These are now your primary burst-tuning knobs.

### Logging still useful, but not primary control

From TeleOp and your docs:

- breakbeam logging
- dumbshoot RPM logging
- shot-breakbeam sequence logging

These are still very useful for:
- understanding what happened
- measuring spacing
- confirming ball timing
- catching weird faults

But they are now mainly **measurement tools**, not the main thing that will fix burst quality.

---

## New priority order for today

1. Intake containment / popout testing
2. Burst feed timing and feed power tuning
3. Single-ball stationary shot tuning
4. Turret lag tuning
5. Only later: shot-table structure cleanup

Why:
- Intake popout can invalidate burst work.
- Burst consistency is now mostly a feed-shaping problem.
- Single-ball shot tuning should stay separate from burst tuning.
- Turret lag should not be tuned using noisy burst data if avoidable.

---

## Part 1 - Intake containment

## Goal

Determine whether the popout problem is:
- active pushing from intake stages
- incorrect state / incorrect ball count
- or a purely mechanical compression issue

## Important code fact

In normal intake:
- with 2 balls already in the robot, only `m1` should still run
- with 3 balls already in the robot, all stages should already be off

So the wall-ball test is a very strong mechanical vs software discriminator.

## Checklist

- [x] Watch telemetry for:
  - `ballCount`
  - `INT_m1Enabled`
  - `INT_m2Enabled`
  - `INT_m3Enabled`
  - `INT_isIntaking`
  - `INT_shooting`
  - `INT_singleBallFeedActive`
  - `INT_multiSingleShotActive`
- [x] Test third-ball intake while sweeping `M1_TARGET_RPM_BALL2`
- [x] Test driving a full robot into a wall ball while confirming all stages are off
- [x] Record whether popout still occurs when telemetry says everything is disabled

## Result from today's intake work

- Intake testing is complete.
- A small occupied-stage hold in `IntakeWithSensorsSubsystem` helped contain already-loaded balls.
- That hold should be treated as an **idle containment tool**, not an active forward-intake tool, because applying it during 3rd-ball pickup made the jam/compression case noticeably worse.
- The burst work can now proceed without the intake popout issue being the main unknown.

## Decision rule

- If popout still happens with all stages off, this is mainly mechanical containment.
- If slowing `M1_TARGET_RPM_BALL2` helps third-ball intake but wall-ball still fails, you likely need both:
  - a softer 3rd-ball software profile
  - and a mechanical blocker or more internal free space

---

## Part 2 - Burst timing / burst observer testing

## Main objective

Before changing burst logic again, measure the timing relationship between:

- feed start
- current hybrid expected-contact timing
- breakbeam edge timing
- first measurable RPM collapse
- actual shooter power rise

The goal is to determine the best layered trigger for:

**"ball first touches the flywheel -> command 100% recovery power immediately."**

This phase is intentionally an **observer-mode test**, not a final tuning pass.

---

## Working hypothesis

The most important burst-control question right now is not "what final boost curve is best?"

It is:

**"What is the earliest reliable signal that the first ball has actually loaded the flywheel?"**

Candidate signals:

1. **current time-based expected contact**
2. **breakbeam edges**
3. **RPM collapse / RPM-delta collapse**
4. **a layered trigger using time + beam + RPM**

---

## Burst testing rules for today

### Rule 1
Do not let the shot test materially change burst behavior while gathering timing data.

Use the current burst path, but make boost effectively inert so the logs capture:
- when the current timed transition would have happened
- when beam events actually happen
- when RPM actually collapses

### Rule 2
Start with **1-ball tests** to identify the cleanest first-contact signal.

Then use **3-ball bursts** to validate whether that same trigger still works under overlapping events.

### Rule 3
Wait on random driving tests until structured stationary data has been reviewed and at least one burst-logic change has been chosen.

---

## Burst Session Setup

- [ ] `ENABLE_DUMBSHOOT_RPM_LOGGING = true`
- [ ] `ENABLE_SHOT_INFO_LOGGING = true`
- [ ] `ENABLE_SOTM_LOGGING = false`
- [ ] `ENABLE_TURRET_LOGGING = false`
- [ ] `ENABLE_SHOT_TUNING_LOGGING = false`
- [ ] Keep `BURST_PROFILE_ID` updated for each profile
- [ ] Use consistent starting pose and loading method
- [ ] Start with fixed stationary pose / fixed target RPM / fixed hood
- [ ] Collect near and far separately if time allows

### Observer-mode boost settings

Keep the hybrid controller active for logging, but prevent it from meaningfully boosting during the shot:

- [ ] `ENABLE_HYBRID_SHOT_FEED_BOOST = true`
- [ ] `HYBRID_PREBOOST1_LEAD_MS = 0`
- [ ] `HYBRID_PREBOOST2_LEAD_MS = 0`
- [ ] `HYBRID_PREBOOST3_LEAD_MS = 0`
- [ ] `HYBRID_PREBOOST1_AMOUNT = 0.0`
- [ ] `HYBRID_PREBOOST2_AMOUNT = 0.0`
- [ ] `HYBRID_PREBOOST3_AMOUNT = 0.0`
- [ ] `BOOST_DELAY_MS = 5000` or larger for observer-mode timing runs
- [ ] `BOOST_STAGE1_MULTIPLIER_NEAR = 1.0`
- [ ] `BOOST_STAGE2_MULTIPLIER_NEAR = 1.0`
- [ ] `BOOST_STAGE1_MULTIPLIER_FAR = 1.0`
- [ ] `BOOST_STAGE2_MULTIPLIER_FAR = 1.0`

This preserves the hybrid timing and logging fields while preventing the burst run from actually receiving meaningful boost.

---

## Burst testing order

### Phase A - 1-ball timing characterization first

Why first:
- single-ball tests give the cleanest view of first-contact timing
- beam and RPM can be compared without overlapping later-ball events
- this is the best way to judge whether time, beam, RPM, or layered logic should trigger 100% recovery

### What to compare
- `hybrid_expected_contact_ms`
- first relevant breakbeam edge
- first meaningful `shooter_rpm_delta_avg` collapse
- first clear `shooter_avg_rpm` drop
- actual power rise timing

### Suggested approach
Hold feed profile, distance, hood, and target RPM fixed.
Repeat enough times to measure timing jitter.

#### Recommended block
- [ ] 10 to 20 one-ball shots from one fixed pose/distance
- [ ] Repeat at a second distance only if time allows

### What to record
- [ ] which beam edge, if any, is earliest and reliable
- [ ] average delay from beam edge to RPM collapse
- [ ] average error between current timed threshold and actual RPM collapse
- [ ] run-to-run jitter for each candidate signal

### What to watch
- [ ] Is breakbeam early enough and reliable enough to be primary?
- [ ] Is RPM collapse only one loop late and therefore acceptable?
- [ ] Is the current timed estimate systematically early or late?

---

## Phase B - 3-ball burst validation second

Why second:
- validates whether the candidate trigger still works when events overlap
- shows whether ball 2 / ball 3 need the same or different contact logic

### Suggested approach
- [ ] 5 to 10 baseline 3-ball bursts at the same fixed distance
- [ ] compare beam/RPM/timed relationships across all three contact events
- [ ] only after review, choose a boost-logic change to implement

### What to watch
- [ ] does the same trigger work for ball 1 and still make sense for later balls?
- [ ] do later balls need separate thresholds/windows?
- [ ] is there enough consistency to move from observer mode into real boost changes?

---

## Phase C - Only after reviewing timing data

Possible next logic directions:

- layered trigger: time window + beam edge + RPM-drop fallback
- RPM-drop-first trigger if it is consistently fast enough
- beam-first trigger if a specific edge is consistently earliest
- transition to immediate `100%` power on accepted contact event

Only if shots still fall short **after** the best recovery trigger is found:

- consider later-ball hood adjustment as a secondary correction layer

---

## Burst test matrix

Keep it small enough to finish.

### Recommended structure

#### Block 1 - 1-ball observer mode baseline
- [ ] 10 to 20 sequences

#### Block 2 - 3-ball observer mode baseline
- [ ] 5 to 10 sequences

#### Block 3 - optional second-distance observer mode
- [ ] repeat Block 1 and Block 2 at a farther distance if time allows

Only after data review:

#### Block 4 - revised boost logic candidate
- [ ] repeat the same structured runs

---

## What to record for each profile

- [ ] completion rate
- [ ] shot 2 distance consistency
- [ ] shot 3 distance consistency
- [ ] left-right spread
- [ ] obvious jams / hesitation
- [ ] whether balls seem to “slam” into the wheel vs enter smoothly

---

## How to interpret outcomes

### If a specific beam edge is earliest and low-jitter
Then:
- use that edge as the earliest trigger layer
- keep time and RPM as backup / sanity checks

### If RPM collapse is only one loop later but much more reliable
Then:
- RPM can be the primary trigger
- beam can remain auxiliary telemetry / cross-check

### If the current timed threshold is consistently biased but low-jitter
Then:
- retune the expected contact timing
- still consider layering beam/RPM for robustness

### If no single signal is both early and reliable
Then:
- use a layered trigger:
  - timed eligibility window
  - accept beam edge if valid
  - otherwise accept RPM collapse

### If full-power recovery still leaves later shots short
Then:
- add hood adjustment only after the recovery trigger is as good as possible

---

## Part 3 - Single-ball shot tuning stays separate

## Goal

Find clean stationary RPM/hood/target anchor points without burst interference.

Do not use burst data to tune the base shot map.

## Controls already in code

- `g2RB`: toggle flywheel
- `g2A`: fire one ball
- `g2Y/X`: hood up/down
- `g2DpadUp/Down`: RPM up/down
- `g1Dpad`: move target
- `g1X`: KEEP
- `g1B`: IGNORE

## Procedure

- [ ] Turn on shot tuning mode
- [ ] Use single-ball tests only
- [ ] Collect near / mid / far anchor points
- [ ] Add one left-offset and one right-offset anchor
- [ ] Label every shot

## Decision after data

- If behavior mostly follows distance, use a distance-based interpolation table
- If same distance behaves differently by lateral position, move to anchor-point / zone-based map

---

## Part 4 - Turret lag tuning

Keep this narrow and separate from burst work.

## Main parameter
- `SOTM_TURRET_LAG_COMP_SEC`

## Suggested tuning
- [ ] start from current value
- [ ] move in `0.02` steps
- [ ] do turn-in-place tracking first
- [ ] then strafe tracking
- [ ] only then test moving shots

## If residual lag remains
Then inspect:
- `OUTER_LOOP_KP`
- `OUTER_LOOP_MAX_TRIM_DEGREES`
- turret logs for steady-state error / stiction

---

## Best likely outcomes from today

### Intake
- Know whether the wall-ball popout is fundamentally mechanical
- Possibly improve third-ball intake by lowering `M1_TARGET_RPM_BALL2`

### Burst
- Identify a better timer/power profile
- Improve both distance consistency and left-right consistency without pretending the shooter can recover instantly

### Single-ball shots
- Collect clean anchor data for RPM/hood/target mapping

### Turret
- Narrow the best `SOTM_TURRET_LAG_COMP_SEC` range

---

## Strongest practical opinion

At this point the burst problem should be framed like this:

**The shooter is saturated after shot 1, so the next big performance gains come from controlling how the next balls arrive, not from detecting their arrival more cleverly.**

That means the most valuable burst work today is:
- simpler logic
- better timing
- better feed power shaping
- and only later, geometry changes if software gains level off
