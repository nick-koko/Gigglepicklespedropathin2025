# Breakbeam Fault Diagnostics (Draft)

## Goal

Add a lightweight runtime diagnostic that raises a **possible BB2 wiring/sensor fault** when feed behavior and sensor order disagree.

This was motivated by a real failure where BB2 wiring was disconnected, so BB2 never reported broken and the associated feed path stayed enabled.

## Core Fault Rule

Raise `POSSIBLE_BB2_FAULT` when all of these happen in one feed cycle:

- The BB2-associated feed motor/path is enabled (currently this is the `m3` stage in intake logic).
- A `BB1` break event is observed.
- Then a `BB0` break event is observed (expected downstream progression).
- `BB2` does **not** show a break event within a timeout window.

In plain terms: if the robot sees later sensors trip but never sees BB2 trip, BB2 is suspicious.

## Suggested Timing Windows

Start conservative and tune from logs:

- `BB2_FAULT_MAX_WAIT_MS = 350` (time allowed for BB2 to break after BB1/BB0 evidence starts)
- `BB2_FAULT_REARM_CLEAR_MS = 500` (time BB2 must look healthy before clearing fault)
- `BB2_FAULT_MIN_CONSECUTIVE_CYCLES = 2` (optional debounce before latching)

## State Machine Sketch

- `IDLE`
  - Wait for a cycle where BB2 feed path is enabled.
- `MONITORING`
  - Track whether `bb1BreakSeen` and `bb0BreakSeen` occur in order.
  - If `bb2BreakSeen`, clear monitor state and return healthy.
  - If `bb1BreakSeen && bb0BreakSeen && !bb2BreakSeen` for longer than timeout, flag fault.
- `FAULT_LATCHED`
  - Keep telemetry/LED warning active.
  - Optionally auto-clear only after healthy BB2 transitions are observed again.

## Pseudocode

```java
if (bb2FeedEnabled) {
    if (!monitorActive) {
        monitorActive = true;
        monitorStartMs = nowMs;
        bb1BreakSeen = false;
        bb0BreakSeen = false;
        bb2BreakSeen = false;
    }

    if (bb1FallingEdge) bb1BreakSeen = true;
    if (bb0FallingEdge && bb1BreakSeen) bb0BreakSeen = true;
    if (bb2FallingEdge) bb2BreakSeen = true;

    if (bb2BreakSeen) {
        clearBb2FaultCandidate();
    } else if (bb1BreakSeen && bb0BreakSeen &&
               (nowMs - monitorStartMs) > BB2_FAULT_MAX_WAIT_MS) {
        setPossibleBb2Fault();
    }
} else {
    monitorActive = false;
}
```

## What To Do When Fault Triggers

- Telemetry: show `POSSIBLE_BB2_FAULT = true`.
- Driver feedback: LED pattern (example: orange strobe).
- Log fields:
  - `bb2_fault_active`
  - `bb2_fault_reason`
  - `bb2_fault_cycle_ms`
  - `bb0_break_count`, `bb1_break_count`, `bb2_break_count`
- Safety option:
  - Stop or reduce the BB2-associated feed motor/path if fault remains active for > N ms.

## False Positive Controls

- Require order (`BB1` then `BB0`) instead of any random transitions.
- Only evaluate while feed path is actually commanded.
- Use edge-based checks, not level-only checks.
- Add debounce (`MIN_CONSECUTIVE_CYCLES`) before latching.

## Student Checklist (Pit Test)

- Unplug BB2 intentionally -> verify fault appears quickly.
- Reconnect BB2 -> verify fault clears after healthy transitions.
- Run normal 3-ball sequence -> verify no false fault.
- Run jitter/noise scenario -> verify debounce prevents false alarms.

## Notes

- Sensor naming and physical order must stay documented and consistent.
- If mechanical order differs from logical order on this robot, update the rule ordering accordingly.
