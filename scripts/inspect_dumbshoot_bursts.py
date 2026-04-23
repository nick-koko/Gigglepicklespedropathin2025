"""Quick scan of a SOTM CSV to inspect dumbShoot bursts.

Prints every dumbShoot start (dumbshoot_timer_active false -> true) and the
following ~1.6 seconds of rows showing ball_count, shooter RPMs, and intake
encoder positions.
"""
import csv
import sys


def inspect(path: str) -> None:
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))

    prev_ds = "false"
    bursts = 0
    for i, row in enumerate(rows):
        ds = row["dumbshoot_timer_active"]
        if ds == "true" and prev_ds == "false":
            bursts += 1
            t0 = int(row["t_ms"])
            start_bc = "?"
            # ball_count in the row BEFORE the burst started is more useful
            if i > 0:
                start_bc = rows[i - 1]["ball_count"]
            # find timer end
            timer_end_idx = None
            for j in range(i, min(len(rows), i + 400)):
                if rows[j]["dumbshoot_timer_active"] == "false":
                    timer_end_idx = j
                    break
            if timer_end_idx is None:
                timer_end_idx = min(len(rows) - 1, i + 200)
            timer_end_dt = int(rows[timer_end_idx]["t_ms"]) - t0
            # m3 ticks moved during the burst (forward transfer work)
            m3_start = int(rows[i]["intake_m3_ticks"])
            m3_at_end = int(rows[timer_end_idx]["intake_m3_ticks"])
            # Phase 1 natural exit -> target_rpm should drop to 0 at end. If it
            # stays >0 at end, something else (gamepad button, re-spin-up)
            # cancelled the timer early. Line 1495 re-spins up shooter when
            # sotmFireRequestActive=true so we also look at the fire input.
            end_tgt_rpm = float(rows[timer_end_idx]["shooter_target_rpm"])
            end_fire = rows[timer_end_idx]["sotm_fire_request_active"]
            fire_at_end = end_fire == "true"
            # scan 3 sec post-timer for ball_count changes
            post_bc_trace = []
            last_bc = rows[timer_end_idx]["ball_count"]
            post_bc_trace.append((timer_end_dt, last_bc))
            for j in range(timer_end_idx + 1, min(len(rows), timer_end_idx + 120)):
                bc = rows[j]["ball_count"]
                if bc != last_bc:
                    dt = int(rows[j]["t_ms"]) - t0
                    post_bc_trace.append((dt, bc))
                    last_bc = bc
            print(
                f"burst {bursts}: row {i} t={t0} pre_bc={start_bc} "
                f"dur={timer_end_dt:4d}ms m3_ticks={m3_at_end - m3_start:4d} "
                f"end_tgt={end_tgt_rpm:6.0f} fire_at_end={fire_at_end} "
                f"bc_trace={post_bc_trace}"
            )
        prev_ds = ds

    if bursts == 0:
        print("No dumbShoot bursts detected in this log.")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("usage: python inspect_dumbshoot_bursts.py <path-to-sotm-csv>")
        sys.exit(1)
    inspect(sys.argv[1])
