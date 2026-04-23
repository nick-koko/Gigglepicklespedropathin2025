"""Print a focused trace of dumbShoot-related columns for a specific row range."""
import csv
import sys


def main():
    path = sys.argv[1]
    start = int(sys.argv[2])
    end = int(sys.argv[3])
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    print(
        f"{'row':>4} {'t_ms':>14} {'match':>6} "
        f"{'fire':>5} {'rtrig':>5} {'ds':>5} {'bc':>2} "
        f"{'m1t':>7} {'m3t':>6} {'tgt_rpm':>8} {'rpm1':>6} "
        f"{'atsp75':>6} {'sat':>5} {'hold':>5} {'loop':>4}"
    )
    for i in range(start, end + 1):
        r = rows[i - 1]
        print(
            f"{i:4d} {r['t_ms']:>14} {r['match_t_ms']:>6} "
            f"{r['sotm_fire_request_active'][0]:>5} "
            f"{r['right_trigger_active'][0]:>5} "
            f"{r['dumbshoot_timer_active'][0]:>5} "
            f"{r['ball_count']:>2} "
            f"{r['intake_m1_ticks']:>7} {r['intake_m3_ticks']:>6} "
            f"{float(r['shooter_target_rpm']):>8.1f} "
            f"{float(r['shooter_rpm1']):>6.0f} "
            f"{r['shooter_at_speed_75'][0]:>6} "
            f"{r['shooter_cmd_saturated'][0]:>5} "
            f"{r['hold_state'][0]:>5} "
            f"{r['loop_time_ms']:>4}"
        )


if __name__ == "__main__":
    main()
