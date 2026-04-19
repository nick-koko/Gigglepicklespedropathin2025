"""
Analyze RPM and current observer data to find the best trigger for
first-ball-contact detection (for boost transition after ball 1).

Usage: python analyze_ball_contact.py
"""

import pandas as pd
import numpy as np
import os

DATA_DIR = r"C:\Users\nicko\StudioProjects\Gigglepicklespedropathin2025\Data\April18"
FILES = {
    "single": "pickles2025_dumbshoot_rpm_20260418_101347.csv",
    "burst3": "pickles2025_dumbshoot_rpm_20260418_101558.csv",
}


def load(name):
    return pd.read_csv(os.path.join(DATA_DIR, FILES[name]))


def per_sequence_feed_aligned(df):
    """Return dict of sequence_id -> DataFrame restricted to the shot feed window
    (hybrid_feed_boost_active True) with columns of interest, re-indexed by
    hybrid_t_since_shot_feed_start_ms.
    """
    out = {}
    for sid, sdf in df.groupby("sequence_id"):
        feed = sdf[sdf["hybrid_feed_boost_active"] == True].copy()
        if feed.empty:
            continue
        feed = feed.sort_values("hybrid_t_since_shot_feed_start_ms")
        out[sid] = feed
    return out


def analyze_ball1_contact(df, label):
    """For each sequence, identify ball-1 contact candidates from several
    signals and report timing/values."""
    print(f"\n{'='*90}")
    print(f"  {label}   (file={FILES[label]})")
    print(f"{'='*90}")
    seqs = per_sequence_feed_aligned(df)

    rows = []
    for sid, fdf in seqs.items():
        # Ground-truth style: bb2 fall edge if present during WAIT_BALL1_CONTACT,
        # otherwise use RPM and current signatures.
        b1_phase = fdf[fdf["hybrid_phase"] == "WAIT_BALL1_CONTACT"]
        if b1_phase.empty:
            continue

        # Advance reason logged at the advance point
        b2_phase = fdf[fdf["hybrid_phase"] == "WAIT_BALL2_CONTACT"]
        if not b2_phase.empty:
            adv_reason = b2_phase.iloc[0]["hybrid_last_advance_reason"]
            adv_rel = b2_phase.iloc[0]["hybrid_last_advance_rel_ms"]
        else:
            adv_reason = "NONE"
            adv_rel = None

        # bb2_fall_edge during WAIT_BALL1_CONTACT (true contact indicator)
        bb2_true = b1_phase[b1_phase["bb2_fall_edge"] == True]
        bb2_contact_ms = (
            bb2_true.iloc[0]["hybrid_t_since_shot_feed_start_ms"] if not bb2_true.empty else None
        )

        # Look for earliest strong current derivative spike (> 10 A/s) during WAIT_BALL1
        cur_spike = b1_phase[b1_phase["shooter_current_derivative_a_per_sec"] > 10.0]
        cur_spike_ms = (
            cur_spike.iloc[0]["hybrid_t_since_shot_feed_start_ms"] if not cur_spike.empty else None
        )

        # Look for earliest current delta from baseline > 0.5A
        cur_delta = b1_phase[b1_phase["shooter_current_delta_from_baseline_a"] > 0.5]
        cur_delta_ms = (
            cur_delta.iloc[0]["hybrid_t_since_shot_feed_start_ms"] if not cur_delta.empty else None
        )
        cur_delta_ms_1p0 = None
        tmp = b1_phase[b1_phase["shooter_current_delta_from_baseline_a"] > 1.0]
        if not tmp.empty:
            cur_delta_ms_1p0 = tmp.iloc[0]["hybrid_t_since_shot_feed_start_ms"]

        # Look for earliest strong RPM drop (negative derivative beyond -2000 rpm/s)
        rpm_drop = b1_phase[b1_phase["shooter_rpm_derivative_rpm_per_sec"] < -2000.0]
        rpm_drop_ms = (
            rpm_drop.iloc[0]["hybrid_t_since_shot_feed_start_ms"] if not rpm_drop.empty else None
        )
        # rpm_delta_from_baseline > 150
        rpm_db = b1_phase[b1_phase["rpm_delta_from_baseline"] > 150.0]
        rpm_db_ms = (
            rpm_db.iloc[0]["hybrid_t_since_shot_feed_start_ms"] if not rpm_db.empty else None
        )
        rpm_db_200 = b1_phase[b1_phase["rpm_delta_from_baseline"] > 200.0]
        rpm_db_200_ms = (
            rpm_db_200.iloc[0]["hybrid_t_since_shot_feed_start_ms"]
            if not rpm_db_200.empty
            else None
        )

        rows.append(
            {
                "seq": sid,
                "adv_reason": adv_reason,
                "adv_ms": adv_rel,
                "bb2_fall_ms": bb2_contact_ms,
                "cur_deriv>10_ms": cur_spike_ms,
                "cur_delta>0.5A_ms": cur_delta_ms,
                "cur_delta>1.0A_ms": cur_delta_ms_1p0,
                "rpm_deriv<-2k_ms": rpm_drop_ms,
                "rpm_db>150_ms": rpm_db_ms,
                "rpm_db>200_ms": rpm_db_200_ms,
            }
        )

    summary = pd.DataFrame(rows)
    print("\nPer-sequence first-ball-contact detector fire times (ms since shot feed start):")
    print(summary.to_string(index=False))

    # Statistics on each signal
    print("\nDetector fire-time statistics (ms since shot feed start):")
    for col in [
        "bb2_fall_ms",
        "cur_deriv>10_ms",
        "cur_delta>0.5A_ms",
        "cur_delta>1.0A_ms",
        "rpm_deriv<-2k_ms",
        "rpm_db>150_ms",
        "rpm_db>200_ms",
    ]:
        s = summary[col].dropna()
        if s.empty:
            print(f"  {col:22s}  fired=0/{len(summary)}")
            continue
        print(
            f"  {col:22s}  fired={len(s)}/{len(summary)}  "
            f"min={s.min():.0f}  p25={s.quantile(0.25):.0f}  "
            f"med={s.median():.0f}  p75={s.quantile(0.75):.0f}  max={s.max():.0f}"
        )

    return summary


def trace_around_contact(df, label):
    """Show per-sample trace relative to feed start for every sequence."""
    print(f"\n{'-'*90}")
    print(f"  Per-sample feed-start-aligned traces ({label})")
    print(f"{'-'*90}")
    seqs = per_sequence_feed_aligned(df)
    for sid, fdf in seqs.items():
        print(f"\n--- sequence {sid} ---")
        sub = fdf[
            [
                "hybrid_t_since_shot_feed_start_ms",
                "hybrid_phase",
                "hybrid_last_advance_reason",
                "hybrid_last_advance_rel_ms",
                "shooter_avg_rpm",
                "shooter_rpm_derivative_rpm_per_sec",
                "rpm_delta_from_baseline",
                "shooter_current_avg_a",
                "shooter_current_delta_from_baseline_a",
                "shooter_current_derivative_a_per_sec",
                "bb2",
                "bb2_fall_edge",
            ]
        ].copy()
        sub = sub.rename(
            columns={
                "hybrid_t_since_shot_feed_start_ms": "t",
                "hybrid_phase": "phase",
                "hybrid_last_advance_reason": "reason",
                "hybrid_last_advance_rel_ms": "r_ms",
                "shooter_avg_rpm": "rpm",
                "shooter_rpm_derivative_rpm_per_sec": "rpm_d",
                "rpm_delta_from_baseline": "rpm_db",
                "shooter_current_avg_a": "curA",
                "shooter_current_delta_from_baseline_a": "cur_db",
                "shooter_current_derivative_a_per_sec": "cur_d",
            }
        )
        print(sub.to_string(index=False))


def main():
    for label in ("burst3", "single"):
        df = load(label)
        analyze_ball1_contact(df, label)
    # Also dump detailed traces for burst3 so we can eyeball patterns
    trace_around_contact(load("burst3"), "burst3")


if __name__ == "__main__":
    main()
