"""
Find the best trigger for first-ball-flywheel contact for boost transition.

Strategy: for every sequence, compute the "ground-truth" contact time as the
first sample where RPM has dropped > 150 rpm AND current is trending up
(current delta > 0.2A or current derivative > 8 A/s). Then evaluate how early
various single/composite triggers fire relative to ground truth.
"""

import pandas as pd
import numpy as np
import os

DATA_DIR = r"C:\Users\nicko\StudioProjects\Gigglepicklespedropathin2025\Data\April18"
FILES = {
    "single": "pickles2025_dumbshoot_rpm_20260418_101347.csv",
    "burst3": "pickles2025_dumbshoot_rpm_20260418_101558.csv",
}

# Only consider samples in the shot feed window from feed_start_ms >= MIN_MS
MIN_MS = 50  # skip the first 50ms to avoid feed-servo transient / filter settling


def load(name):
    return pd.read_csv(os.path.join(DATA_DIR, FILES[name]))


def sequence_windows(df):
    """Return list of (seq_id, feed_df, ball1_df) where feed_df is the full
    shot-feed window and ball1_df is the WAIT_BALL1_CONTACT portion + the first
    WAIT_BALL2_CONTACT sample (so we can see the advance moment).
    """
    out = []
    for sid, sdf in df.groupby("sequence_id"):
        feed = sdf[sdf["hybrid_feed_boost_active"] == True].copy()
        if feed.empty:
            continue
        feed = feed.sort_values("hybrid_t_since_shot_feed_start_ms").reset_index(drop=True)
        b1 = feed[feed["hybrid_phase"] == "WAIT_BALL1_CONTACT"]
        b2_idx = feed.index[feed["hybrid_phase"] == "WAIT_BALL2_CONTACT"]
        if len(b2_idx) > 0:
            b1_plus = feed.loc[: b2_idx[0]]
        else:
            b1_plus = b1
        out.append((sid, feed, b1, b1_plus))
    return out


def first_fire(fdf, cond_mask, min_ms=MIN_MS):
    """Return earliest t_since_feed_start where cond_mask is True and t >= min_ms."""
    t = fdf["hybrid_t_since_shot_feed_start_ms"]
    m = cond_mask & (t >= min_ms)
    hits = fdf[m]
    if hits.empty:
        return None
    return float(hits.iloc[0]["hybrid_t_since_shot_feed_start_ms"])


def evaluate_triggers(df, label):
    print(f"\n{'='*100}")
    print(f"  {label}   (file={FILES[label]})")
    print(f"{'='*100}")
    seqs = sequence_windows(df)

    rows = []
    for sid, feed, b1, b1_plus in seqs:
        # Restrict detection to the WAIT_BALL1_CONTACT phase + first post-advance
        # sample. Going beyond would pick up ball 2 signals.
        win = b1_plus

        rpm_db = win["rpm_delta_from_baseline"]
        rpm_d = win["shooter_rpm_derivative_rpm_per_sec"]
        cur_db = win["shooter_current_delta_from_baseline_a"]
        cur_d = win["shooter_current_derivative_a_per_sec"]
        bb2f = win["bb2_fall_edge"]

        # Ground-truth contact: both RPM and current moved together
        gt = (rpm_db > 100) & ((cur_db > 0.25) | (cur_d > 8))
        t_gt = first_fire(win, gt)

        # Candidate triggers
        triggers = {
            "bb2_fall": first_fire(win, bb2f == True, min_ms=0),
            "cur_db>0.3A": first_fire(win, cur_db > 0.3),
            "cur_db>0.4A": first_fire(win, cur_db > 0.4),
            "cur_d>10A/s": first_fire(win, cur_d > 10),
            "cur_d>12A/s": first_fire(win, cur_d > 12),
            "rpm_db>150": first_fire(win, rpm_db > 150),
            "rpm_db>200": first_fire(win, rpm_db > 200),
            "rpm_d<-3000": first_fire(win, rpm_d < -3000),
            "rpm_d<-4000": first_fire(win, rpm_d < -4000),
            # Composite A: either current or rpm signal
            "cur_db>0.3|rpm_db>150": first_fire(win, (cur_db > 0.3) | (rpm_db > 150)),
            # Composite B: require both for confirmation
            "cur_db>0.25 & rpm_db>100": first_fire(win, (cur_db > 0.25) & (rpm_db > 100)),
            # Composite C: bb2 OR (rpm_d<-3k & cur rising)
            "bb2|rpm_d<-3k&cur_d>5": first_fire(
                win, (bb2f == True) | ((rpm_d < -3000) & (cur_d > 5)), min_ms=0
            ),
            # Composite D: aggressive early (bb2) OR current rising alone
            "bb2|cur_d>10": first_fire(win, (bb2f == True) | (cur_d > 10), min_ms=0),
        }

        # Existing logic's fire time (first WAIT_BALL2_CONTACT sample reason + rel_ms)
        b2_phase = feed[feed["hybrid_phase"] == "WAIT_BALL2_CONTACT"]
        if not b2_phase.empty:
            adv_reason = b2_phase.iloc[0]["hybrid_last_advance_reason"]
            adv_rel = float(b2_phase.iloc[0]["hybrid_last_advance_rel_ms"])
        else:
            adv_reason, adv_rel = "NONE", None

        row = {"seq": sid, "gt": t_gt, "existing_adv_ms": adv_rel, "existing_reason": adv_reason}
        row.update(triggers)
        rows.append(row)

    summary = pd.DataFrame(rows)
    print("\nPer-sequence trigger fire times (ms since feed start). 'gt' = synthetic ground truth.")
    print(summary.to_string(index=False))

    # Stats
    print("\nDetector statistics (fire-time distribution and lead vs ground truth):")
    cols = [c for c in summary.columns if c not in ("seq", "gt", "existing_reason")]
    for col in cols:
        s = summary[col].dropna()
        n_fired = len(s)
        n_total = len(summary)
        lead = summary["gt"] - summary[col]  # positive = fires earlier than gt
        lead = lead.dropna()
        if not s.empty:
            print(
                f"  {col:28s} fired={n_fired}/{n_total}  "
                f"min={s.min():6.0f}  med={s.median():6.0f}  max={s.max():6.0f}  "
                f"lead_med={lead.median() if not lead.empty else float('nan'):+6.0f}ms"
            )
        else:
            print(f"  {col:28s} fired=0/{n_total}")

    return summary


def main():
    for label in ("burst3", "single"):
        df = load(label)
        evaluate_triggers(df, label)


if __name__ == "__main__":
    main()
