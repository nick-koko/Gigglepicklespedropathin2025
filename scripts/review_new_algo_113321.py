"""Quick review of the April 18 11:33 data collected with the new ball-1 composite trigger."""
import pandas as pd

pd.set_option("display.width", 200)
df = pd.read_csv(
    r"C:\Users\nicko\StudioProjects\Gigglepicklespedropathin2025\Data\April18\pickles2025_dumbshoot_rpm_20260418_113321.csv"
)

rows = []
for sid, sdf in df.groupby("sequence_id"):
    feed = sdf[sdf["hybrid_feed_boost_active"] == True].sort_values(
        "hybrid_t_since_shot_feed_start_ms"
    )
    if feed.empty:
        continue
    b2p = feed[feed["hybrid_phase"] == "WAIT_BALL2_CONTACT"]
    b1_reason = b2p.iloc[0]["hybrid_last_advance_reason"] if not b2p.empty else "NONE"
    b1_t = b2p.iloc[0]["hybrid_last_advance_rel_ms"] if not b2p.empty else None
    full = sdf.sort_values("t_ms")
    rpm_peak = full["rpm_delta_from_baseline"].max()
    cur_peak = full["shooter_current_delta_from_baseline_a"].max()
    b1w = feed[feed["hybrid_phase"] == "WAIT_BALL1_CONTACT"]
    bb2_fell_in_b1 = bool(b1w["bb2_fall_edge"].any())
    rows.append(
        dict(
            seq=sid,
            b1_reason=b1_reason,
            b1_ms=b1_t,
            rpm_peak_drop=rpm_peak,
            cur_peak_spike=cur_peak,
            bb2_fell_b1=bb2_fell_in_b1,
        )
    )

s = pd.DataFrame(rows)
print("Per sequence:")
print(s.to_string(index=False))
print()
print("B1 advance reason summary:")
print(
    s.groupby("b1_reason")
    .agg(n=("b1_ms", "count"), min_ms=("b1_ms", "min"), med_ms=("b1_ms", "median"), max_ms=("b1_ms", "max"))
    .reset_index()
)
print()
s["real_shot"] = (s["rpm_peak_drop"] > 150) & (s["cur_peak_spike"] > 0.5)
print("Ball actually shot? (rpm_peak>150 AND cur_peak>0.5A):")
print(s.groupby("real_shot").agg(n=("seq", "count"), b1_mean_ms=("b1_ms", "mean")).reset_index())
print()
print("Summary vs. timer-fallback baseline:")
median_b1 = s["b1_ms"].median()
mean_b1 = s["b1_ms"].mean()
n_timer = int((s["b1_reason"] == "BALL1_TIMER_FALLBACK").sum())
n_bb2 = int((s["b1_reason"] == "BALL1_BB2_FALL_EDGE").sum())
n_rpm = int(s["b1_reason"].str.startswith("BALL1_RPM").sum())
n_cur = int(s["b1_reason"].str.startswith("BALL1_CURRENT").sum())
print(f"  New algo ball-1 advance time: median={median_b1:.0f} ms, mean={mean_b1:.0f} ms")
print(f"  Timer fallback: {n_timer}/{len(s)}")
print(f"  BB2 edge:       {n_bb2}/{len(s)}")
print(f"  RPM triggers:   {n_rpm}/{len(s)}  (baseline + derivative)")
print(f"  Current trigs:  {n_cur}/{len(s)}  (baseline + derivative)")
print()
print("Firing-time distribution by reason class:")
for cls, mask in [
    ("bb2_fall_edge", s["b1_reason"] == "BALL1_BB2_FALL_EDGE"),
    ("rpm_triggers", s["b1_reason"].str.startswith("BALL1_RPM")),
    ("cur_triggers", s["b1_reason"].str.startswith("BALL1_CURRENT")),
    ("timer_fallback", s["b1_reason"] == "BALL1_TIMER_FALLBACK"),
]:
    t = s.loc[mask, "b1_ms"].dropna()
    if not t.empty:
        print(f"  {cls:14s}: n={len(t)}  min={t.min():.0f}  med={t.median():.0f}  max={t.max():.0f}")
    else:
        print(f"  {cls:14s}: n=0")
