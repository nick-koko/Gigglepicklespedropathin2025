# Shot Table Diagnostic

Inputs: live `ShotCalibrationTable.java`, leave-one-out IDW with `k=3`, `power=2.0`.

This report does **not** prove a point is wrong. It highlights rows whose values differ the most from what nearby points would have predicted.

## Diagram

- SVG: `scripts\shot_table_diagnostic.svg`
- Includes two interpolated x/y surfaces: RPM and hood.
- Surface panels also show dashed lines from each shot point to its target position.

## Top review candidates

1. `pt 4` at `(40, 134)` zone `A`: score `3.81`, rpm delta `-362`, hood delta `-0.334`, aimX delta `-0.02`, aimY delta `+1.52`; neighbors `1@26.3in, 6@32.2in, 3@41.2in`.
2. `pt 12` at `(90, 14)` zone `B`: score `2.33`, rpm delta `+275`, hood delta `+0.000`, aimX delta `-3.19`, aimY delta `+5.00`; neighbors `11@18.1in, 9@20.6in, 10@36.2in`.
3. `pt 2` at `(108, 108)` zone `A`: score `1.98`, rpm delta `+327`, hood delta `+0.016`, aimX delta `-1.19`, aimY delta `-4.65`; neighbors `8@22.1in, 7@29.1in, 3@36.0in`.
4. `pt 1` at `(36, 108)` zone `A`: score `1.87`, rpm delta `-302`, hood delta `-0.087`, aimX delta `-1.22`, aimY delta `-1.42`; neighbors `5@25.2in, 4@26.3in, 3@36.0in`.
5. `pt 5` at `(50, 87)` zone `A`: score `1.75`, rpm delta `+157`, hood delta `+0.141`, aimX delta `+1.04`, aimY delta `+2.22`; neighbors `1@25.2in, 0@26.6in, 3@30.4in`.
6. `pt 8` at `(110, 130)` zone `A`: score `1.32`, rpm delta `+130`, hood delta `+0.032`, aimX delta `+0.26`, aimY delta `+5.51`; neighbors `2@22.1in, 6@38.0in, 3@43.9in`.
7. `pt 6` at `(72, 130)` zone `A`: score `1.05`, rpm delta `-57`, hood delta `+0.098`, aimX delta `+0.32`, aimY delta `-1.08`; neighbors `3@22.0in, 4@32.2in, 8@38.0in`.
8. `pt 0` at `(72, 72)` zone `A`: score `1.03`, rpm delta `-24`, hood delta `+0.015`, aimX delta `+1.15`, aimY delta `+4.13`; neighbors `7@22.0in, 5@26.6in, 3@36.0in`.
9. `pt 10` at `(54, 10)` zone `B`: score `1.01`, rpm delta `-66`, hood delta `+0.000`, aimX delta `+1.90`, aimY delta `-0.66`; neighbors `11@18.1in, 9@22.8in, 12@36.2in`.
10. `pt 7` at `(86, 89)` zone `A`: score `0.95`, rpm delta `-41`, hood delta `+0.030`, aimX delta `-0.94`, aimY delta `-3.70`; neighbors `0@22.0in, 3@23.6in, 2@29.1in`.
11. `pt 9` at `(72, 24)` zone `B`: score `0.55`, rpm delta `-94`, hood delta `+0.000`, aimX delta `+0.37`, aimY delta `-1.05`; neighbors `11@12.0in, 12@20.6in, 10@22.8in`.
12. `pt 3` at `(72, 108)` zone `A`: score `0.55`, rpm delta `+43`, hood delta `-0.049`, aimX delta `+0.14`, aimY delta `+0.21`; neighbors `6@22.0in, 7@23.6in, 5@30.4in`.
13. `pt 11` at `(72, 12)` zone `B`: score `0.31`, rpm delta `-20`, hood delta `-0.000`, aimX delta `+0.35`, aimY delta `-1.17`; neighbors `9@12.0in, 10@18.1in, 12@18.1in`.

## Specific points you mentioned

- `pt 0` / `(72,72)`: neighbor disagreement is dominated by aim (`aimX -6`, `aimY 144`) more than RPM. The recent RPM increase to `3400` barely moved its local outlier score; if it still shoots short after batteries recover, the issue may be more trajectory/target shape than pure RPM alone.
- `pt 3` / `(72,108)`: this point does **not** look like a strong numeric outlier relative to nearby rows. If the drive team says it is bad, I would suspect repeatability at that location, battery state, or a target/hood interaction rather than a wildly wrong table value.
- `pt 5` / `(50,87)`: shoots well, but its values are noticeably more aggressive than nearby left-side neighbors, especially hood and aimX. That can be valid, but it also means interpolation between `pt 5`, `pt 1`, and `pt 0` may bend sharply in that region.
- `pt 4` / `(40,134)`: very strong structural outlier because `hood=0.000` is intentionally extreme. Since this is a known close-lip shot, treat it as a valid exception, not an automatic fix target.

## Suggested retune order

1. Recheck `pt 0 (72,72)` after batteries charge, since you already observed it short.
2. Recheck an in-between shot near `(72,90-96)` or `(60,96)` to see whether the left/interior surface between `pt 0`, `pt 3`, and `pt 5` is bending the way you expect.
3. Recheck `pt 3 (72,108)` with the same battery state as the good `pt 5` run; its local math looks reasonable, so the failure may be operational rather than table-shape.
4. If upper-left interpolated shots look too flat, add one more nearby point instead of changing `pt 4` away from `hood=0.000`.

## Raw table

| idx | zone | x | y | rpm | hood | aimX | aimY | note |
|---:|:---:|---:|---:|---:|---:|---:|---:|---|
| 0 | A | 72 | 72 | 3400 | 0.525 | 1.5 | 136.0 | A apex |
| 1 | A | 36 | 108 | 2800 | 0.220 | -0.5 | 133.0 | A mid-left |
| 2 | A | 108 | 108 | 4000 | 0.525 | -1.5 | 128.0 | A mid-right hood-max |
| 3 | A | 72 | 108 | 3400 | 0.440 | 0.5 | 132.0 | A mid-center |
| 4 | A | 40 | 134 | 2700 | 0.000 | 0.0 | 134.0 | A upper-left edge |
| 5 | A | 50 | 87 | 3325 | 0.525 | 1.5 | 136.0 | A left interior hood-max |
| 6 | A | 72 | 130 | 3250 | 0.440 | 0.5 | 132.0 | A upper-center |
| 7 | A | 86 | 89 | 3500 | 0.525 | -0.5 | 129.0 | A right interior hood-max |
| 8 | A | 110 | 130 | 3875 | 0.525 | -0.5 | 135.0 | A upper-right hood-max |
| 9 | B | 72 | 24 | 4150 | 0.525 | 1.5 | 129.0 | B apex hood-max |
| 10 | B | 54 | 10 | 4150 | 0.525 | 3.0 | 129.0 | B left hood-max |
| 11 | B | 72 | 12 | 4200 | 0.525 | 1.5 | 129.0 | B center hood-max |
| 12 | B | 90 | 14 | 4450 | 0.525 | -1.5 | 134.0 | B right hood-max |
