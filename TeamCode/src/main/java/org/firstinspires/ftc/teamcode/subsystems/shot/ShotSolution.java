package org.firstinspires.ftc.teamcode.subsystems.shot;

/**
 * Result of a {@link ShotCalibrationTable#lookup(double, double)}.
 *
 * Always non-null. When the robot pose is exactly on a calibration sample the
 * result has one source index with weight 1.0 and {@code extrapolated == false};
 * otherwise it is a convex blend of the k nearest samples by inverse-distance
 * weighting. {@code extrapolated} is a soft flag meaning the nearest calibrated
 * sample is farther away than the warn threshold (telemetry hint only; the
 * values are still the best blend the table can produce).
 */
public final class ShotSolution {
    public final double rpm;
    public final double hoodPos;
    public final double aimX;
    public final double aimY;
    public final int[] sourceIdxs;
    public final double[] weights;
    public final boolean extrapolated;
    public final double nearestDistanceIn;

    public ShotSolution(double rpm,
                        double hoodPos,
                        double aimX,
                        double aimY,
                        int[] sourceIdxs,
                        double[] weights,
                        boolean extrapolated,
                        double nearestDistanceIn) {
        this.rpm = rpm;
        this.hoodPos = hoodPos;
        this.aimX = aimX;
        this.aimY = aimY;
        this.sourceIdxs = sourceIdxs;
        this.weights = weights;
        this.extrapolated = extrapolated;
        this.nearestDistanceIn = nearestDistanceIn;
    }
}
