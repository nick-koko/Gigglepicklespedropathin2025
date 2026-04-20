package org.firstinspires.ftc.teamcode.subsystems.shot;

/**
 * One calibration waypoint for the shot-calibration table.
 *
 * All coordinates are in field inches using the Pedro-pathing coordinate system.
 * A sample binds a pinned robot location (x, y) to the RPM, hood position, and
 * turret aim point that the operator found produced a good shot at that spot.
 * Per-point aim lets the table encode things a single goal pose cannot, like
 * the observation that shots from the left side of zone A want a slight
 * rightward aim offset and vice versa.
 */
public final class ShotSample {
    public final int index;
    public final double x;
    public final double y;
    public final double rpm;
    public final double hoodPos;
    public final double aimX;
    public final double aimY;
    public final String zone;
    public final String notes;

    public ShotSample(int index,
                      double x,
                      double y,
                      double rpm,
                      double hoodPos,
                      double aimX,
                      double aimY,
                      String zone,
                      String notes) {
        this.index = index;
        this.x = x;
        this.y = y;
        this.rpm = rpm;
        this.hoodPos = hoodPos;
        this.aimX = aimX;
        this.aimY = aimY;
        this.zone = zone;
        this.notes = notes;
    }

    public double distanceTo(double botX, double botY) {
        double dx = x - botX;
        double dy = y - botY;
        return Math.sqrt(dx * dx + dy * dy);
    }
}
