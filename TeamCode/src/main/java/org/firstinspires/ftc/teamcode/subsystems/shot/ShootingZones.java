package org.firstinspires.ftc.teamcode.subsystems.shot;

/**
 * Legal shooting zones defined by the season's rules. Two triangles in field
 * coordinates:
 *   Zone A: (72,72), (0,144), (144,144)
 *   Zone B: (48,0),  (96,0),  (72,24)
 *
 * The zone check here uses the center of the robot pose for simplicity. The
 * actual rule is "any part of the robot inside or touching a triangle line"
 * which a 18x15 robot can satisfy while its center is slightly outside. We do
 * not model that buffer here because the zone label is used only for telemetry
 * and CSV tagging; legality is enforced by the driver watching the zone
 * readout.
 */
public final class ShootingZones {

    private ShootingZones() {}

    private static final double[][] ZONE_A = {
            {72.0, 72.0},
            {0.0, 144.0},
            {144.0, 144.0}
    };

    private static final double[][] ZONE_B = {
            {48.0, 0.0},
            {96.0, 0.0},
            {72.0, 24.0}
    };

    public static boolean inZoneA(double x, double y) {
        return pointInTriangle(x, y, ZONE_A);
    }

    public static boolean inZoneB(double x, double y) {
        return pointInTriangle(x, y, ZONE_B);
    }

    public static String zoneLabel(double x, double y) {
        if (inZoneA(x, y)) return "A";
        if (inZoneB(x, y)) return "B";
        return "OUT";
    }

    private static boolean pointInTriangle(double px, double py, double[][] tri) {
        double ax = tri[0][0], ay = tri[0][1];
        double bx = tri[1][0], by = tri[1][1];
        double cx = tri[2][0], cy = tri[2][1];
        double d1 = sign(px, py, ax, ay, bx, by);
        double d2 = sign(px, py, bx, by, cx, cy);
        double d3 = sign(px, py, cx, cy, ax, ay);
        boolean hasNeg = (d1 < 0) || (d2 < 0) || (d3 < 0);
        boolean hasPos = (d1 > 0) || (d2 > 0) || (d3 > 0);
        return !(hasNeg && hasPos);
    }

    private static double sign(double px, double py,
                               double ax, double ay,
                               double bx, double by) {
        return (px - bx) * (ay - by) - (ax - bx) * (py - by);
    }
}
