package frc.robot.utils;

/**
 * Simple yaw helper for turret targeting.
 *
 * Uses field-relative robot position, field-relative target position,
 * camera-derived robot heading, and a turret mounting offset.
 */
public final class CalculateYaw {

    private CalculateYaw() {}

    /**
     * Calculates turret yaw setpoint (degrees, robot-relative).
     *
     * @param robotX                   robot field X (m)
     * @param robotY                   robot field Y (m)
     * @param targetX                  target field X (m)
     * @param targetY                  target field Y (m)
     * @param cameraHeadingDeg         robot heading from camera odometry (deg)
     * @param turretMountingOffsetDeg  turret mechanical offset (deg)
     * @return normalized turret yaw setpoint in [-180, 180]
     */
    public static double calculateTurretYawSetpoint(
            double robotX,
            double robotY,
            double targetX,
            double targetY,
            double cameraHeadingDeg,
            double turretMountingOffsetDeg) {

        double dx = targetX - robotX;
        double dy = targetY - robotY;

        // Field-relative angle to target
        double fieldRelativeTargetAngleDeg = Math.toDegrees(Math.atan2(dy, dx));

        // Convert to robot-relative using camera heading (gyro-independent when camera pose exists)
        double robotRelativeTargetAngleDeg = fieldRelativeTargetAngleDeg - cameraHeadingDeg;

        // Apply turret mounting correction
        double rawTurretYawDeg = robotRelativeTargetAngleDeg + turretMountingOffsetDeg;

        return normalizeTo180(rawTurretYawDeg);
    }

    /** Normalize angle (degrees) into [-180, 180]. */
    public static double normalizeTo180(double angleDeg) {
        while (angleDeg > 180.0) angleDeg -= 360.0;
        while (angleDeg < -180.0) angleDeg += 360.0;
        return angleDeg;
    }
}