package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Turret aiming helper that leads the shot by predicting robot pose forward in time.
 *
 * Assumptions:
 * - vx, vy are FIELD-relative robot translational velocities (m/s).
 * - omegaRadPerSec is robot FIELD yaw rate (rad/s), CCW positive.
 * - goalPos is in FIELD coordinates (same frame as robotPos).
 * - tSeconds is lookahead time (seconds).
 */
public final class CalculateYaw {

    private CalculateYaw() {}

    /** Result angles for turret aiming. */
    public record AimAngles(Rotation2d fieldAngle, Rotation2d robotRelativeAngle) {}

    /**
     * Computes turret aim angles with lookahead.
     *
     * @param robotPosField     robot translation on field
     * @param robotYawField     robot rotation on field (CCW+)
     * @param vxFieldMps        robot field-relative vx (m/s)
     * @param vyFieldMps        robot field-relative vy (m/s)
     * @param omegaRadPerSec    robot yaw rate (rad/s), CCW+
     * @param goalPosField      goal translation on field
     * @param tSeconds          lookahead time (s)
     * @return AimAngles: (fieldAngle, robotRelativeAngle)
     */
    public static AimAngles aimWithLookahead(
            Translation2d robotPosField,
            Rotation2d robotYawField,
            double vxFieldMps,
            double vyFieldMps,
            double omegaRadPerSec,
            Translation2d goalPosField,
            double tSeconds
    ) {
        // Predict robot pose at t seconds in the future
        Translation2d robotPosFuture = robotPosField.plus(
                new Translation2d(vxFieldMps * tSeconds, vyFieldMps * tSeconds)
        );

    Rotation2d robotYawFuture = robotYawField.plus(
        Rotation2d.fromRadians(omegaRadPerSec * tSeconds)
    );

        // Vector from future robot position to goal (field frame)
        Translation2d toGoal = goalPosField.minus(robotPosFuture);

        // Field-relative turret angle to face goal
        Rotation2d turretFieldAngle = new Rotation2d(toGoal.getX(), toGoal.getY());

        // Turret angle relative to robot (what turret usually controls)
        Rotation2d turretRobotRelative = wrapToPi(turretFieldAngle.minus(robotYawFuture));

        return new AimAngles(turretFieldAngle, turretRobotRelative);
    }

    /** Overload if you don't want to model omega (assumes omega = 0). */
    public static AimAngles aimWithLookahead(
            Translation2d robotPosField,
            Rotation2d robotYawField,
            double vxFieldMps,
            double vyFieldMps,
            Translation2d goalPosField,
            double tSeconds
    ) {
        return aimWithLookahead(robotPosField, robotYawField, vxFieldMps, vyFieldMps, 0.0, goalPosField, tSeconds);
    }

    /** Wraps angle to [-pi, pi]. */
    private static Rotation2d wrapToPi(Rotation2d angle) {
        double r = angle.getRadians();
        r = Math.atan2(Math.sin(r), Math.cos(r));
        return Rotation2d.fromRadians(r);
    }
}