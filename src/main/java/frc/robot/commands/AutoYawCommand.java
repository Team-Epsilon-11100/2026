package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constAutoAim;
import frc.robot.Constants.constTurret;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utils.CalculateYaw;

/**
 * Continuously aims the turret using one of two modes:
 *
 * TEST MODE (constAutoAim.useTagYawForTesting = true):
 *   Aims directly at the closest visible AprilTag. Simple and useful for verifying
 *   turret mechanics, PID, and coordinate system without relying on odometry.
 *   Holds last known angle when no tag is visible.
 *
 * COMPETITION MODE (constAutoAim.useTagYawForTesting = false):
 *   Uses drivetrain odometry fused with vision to determine the robot's field position,
 *   then selects the correct target:
 *     - Outside neutral zone: aim straight at alliance HUB center.
 *     - Inside neutral zone (X between 4.59–11.95m, between the BUMPS): aim at the
 *       lateral ferry midpoint closest to the nearest guardrail, so the ball arcs
 *       safely into the HUB from the side.
 *   Uses direct field-geometry trig with camera-heading as the primary heading source.
 *
 * Both modes apply the same ±180° angle wrapping used by the manual POV nudge buttons.
 */
public class AutoYawCommand extends Command {

    private final Turret turret;
    private final Drivetrain drivetrain;

    // Holds the last commanded angle so the turret doesn't snap to 0 when tags are lost
    private double lastTargetAngle = 0.0;
    public AutoYawCommand(Turret turret, Drivetrain drivetrain) {
        this.turret = turret;
        this.drivetrain = drivetrain;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        // Seed lastTargetAngle from the turret's actual current position so that
        // if no tag is ever seen the turret holds where it physically is, not 0°.
        lastTargetAngle = turret.getAngle();
        SmartDashboard.putString("AutoYaw/Status", "Active");
        System.out.println("AutoYaw: started (mode=" +
            (constAutoAim.useTagYawForTesting ? "TAG-TEST" : "COMPETITION") + ")");
    }

    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getPose();
        Pose2d visionPose = Vision.getLatestVisionPose();
        // Prefer camera-derived heading when available (immune to gyro reset effects).
        double headingDeg = visionPose != null
                ? visionPose.getRotation().getDegrees()
                : robotPose.getRotation().getDegrees();

        // Log both so you can verify which one is being used
        SmartDashboard.putNumber("AutoYaw/VisionHeadingDeg", visionPose != null ? visionPose.getRotation().getDegrees() : Double.NaN);
        SmartDashboard.putNumber("AutoYaw/GyroHeadingDeg",   robotPose.getRotation().getDegrees());
        SmartDashboard.putBoolean("AutoYaw/UsingVisionHeading", visionPose != null);

        double targetAngle;
        double targetX;
        double targetY;

        if (constAutoAim.useTagYawForTesting) {
            // ── TEST MODE ────────────────────────────────────────────────────────────
            // Aim at the closest visible AprilTag using vision only.
            Pose3d tag = Vision.getClosestVisibleTag(robotPose);

            if (tag == null) {
                // No tag visible — hold last angle
                turret.setAngle(lastTargetAngle);
                SmartDashboard.putString("AutoYaw/Status", "TEST: No tag - holding last");
                return;
            }
            targetX = tag.getX();
            targetY = tag.getY();
            targetAngle = CalculateYaw.calculateTurretYawSetpoint(
                    robotPose.getX(),
                    robotPose.getY(),
                    targetX,
                    targetY,
                    headingDeg,
                    constTurret.turretAngleOffsetDegrees);
            SmartDashboard.putString("AutoYaw/Status", "TEST: Tracking tag");

        } else {
            // ── COMPETITION MODE ─────────────────────────────────────────────────────
            // Select target from odometry position: HUB or ferry point in neutral zone.
            double robotX = robotPose.getX();
            double robotY = robotPose.getY();

            boolean isRed = DriverStation.getAlliance()
                .map(a -> a == Alliance.Red)
                .orElse(false);

            Translation3d target3d = selectTarget(robotX, robotY, isRed);
            targetX = target3d.getX();
            targetY = target3d.getY();

            targetAngle = CalculateYaw.calculateTurretYawSetpoint(
                    robotX,
                    robotY,
                    targetX,
                    targetY,
                    headingDeg,
                    constTurret.turretAngleOffsetDegrees);

            boolean inNeutralZone = isInNeutralZone(robotX);
            SmartDashboard.putString("AutoYaw/Status",        inNeutralZone ? "COMP: Ferry" : "COMP: HUB");
            SmartDashboard.putBoolean("AutoYaw/InNeutralZone", inNeutralZone);
            SmartDashboard.putNumber("AutoYaw/TargetX",        targetX);
            SmartDashboard.putNumber("AutoYaw/TargetY",        targetY);
        }

        // Keep target in turret's expected wrapping interval
        targetAngle = CalculateYaw.normalizeTo180(targetAngle);

        turret.setAngle(targetAngle);
        lastTargetAngle = targetAngle;

        SmartDashboard.putNumber("AutoYaw/TargetAngle",      targetAngle);
        SmartDashboard.putNumber("AutoYaw/CurrentAngle",     turret.getAngle());
        SmartDashboard.putNumber("AutoYaw/RobotHeadingDeg",  headingDeg);
    }

    /**
     * Selects the 3D target for competition mode.
     *
     * Neutral zone (X between the BUMPS): aim at the ferry midpoint on whichever
     * side of the HUB is closer to the robot's current Y (nearest guardrail).
     * Outside neutral zone: aim straight at alliance HUB center.
     */
    private Translation3d selectTarget(double robotX, double robotY, boolean isRed) {
        if (isInNeutralZone(robotX)) {
            boolean closerToRight = robotY < (constAutoAim.fieldWidth / 2.0); // right = y=0 side
            if (isRed) {
                return closerToRight ? constAutoAim.redFerryPointRight : constAutoAim.redFerryPointLeft;
            } else {
                return closerToRight ? constAutoAim.blueFerryPointRight : constAutoAim.blueFerryPointLeft;
            }
        }
        return isRed ? constAutoAim.redHubPosition : constAutoAim.blueHubPosition;
    }

    /** True when the robot X is between the two BUMPS (the neutral/danger zone). */
    private boolean isInNeutralZone(double robotX) {
        return robotX >= constAutoAim.neutralZoneMinX && robotX <= constAutoAim.neutralZoneMaxX;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("AutoYaw/Status", "Stopped");
        System.out.println("AutoYaw: stopped");
    }
}
