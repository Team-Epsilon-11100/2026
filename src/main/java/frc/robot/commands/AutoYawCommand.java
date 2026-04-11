package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constAutoAim;
import frc.robot.Constants.constTurret;
import frc.robot.Constants.constVision;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;

/**
 * Continuously aims the turret at the current alliance HUB center.
 *
 * Pose source preference:
 *  - use fresh vision translation/heading when available;
 *  - otherwise fall back to odometry with heading-bias continuity through gyro resets.
 *
 * The resulting robot-relative yaw is offset and wrapped to the turret's [-180, 180]
 * operating range before commanding turret position.
 */
public class AutoYawCommand extends Command {

    private final Turret turret;
    private final Drivetrain drivetrain;
    private long executeCounter = 0;

    public AutoYawCommand(Turret turret, Drivetrain drivetrain) {
        this.turret = turret;
        this.drivetrain = drivetrain;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        executeCounter = 0;
        SmartDashboard.putString("AutoYaw/Status", "Active");
        System.out.println("AutoYaw: started (HUB target mode)");
    }

    @Override
    public void execute() {
        Pose2d odomPose = drivetrain.getPose();
        Pose2d visionPose = Vision.getLatestVisionPose();
        double visionTimestamp = Vision.getLatestVisionTimestamp();
        double visionAgeSec = Timer.getFPGATimestamp() - visionTimestamp;
        boolean hasFreshVisionPose = visionPose != null && visionAgeSec <= constVision.latestVisionMaxAgeSec;

        // Always use drivetrain heading for yaw math (gyro/odometry), even when using
        // vision translation. This prevents frozen/stale vision heading from locking target yaw.
        Rotation2d robotHeading = odomPose.getRotation();
        Translation2d robotTranslation = hasFreshVisionPose
            ? visionPose.getTranslation()
            : odomPose.getTranslation();

        executeCounter++;
        SmartDashboard.putNumber("AutoYaw/ExecuteCounter", executeCounter);

        // Aim from shooter/turret center rather than robot center.
        Translation2d shooterOffsetRobot = new Translation2d(
            constTurret.shooterOffsetXMeters,
            constTurret.shooterOffsetYMeters
        );
        Translation2d shooterPosField = robotTranslation.plus(
            shooterOffsetRobot.rotateBy(robotHeading)
        );

        // Log both so you can verify camera heading vs estimator heading.
        SmartDashboard.putNumber("AutoYaw/VisionHeadingDeg", visionPose != null ? visionPose.getRotation().getDegrees() : Double.NaN);
        SmartDashboard.putNumber("AutoYaw/GyroHeadingDeg",   odomPose.getRotation().getDegrees());
    SmartDashboard.putBoolean("AutoYaw/UsingVisionHeading", false);
        SmartDashboard.putBoolean("AutoYaw/UsingVisionTranslation", hasFreshVisionPose);
        SmartDashboard.putNumber("AutoYaw/VisionPoseAgeSec", visionAgeSec);
        SmartDashboard.putNumber("AutoYaw/VisionTimestamp", visionTimestamp);
        SmartDashboard.putNumber("AutoYaw/HeadingInnovationDeg", 0.0);
        SmartDashboard.putBoolean("AutoYaw/OdomResetDetected", false);
        SmartDashboard.putNumber("AutoYaw/HeadingBiasDeg", 0.0);

        Translation3d hubTarget = getAllianceHubTarget();

        // Direct hub-yaw logic:
        // 1) field-relative target angle = atan2(HUB_Y - robotY, HUB_X - robotX)
        // 2) robot-relative target angle = field angle - camera heading
        // 3) apply turret mounting offset
        // 4) normalize to [-180, 180]
        double robotX = shooterPosField.getX();
        double robotY = shooterPosField.getY();
        double cameraHeading = robotHeading.getDegrees();

        double dx = hubTarget.getX() - robotX;
        double dy = hubTarget.getY() - robotY;

        double fieldRelativeTargetAngle = Math.toDegrees(Math.atan2(dy, dx));
        double robotRelativeTargetAngle = fieldRelativeTargetAngle - cameraHeading;
        double rawTurretYaw = robotRelativeTargetAngle + constTurret.turretAngleOffsetDegrees;
        double targetAngle = normalizeTo180(rawTurretYaw);

        SmartDashboard.putString("AutoYaw/Status", "Tracking alliance HUB");
        SmartDashboard.putNumber("AutoYaw/TargetX", hubTarget.getX());
        SmartDashboard.putNumber("AutoYaw/TargetY", hubTarget.getY());
        SmartDashboard.putNumber("AutoYaw/TargetZ", hubTarget.getZ());
        SmartDashboard.putNumber("AutoYaw/FieldRelativeTargetAngle", fieldRelativeTargetAngle);
        SmartDashboard.putNumber("AutoYaw/RobotRelativeTargetAngle", robotRelativeTargetAngle);
        SmartDashboard.putNumber("AutoYaw/RawTurretYaw", rawTurretYaw);

        turret.setAngle(targetAngle);

        SmartDashboard.putNumber("AutoYaw/TargetAngle",      targetAngle);
        SmartDashboard.putNumber("AutoYaw/CurrentAngle",     turret.getAngle());
        SmartDashboard.putNumber("AutoYaw/RobotHeadingDeg",  robotHeading.getDegrees());
        SmartDashboard.putNumber("AutoYaw/ShooterFieldX",    shooterPosField.getX());
        SmartDashboard.putNumber("AutoYaw/ShooterFieldY",    shooterPosField.getY());
    }

    /** Returns the current alliance hub target (always hub, never nearest tag/ferry). */
    private Translation3d getAllianceHubTarget() {
        boolean isRed = DriverStation.getAlliance()
            .map(a -> a == Alliance.Red)
            .orElse(false);
        return isRed ? constAutoAim.redHubPosition : constAutoAim.blueHubPosition;
    }

    private double normalizeTo180(double angleDeg) {
        double wrapped = angleDeg;
        while (wrapped > 180.0) wrapped -= 360.0;
        while (wrapped < -180.0) wrapped += 360.0;
        return wrapped;
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
