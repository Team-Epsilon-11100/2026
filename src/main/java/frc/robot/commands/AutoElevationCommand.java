package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constAutoAim;
import frc.robot.Constants.constBallisticSolver;
import frc.robot.Constants.constHood;
import frc.robot.Constants.constTurret;
import frc.robot.Constants.constVision;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utils.BallisticSolver;
import frc.robot.utils.BallisticSolver.Solution;
import frc.robot.utils.BallisticSolver.Config;

/**
 * Continuously calculates and applies auto-aiming flywheel adjustments while keeping
 * the hood fixed at a steep angle. Runs as a default command, updating every cycle.
 *
 * Uses the ballistic solver: sweeps RPM low->high, tries the highest launch angle at each
 * RPM, and returns the first valid descending solution.
 */
public class AutoElevationCommand extends Command {
    private final Hood hood;
    private final Flywheel flywheel;
    private final Drivetrain drivetrain;
    
    // Reusable config object to avoid creating new objects every cycle
    private final Config config;
    
    // Fixed steep hood setting (mechanism angle), plus last valid RPM
    private final double fixedHoodAngleDeg;
    private double lastTargetRpm = 5500.0; // Start at preferred RPM

    // Enable/disable flag - toggled by POV up
    private boolean enabled = true;
    
    // Throttle dashboard updates (update every N cycles to reduce overhead)
    private int updateCounter = 0;
    private static final int UPDATE_PERIOD = 5; // Update dashboard every 5 cycles (~100ms)
    private Rotation2d headingBias = new Rotation2d();
    private Rotation2d lastOdomHeading = null;
    private static final double ODOM_RESET_JUMP_DEG = 35.0;
    // Mirror minMotorRPM from constants so the valid-RPM check stays in sync
    private static final double MIN_VALID_RPM = constBallisticSolver.minMotorRPM;
    
    public AutoElevationCommand(Hood hood, Flywheel flywheel, Drivetrain drivetrain) {
        this.hood = hood;
        this.flywheel = flywheel;
        this.drivetrain = drivetrain;
        
        // Create config once - uses values from Constants.constBallisticSolver
        this.config = new Config();

        // Lock solver to a single steep launch angle so hood does not move each cycle.
        double fixedLaunchAngleDeg = constHood.maxLaunchAngleSoftDegrees;
        this.config.minAngleDeg = fixedLaunchAngleDeg;
        this.config.maxAngleDeg = fixedLaunchAngleDeg;

        this.fixedHoodAngleDeg = constHood.launchAngleToHoodAngleDeg(fixedLaunchAngleDeg);
        // Require both Hood and Flywheel subsystems
        addRequirements(hood, flywheel);
    }
    
    /** Toggle flywheel/auto-aim on or off without cancelling the default command. */
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        if (!enabled) {
            flywheel.stop();
            SmartDashboard.putString("AutoElev/Status", "Disabled");
        }
    }

    @Override
    public void initialize() {
        hood.setAngle(fixedHoodAngleDeg);
        headingBias = new Rotation2d();
        lastOdomHeading = null;
        SmartDashboard.putString("AutoElev/Status", "Active");
        System.out.println("AutoElevation: Continuous auto-aiming started");
    }
    
    @Override
    public void execute() {
        if (!enabled) return; // Flywheel disabled - do nothing, motor already stopped in setEnabled()

        // Use fresh vision when available; preserve heading continuity across gyro resets.
        Pose2d odomPose = drivetrain.getPose();
        Pose2d visionPose = Vision.getLatestVisionPose();
        double visionAgeSec = Timer.getFPGATimestamp() - Vision.getLatestVisionTimestamp();
        boolean hasFreshVisionPose = visionPose != null && visionAgeSec <= constVision.latestVisionMaxAgeSec;

        Rotation2d odomHeading = odomPose.getRotation();
        boolean odomResetDetected = false;
        if (lastOdomHeading != null) {
            Rotation2d odomDelta = odomHeading.minus(lastOdomHeading);
            if (Math.abs(odomDelta.getDegrees()) > ODOM_RESET_JUMP_DEG) {
                headingBias = headingBias.minus(odomDelta);
                odomResetDetected = true;
            }
        }
        lastOdomHeading = odomHeading;

        if (hasFreshVisionPose) {
            headingBias = visionPose.getRotation().minus(odomHeading);
        }

        Translation2d robotTranslation = hasFreshVisionPose
            ? visionPose.getTranslation()
            : odomPose.getTranslation();
        Rotation2d robotHeading = odomHeading.plus(headingBias);
        Pose2d poseForAim = new Pose2d(robotTranslation, robotHeading);

        // Compute shooter/turret center field position from robot center + robot-frame offset.
        Translation2d shooterOffsetRobot = new Translation2d(
            constTurret.shooterOffsetXMeters,
            constTurret.shooterOffsetYMeters
        );
        Translation2d shooterPosField = robotTranslation.plus(
            shooterOffsetRobot.rotateBy(robotHeading)
        );
        
        // Get closest visible tag using the same pose basis as aiming
        Pose3d targetPose = Vision.getClosestVisibleTag(poseForAim);
        
        // Increment update counter
        updateCounter++;
        boolean shouldUpdateDashboard = (updateCounter >= UPDATE_PERIOD);
        if (shouldUpdateDashboard) {
            updateCounter = 0;
        }
        
        if (targetPose == null) {
            // No tags visible - maintain last settings and ALWAYS command motors
            if (shouldUpdateDashboard) {
                SmartDashboard.putString("AutoElev/Status", "No tags visible - using last");
            }
            
            // ALWAYS command motors with last valid settings
            hood.setAngle(fixedHoodAngleDeg);
            flywheel.setFlywheelRpm(lastTargetRpm);
            return;
        }

        // Calculate horizontal distance to target
    double xMeters = targetPose.getX() - shooterPosField.getX();
    double yMeters = targetPose.getY() - shooterPosField.getY();
        
        // Determine goal height based on testing/competition mode
        double goalZMeters;
        if (constAutoAim.useTagCenterForTesting) {
            // TESTING: Aim directly at the AprilTag center (absolute Z position from field)
            goalZMeters = targetPose.getZ();
        } else {
            // COMPETITION: Select the same 3D target that AutoYawCommand uses —
            // HUB center normally, or the nearest ferry point when in the neutral zone.
            // This ensures elevation and yaw always agree on the target.
            boolean isRedAlliance = DriverStation.getAlliance()
                .map(a -> a == Alliance.Red)
                .orElse(false);
            Translation3d target = selectTarget(poseForAim.getX(), poseForAim.getY(), isRedAlliance);
            xMeters    = target.getX() - shooterPosField.getX();
            yMeters    = target.getY() - shooterPosField.getY();
            goalZMeters = target.getZ();
        }
        
    // Solve ballistics: prefer highest launch angle at each RPM
        Solution s = BallisticSolver.solveLowestRpmPreferImpact(
            xMeters, 
            yMeters, 
            goalZMeters,
            config
        );

        if (s.valid() && s.motorRpm() > MIN_VALID_RPM) {
            // Valid solution found - apply to subsystems immediately
            lastTargetRpm = s.motorRpm();
            
            hood.setAngle(fixedHoodAngleDeg);
            flywheel.setFlywheelRpm(lastTargetRpm);
            
            // Update dashboard only periodically to reduce overhead
            if (shouldUpdateDashboard) {
                SmartDashboard.putString("AutoElev/Status", "Tracking");
                SmartDashboard.putNumber("AutoElev/TargetAngle", fixedHoodAngleDeg);
                SmartDashboard.putNumber("AutoElev/LaunchAngle", s.launchAngleDeg());
                SmartDashboard.putNumber("AutoElev/TargetRPM", lastTargetRpm);
                SmartDashboard.putNumber("AutoElev/Distance", Math.hypot(xMeters, yMeters));
                SmartDashboard.putNumber("AutoElev/ExitSpeed", s.exitSpeedMps());
                SmartDashboard.putNumber("AutoElev/ImpactAngle", s.impactAngleDeg());
                SmartDashboard.putNumber("AutoElev/ShooterFieldX", shooterPosField.getX());
                SmartDashboard.putNumber("AutoElev/ShooterFieldY", shooterPosField.getY());
                SmartDashboard.putBoolean("AutoElev/UsingVisionHeading", hasFreshVisionPose);
                SmartDashboard.putBoolean("AutoElev/UsingVisionTranslation", hasFreshVisionPose);
                SmartDashboard.putNumber("AutoElev/VisionPoseAgeSec", visionAgeSec);
                SmartDashboard.putBoolean("AutoElev/OdomResetDetected", odomResetDetected);
                SmartDashboard.putNumber("AutoElev/HeadingBiasDeg", headingBias.getDegrees());
            }
        } else {
            // No valid solution or RPM too low - use last valid settings
            hood.setAngle(fixedHoodAngleDeg);
            flywheel.setFlywheelRpm(lastTargetRpm);
            
            if (shouldUpdateDashboard) {
                String reason = s.valid() ? "RPM too low" : s.reason();
                SmartDashboard.putString("AutoElev/Status", "No solution: " + reason + " - using last");
            }
        }
    }
    
    @Override
    public boolean isFinished() {
        // Never finish - runs continuously as default command
        return false;
    }

    /**
     * Mirrors AutoYawCommand.selectTarget so both commands always agree on the target.
     * Hub center normally; nearest ferry point when in the neutral zone.
     */
    private Translation3d selectTarget(double robotX, double robotY, boolean isRed) {
        if (robotX >= constAutoAim.neutralZoneMinX && robotX <= constAutoAim.neutralZoneMaxX) {
            boolean closerToRight = robotY < (constAutoAim.fieldWidth / 2.0);
            if (isRed) {
                return closerToRight ? constAutoAim.redFerryPointRight : constAutoAim.redFerryPointLeft;
            } else {
                return closerToRight ? constAutoAim.blueFerryPointRight : constAutoAim.blueFerryPointLeft;
            }
        }
        return isRed ? constAutoAim.redHubPosition : constAutoAim.blueHubPosition;
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("AutoElev/Status", "Stopped");
        System.out.println("AutoElevation: Continuous auto-aiming stopped");
    }
}