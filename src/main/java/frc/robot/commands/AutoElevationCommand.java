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

    /** Latest commanded flywheel target RPM from ballistic solver path. */
    public double getTargetRpm() {
        return lastTargetRpm;
    }

    @Override
    public void initialize() {
        hood.setAngle(fixedHoodAngleDeg);
        SmartDashboard.putString("AutoElev/Status", "Active");
        System.out.println("AutoElevation: Continuous auto-aiming started");
    }
    
    @Override
    public void execute() {
        if (!enabled) return; // Flywheel disabled - do nothing, motor already stopped in setEnabled()

        // Prefer fresh vision pose, but fall back to odometry pose if vision is stale.
        Pose2d odomPose = drivetrain.getPose();
        Pose2d visionPose = Vision.getLatestVisionPose();
        double visionTimestamp = Vision.getLatestVisionTimestamp();
        double visionAgeSec = Timer.getFPGATimestamp() - visionTimestamp;
        boolean hasFreshVisionPose = visionPose != null && visionAgeSec <= constVision.latestVisionMaxAgeSec;
        SmartDashboard.putNumber("AutoElev/GyroHeadingDeg", odomPose.getRotation().getDegrees());

        Pose2d poseForAim = hasFreshVisionPose ? visionPose : odomPose;
        Translation2d robotTranslation = poseForAim.getTranslation();
        Rotation2d robotHeading = poseForAim.getRotation();
        // Compute shooter/turret center field position from robot center + robot-frame offset.
        Translation2d shooterOffsetRobot = new Translation2d(
            constTurret.shooterOffsetXMeters,
            constTurret.shooterOffsetYMeters
        );
        Translation2d shooterPosField = robotTranslation.plus(
            shooterOffsetRobot.rotateBy(robotHeading)
        );
        
        // Increment update counter
        updateCounter++;
        boolean shouldUpdateDashboard = (updateCounter >= UPDATE_PERIOD);
        if (shouldUpdateDashboard) {
            updateCounter = 0;
        }
        
        Translation3d hubTarget = getAllianceHubTarget();
        double xMeters = hubTarget.getX() - shooterPosField.getX();
        double yMeters = hubTarget.getY() - shooterPosField.getY();
        double goalZMeters = hubTarget.getZ();
        
    // Solve ballistics: prefer highest launch angle at each RPM
        Solution s = BallisticSolver.solveLowestRpmPreferImpact(
            xMeters, 
            yMeters, 
            goalZMeters,
            config
        );

        if (s.valid() && s.motorRpm() > MIN_VALID_RPM) {
            // Valid solution found - apply to subsystems immediately
            lastTargetRpm = s.motorRpm() * constBallisticSolver.speedMod;
            
            hood.setAngle(fixedHoodAngleDeg);
            flywheel.setFlywheelRpm(lastTargetRpm);
            
            // Update dashboard only periodically to reduce overhead
            if (shouldUpdateDashboard) {
                SmartDashboard.putString("AutoElev/Status", hasFreshVisionPose ? "Tracking (Vision)" : "Tracking (Odom Fallback)");
                SmartDashboard.putNumber("AutoElev/TargetAngle", fixedHoodAngleDeg);
                SmartDashboard.putNumber("AutoElev/LaunchAngle", s.launchAngleDeg());
                SmartDashboard.putNumber("AutoElev/TargetRPM", lastTargetRpm);
                SmartDashboard.putNumber("AutoElev/TargetX", hubTarget.getX());
                SmartDashboard.putNumber("AutoElev/TargetY", hubTarget.getY());
                SmartDashboard.putNumber("AutoElev/TargetZ", hubTarget.getZ());
                SmartDashboard.putNumber("AutoElev/Distance", Math.hypot(xMeters, yMeters));
                SmartDashboard.putNumber("AutoElev/ExitSpeed", s.exitSpeedMps());
                SmartDashboard.putNumber("AutoElev/ImpactAngle", s.impactAngleDeg());
                SmartDashboard.putNumber("AutoElev/ShooterFieldX", shooterPosField.getX());
                SmartDashboard.putNumber("AutoElev/ShooterFieldY", shooterPosField.getY());
                SmartDashboard.putBoolean("AutoElev/UsingVisionHeading", hasFreshVisionPose);
                SmartDashboard.putBoolean("AutoElev/UsingVisionTranslation", hasFreshVisionPose);
                SmartDashboard.putNumber("AutoElev/VisionPoseAgeSec", visionAgeSec);
                SmartDashboard.putNumber("AutoElev/VisionTimestamp", visionTimestamp);
                SmartDashboard.putBoolean("AutoElev/OdomResetDetected", false);
                SmartDashboard.putNumber("AutoElev/HeadingBiasDeg", 0.0);
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

    /** Returns the current alliance hub target (always hub, never nearest tag/ferry). */
    private Translation3d getAllianceHubTarget() {
        boolean isRedAlliance = DriverStation.getAlliance()
            .map(a -> a == Alliance.Red)
            .orElse(false);
        return isRedAlliance ? constAutoAim.redHubPosition : constAutoAim.blueHubPosition;
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("AutoElev/Status", "Stopped");
        System.out.println("AutoElevation: Continuous auto-aiming stopped");
    }
}