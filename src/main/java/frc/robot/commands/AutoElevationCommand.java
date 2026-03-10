package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constAutoAim;
import frc.robot.Constants.constBallisticSolver;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utils.BallisticSolver;
import frc.robot.utils.BallisticSolver.Solution;
import frc.robot.utils.BallisticSolver.Config;

/**
 * Continuously calculates and applies auto-aiming adjustments to the hood and flywheel
 * based on the closest visible AprilTag. Runs as a default command, updating every cycle.
 *
 * Uses the ballistic solver: sweeps RPM low->high, picks the lowest launch angle at each
 * RPM (flat launch = steep descent arc), returns the first solution whose impact angle
 * lands in the configured band [-70, -45 deg]. Falls back to best-scoring descending
 * shot if no in-band solution exists.
 */
public class AutoElevationCommand extends Command {
    private final Hood hood;
    private final Flywheel flywheel;
    private final Drivetrain drivetrain;
    
    // Reusable config object to avoid creating new objects every cycle
    private final Config config;
    
    // Track last calculated values
    private double lastTargetAngle = 25.0; // Start at safe mid-range angle
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
        SmartDashboard.putString("AutoElev/Status", "Active");
        System.out.println("AutoElevation: Continuous auto-aiming started");
    }
    
    @Override
    public void execute() {
        if (!enabled) return; // Flywheel disabled - do nothing, motor already stopped in setEnabled()

        // Get current robot pose from drivetrain odometry
        Pose2d robotPose = drivetrain.getPose();
        
        // Get closest visible tag using current robot pose
        Pose3d targetPose = Vision.getClosestVisibleTag(robotPose);
        
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
            hood.setAngle(lastTargetAngle);
            flywheel.setFlywheelRpm(lastTargetRpm * constBallisticSolver.speedMod);
            return;
        }

        // Calculate horizontal distance to target
        double xMeters = targetPose.getX() - robotPose.getX();
        double yMeters = targetPose.getY() - robotPose.getY();
        
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
            Translation3d target = selectTarget(robotPose.getX(), robotPose.getY(), isRedAlliance);
            xMeters    = target.getX() - robotPose.getX();
            yMeters    = target.getY() - robotPose.getY();
            goalZMeters = target.getZ();
        }
        
        // Solve ballistics: prefer lowest launch angle at each RPM for steepest impact arc
        Solution s = BallisticSolver.solveLowestRpmPreferImpact(
            xMeters, 
            yMeters, 
            goalZMeters,
            config
        );

        if (s.valid() && s.motorRpm() > MIN_VALID_RPM) {
            // Valid solution found - apply to subsystems immediately
            lastTargetAngle = s.launchAngleDeg();
            lastTargetRpm = s.motorRpm();
            
            hood.setAngle(lastTargetAngle);
            flywheel.setFlywheelRpm(lastTargetRpm * constBallisticSolver.speedMod); // Apply speed modifier
            
            // Update dashboard only periodically to reduce overhead
            if (shouldUpdateDashboard) {
                SmartDashboard.putString("AutoElev/Status", "Tracking");
                SmartDashboard.putNumber("AutoElev/TargetAngle", lastTargetAngle);
                SmartDashboard.putNumber("AutoElev/TargetRPM", lastTargetRpm);
                SmartDashboard.putNumber("AutoElev/Distance", Math.hypot(xMeters, yMeters));
                SmartDashboard.putNumber("AutoElev/ExitSpeed", s.exitSpeedMps());
                SmartDashboard.putNumber("AutoElev/ImpactAngle", s.impactAngleDeg());
            }
        } else {
            // No valid solution or RPM too low - use last valid settings
            hood.setAngle(lastTargetAngle);
            flywheel.setFlywheelRpm(lastTargetRpm * constBallisticSolver.speedMod);
            
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