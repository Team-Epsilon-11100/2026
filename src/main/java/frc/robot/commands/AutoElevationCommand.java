package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constAutoAim;
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
 * OPTIMIZED for real-time performance:
 * - Uses 0.5° angle steps (72 iterations vs 720)
 * - Inlined ballistic calculations
 * - Minimized SmartDashboard updates
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
    private boolean lastSolutionValid = false;
    
    // Throttle dashboard updates (update every N cycles to reduce overhead)
    private int updateCounter = 0;
    private static final int UPDATE_PERIOD = 5; // Update dashboard every 5 cycles (~100ms)
    private static final double MIN_VALID_RPM = 1000.0; // Minimum reasonable flywheel RPM
    
    public AutoElevationCommand(Hood hood, Flywheel flywheel, Drivetrain drivetrain) {
        this.hood = hood;
        this.flywheel = flywheel;
        this.drivetrain = drivetrain;
        
        // Create config once - uses values from Constants.constBallisticSolver
        this.config = new Config();
        
        // Require both Hood and Flywheel subsystems
        addRequirements(hood, flywheel);
    }
    
    @Override
    public void initialize() {
        SmartDashboard.putString("AutoElev/Status", "Active");
        System.out.println("AutoElevation: Continuous auto-aiming started");
    }
    
    @Override
    public void execute() {
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
            flywheel.setFlywheelRpm(lastTargetRpm);
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
            // COMPETITION: Aim at absolute goal height
            goalZMeters = constAutoAim.absoluteGoalHeightMeters;
        }
        
        // Solve ballistics using preferred RPM
        Solution s = BallisticSolver.solvePreferConstantRpm(
            xMeters, 
            yMeters, 
            goalZMeters, 
            5500,   // Preferred motor RPM for consistent shots
            config
        );

        if (s.valid() && s.motorRpm() > MIN_VALID_RPM) {
            // Valid solution found with reasonable RPM - apply to subsystems immediately
            lastTargetAngle = s.angleDeg();
            lastTargetRpm = s.motorRpm();
            lastSolutionValid = true;
            
            hood.setAngle(lastTargetAngle);
            flywheel.setFlywheelRpm(lastTargetRpm);
            
            // Update dashboard only periodically to reduce overhead
            if (shouldUpdateDashboard) {
                SmartDashboard.putString("AutoElev/Status", "Tracking");
                SmartDashboard.putNumber("AutoElev/TargetAngle", lastTargetAngle);
                SmartDashboard.putNumber("AutoElev/TargetRPM", lastTargetRpm);
                SmartDashboard.putNumber("AutoElev/Distance", Math.hypot(xMeters, yMeters));
            }
        } else {
            // No valid solution or RPM too low - use last valid settings
            hood.setAngle(lastTargetAngle);
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
    
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("AutoElev/Status", "Stopped");
        System.out.println("AutoElevation: Continuous auto-aiming stopped");
    }
}