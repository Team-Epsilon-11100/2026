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

public class AutoElevationCommand extends Command {
    private final Hood hood;
    private final Flywheel flywheel;
    private final Drivetrain drivetrain;
    
    private double targetAngle;
    private double targetRpm;
    private boolean solutionFound = false;
    
    public AutoElevationCommand(Hood hood, Flywheel flywheel, Drivetrain drivetrain) {
        this.hood = hood;
        this.flywheel = flywheel;
        this.drivetrain = drivetrain;
        
        // Only require Hood since Flywheel doesn't extend SubsystemBase
        addRequirements(hood);
    }
    
    @Override
    public void initialize() {
        // Calculate ballistic solution when command starts
        Pose2d robotPose = drivetrain.getPose();
        Pose3d targetPose = Vision.getClosestVisibleTag();
        
        if (targetPose == null) {
            System.out.println("AutoElevation: No AprilTags visible!");
            solutionFound = false;
            return;
        }

        SmartDashboard.putNumberArray("AutoElev/TargetPos", 
            new double[]{targetPose.getX(), targetPose.getY(), targetPose.getZ()});

        // Calculate horizontal distance to target
        double xMeters = targetPose.getX() - robotPose.getX();
        double yMeters = targetPose.getY() - robotPose.getY();
        
        // Determine goal height based on testing/competition mode
        double goalZMeters;
        if (constAutoAim.useTagCenterForTesting) {
            // TESTING: Aim directly at the AprilTag center
            goalZMeters = targetPose.getZ();
            SmartDashboard.putString("AutoElev/Mode", "TEST: Aiming at tag center");
        } else {
            // COMPETITION: Aim at absolute goal height
            goalZMeters = constAutoAim.absoluteGoalHeightMeters;
            SmartDashboard.putString("AutoElev/Mode", "COMP: Aiming at absolute goal height");
        }
        
        SmartDashboard.putNumber("AutoElev/GoalHeight", goalZMeters);
        
        // Create config - uses values from Constants.constBallisticSolver
        Config config = new Config();
        
        // Solve using preferred RPM (now accepts RPM directly!)
        Solution s = BallisticSolver.solvePreferConstantRpm(
            xMeters, 
            yMeters, 
            goalZMeters, 
            5500,   // Preferred motor RPM for consistent shots
            config
        );

        if (s.valid()) {
            targetAngle = s.angleDeg();
            targetRpm = s.motorRpm();
            solutionFound = true;
            
            SmartDashboard.putNumber("AutoElev/TargetAngle", targetAngle);
            SmartDashboard.putNumber("AutoElev/TargetRPM", targetRpm);
            SmartDashboard.putNumber("AutoElev/FlightTime", s.timeSec());
            
            System.out.println("AutoElevation: Solution found - Angle: " + 
                String.format("%.1f°", targetAngle) + ", RPM: " + String.format("%.0f", targetRpm));
        } else {
            solutionFound = false;
            System.out.println("AutoElevation: No ballistic solution - " + s.reason());
        }
    }
    
    @Override
    public void execute() {
        // Apply the calculated values to subsystems
        if (solutionFound) {
            hood.setAngle(targetAngle);
            flywheel.setFlywheelRpm(targetRpm);
        }
    }
    
    @Override
    public boolean isFinished() {
        // Command finishes immediately after calculating and setting values
        return !solutionFound; // Finish if no solution found
    }
    
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("AutoElevation: Command interrupted");
        }
    }
}