package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constTurret;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utils.CalculateYaw;

/**
 * Continuously aims the turret at the closest visible AprilTag with predictive lookahead.
 * Runs as a default command on the turret subsystem.
 */
public class AutoYawCommand extends Command {

    private final Turret turret;
    private final Drivetrain drivetrain;
    
    // Track last valid angle
    private double lastValidAngle = 0.0; // Start facing forward

    public AutoYawCommand(Turret turret, Drivetrain drivetrain) {
        this.turret = turret;
        this.drivetrain = drivetrain;
        
        // Require turret subsystem
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("AutoYaw/Status", "Active");
        System.out.println("AutoYaw: Continuous turret aiming started");
    }

    @Override
    public void execute() {
        // Get current robot state
        Pose2d robotPose = drivetrain.getPose();
        double[] velocities = drivetrain.getFieldVelocities(); // [vx, vy, omega]
        
        // Get target from vision
        Pose3d target = Vision.getClosestVisibleTag(robotPose);
        
        if (target == null) {
            // No tags visible - maintain last angle
            turret.setAngle(lastValidAngle);
            SmartDashboard.putString("AutoYaw/Status", "No tags - holding last");
            return;
        }

        // Calculate turret aim with lookahead
        CalculateYaw.AimAngles aim = CalculateYaw.aimWithLookahead(
            robotPose.getTranslation(),    // Robot position
            robotPose.getRotation(),       // Robot heading
            velocities[0],                 // vx (m/s)
            velocities[1],                 // vy (m/s)
            velocities[2],                 // omega (rad/s)
            target.getTranslation().toTranslation2d(),  // Target 2D position
            constTurret.lookaheadTimeMs / 1000.0 // Convert ms to seconds
        );

        // Update turret angle
        double targetAngle = aim.robotRelativeAngle().getDegrees();
        turret.setAngle(targetAngle);
        lastValidAngle = targetAngle;
        
        // Update dashboard
        SmartDashboard.putString("AutoYaw/Status", "Tracking");
        SmartDashboard.putNumber("AutoYaw/TargetAngle", targetAngle);
        SmartDashboard.putNumber("AutoYaw/CurrentAngle", turret.getAngle());
    }

    @Override
    public boolean isFinished() {
        // Never finish - runs continuously as default command
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("AutoYaw/Status", "Stopped");
        System.out.println("AutoYaw: Continuous turret aiming stopped");
    }
}

