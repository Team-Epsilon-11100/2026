package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.constTurret;
import frc.robot.Constants.constVision;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class Turret extends SubsystemBase {


    TalonFXS turretYawMotor;
    Mechanism2d turretMech = new Mechanism2d(15, 15);
    MechanismRoot2d turretMechRoot = turretMech.getRoot("turret", 8.5 , 8.5);
    MechanismLigament2d turretMechYaw = turretMechRoot.append(new MechanismLigament2d("omega", 6, 0));
    MechanismLigament2d turretMechTargetYaw = turretMechRoot.append(new MechanismLigament2d("target", 6, 0));
    public double targetYaw;

    Slot1Configs turretYawConfigs = new Slot1Configs();
    
    Drivetrain drivetrain;

    public Turret(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        turretYawMotor = new TalonFXS(constTurret.turretYawMotorID);
        targetYaw = 0;
        turretYawConfigs.kP = constTurret.pYaw;
        turretYawConfigs.kI = constTurret.iYaw;
        turretYawConfigs.kD = constTurret.dYaw;
        turretYawMotor.getConfigurator().apply(turretYawConfigs);
        
    }

    @Override
    public void periodic() {
        autoYaw(10);
        // Use PositionVoltage for closed-loop PID control instead of PositionDutyCycle (open-loop)
        turretYawMotor.setControl(new PositionVoltage(Units.degreesToRotations(targetYaw)));
        turretMechYaw.setAngle(-Units.rotationsToDegrees(turretYawMotor.getPosition().getValueAsDouble()));
        turretMechTargetYaw.setAngle(-targetYaw);
        SmartDashboard.putData("Turret Mechanism", turretMech);
        SmartDashboard.putNumber("Turret Target Yaw", targetYaw);
        SmartDashboard.putNumber("Turret Current Yaw", turretYawMotor.getPosition().getValueAsDouble());

    }

    public void updateTarget(double yaw) {
        targetYaw = yaw;
    }

    public double getTargetYaw() {
        return targetYaw;
    }

    public void autoYaw(int tagId) {
        Pose2d robotPose = drivetrain.getPose();
        Pose3d goalPose = constVision.aprilTagLayout.getTagPose(tagId).get();

        // Convert 3D goal pose to 2D (use x, y coordinates)
        Pose2d goalPose2d = new Pose2d(goalPose.getX(), goalPose.getY(), goalPose.getRotation().toRotation2d());
        
        // Calculate the vector from robot to goal
        double deltaX = goalPose2d.getX() - robotPose.getX();
        double deltaY = goalPose2d.getY() - robotPose.getY();
        
        // Calculate the angle in radians using atan2
        double angleToTagRadians = Math.atan2(deltaY, deltaX);
        
        // Convert to degrees
        double angleToTagDegrees = Math.toDegrees(angleToTagRadians);
        
        // Account for robot's current heading
        double robotHeadingDegrees = robotPose.getRotation().getDegrees();
        
        // Calculate relative angle (angle from robot's perspective)
        double relativeAngleDegrees = angleToTagDegrees - robotHeadingDegrees;
        
        // Normalize to [-180, 180] range
        while (relativeAngleDegrees > 180) {
            relativeAngleDegrees -= 360;
        }
        while (relativeAngleDegrees < -180) {
            relativeAngleDegrees += 360;
        }
        
        // Set target yaw
        targetYaw = relativeAngleDegrees;
    }
}