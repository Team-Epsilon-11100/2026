package frc.robot.subsystems.turret;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.sim.TalonFXSSimState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.constTurret;
import frc.robot.Constants.constVision;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.FindLaunchAngle;

public class Turret extends SubsystemBase {

    // ===== YAW (AZIMUTH) MOTOR =====
    protected final TalonFXS turretYawMotor;
    protected final MotionMagicVoltage yawControl;
    protected double targetYawDegrees = 0.0;
    protected boolean hasYawTarget = false;
    private final TalonFXSSimState yawSimState;
    
    // ===== PITCH (ELEVATION) MOTOR =====
    protected final TalonFXS turretPitchMotor;
    protected final MotionMagicVoltage pitchControl;
    protected double targetPitchDegrees = 45.0;  // Default to 45 degree launch angle
    protected boolean hasPitchTarget = false;
    private final TalonFXSSimState pitchSimState;
    
    // ===== 2D VISUALIZATION (TOP VIEW - YAW) =====
    Mechanism2d turretMechTopView = new Mechanism2d(Units.inchesToMeters(15), Units.inchesToMeters(15));
    MechanismRoot2d turretMechRoot = turretMechTopView.getRoot("turret", Units.inchesToMeters(7.5), Units.inchesToMeters(7.5));
    MechanismLigament2d turretMechYaw = turretMechRoot.append(
        new MechanismLigament2d("yaw", Units.inchesToMeters(5), 0, 6, new Color8Bit(Color.kBlue)));
    MechanismLigament2d turretMechTargetYaw = turretMechRoot.append(
        new MechanismLigament2d("target_yaw", Units.inchesToMeters(4), 0, 2, new Color8Bit(Color.kGreen)));
    
    // ===== 2D VISUALIZATION (SIDE VIEW - PITCH) =====
    Mechanism2d turretMechSideView = new Mechanism2d(Units.inchesToMeters(15), Units.inchesToMeters(15));
    MechanismRoot2d pitchMechRoot = turretMechSideView.getRoot("pitch_base", Units.inchesToMeters(7.5), Units.inchesToMeters(2));
    MechanismLigament2d turretMechPitch = pitchMechRoot.append(
        new MechanismLigament2d("pitch", Units.inchesToMeters(6), 0, 6, new Color8Bit(Color.kRed)));
    MechanismLigament2d turretMechTargetPitch = pitchMechRoot.append(
        new MechanismLigament2d("target_pitch", Units.inchesToMeters(5), 0, 2, new Color8Bit(Color.kYellow)));
    
    Drivetrain drivetrain;

    public Turret(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        
        // ===== CONFIGURE YAW MOTOR =====
        turretYawMotor = new TalonFXS(constTurret.turretYawMotorID);
        yawControl = new MotionMagicVoltage(0);
        yawSimState = turretYawMotor.getSimState();
        
        TalonFXSConfiguration yawConfig = new TalonFXSConfiguration();
        
        // Slot 0 PID and feedforward gains for yaw
        yawConfig.Slot0.kG = constTurret.yawKG;
        yawConfig.Slot0.kS = constTurret.yawKS;
        yawConfig.Slot0.kV = constTurret.yawKV;
        yawConfig.Slot0.kA = constTurret.yawKA;
        yawConfig.Slot0.kP = constTurret.yawKP;
        yawConfig.Slot0.kI = constTurret.yawKI;
        yawConfig.Slot0.kD = constTurret.yawKD;
        
        // Motion Magic configuration for yaw
        MotionMagicConfigs yawMotionMagic = yawConfig.MotionMagic;
        yawMotionMagic.MotionMagicCruiseVelocity = constTurret.yawMotionVelocity * constTurret.rotationsPerDegree;
        yawMotionMagic.MotionMagicAcceleration = constTurret.yawMotionAcceleration * constTurret.rotationsPerDegree;
        yawMotionMagic.MotionMagicExpo_kA = constTurret.yawExpoKA;
        yawMotionMagic.MotionMagicExpo_kV = constTurret.yawExpoKV;
        
        // Software limits for yaw
        SoftwareLimitSwitchConfigs yawSoftLimits = yawConfig.SoftwareLimitSwitch;
        yawSoftLimits.ForwardSoftLimitEnable = true;
        yawSoftLimits.ReverseSoftLimitEnable = true;
        yawSoftLimits.ForwardSoftLimitThreshold = constTurret.maxYawDegrees * constTurret.rotationsPerDegree;
        yawSoftLimits.ReverseSoftLimitThreshold = constTurret.minYawDegrees * constTurret.rotationsPerDegree;
        
        turretYawMotor.getConfigurator().apply(yawConfig);
        
        // ===== CONFIGURE PITCH MOTOR =====
        turretPitchMotor = new TalonFXS(constTurret.turretPitchMotorID);
        pitchControl = new MotionMagicVoltage(0);
        pitchSimState = turretPitchMotor.getSimState();
        
        TalonFXSConfiguration pitchConfig = new TalonFXSConfiguration();
        
        // Slot 0 PID and feedforward gains for pitch (includes gravity compensation!)
        pitchConfig.Slot0.kG = constTurret.pitchKG;  // IMPORTANT: Gravity feedforward
        pitchConfig.Slot0.kS = constTurret.pitchKS;
        pitchConfig.Slot0.kV = constTurret.pitchKV;
        pitchConfig.Slot0.kA = constTurret.pitchKA;
        pitchConfig.Slot0.kP = constTurret.pitchKP;
        pitchConfig.Slot0.kI = constTurret.pitchKI;
        pitchConfig.Slot0.kD = constTurret.pitchKD;
        
        // Motion Magic configuration for pitch
        MotionMagicConfigs pitchMotionMagic = pitchConfig.MotionMagic;
        pitchMotionMagic.MotionMagicCruiseVelocity = constTurret.pitchMotionVelocity * constTurret.rotationsPerDegree;
        pitchMotionMagic.MotionMagicAcceleration = constTurret.pitchMotionAcceleration * constTurret.rotationsPerDegree;
        pitchMotionMagic.MotionMagicExpo_kA = constTurret.pitchExpoKA;
        pitchMotionMagic.MotionMagicExpo_kV = constTurret.pitchExpoKV;
        
        // Software limits for pitch (launch angle range: 0° to 90°)
        SoftwareLimitSwitchConfigs pitchSoftLimits = pitchConfig.SoftwareLimitSwitch;
        pitchSoftLimits.ForwardSoftLimitEnable = true;
        pitchSoftLimits.ReverseSoftLimitEnable = true;
        pitchSoftLimits.ForwardSoftLimitThreshold = constTurret.maxPitchDegrees * constTurret.rotationsPerDegree;
        pitchSoftLimits.ReverseSoftLimitThreshold = constTurret.minPitchDegrees * constTurret.rotationsPerDegree;
        
        turretPitchMotor.getConfigurator().apply(pitchConfig);
        
        // Initialize simulation with supply voltage
        if (Utils.isSimulation()) {
            yawSimState.setSupplyVoltage(12.0);
            pitchSimState.setSupplyVoltage(12.0);
        }
    }

    @Override
    public void periodic() {
        // Update simulation if in sim mode
        if (Utils.isSimulation()) {
            updateSimulation();
        }
        
        // Automatically calculate and set yaw and pitch to aim at tag
        autoAim(10);
        
        // ===== UPDATE YAW CONTROL =====
        // Re-assert the current MotionMagic position command only if someone requested a target
        try {
            if (hasYawTarget && !Double.isNaN(targetYawDegrees) && !Double.isInfinite(targetYawDegrees)) {
                turretYawMotor.setControl(yawControl.withPosition(targetYawDegrees * constTurret.rotationsPerDegree));
            }
        } catch (Exception e) {
            System.out.println("Turret Yaw: failed to reapply control: " + e.getMessage());
        }
        
        // ===== UPDATE PITCH CONTROL =====
        // Re-assert the current MotionMagic position command only if someone requested a target
        try {
            if (hasPitchTarget && !Double.isNaN(targetPitchDegrees) && !Double.isInfinite(targetPitchDegrees)) {
                turretPitchMotor.setControl(pitchControl.withPosition(targetPitchDegrees * constTurret.rotationsPerDegree));
            }
        } catch (Exception e) {
            System.out.println("Turret Pitch: failed to reapply control: " + e.getMessage());
        }
        
        // ===== READ CURRENT POSITIONS =====
        // Yaw position
        double currentYawRotations = turretYawMotor.getPosition().getValueAsDouble();
        double currentYawDegrees = currentYawRotations / constTurret.rotationsPerDegree;
        double clampedYaw = Math.max(constTurret.minYawDegrees, Math.min(constTurret.maxYawDegrees, currentYawDegrees));
        
        // Pitch position
        double currentPitchRotations = turretPitchMotor.getPosition().getValueAsDouble();
        double currentPitchDegrees = currentPitchRotations / constTurret.rotationsPerDegree;
        double clampedPitch = Math.max(constTurret.minPitchDegrees, Math.min(constTurret.maxPitchDegrees, currentPitchDegrees));
        
        // ===== UPDATE VISUALIZATIONS =====
        // Top view - yaw (azimuth)
        turretMechYaw.setAngle(clampedYaw);
        turretMechTargetYaw.setAngle(targetYawDegrees);
        
        // Side view - pitch (elevation)
        turretMechPitch.setAngle(clampedPitch);
        turretMechTargetPitch.setAngle(targetPitchDegrees);
        
        // ===== SMARTDASHBOARD OUTPUTS =====
        SmartDashboard.putData("Turret Top View (Yaw)", turretMechTopView);
        SmartDashboard.putData("Turret Side View (Pitch)", turretMechSideView);
        
        // Yaw data
        SmartDashboard.putNumber("Turret Target Yaw (deg)", targetYawDegrees);
        SmartDashboard.putNumber("Turret Current Yaw (deg)", clampedYaw);
        SmartDashboard.putNumber("Turret Yaw Error (deg)", targetYawDegrees - clampedYaw);
        
        // Pitch data
        SmartDashboard.putNumber("Turret Target Pitch (deg)", targetPitchDegrees);
        SmartDashboard.putNumber("Turret Current Pitch (deg)", clampedPitch);
        SmartDashboard.putNumber("Turret Pitch Error (deg)", targetPitchDegrees - clampedPitch);
        
        // Combined launch angle display
        SmartDashboard.putString("Turret Launch Angle", 
            String.format("Yaw: %.1f° | Pitch: %.1f°", clampedYaw, clampedPitch));
    }
    
    private void updateSimulation() {
        // ===== SIMULATE YAW MOTOR =====
        double currentYawRotations = turretYawMotor.getPosition().getValueAsDouble();
        double targetYawRotations = targetYawDegrees * constTurret.rotationsPerDegree;
        double yawError = targetYawRotations - currentYawRotations;
        
        double yawMaxDelta = constTurret.yawMotionVelocity * constTurret.rotationsPerDegree * 0.02;
        double yawDelta = Math.max(-yawMaxDelta, Math.min(yawMaxDelta, yawError * 0.1));
        double newYawPosition = currentYawRotations + yawDelta;
        
        yawSimState.setRawRotorPosition(newYawPosition);
        yawSimState.setRotorVelocity(yawDelta / 0.02);
        
        // ===== SIMULATE PITCH MOTOR =====
        double currentPitchRotations = turretPitchMotor.getPosition().getValueAsDouble();
        double targetPitchRotations = targetPitchDegrees * constTurret.rotationsPerDegree;
        double pitchError = targetPitchRotations - currentPitchRotations;
        
        double pitchMaxDelta = constTurret.pitchMotionVelocity * constTurret.rotationsPerDegree * 0.02;
        double pitchDelta = Math.max(-pitchMaxDelta, Math.min(pitchMaxDelta, pitchError * 0.1));
        double newPitchPosition = currentPitchRotations + pitchDelta;
        
        pitchSimState.setRawRotorPosition(newPitchPosition);
        pitchSimState.setRotorVelocity(pitchDelta / 0.02);
    }

    // ===== YAW (AZIMUTH) CONTROL METHODS =====
    /**
     * Set the turret yaw (azimuth) angle
     * @param degrees Target yaw angle in degrees (-180 to 180)
     */
    public void setYaw(double degrees) {
        double clamped = Math.max(constTurret.minYawDegrees, Math.min(constTurret.maxYawDegrees, degrees));
        targetYawDegrees = clamped;
        hasYawTarget = true;
        turretYawMotor.setControl(yawControl.withPosition(clamped * constTurret.rotationsPerDegree));
    }

    public void updateTarget(double yaw) {
        setYaw(yaw);
    }

    public double getTargetYaw() {
        return targetYawDegrees;
    }
    
    public void clearYawHold() {
        hasYawTarget = false;
    }
    
    // ===== PITCH (ELEVATION) CONTROL METHODS =====
    /**
     * Set the turret pitch (elevation) angle - THIS IS YOUR LAUNCH ANGLE
     * @param degrees Target pitch angle in degrees (0 to 90)
     *                0° = horizontal, 45° = typical launch angle, 90° = vertical
     */
    public void setPitch(double degrees) {
        double clamped = Math.max(constTurret.minPitchDegrees, Math.min(constTurret.maxPitchDegrees, degrees));
        targetPitchDegrees = clamped;
        hasPitchTarget = true;
        turretPitchMotor.setControl(pitchControl.withPosition(clamped * constTurret.rotationsPerDegree));
    }
    
    public double getTargetPitch() {
        return targetPitchDegrees;
    }
    
    public void clearPitchHold() {
        hasPitchTarget = false;
    }
    
    // ===== COMBINED CONTROL METHODS =====
    /**
     * Set both yaw and pitch simultaneously for a complete aim solution
     * @param yawDegrees Target yaw (azimuth) angle
     * @param pitchDegrees Target pitch (elevation/launch) angle
     */
    public void setAim(double yawDegrees, double pitchDegrees) {
        setYaw(yawDegrees);
        setPitch(pitchDegrees);
    }

    public void autoYaw(int tagId) {
        try {
            Pose2d robotPose = drivetrain.getPose();
            Pose3d goalPose = constVision.aprilTagLayout.getTagPose(tagId).get();
            
            // Convert 3D goal pose to 2D (use x, y coordinates)
            Pose2d goalPose2d = new Pose2d(goalPose.getX(), goalPose.getY(), goalPose.getRotation().toRotation2d());
            SmartDashboard.putString("AutoYaw - Tag Pose", goalPose.toString());
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
            
            // Set target yaw using the setYaw method (which handles clamping and hasTarget flag)
            setYaw(relativeAngleDegrees);
            
            SmartDashboard.putNumber("AutoYaw - Robot Pose X", robotPose.getX());
            SmartDashboard.putNumber("AutoYaw - Robot Pose Y", robotPose.getY());
            SmartDashboard.putNumber("AutoYaw - Tag Pose X", goalPose2d.getX());
            SmartDashboard.putNumber("AutoYaw - Tag Pose Y", goalPose2d.getY());
            SmartDashboard.putNumber("AutoYaw - Angle to Tag (deg)", angleToTagDegrees);
            SmartDashboard.putNumber("AutoYaw - Robot Heading (deg)", robotHeadingDegrees);
            SmartDashboard.putNumber("AutoYaw - Calculated Target (deg)", relativeAngleDegrees);
        } catch (Exception e) {
            System.err.println("Error in autoYaw: " + e.getMessage());
            e.printStackTrace();
        }
    }
    
    /**
     * Automatically calculates and sets the pitch (elevation/launch angle) based on the
     * robot's distance from the target AprilTag using projectile motion physics.
     * 
     * @param tagId The AprilTag ID to aim at
     */
    public void autoPitch(int tagId) {
        try {
            Pose2d robotPose = drivetrain.getPose();
            Pose3d goalPose = constVision.aprilTagLayout.getTagPose(tagId).get();
            
            // Calculate horizontal distance to target (2D distance on ground plane)
            double deltaX = goalPose.getX() - robotPose.getX();
            double deltaY = goalPose.getY() - robotPose.getY();
            double horizontalDistance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
            
            // Calculate launch angle using projectile motion physics
            double calculatedAngle = FindLaunchAngle.calculateLaunchAngle(
                constTurret.shooterVelocity,   // Launch velocity (m/s)
                horizontalDistance,             // Distance to target (m)
                constTurret.shooterHeight,     // Shooter height (m)
                constTurret.targetHeight        // Target height (m)
            );
            
            // Set the calculated pitch angle
            setPitch(calculatedAngle);
            
            // SmartDashboard telemetry
            SmartDashboard.putNumber("AutoPitch - Distance (m)", horizontalDistance);
            SmartDashboard.putNumber("AutoPitch - Calculated Angle (deg)", calculatedAngle);
            SmartDashboard.putNumber("AutoPitch - Shooter Velocity (m/s)", constTurret.shooterVelocity);
            SmartDashboard.putNumber("AutoPitch - Height Diff (m)", constTurret.targetHeight - constTurret.shooterHeight);
            
        } catch (IllegalArgumentException e) {
            // Target is out of range with current velocity
            System.err.println("AutoPitch Error: " + e.getMessage());
            SmartDashboard.putString("AutoPitch - Status", "OUT OF RANGE");
        } catch (Exception e) {
            System.err.println("Error in autoPitch: " + e.getMessage());
            e.printStackTrace();
        }
    }
    
    /**
     * Automatically calculates and sets both yaw and pitch to aim at the target AprilTag.
     * This combines autoYaw() and autoPitch() for complete autonomous aiming.
     * 
     * @param tagId The AprilTag ID to aim at
     */
    public void autoAim(int tagId) {
        autoYaw(tagId);
        autoPitch(tagId);
    }
}