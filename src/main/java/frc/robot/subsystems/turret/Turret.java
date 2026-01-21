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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.constTurret;
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
        
        // ===== CALCULATE SHOOTER TRAJECTORY VECTOR =====
        // Get robot pose for field-relative conversion
        Pose2d robotPose = drivetrain.getPose();
        double robotHeadingRad = Math.toRadians(robotPose.getRotation().getDegrees());
        
        // Convert yaw and pitch to radians (yaw is relative to robot)
        double turretYawRad = Math.toRadians(clampedYaw);
        double pitchRad = Math.toRadians(clampedPitch);
        
        // Unit vector in turret-relative direction (pitch and yaw combined)
        double horizontalDistance = Math.cos(pitchRad);  // Horizontal component magnitude
        double verticalDistance = Math.sin(pitchRad);    // Vertical component magnitude
        
        // Apply turret yaw rotation to horizontal component (robot-relative)
        double turretRelativeX = horizontalDistance * Math.cos(turretYawRad);
        double turretRelativeY = horizontalDistance * Math.sin(turretYawRad);
        
        // Convert to field-relative by rotating by robot heading
        double fieldRelativeX = turretRelativeX * Math.cos(robotHeadingRad) - turretRelativeY * Math.sin(robotHeadingRad);
        double fieldRelativeY = turretRelativeX * Math.sin(robotHeadingRad) + turretRelativeY * Math.cos(robotHeadingRad);
        double fieldRelativeZ = verticalDistance;
        
        // Calculate end point in field space (for vector visualization)
        double trajectoryEndX = robotPose.getX() + fieldRelativeX;
        double trajectoryEndY = robotPose.getY() + fieldRelativeY;
        
        // Telemetry - Field-relative coordinates
        SmartDashboard.putNumber("Trajectory - X", fieldRelativeX);
        SmartDashboard.putNumber("Trajectory - Y", fieldRelativeY);
        SmartDashboard.putNumber("Trajectory - Z", fieldRelativeZ);
        SmartDashboard.putNumberArray("Trajectory", new double[] {trajectoryEndX, trajectoryEndY, fieldRelativeZ});
        
        // Display the target pose from constants
        Pose3d targetPose = DriverStation.getAlliance().isPresent() && 
                           DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
            ? constTurret.blueTarget
            : constTurret.redTarget;
        SmartDashboard.putNumberArray("Target Position", new double[] {targetPose.getX(), targetPose.getY(), targetPose.getZ()});
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

    /**
     * Calculates the time of flight for a projectile to reach the target.
     * Uses the horizontal distance and launch angle to determine flight time.
     * 
     * @param distance Horizontal distance to target (m)
     * @param launchAngleDegrees Launch angle in degrees
     * @return Time of flight in seconds
     */
    private double calculateTimeOfFlight(double distance, double launchAngleDegrees) {
        double launchAngleRad = Math.toRadians(launchAngleDegrees);
        double horizontalVelocity = constTurret.shooterVelocity * Math.cos(launchAngleRad);
        
        if (horizontalVelocity <= 0) {
            return 0.0;  // Prevent division by zero
        }
        
        return distance / horizontalVelocity;
    }

    /**
     * Automatically calculates yaw with motion compensation.
     * Predicts where the robot will be when the projectile reaches the target,
     * accounting for robot translation and rotation.
     * Uses the alliance-specific target pose from Constants instead of AprilTag lookup.
     * 
     * @param tagId Unused - kept for signature compatibility
     */
    public void autoYaw(int tagId) {
        try {
            Pose2d robotPose = drivetrain.getPose();
            
            // Get the target pose based on alliance
            Pose3d goalPose = DriverStation.getAlliance().isPresent() && 
                             DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
                ? constTurret.blueTarget
                : constTurret.redTarget;
            
            // Get robot velocities for motion compensation
            double[] velocities = drivetrain.getFieldVelocities();
            double vx = velocities[0];  // Field-relative X velocity (m/s)
            double vy = velocities[1];  // Field-relative Y velocity (m/s)
            double omega = velocities[2];  // Angular velocity (rad/s)
            
            // Calculate base vector from robot to goal
            double deltaX = goalPose.getX() - robotPose.getX();
            double deltaY = goalPose.getY() - robotPose.getY();
            double currentDistance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
            
            // ===== MOTION COMPENSATION =====
            // Calculate compensated aim point based on robot motion
            double compensatedDeltaX = deltaX;
            double compensatedDeltaY = deltaY;
            double rotationCompensationDeg = 0.0;
            
            if (constTurret.enableMotionCompensation) {
                // Calculate time of flight (approximate using current distance and 45° angle estimate)
                double estimatedFlightTime = calculateTimeOfFlight(currentDistance, 45.0);
                
                // Total look-ahead time = system latency + flight time + base look-ahead
                double totalLookAheadTime = constTurret.systemLatencySeconds + 
                                           constTurret.lookAheadTimeSeconds + 
                                           estimatedFlightTime;
                
                // Predict future robot position based on current velocity
                double speed = Math.sqrt(vx * vx + vy * vy);
                if (speed > constTurret.minVelocityThreshold) {
                    // Adjust target to compensate for robot translation
                    // If robot moves +X, we need to aim more -X relative to current position
                    compensatedDeltaX = deltaX - (vx * totalLookAheadTime);
                    compensatedDeltaY = deltaY - (vy * totalLookAheadTime);
                }
                
                // Compensate for robot rotation
                // If robot is rotating CCW (+omega), the turret needs to lead CW (negative compensation)
                if (Math.abs(omega) > constTurret.minAngularVelocityThreshold) {
                    // Angular compensation: predict how much robot will rotate during flight
                    rotationCompensationDeg = -Math.toDegrees(omega * totalLookAheadTime);
                }
            }
            
            // Calculate the angle to the compensated target position
            double angleToTagRadians = Math.atan2(compensatedDeltaY, compensatedDeltaX);
            double angleToTagDegrees = Math.toDegrees(angleToTagRadians);
            
            // Predict future robot heading if rotating
            double predictedHeadingDegrees = robotPose.getRotation().getDegrees();
            if (constTurret.enableMotionCompensation && Math.abs(omega) > constTurret.minAngularVelocityThreshold) {
                double totalLookAheadTime = constTurret.systemLatencySeconds + constTurret.lookAheadTimeSeconds;
                predictedHeadingDegrees += Math.toDegrees(omega * totalLookAheadTime);
            }
            
            // Calculate relative angle (angle from robot's perspective)
            double relativeAngleDegrees = angleToTagDegrees - predictedHeadingDegrees + rotationCompensationDeg;
            
            // Normalize to [-180, 180] range
            while (relativeAngleDegrees > 180) {
                relativeAngleDegrees -= 360;
            }
            while (relativeAngleDegrees < -180) {
                relativeAngleDegrees += 360;
            }
            
            // Set target yaw
            setYaw(relativeAngleDegrees);
            
            // SmartDashboard telemetry
            SmartDashboard.putNumber("AutoYaw - Robot Pose X", robotPose.getX());
            SmartDashboard.putNumber("AutoYaw - Robot Pose Y", robotPose.getY());
            SmartDashboard.putNumber("AutoYaw - Tag Pose X", goalPose.getX());
            SmartDashboard.putNumber("AutoYaw - Tag Pose Y", goalPose.getY());
            SmartDashboard.putNumber("AutoYaw - Robot Vx (m/s)", vx);
            SmartDashboard.putNumber("AutoYaw - Robot Vy (m/s)", vy);
            SmartDashboard.putNumber("AutoYaw - Robot Omega (deg/s)", Math.toDegrees(omega));
            SmartDashboard.putNumber("AutoYaw - Rotation Compensation (deg)", rotationCompensationDeg);
            SmartDashboard.putNumber("AutoYaw - Calculated Target (deg)", relativeAngleDegrees);
            SmartDashboard.putString("AutoYaw - Tag Pose", goalPose.toString());
            
        } catch (Exception e) {
            System.err.println("Error in autoYaw: " + e.getMessage());
            e.printStackTrace();
        }
    }
    
    /**
     * Automatically calculates and sets the pitch (elevation/launch angle) based on the
     * robot's distance from the target using projectile motion physics.
     * Accounts for robot motion to predict the distance when projectile arrives.
     * Uses the alliance-specific target pose from Constants instead of AprilTag lookup.
     * 
     * @param tagId Unused - kept for signature compatibility
     */
    public void autoPitch(int tagId) {
        try {
            Pose2d robotPose = drivetrain.getPose();
            
            // Get the target pose based on alliance
            Pose3d goalPose = DriverStation.getAlliance().isPresent() && 
                             DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
                ? constTurret.blueTarget
                : constTurret.redTarget;
            
            // Get robot velocities
            double[] velocities = drivetrain.getFieldVelocities();
            double vx = velocities[0];
            double vy = velocities[1];
            
            // Calculate current vector to target
            double deltaX = goalPose.getX() - robotPose.getX();
            double deltaY = goalPose.getY() - robotPose.getY();
            double currentDistance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
            
            // ===== MOTION COMPENSATION FOR PITCH =====
            double compensatedDistance = currentDistance;
            
            if (constTurret.enableMotionCompensation) {
                double speed = Math.sqrt(vx * vx + vy * vy);
                
                if (speed > constTurret.minVelocityThreshold) {
                    // Calculate the component of velocity toward/away from target
                    // Unit vector from robot to target
                    double unitX = deltaX / currentDistance;
                    double unitY = deltaY / currentDistance;
                    
                    // Radial velocity = velocity component along robot-to-target line
                    // Positive = moving toward target, Negative = moving away
                    double radialVelocity = vx * unitX + vy * unitY;
                    
                    // Initial flight time estimate
                    double estimatedFlightTime = calculateTimeOfFlight(currentDistance, 45.0);
                    double totalLookAheadTime = constTurret.systemLatencySeconds + 
                                               constTurret.lookAheadTimeSeconds + 
                                               estimatedFlightTime;
                    
                    // Predict future distance (subtract because positive radial velocity = getting closer)
                    compensatedDistance = currentDistance - (radialVelocity * totalLookAheadTime);
                    
                    // Ensure distance stays positive
                    compensatedDistance = Math.max(0.5, compensatedDistance);  // Min 0.5m
                }
            }
            
            // Calculate launch angle using projectile motion physics with compensated distance
            double calculatedAngle = FindLaunchAngle.calculateLaunchAngle(
                constTurret.shooterVelocity,   // Launch velocity (m/s)
                compensatedDistance,            // Compensated distance to target (m)
                constTurret.shooterHeight,     // Shooter height (m)
                constTurret.targetHeight        // Target height (m)
            );
            
            // Set the calculated pitch angle
            setPitch(calculatedAngle);
            
            // SmartDashboard telemetry
            SmartDashboard.putNumber("AutoPitch - Current Distance (m)", currentDistance);
            SmartDashboard.putNumber("AutoPitch - Compensated Distance (m)", compensatedDistance);
            SmartDashboard.putNumber("AutoPitch - Calculated Angle (deg)", calculatedAngle);
            SmartDashboard.putNumber("AutoPitch - Shooter Velocity (m/s)", constTurret.shooterVelocity);
            SmartDashboard.putNumber("AutoPitch - Height Diff (m)", constTurret.targetHeight - constTurret.shooterHeight);
            SmartDashboard.putString("AutoPitch - Status", "OK");
            
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
     * This combines autoYaw() and autoPitch() for complete autonomous aiming with
     * full motion compensation for robot translation and rotation.
     * 
     * @param tagId The AprilTag ID to aim at
     */
    public void autoAim(int tagId) {
        autoYaw(tagId);
        autoPitch(tagId);
    }
}