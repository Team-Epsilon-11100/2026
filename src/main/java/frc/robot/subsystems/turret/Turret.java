package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constTurret;

public class Turret extends SubsystemBase {

    private final TalonFX turretMotor;
    private final MotionMagicExpoVoltage turretPID;

    public Turret() {
        turretMotor = new TalonFX(constTurret.turretMotorId);

        var turretMotorConfigs = new TalonFXConfiguration();

        // Set Slot 0 PID gains
        var slot0Configs = turretMotorConfigs.Slot0;
        slot0Configs.kS = 0; // Static friction compensation
        slot0Configs.kV = 0; // Velocity feedforward
        slot0Configs.kA = 0; // Acceleration feedforward
        slot0Configs.kP = constTurret.kP;
        slot0Configs.kI = constTurret.kI;
        slot0Configs.kD = constTurret.kD;

        // Set Motion Magic Expo settings
        var motionMagicConfigs = turretMotorConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 0; // Unlimited cruise velocity
        motionMagicConfigs.MotionMagicExpo_kV = 0;
        motionMagicConfigs.MotionMagicExpo_kA = 0;

        // ⚠️ CRITICAL: Software limits to prevent cable wrap
        var softwareLimitConfigs = turretMotorConfigs.SoftwareLimitSwitch;
        softwareLimitConfigs.ForwardSoftLimitEnable = true;
        softwareLimitConfigs.ForwardSoftLimitThreshold = constTurret.maxTurretMotorPos;
        softwareLimitConfigs.ReverseSoftLimitEnable = true;
        softwareLimitConfigs.ReverseSoftLimitThreshold = constTurret.minTurretMotorPos;

        turretMotor.getConfigurator().apply(turretMotorConfigs);
        turretPID = new MotionMagicExpoVoltage(0); // Start at 0° (forward)

        System.out.println("Turret initialized with software limits: [" + 
            constTurret.minTurretMotorPos + ", " + constTurret.maxTurretMotorPos + "] rotations");
    }

    /**
     * Set turret angle in degrees (robot-relative).
     * 0° = forward, positive = CCW, negative = CW
     */
    public void setAngle(double angleDegrees) {
        // Clamp angle to safe limits
        double clampedAngle = Math.max(constTurret.minTurretAngleDegrees, 
                                       Math.min(angleDegrees, constTurret.maxTurretAngleDegrees));
        setMotorPos(angleToMotorPos(clampedAngle));
    }

    /**
     * Set turret motor position directly (rotations).
     */
    public void setMotorPos(double positionRotations) {
        turretMotor.setControl(turretPID.withPosition(positionRotations));
    }

    /**
     * Get current turret angle in degrees (robot-relative).
     */
    public double getAngle() {
        return motorPosToAngle(getMotorPos());
    }

    /**
     * Get current turret motor position (rotations).
     */
    public double getMotorPos() {
        return turretMotor.getPosition().getValueAsDouble();
    }

    /**
     * Convert angle (degrees) to motor position (rotations).
     */
    private double angleToMotorPos(double angleDegrees) {
        return angleDegrees * constTurret.angleToPosFactor;
    }

    /**
     * Convert motor position (rotations) to angle (degrees).
     */
    private double motorPosToAngle(double positionRotations) {
        return positionRotations / constTurret.angleToPosFactor;
    }

    @Override
    public void periodic() {
        // Log turret status to SmartDashboard
        SmartDashboard.putNumber("Turret/AngleDeg", getAngle());
        SmartDashboard.putNumber("Turret/MotorPos", getMotorPos());
        SmartDashboard.putNumber("Turret/Current", turretMotor.getStatorCurrent().getValueAsDouble());
    }
}
