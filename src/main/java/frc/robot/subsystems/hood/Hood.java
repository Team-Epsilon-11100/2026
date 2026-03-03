package frc.robot.subsystems.hood;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constHood;

public class Hood extends SubsystemBase {


    TalonFX hoodMotor;
    MotionMagicExpoVoltage hoodPID;
    public Hood() {
        hoodMotor = new TalonFX(constHood.hoodMotorId);

        var hoodMotorConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = hoodMotorConfigs.Slot0;
        slot0Configs.kS = 0; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = constHood.kP; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = constHood.kI; // no output for integrated error
        slot0Configs.kD = constHood.kD; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic Expo settings
        var motionMagicConfigs = hoodMotorConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 0; // Unlimited cruise velocity
        motionMagicConfigs.MotionMagicExpo_kV = 0; // kV is around 0.12 V/rps
        motionMagicConfigs.MotionMagicExpo_kA = 0; // Use a slower kA of 0.1 V/(rps/s)

        // ⚠️ CRITICAL: Software limits to prevent mechanical damage
        var softwareLimitConfigs = hoodMotorConfigs.SoftwareLimitSwitch;
        softwareLimitConfigs.ForwardSoftLimitEnable = true;
        softwareLimitConfigs.ForwardSoftLimitThreshold = constHood.maxHoodMotorPos; // 12.5 rotations max
        softwareLimitConfigs.ReverseSoftLimitEnable = true;
        softwareLimitConfigs.ReverseSoftLimitThreshold = constHood.minHoodMotorPos; // 0.5 rotations min

        hoodMotor.getConfigurator().apply(hoodMotorConfigs);
        hoodPID = new MotionMagicExpoVoltage(constHood.minHoodMotorPos);
        
        System.out.println("Hood initialized with software limits: [" + 
            constHood.minHoodMotorPos + ", " + constHood.maxHoodMotorPos + "] rotations");
    }

    public void setMotorPos(double position) {
        hoodMotor.setControl(hoodPID.withPosition(position));
    }
    public void setAngle(double angle) {
        // Clamp angle to safe limits BEFORE converting to motor position
        double clampedAngle = Math.max(constHood.minHoodAngleDegrees, 
                                       Math.min(angle, constHood.maxHoodAngleDegrees));
        setMotorPos(angleToMotorPos(clampedAngle));
    }

    public double getMotorPos() {
        return hoodMotor.getPosition().getValueAsDouble();
    }

    public double getAngle() {
        return motorPosToAngle(getMotorPos());
    }

    public double angleToMotorPos(double angle) {
        return constHood.minHoodMotorPos +
            (angle - constHood.minHoodAngleDegrees) * constHood.angleToPosFactor;
    }

    public double motorPosToAngle(double motorPos) {
        return constHood.minHoodAngleDegrees +
            (motorPos - constHood.minHoodMotorPos) / constHood.angleToPosFactor;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
