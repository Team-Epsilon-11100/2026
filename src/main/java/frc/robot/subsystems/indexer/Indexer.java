package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constIndexer;

public class Indexer extends SubsystemBase {
    private final TalonFX indexerMotor;
    private final MotionMagicVelocityVoltage indexerPID;
    private double lastTargetRpm = 0.0;
    
    public Indexer() {
        indexerMotor = new TalonFX(constIndexer.indexerMotorId);

        var motorConfigs = new TalonFXConfiguration();

        // Set slot 0 gains (velocity PID + feedforward)
        var slot0Configs = motorConfigs.Slot0;
        slot0Configs.kS = constIndexer.kS;
        slot0Configs.kV = constIndexer.kV;
        slot0Configs.kA = constIndexer.kA;
        slot0Configs.kP = constIndexer.kP;
        slot0Configs.kI = constIndexer.kI;
        slot0Configs.kD = constIndexer.kD;

        // Set Motion Magic settings
        var motionMagicConfigs = motorConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 0; // Unlimited cruise velocity
        motionMagicConfigs.MotionMagicExpo_kV = 0;
        motionMagicConfigs.MotionMagicExpo_kA = 0;
        motionMagicConfigs.MotionMagicAcceleration = constIndexer.maxIndexerRPM / 60.0 * 2;

        indexerMotor.getConfigurator().apply(motorConfigs);
        indexerPID = new MotionMagicVelocityVoltage(0).withSlot(0);

        System.out.println("Indexer initialized on motor ID: " + constIndexer.indexerMotorId);
    }

    /**
     * Set the indexer motor duty cycle.
     * 
     * @param dutyCycle Target duty cycle (-1.0 to 1.0)
     */
    public void setDutyCycle(double dutyCycle) {
        // Backward-compatible mapping from duty [-1..1] to closed-loop RPM target.
        setIndexerRpm(dutyCycle * constIndexer.targetRpm);
    }

    /** Set indexer target speed in motor RPM (velocity closed-loop). */
    public void setIndexerRpm(double rpm) {
        double clampedRpm = Math.max(constIndexer.minIndexerRPM, Math.min(rpm, constIndexer.maxIndexerRPM));
        lastTargetRpm = clampedRpm;
        indexerMotor.setControl(indexerPID.withVelocity(clampedRpm / 60.0));
    }

    /** Get measured indexer speed in motor RPM. */
    public double getIndexerRpm() {
        return indexerMotor.getVelocity().getValueAsDouble() * 60.0;
    }

    /**
     * Stop the indexer motor.
     */
    public void stop() {
        setIndexerRpm(0.0);
    }

    @Override
    public void periodic() {
        // Log indexer status to SmartDashboard
        SmartDashboard.putNumber("Indexer/TargetRPM", lastTargetRpm);
        SmartDashboard.putNumber("Indexer/RPM", getIndexerRpm());
        SmartDashboard.putNumber("Indexer/Current", indexerMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Indexer/Voltage", indexerMotor.getMotorVoltage().getValueAsDouble());
    }
}