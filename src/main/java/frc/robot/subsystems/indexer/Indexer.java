package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constIndexer;

public class Indexer extends SubsystemBase {
    private final TalonFX indexerMotor;
    private final DutyCycleOut indexerOut;
    
    public Indexer() {
        indexerMotor = new TalonFX(constIndexer.indexerMotorId);

        var motorConfigs = new TalonFXConfiguration();

        // Set PID gains for velocity control (Slot 2)
        var slot2Configs = motorConfigs.Slot2;
        slot2Configs.kS = 0; // Static friction compensation
        slot2Configs.kV = 0; // Velocity feedforward
        slot2Configs.kA = 0; // Acceleration feedforward
        slot2Configs.kP = constIndexer.kP;
        slot2Configs.kI = constIndexer.kI;
        slot2Configs.kD = constIndexer.kD;

        // Set Motion Magic settings
        var motionMagicConfigs = motorConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 0; // Unlimited cruise velocity
        motionMagicConfigs.MotionMagicExpo_kV = 0;
        motionMagicConfigs.MotionMagicExpo_kA = 0;

        indexerMotor.getConfigurator().apply(motorConfigs);
        indexerOut = new DutyCycleOut(0);
    }

    /**
     * Set the indexer motor duty cycle.
     * 
     * @param dutyCycle Target duty cycle (-1.0 to 1.0)
     */
    public void setDutyCycle(double dutyCycle) {
        indexerMotor.setControl(indexerOut.withOutput(dutyCycle));
    }

    /**
     * Stop the indexer motor.
     */
    public void stop() {
        setDutyCycle(0);
    }

    @Override
    public void periodic() {
        // Log indexer status to SmartDashboard
        SmartDashboard.putNumber("Indexer/Current", indexerMotor.getSupplyCurrent().getValueAsDouble());
    }
}