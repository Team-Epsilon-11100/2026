package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constIndexer;

public class Indexer extends SubsystemBase {
    private final TalonFX indexerMotor;
    private final MotionMagicVelocityVoltage velocityControl;
    
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
        velocityControl = new MotionMagicVelocityVoltage(0).withSlot(2);
    }

    /**
     * Set the indexer motor RPM.
     * 
     * @param rpm Target RPM (positive = forward, negative = reverse)
     */
    public void setRpm(double rpm) {
        // Convert RPM to rotations per second (RPS)
        indexerMotor.setControl(velocityControl.withVelocity(rpm / 60.0));
    }

    /**
     * Get the current indexer motor RPM.
     * 
     * @return Current RPM
     */
    public double getRpm() {
        return indexerMotor.getVelocity().getValueAsDouble() * 60.0;
    }

    /**
     * Stop the indexer motor.
     */
    public void stop() {
        setRpm(0);
    }

    @Override
    public void periodic() {
        // Log indexer status to SmartDashboard
        SmartDashboard.putNumber("Indexer/RPM", getRpm());
        SmartDashboard.putNumber("Indexer/Current", indexerMotor.getSupplyCurrent().getValueAsDouble());
    }
}

