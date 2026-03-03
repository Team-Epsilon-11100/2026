package frc.robot.subsystems.kicker;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constKicker;

public class Kicker extends SubsystemBase {
    private final TalonFX kickerMotor;
    private final MotionMagicVelocityVoltage velocityControl;
    
    public Kicker() {
        kickerMotor = new TalonFX(constKicker.kickerMotorId);

        var motorConfigs = new TalonFXConfiguration();

        // Set PID gains for velocity control (Slot 1)
        var slot2Configs = motorConfigs.Slot2;
        slot2Configs.kS = 0; // Static friction compensation
        slot2Configs.kV = 0; // Velocity feedforward
        slot2Configs.kA = 0; // Acceleration feedforward
        slot2Configs.kP = constKicker.kP;
        slot2Configs.kI = constKicker.kI;
        slot2Configs.kD = constKicker.kD;

        // Set Motion Magic settings
        var motionMagicConfigs = motorConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 0; // Unlimited cruise velocity
        motionMagicConfigs.MotionMagicExpo_kV = 0;
        motionMagicConfigs.MotionMagicExpo_kA = 0;

        kickerMotor.getConfigurator().apply(motorConfigs);
        velocityControl = new MotionMagicVelocityVoltage(0).withSlot(2);
    }

    /**
     * Set the kicker motor RPM.
     * 
     * @param rpm Target RPM (positive = forward, negative = reverse)
     */
    public void setRpm(double rpm) {
        // Convert RPM to rotations per second (RPS)
        kickerMotor.setControl(velocityControl.withVelocity(rpm / 60.0));
    }

    /**
     * Get the current kicker motor RPM.
     * 
     * @return Current RPM
     */
    public double getRpm() {
        return kickerMotor.getVelocity().getValueAsDouble() * 60.0;
    }

    /**
     * Stop the kicker motor.
     */
    public void stop() {
        setRpm(0);
    }

    @Override
    public void periodic() {
        // Log kicker status to SmartDashboard
        SmartDashboard.putNumber("Kicker/RPM", getRpm());
        SmartDashboard.putNumber("Kicker/Current", kickerMotor.getSupplyCurrent().getValueAsDouble());
    }
}

