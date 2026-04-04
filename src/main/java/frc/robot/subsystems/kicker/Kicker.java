package frc.robot.subsystems.kicker;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constKicker;

public class Kicker extends SubsystemBase {
    private final TalonFX kickerMotor;
    private final DutyCycleOut kickerOut;
    
    public Kicker() {
        kickerMotor = new TalonFX(constKicker.kickerMotorId);

        var motorConfigs = new TalonFXConfiguration();

     
        // Set Motion Magic settings
        var motionMagicConfigs = motorConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 0; // Unlimited cruise velocity
        motionMagicConfigs.MotionMagicExpo_kV = 0;
        motionMagicConfigs.MotionMagicExpo_kA = 0;

        kickerMotor.getConfigurator().apply(motorConfigs);
        kickerOut = new DutyCycleOut(0);
    }

    /**
     * Set the kicker motor RPM.
     * 
     * @param rpm Target RPM (positive = forward, negative = reverse)
     */
    public void setDutyCycle(double dutyCycle) {
        // Convert RPM to rotations per second (RPS)
        kickerMotor.setControl(kickerOut.withOutput(dutyCycle));
    }



    /**
     * Stop the kicker motor.
     */
    public void stop() {
        setDutyCycle(0);
    }

    @Override
    public void periodic() {
        // Log kicker status to SmartDashboard
       
        SmartDashboard.putNumber("Kicker/Current", kickerMotor.getSupplyCurrent().getValueAsDouble());
    }
}

