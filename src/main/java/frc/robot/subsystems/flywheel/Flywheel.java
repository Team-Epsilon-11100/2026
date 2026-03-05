package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constFlywheel;

public class Flywheel extends SubsystemBase {
    TalonFX flywheelMotor;
    MotionMagicVelocityVoltage flywheelPID;
    
    public Flywheel() {
        flywheelMotor = new TalonFX(constFlywheel.flywheelMotorId);

        var flywheelMotorConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = flywheelMotorConfigs.Slot0;
        slot0Configs.kS = constFlywheel.kS; // Static friction compensation
        slot0Configs.kV = constFlywheel.kV; // Velocity feedforward
        slot0Configs.kA = constFlywheel.kA; // Acceleration feedforward
        slot0Configs.kP = constFlywheel.kP; // Proportional gain
        slot0Configs.kI = constFlywheel.kI; // Integral gain
        slot0Configs.kD = constFlywheel.kD; // Derivative gain

        // set Motion Magic Expo settings
        var motionMagicConfigs = flywheelMotorConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 0; // Unlimited cruise velocity
        motionMagicConfigs.MotionMagicExpo_kV = 0; 
        motionMagicConfigs.MotionMagicExpo_kA = 0; 
        motionMagicConfigs.MotionMagicAcceleration = constFlywheel.maxFlywheelRPM / 60.0 * 2; // Convert max RPM to RPS and set as acceleration limit
        flywheelMotor.getConfigurator().apply(flywheelMotorConfigs);
        flywheelPID = new MotionMagicVelocityVoltage(0).withSlot(0);
        
        System.out.println("Flywheel initialized on motor ID: " + constFlywheel.flywheelMotorId);
    }

    public void setFlywheelRpm(double rpm) {
        // Convert RPM to RPS for motor control
        flywheelMotor.setControl(flywheelPID.withVelocity(-rpm / 60.0));
    }

    public double getFlywheelRpm() {
        return -flywheelMotor.getVelocity().getValueAsDouble() * 60.0;
    }
    
    public void stop() {
        setFlywheelRpm(0);
    }
    
    @Override
    public void periodic() {
        // Log flywheel status to SmartDashboard
        SmartDashboard.putNumber("Flywheel/RPM", getFlywheelRpm());
        SmartDashboard.putNumber("Flywheel/Current", flywheelMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Flywheel/Voltage", flywheelMotor.getMotorVoltage().getValueAsDouble());
    }
}
