package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.constFlywheel;
public class Flywheel {
    TalonFX flywheelMotor;
    MotionMagicVelocityVoltage flywheelPID;
    public Flywheel() {
        flywheelMotor = new TalonFX(constFlywheel.flywheelMotorId);

        var flywheelMotorConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = flywheelMotorConfigs.Slot0;
        slot0Configs.kS = 0; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = constFlywheel.kP; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = constFlywheel.kI; // no output for integrated error
        slot0Configs.kD = constFlywheel.kD; // A velocity error of 1 rps results in 0.1 V output

        // set Motion Magic Expo settings
        var motionMagicConfigs = flywheelMotorConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 0; // Unlimited cruise velocity
        motionMagicConfigs.MotionMagicExpo_kV = 0; // kV is around 0.12 V/rps
        motionMagicConfigs.MotionMagicExpo_kA = 0; // Use a slower kA of 0.1 V/(rps/s)

        flywheelMotor.getConfigurator().apply(flywheelMotorConfigs);
        flywheelPID = new MotionMagicVelocityVoltage(0).withSlot(0);
    }

        public void setFlywheelRpm(double rpm) {
            flywheelMotor.setControl(flywheelPID.withVelocity(rpm/60));
        }

        public double getFlywheelRpm() {
            return flywheelMotor.getVelocity().getValueAsDouble() * 60;
        }
    }
