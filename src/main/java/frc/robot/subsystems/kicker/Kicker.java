package frc.robot.subsystems.kicker;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constBallisticSolver;
import frc.robot.Constants.constKicker;

public class Kicker extends SubsystemBase {
    private final TalonFX kickerMotor;
    private final MotionMagicVelocityVoltage kickerPID;
    private double lastTargetRpm = 0.0;
    private double lastFlywheelInputRpm = 0.0;
    private double lastTargetSurfaceSpeedMps = 0.0;
    
    public Kicker() {
        kickerMotor = new TalonFX(constKicker.kickerMotorId);

        var motorConfigs = new TalonFXConfiguration();

        // Set slot 0 gains (velocity PID + feedforward)
        var slot0Configs = motorConfigs.Slot0;
        slot0Configs.kS = constKicker.kS;
        slot0Configs.kV = constKicker.kV;
        slot0Configs.kA = constKicker.kA;
        slot0Configs.kP = constKicker.kP;
        slot0Configs.kI = constKicker.kI;
        slot0Configs.kD = constKicker.kD;

        // Set Motion Magic settings
        var motionMagicConfigs = motorConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 0; // Unlimited cruise velocity
        motionMagicConfigs.MotionMagicExpo_kV = 0;
        motionMagicConfigs.MotionMagicExpo_kA = 0;
        motionMagicConfigs.MotionMagicAcceleration = constKicker.maxKickerRPM / 60.0 * 2;

        kickerMotor.getConfigurator().apply(motorConfigs);
        kickerPID = new MotionMagicVelocityVoltage(0).withSlot(0);

        System.out.println("Kicker initialized on motor ID: " + constKicker.kickerMotorId);
    }

    /**
     * Set the kicker motor RPM.
     * 
     * @param rpm Target RPM (positive = forward, negative = reverse)
     */
    public void setDutyCycle(double dutyCycle) {
        // Backward-compatible mapping from duty [-1..1] to closed-loop RPM target.
        setKickerRpm(dutyCycle * constKicker.maxKickerRPM);
    }

    /** Set kicker target speed in motor RPM (velocity closed-loop). */
    public void setKickerRpm(double rpm) {
        double clampedRpm = Math.max(constKicker.minKickerRPM, Math.min(rpm, constKicker.maxKickerRPM));
        lastTargetRpm = clampedRpm;
        kickerMotor.setControl(kickerPID.withVelocity(clampedRpm / 60.0));
    }

    /**
     * Follow flywheel wheel surface speed.
     * Input is flywheel motor RPM; output is kicker motor RPM computed from physical ratios.
     */
    public void setFromFlywheelRpm(double flywheelRpm) {
        lastFlywheelInputRpm = flywheelRpm;
        double flywheelWheelRpm = flywheelRpm / constBallisticSolver.gearRatioMotorToWheel;
        double flywheelSurfaceSpeedMps = (flywheelWheelRpm / 60.0) * (Math.PI * constBallisticSolver.flywheelDiameterMeters);

        double kickerWheelRpm = (flywheelSurfaceSpeedMps * 60.0) / (Math.PI * constKicker.wheelDiameterMeters);
        double kickerMotorRpm = kickerWheelRpm * constKicker.gearRatioMotorToWheel * constKicker.surfaceSpeedSign;

        lastTargetSurfaceSpeedMps = flywheelSurfaceSpeedMps;
        setKickerRpm(kickerMotorRpm);
    }

    /** Get measured kicker speed in motor RPM. */
    public double getKickerRpm() {
        return kickerMotor.getVelocity().getValueAsDouble() * 60.0;
    }



    /**
     * Stop the kicker motor.
     */
    public void stop() {
        setKickerRpm(0);
    }

    @Override
    public void periodic() {
        // Log kicker status to SmartDashboard
        SmartDashboard.putNumber("Kicker/FlywheelInputRPM", lastFlywheelInputRpm);
        SmartDashboard.putNumber("Kicker/TargetSurfaceSpeedMps", lastTargetSurfaceSpeedMps);
        SmartDashboard.putNumber("Kicker/TargetRPM", lastTargetRpm);
        SmartDashboard.putNumber("Kicker/RPM", getKickerRpm());
        SmartDashboard.putNumber("Kicker/Current", kickerMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Kicker/Voltage", kickerMotor.getMotorVoltage().getValueAsDouble());
    }
}

