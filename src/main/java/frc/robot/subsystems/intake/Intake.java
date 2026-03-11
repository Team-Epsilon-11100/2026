package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constIntake;

public class Intake extends SubsystemBase {
    private final TalonFX intakeMotor;
    private final TalonFX pivotMotor;
    private final DutyCycleOut intakeOut;
    private final MotionMagicExpoVoltage pivotPositionControl;

    public Intake() {
        intakeMotor = new TalonFX(constIntake.intakeMotorId);
        pivotMotor = new TalonFX(constIntake.pivotMotorId);

        // Configure intake motor for duty cycle control
        var intakeConfigs = new TalonFXConfiguration();
        intakeMotor.getConfigurator().apply(intakeConfigs);
        intakeOut = new DutyCycleOut(0);
        pivotPositionControl = new MotionMagicExpoVoltage(0);

        // Configure pivot motor (position control can be added later if needed)
        var pivotConfigs = new TalonFXConfiguration();
        var pivotSlot0 = pivotConfigs.Slot0;
        pivotSlot0.kP = constIntake.pivotKp;
        pivotSlot0.kI = constIntake.pivotKi;
        pivotSlot0.kD = constIntake.pivotKd;
        
        pivotMotor.getConfigurator().apply(pivotConfigs);
    }

    /**
     * Set the intake motor duty cycle.
     *
     * @param dutyCycle Target duty cycle (-1.0 to 1.0, positive = intake, negative = eject)
     */
    public void setDutyCycle(double dutyCycle) {
        intakeMotor.setControl(intakeOut.withOutput(-dutyCycle));
    }

    /**
     * Stop the intake motor.
     */
    public void stopIntake() {
        setDutyCycle(0);
    }

    public void setPivotPos(double positionRotations) {
        pivotMotor.setControl(pivotPositionControl.withPosition(positionRotations));
    }

    /**
     * Stop both intake and pivot motors.
     */
    public void stop() {
        stopIntake();
    }

    public void deploy() {
        setPivotPos(constIntake.deployedPos);
    }

    /**
     * Returns true when the pivot is within tolerance of the fully deployed position.
     */
    public boolean isDeployed() {
        double pos = pivotMotor.getPosition().getValueAsDouble();
        return Math.abs(pos - constIntake.deployedPos) <= constIntake.deployedTolerance;
    }

    public void retract() {
        setPivotPos(constIntake.retractedPos);
    }

    public void pump() {
        setPivotPos(constIntake.pumpPos);
    }

    @Override
    public void periodic() {
        // Log intake status to SmartDashboard
        SmartDashboard.putNumber("Intake/Current", intakeMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Pivot/Position", pivotMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Pivot/Current", pivotMotor.getSupplyCurrent().getValueAsDouble());
    }
}

