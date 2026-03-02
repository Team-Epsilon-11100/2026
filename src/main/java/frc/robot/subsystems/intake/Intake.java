package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constIntake;

public class Intake extends SubsystemBase {
    private final TalonFX intakeMotor;
    private final TalonFX pivotMotor;
    private final MotionMagicVelocityVoltage velocityControl;
    private final MotionMagicExpoVoltage pivotPositionControl;

    public Intake() {
        intakeMotor = new TalonFX(constIntake.intakeMotorId);
        pivotMotor = new TalonFX(constIntake.pivotMotorId);

        // Configure intake motor for velocity control
        var intakeConfigs = new TalonFXConfiguration();
        
        // Set PID gains for velocity control (Slot 1)
        var slot1Configs = intakeConfigs.Slot1;
        slot1Configs.kS = 0; // Static friction compensation
        slot1Configs.kV = 0; // Velocity feedforward
        slot1Configs.kA = 0; // Acceleration feedforward
        slot1Configs.kP = constIntake.intakeKp;
        slot1Configs.kI = constIntake.intakeKi;
        slot1Configs.kD = constIntake.intakeKd;

        // Set Motion Magic settings
        var motionMagicConfigs = intakeConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 0; // Unlimited cruise velocity
        motionMagicConfigs.MotionMagicExpo_kV = 0;
        motionMagicConfigs.MotionMagicExpo_kA = 0;

        intakeMotor.getConfigurator().apply(intakeConfigs);
        velocityControl = new MotionMagicVelocityVoltage(0).withSlot(1);
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
     * Set the intake motor RPM.
     * 
     * @param rpm Target RPM (positive = intake, negative = eject)
     */
    public void setIntakeRpm(double rpm) {
        // Convert RPM to rotations per second (RPS)
        intakeMotor.setControl(velocityControl.withVelocity(rpm / 60.0));
    }

    /**
     * Get the current intake motor RPM.
     * 
     * @return Current RPM
     */
    public double getIntakeRpm() {
        return intakeMotor.getVelocity().getValueAsDouble() * 60.0;
    }

    /**
     * Stop the intake motor.
     */
    public void stopIntake() {
        setIntakeRpm(0);
    }

    /**
     * Set the pivot motor voltage (percent output).
     * 
     * @param percent Percent output (-1.0 to 1.0)
     */
    public void setPivotPercent(double percent) {
        pivotMotor.set(percent);
    }

    /**
     * Stop the pivot motor.
     */
    public void stopPivot() {
        pivotMotor.set(0);
    }

    /**
     * Stop both intake and pivot motors.
     */
    public void stop() {
        stopIntake();
        stopPivot();
    }

    @Override
    public void periodic() {
        // Log intake status to SmartDashboard
        SmartDashboard.putNumber("Intake/RPM", getIntakeRpm());
        SmartDashboard.putNumber("Intake/Current", intakeMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Pivot/Position", pivotMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Pivot/Current", pivotMotor.getSupplyCurrent().getValueAsDouble());
    }
}

