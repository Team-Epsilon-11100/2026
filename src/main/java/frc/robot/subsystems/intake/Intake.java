package frc.robot.subsystems.intake;


import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constIntake;

public class Intake extends SubsystemBase {
     TalonFX intakeMotor;
     TalonFX pivotMotor;
    public Intake() {
        intakeMotor = new TalonFX(constIntake.intakeMotorId);
        pivotMotor = new TalonFX(constIntake.pivotMotorId);
    }
}
