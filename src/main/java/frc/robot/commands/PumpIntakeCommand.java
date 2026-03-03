package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.constIntake;
import frc.robot.subsystems.intake.Intake;

/**
 * Pumps the intake between deployed and pumped positions repeatedly.
 * Cycle: deployed → pump → deployed → pump → ...
 * Stops when button is released and returns to deployed position.
 */
public class PumpIntakeCommand extends Command {
    
    private final Intake intake;
    private final Timer timer;
    private boolean isAtDeployed;

    public PumpIntakeCommand(Intake intake) {
        this.intake = intake;
        this.timer = new Timer();
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        // Start at deployed position
        intake.deploy();
        isAtDeployed = true;
        timer.restart();
        System.out.println("PumpIntake: Started pumping");
    }

    @Override
    public void execute() {
        // Check if enough time has passed to switch positions
        if (timer.hasElapsed(constIntake.pumpDelaySeconds)) {
            if (isAtDeployed) {
                // Switch to pumped position
                intake.pump();
                isAtDeployed = false;
            } else {
                // Switch back to deployed position
                intake.deploy();
                isAtDeployed = true;
            }
            // Reset timer for next cycle
            timer.restart();
        }
    }

    @Override
    public boolean isFinished() {
        // Never finish - runs until button released
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Return to deployed position when command ends
        intake.deploy();
        timer.stop();
        System.out.println("PumpIntake: Stopped, returned to deployed");
    }
}

