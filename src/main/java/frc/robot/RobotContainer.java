// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.constDrivetrain;
import frc.robot.Constants.constVision;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.AutoElevationCommand;
import frc.robot.commands.AutoYawCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;

public class RobotContainer {
  // Subsystems
  private final Drivetrain drivetrain;
  private final Vision vision;
  private final Flywheel flywheel;
  private final Hood hood;
  private final Intake intake;
  private final Kicker kicker;
  private final Indexer indexer;
  private final Turret turret;
  

  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(constDrivetrain.joystickPort);

  public RobotContainer() {
    // Initialize drivetrain
    drivetrain = TunerConstants.createDrivetrain();

    // Initialize vision with 3 cameras for optimal field coverage
    vision = new Vision(
        drivetrain::addVisionMeasurement,
        new VisionIOPhotonVision(constVision.mainCameraName, constVision.mainCameraOffset),
        new VisionIOPhotonVision(constVision.leftCameraName, constVision.leftCameraOffset),
        new VisionIOPhotonVision(constVision.rightCameraName, constVision.rightCameraOffset));

    flywheel = new Flywheel();
    hood = new Hood();
    intake = new Intake();
    kicker = new Kicker();
    indexer = new Indexer();
    turret = new Turret();

    intake.setIntakeRpm(Constants.intakeRpm);
    indexer.setIndexerRpm(Constants.indexerRpm);

    // Configure button bindings and default commands
    configureBindings();
  }



  private void configureBindings() {
    // Default command: Advanced drive with heading lock, input curves, and slow mode
    drivetrain.setDefaultCommand(
        new DriveCommand(
            drivetrain,
            () -> -driverController.getLeftY(), // Forward/backward (negated for correct direction)
            () -> -driverController.getLeftX(), // Left/right (negated for correct direction)
            () -> -driverController.getRightX(), // Rotation (negated for correct direction)
            driverController.leftBumper(), // Slow drive mode (hold left bumper)
            constDrivetrain.maxSpeed, // Max speed
            constDrivetrain.maxAngularRate, // Max angular rate
            vision // Vision subsystem
        ));
    
    // Default command: Continuous auto-aiming based on closest visible AprilTag
    // Runs automatically, no button press needed
    // This will continuously command both hood and flywheel with calculated values
    hood.setDefaultCommand(
      new AutoElevationCommand(hood, flywheel, drivetrain)
    );

    turret.setDefaultCommand(
      new AutoYawCommand(turret, drivetrain)
    );
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
