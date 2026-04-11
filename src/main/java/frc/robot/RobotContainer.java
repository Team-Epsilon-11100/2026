// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.constDrivetrain;
import frc.robot.Constants.constIndexer;
import frc.robot.Constants.constIntake;
import frc.robot.Constants.constKicker;
import frc.robot.Constants.constTurret;
import frc.robot.Constants.constVision;

import frc.robot.commands.AutoElevationCommand;
import frc.robot.commands.AutoYawCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.PumpIntakeCommand;
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

  // Intake toggle state
  private boolean intakeDeployed = true; // starts deployed (matches intake.deploy() in constructor)

  // Flywheel/auto-aim toggle state
  private boolean flywheelEnabled = true;

  // Turret manual nudge state (testing only — POV left/right)
  private double turretTargetAngle = 0.0; // degrees, 0 = forward

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
    
    intake.deploy(); // Start with intake deployed
      indexer.setDutyCycle(constIndexer.idleDutyCycle);
    // Configure button bindings and default commands
    configureBindings();
  }

  private void configureBindings() {
    // Default command: Advanced drive with heading lock, input curves, and slow
    // mode
    drivetrain.setDefaultCommand(
        new DriveCommand(
            drivetrain,
            () -> -driverController.getLeftY(), // Forward/backward (negated for correct direction)
            () -> -driverController.getLeftX(), // Left/right (negated for correct direction)
            () -> -driverController.getRightX(), // Rotation (negated for correct direction)
            driverController.b(), // Slow drive mode (hold left bumper)
            constDrivetrain.maxSpeed, // Max speed
            constDrivetrain.maxAngularRate, // Max angular rate
            vision // Vision subsyst em
        ));

    // Default command: Continuous auto-aiming based on closest visible AprilTag
    // Runs automatically, no button press needed
    // This will continuously command both hood and flywheel with calculated values
    AutoElevationCommand autoElevation = new AutoElevationCommand(hood, flywheel, drivetrain);
    hood.setDefaultCommand(
    Commands.waitUntil(intake::isDeployed)
      .withTimeout(constIntake.deployWaitTimeoutSeconds)
      .andThen(autoElevation));

    // POV up: toggle flywheel/auto-aim on or off
    driverController.povUp().onTrue(
        Commands.runOnce(() -> {
          flywheelEnabled = !flywheelEnabled;
          autoElevation.setEnabled(flywheelEnabled);
          SmartDashboard.putBoolean("AutoElev/FlywheelEnabled", flywheelEnabled);
        })
    );

  // Keep yaw active continuously so turret aiming can't be blocked by intake-state tolerance.
  turret.setDefaultCommand(new AutoYawCommand(turret, drivetrain));

    driverController.rightTrigger().whileTrue(
        Commands.run(() -> {
          kicker.setFromFlywheelRpm(flywheel.getFlywheelRpm());
          indexer.setDutyCycle(constIndexer.dutyCycle);
        }, kicker, indexer)).whileFalse(
            Commands.runOnce(() -> {
              kicker.stop();
              indexer.setDutyCycle(constIndexer.idleDutyCycle);
            }, kicker, indexer));

    // Right bumper: run intake while held, stop when released
    driverController.rightBumper().whileTrue(
        Commands.runOnce(() -> {
          intake.setDutyCycle(constIntake.dutyCycle);
        }, intake)
    ).onFalse(
        Commands.runOnce(() -> {
          intake.setDutyCycle(0);
        }, intake)
    );

    // Left bumper: toggle intake deploy/retract on each press
    // driverController.leftBumper().onTrue(
    //     Commands.runOnce(() -> {
    //       if (intakeDeployed) {
    //         intake.retract();
    //       } else {
    //         intake.deploy();
    //       }
    //       intakeDeployed = !intakeDeployed;
    //     }, intake)
    // );

    // Left trigger: Pump intake while held, retract when released
    driverController.leftTrigger().whileTrue(
        new PumpIntakeCommand(intake));

    // POV down: reverse kicker and indexer while held (unjam)
    driverController.povDown().whileTrue(
        Commands.runOnce(() -> {
          kicker.setDutyCycle(constKicker.reverseDutyCycle);
          indexer.setDutyCycle(constIndexer.reverseDutyCycle);
        }, kicker, indexer)
    ).onFalse(
        Commands.runOnce(() -> {
          kicker.stop();
          indexer.setDutyCycle(constIndexer.idleDutyCycle);
        }, kicker, indexer)
    );

    // POV left/right: nudge turret by ±15° for testing (AutoYaw disabled)
    // Wraps around: going past +180° jumps to -180° and vice versa
    driverController.povLeft().onTrue(
        Commands.runOnce(() -> {
          turretTargetAngle -= 15.0;
          if (turretTargetAngle < constTurret.minTurretAngleDegrees) {
            turretTargetAngle += 360.0; // wrap: -195° → +165°
          }
          turret.setAngle(turretTargetAngle);
          SmartDashboard.putNumber("Turret/TargetAngle", turretTargetAngle);
        }, turret)
    );

    driverController.povRight().onTrue(
        Commands.runOnce(() -> {
          turretTargetAngle += 15.0;
          if (turretTargetAngle > constTurret.maxTurretAngleDegrees) {
            turretTargetAngle -= 360.0; // wrap: +195° → -165°
          }
          turret.setAngle(turretTargetAngle);
          SmartDashboard.putNumber("Turret/TargetAngle", turretTargetAngle);
        }, turret)
    );
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
