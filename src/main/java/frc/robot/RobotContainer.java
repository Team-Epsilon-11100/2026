// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.constAutoAim;
import frc.robot.Constants.constDrivetrain;
import frc.robot.Constants.constFlywheel;
import frc.robot.Constants.constHood;
import frc.robot.Constants.constIndexer;
import frc.robot.Constants.constIntake;
import frc.robot.Constants.constKicker;
import frc.robot.Constants.constTurret;
import frc.robot.Constants.constVision;

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
  private final SwerveRequest.ApplyRobotSpeeds autoRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(constDrivetrain.joystickPort);

  // Flywheel/auto-aim toggle state
  private boolean flywheelEnabled = true;

  // Turret manual nudge state (testing only — POV left/right)
  private double turretTargetAngle = 0.0; // degrees, 0 = forward

  // Pump intake toggle state (starts deployed)
  private boolean pumpMode = false; // false = deployed, true = pumped

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

  configurePathPlanner();
    
    intake.deploy(); // Start with intake deployed
      indexer.setDutyCycle(constIndexer.idleDutyCycle);
    // Configure button bindings and default commands
    configureBindings();
  }

  private void configurePathPlanner() {
    try {
      RobotConfig robotConfig = RobotConfig.fromGUISettings();

      AutoBuilder.configure(
      () -> drivetrain.getBestAvailablePose(vision),
          drivetrain::resetPose,
          () -> drivetrain.getState().Speeds,
      (speeds, feedforwards) -> drivetrain.setControl(autoRobotSpeeds.withSpeeds(speeds)),
          new PPHolonomicDriveController(
              new PIDConstants(5.0, 0.0, 0.0),
              new PIDConstants(5.0, 0.0, 0.0)),
          robotConfig,
          () -> DriverStation.getAlliance().map(a -> a == DriverStation.Alliance.Red).orElse(false),
          drivetrain);

      SmartDashboard.putString("Auto/PathPlanner", "Configured");
    } catch (Exception e) {
      SmartDashboard.putString("Auto/PathPlanner", "Config failed: " + e.getMessage());
      System.out.println("PathPlanner configuration failed: " + e.getMessage());
    }
  }

  private void configureBindings() {
    // Default command: Advanced drive with heading lock, input curves, and slow mode
    // B button held = half speed mode
    drivetrain.setDefaultCommand(
        new DriveCommand(
            drivetrain,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            driverController.b(), // B held = half speed
            constDrivetrain.maxSpeed,
            constDrivetrain.maxAngularRate,
            vision
        ));

    // Default command: Continuous auto-aiming based on closest visible AprilTag
    AutoElevationCommand autoElevation = new AutoElevationCommand(hood, flywheel, drivetrain);
    hood.setDefaultCommand(
        Commands.waitUntil(intake::isDeployed).andThen(autoElevation));

    // POV up: toggle flywheel/auto-aim on or off
    driverController.povUp().onTrue(
        Commands.runOnce(() -> {
          flywheelEnabled = !flywheelEnabled;
          autoElevation.setEnabled(flywheelEnabled);
          SmartDashboard.putBoolean("AutoElev/FlywheelEnabled", flywheelEnabled);
        })
    );

    turret.setDefaultCommand(
        Commands.waitUntil(intake::isDeployed).andThen(new AutoYawCommand(turret, drivetrain)));

    // Right trigger: shoot (kicker + indexer)
    driverController.rightTrigger().whileTrue(
        Commands.runOnce(() -> {
          kicker.setDutyCycle(constKicker.dutyCycle);
          indexer.setDutyCycle(constIndexer.dutyCycle);
        }, kicker, indexer)).whileFalse(
            Commands.runOnce(() -> {
              kicker.stop();
              indexer.setDutyCycle(constIndexer.idleDutyCycle);
            }, kicker, indexer));

    // A: run intake while held, stop when released
    driverController.a().whileTrue(
        Commands.runOnce(() -> {
          intake.setDutyCycle(constIntake.dutyCycle);
        }, intake)
    ).onFalse(
        Commands.runOnce(() -> {
          intake.setDutyCycle(0);
        }, intake)
    );

    // B: half speed mode (handled in DriveCommand) + stow hood to minimum angle while held
    driverController.b().whileTrue(
        Commands.run(() -> hood.setAngle(constHood.minHoodAngleDegrees), hood)
    ).onFalse(
        Commands.runOnce(() -> {}, hood) // release hood back to default command
    );

    // Y button: Toggle pump mode (deployed <-> pumped position)
    driverController.y().onTrue(
        Commands.runOnce(() -> {
          pumpMode = !pumpMode;
          if (pumpMode) {
            // Move to pumped position
            intake.setPivotPos(constIntake.pumpPos);
            SmartDashboard.putString("Intake/PumpMode", "PUMPED");
          } else {
            // Return to deployed position
            intake.setPivotPos(constIntake.deployedPos);
            SmartDashboard.putString("Intake/PumpMode", "DEPLOYED");
          }
          SmartDashboard.putBoolean("Intake/PumpToggle", pumpMode);
        }, intake)
    );

    // POV down: reverse kicker and indexer while held (unjam)
    driverController.x().whileTrue(
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

    // Start or Back: reset gyro heading (treat current facing as field forward)
    driverController.start().onTrue(
        Commands.runOnce(() -> drivetrain.seedFieldCentric(), drivetrain)
    );
    driverController.back ().onTrue(
        Commands.runOnce(() -> drivetrain.seedFieldCentric(), drivetrain)
    );

    // POV left/right: nudge turret by ±15° for testing
    driverController.povLeft().onTrue(
        Commands.runOnce(() -> {
          turretTargetAngle -= 15.0;
          if (turretTargetAngle < constTurret.minTurretAngleDegrees) {
            turretTargetAngle += 360.0;
          }
          turret.setAngle(turretTargetAngle);
          SmartDashboard.putNumber("Turret/TargetAngle", turretTargetAngle);
        }, turret)
    );

    driverController.povRight().onTrue(
        Commands.runOnce(() -> {
          turretTargetAngle += 15.0;
          if (turretTargetAngle > constTurret.maxTurretAngleDegrees) {
            turretTargetAngle -= 360.0;
          }
          turret.setAngle(turretTargetAngle);
          SmartDashboard.putNumber("Turret/TargetAngle", turretTargetAngle);
        }, turret)
    );
  }

  public Command getAutonomousCommand() {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");
      Command followPath = AutoBuilder.followPath(path);

      // Periodic autonomous mode switch:
      // - Neutral zone: run intake
      // - Alliance side: run shooter (flywheel + indexer + kicker)
      Command zoneAction = Commands.run(() -> {
        var pose = drivetrain.getBestAvailablePose(vision);
        double x = pose.getX();
        boolean inNeutralZone = x >= constAutoAim.neutralZoneMinX && x <= constAutoAim.neutralZoneMaxX;
        boolean isRed = DriverStation.getAlliance().map(a -> a == DriverStation.Alliance.Red).orElse(false);
        boolean onAllianceSide = isRed ? (x > constAutoAim.neutralZoneMaxX) : (x < constAutoAim.neutralZoneMinX);

        if (inNeutralZone) {
          // Intake mode in neutral zone
          intake.setDutyCycle(constIntake.dutyCycle);
          flywheel.stop();
          kicker.stop();
          indexer.setDutyCycle(constIndexer.idleDutyCycle);
          SmartDashboard.putString("Auto/ZoneMode", "NEUTRAL_INTAKE");
        } else if (onAllianceSide) {
          // Shoot mode on alliance side
          intake.stopIntake();
          flywheel.setFlywheelRpm(constFlywheel.maxFlywheelRPM);
          kicker.setDutyCycle(constKicker.dutyCycle);
          indexer.setDutyCycle(constIndexer.dutyCycle);
          SmartDashboard.putString("Auto/ZoneMode", "ALLIANCE_SHOOT");
        } else {
          // Opponent side / undefined zone: safe idle
          intake.stopIntake();
          flywheel.stop();
          kicker.stop();
          indexer.setDutyCycle(constIndexer.idleDutyCycle);
          SmartDashboard.putString("Auto/ZoneMode", "SAFE_IDLE");
        }
      }, intake, flywheel, kicker, indexer);

      Command cleanup = Commands.runOnce(() -> {
        intake.stopIntake();
        flywheel.stop();
        kicker.stop();
        indexer.setDutyCycle(constIndexer.idleDutyCycle);
        SmartDashboard.putString("Auto/ZoneMode", "DONE");
      }, intake, flywheel, kicker, indexer);

      return Commands.sequence(
          Commands.runOnce(intake::deploy, intake),
      Commands.deadline(followPath, zoneAction),
          cleanup);
    } catch (Exception e) {
      SmartDashboard.putString("Auto/PathPlanner", "Path load failed: " + e.getMessage());
      System.out.println("Failed to load Example Path: " + e.getMessage());
      return Commands.none();
    }
  }
}
