/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team1165.robot.commands.Intake;
import com.team1165.robot.commands.drivetrain.DriveToPose;
import com.team1165.robot.commands.elevator.ElevatorNextPosition;
import com.team1165.robot.commands.elevator.ElevatorPosition;
import com.team1165.robot.commands.flywheels.FlywheelsPercenmt;
import com.team1165.robot.commands.funnel.FunnelPercent;
import com.team1165.robot.subsystems.drive.Drive;
import com.team1165.robot.subsystems.drive.constants.DriveConstants;
import com.team1165.robot.subsystems.drive.constants.TunerConstants;
import com.team1165.robot.subsystems.drive.io.DriveIO;
import com.team1165.robot.subsystems.drive.io.DriveIOMapleSim;
import com.team1165.robot.subsystems.drive.io.DriveIOReal;
import com.team1165.robot.subsystems.elevator.Elevator;
import com.team1165.robot.subsystems.elevator.constants.ElevatorConstants;
import com.team1165.robot.subsystems.elevator.io.ElevatorIO;
import com.team1165.robot.subsystems.elevator.io.ElevatorIOSim;
import com.team1165.robot.subsystems.elevator.io.ElevatorIOTalonFX;
import com.team1165.robot.subsystems.flywheels.Flywheels;
import com.team1165.robot.subsystems.flywheels.io.FlywheelsIO;
import com.team1165.robot.subsystems.flywheels.io.FlywheelsIOSparkMax;
import com.team1165.robot.subsystems.funnel.Funnel;
import com.team1165.robot.subsystems.funnel.io.FunnelIO;
import com.team1165.robot.subsystems.funnel.io.FunnelIOSparkMax;
import com.team1165.robot.subsystems.vision.apriltag.ATVision;
import com.team1165.robot.subsystems.vision.apriltag.ATVision.CameraConfig;
import com.team1165.robot.subsystems.vision.apriltag.io.ATVisionIO;
import com.team1165.robot.subsystems.vision.apriltag.io.ATVisionIOPhoton;
import com.team1165.robot.subsystems.vision.apriltag.io.ATVisionIOPhotonSim;
import com.team1165.robot.subsystems.vision.apriltag.io.ATVisionIOPhotonSim.ATVisionIOPhotonSimConfig;
import com.team1165.robot.util.ChoreoTrajChooser;
import com.team1165.robot.util.TeleopDashboard;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.function.DoubleSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Elevator elevator;
  private final Flywheels flywheels;
  private final Funnel funnel;
  private final ATVision apriltagVision;

  private final TeleopDashboard teleopDash = TeleopDashboard.getInstance();

  // Driver Controllers
  private final CommandXboxController driverController = new CommandXboxController(0);

  // Testing, likely will be changed later
  private final DoubleSupplier MaxSpeed;
  private final DoubleSupplier MaxAngularRate;
  private final SwerveRequest.FieldCentric fieldCentric;
  private final ChoreoTrajChooser trajChooser;

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    switch (Constants.robotMode) {
      case REAL -> {
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new DriveIOReal(
                    DriveConstants.drivetrainConstants, DriveConstants.getModuleConstants()));
        elevator = new Elevator(new ElevatorIOTalonFX());
        flywheels = new Flywheels(new FlywheelsIOSparkMax());
        funnel = new Funnel(new FunnelIOSparkMax());
        apriltagVision =
            new ATVision(
                drive::addVisionMeasurement,
                drive::getRotation,
                new CameraConfig(
                    new ATVisionIOPhoton("Left Camera"),
                    new Transform3d(
                        0.197,
                        0.286,
                        0.0,
                        new Rotation3d(Degrees.zero(), Degrees.of(20), Degrees.of(-10)))),
                new CameraConfig(
                    new ATVisionIOPhoton("Right Camera"),
                    new Transform3d(
                        0.197,
                        -0.286,
                        0.0,
                        new Rotation3d(Degrees.zero(), Degrees.of(20), Degrees.of(10)))));
      }

      case SIM -> {
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new DriveIOMapleSim(
                    DriveConstants.drivetrainConstants,
                    DriveConstants.simConfig,
                    DriveConstants.getModuleConstants()));
        elevator = new Elevator(new ElevatorIOSim());
        flywheels = new Flywheels(new FlywheelsIO() {});
        funnel = new Funnel(new FunnelIO() {});
        apriltagVision =
            new ATVision(
                drive::addVisionMeasurement,
                drive::getRotation,
                new CameraConfig(
                    new ATVisionIOPhotonSim(
                        "test",
                        new ATVisionIOPhotonSimConfig(
                                960,
                                720,
                                new Transform3d(
                                    new Translation3d(0, 0, 0),
                                    new Rotation3d(0, 0.0, Units.degreesToRadians(-30))))
                            .withCalibError(0.15, 0.1)
                            .withLatency(0, 0)
                            .withFPS(150),
                        drive::getSimulationPose),
                    new Transform3d(
                        new Translation3d(0, 0, 0),
                        new Rotation3d(0, 0.0, Units.degreesToRadians(-30)))));
      }

      default -> {
        // Replayed robot, disable IO implementations
        drive = new Drive(new DriveIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        flywheels = new Flywheels(new FlywheelsIO() {});
        funnel = new Funnel(new FunnelIO() {});
        apriltagVision =
            new ATVision(
                drive::addVisionMeasurement,
                drive::getRotation,
                new CameraConfig(new ATVisionIO() {}, new Transform3d()));
      }
    }
    trajChooser =
        new ChoreoTrajChooser(
            drive.getAutoFactory().newRoutine("Traj Testing"), "Testing Trajectory Chooser");

    MaxSpeed =
        () ->
            elevator.getPosition() >= 6.5
                ? TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / 4
                : TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    MaxAngularRate =
        () ->
            elevator.getPosition() >= 6.5
                ? RotationsPerSecond.of(2).in(RadiansPerSecond) / 4
                : RotationsPerSecond.of(2).in(RadiansPerSecond);

    fieldCentric =
        new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed.getAsDouble() * 0.1)
            .withRotationalDeadband(MaxAngularRate.getAsDouble() * 0.1)
            .withDriveRequestType(DriveRequestType.Velocity);

    configureButtonBindings();
    configureDefaultCommands();
  }

  /** Use this method to define your button->command mappings. */
  private void configureButtonBindings() {
    // New mappings
    driverController.start().onTrue(new InstantCommand(drive::seedFieldCentric));
    driverController.leftTrigger().whileTrue(new DriveToPose(drive, () -> teleopDash.getReefLocation().getPose()));
    driverController.rightTrigger().onTrue(new ElevatorPosition(elevator, () -> teleopDash.getLevel().getElevatorHeight()));
    driverController.rightBumper().onTrue(new FlywheelsPercenmt(flywheels, () -> 0.3).withTimeout(0.05));
    driverController.leftBumper().onTrue(new Intake(elevator, flywheels, funnel));

    driverController.b().onTrue(new Intake(elevator, flywheels, funnel));
    driverController
        .x()
        .whileTrue(new DriveToPose(drive, () -> teleopDash.getReefLocation().getPose()));
    driverController
        .y()
        .whileTrue(new ElevatorPosition(elevator, () -> teleopDash.getLevel().getElevatorHeight()));
    driverController.povUp().whileTrue(new ElevatorNextPosition(elevator));
  }

  /** Use this method to define default commands for subsystems. */
  private void configureDefaultCommands() {
    drive.setDefaultCommand(
        drive.applyRequest(
            () ->
                fieldCentric
                    .withVelocityX(-driverController.getLeftY() * MaxSpeed.getAsDouble())
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed.getAsDouble())
                    .withRotationalRate(
                        -driverController.getRightX() * MaxAngularRate.getAsDouble())));
  }

  public Command getAutonomousCommand() {
    return drive
        .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(1.5))
        .withTimeout(0.5);
  }
}
