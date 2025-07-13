/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;
import com.team1165.robot.commands.drivetrain.DriveCommands;
import com.team1165.robot.globalconstants.Constants;
import com.team1165.robot.subsystems.drive.Drive;
import com.team1165.robot.subsystems.drive.constants.DriveConstants;
import com.team1165.robot.subsystems.drive.io.DriveIO;
import com.team1165.robot.subsystems.drive.io.DriveIOMapleSim;
import com.team1165.robot.subsystems.drive.io.DriveIOReal;
import com.team1165.robot.subsystems.elevator.Elevator;
import com.team1165.robot.subsystems.elevator.ElevatorConstants;
import com.team1165.robot.subsystems.elevator.io.ElevatorIO;
import com.team1165.robot.subsystems.elevator.io.ElevatorIOTalonFX;
import com.team1165.robot.subsystems.roller.flywheel.Flywheel;
import com.team1165.robot.subsystems.roller.flywheel.FlywheelConstants;
import com.team1165.robot.subsystems.roller.flywheel.FlywheelState;
import com.team1165.robot.subsystems.roller.funnel.Funnel;
import com.team1165.robot.subsystems.roller.funnel.FunnelConstants;
import com.team1165.robot.subsystems.roller.funnel.FunnelState;
import com.team1165.robot.subsystems.roller.io.RollerIO;
import com.team1165.robot.subsystems.roller.io.RollerIOSim;
import com.team1165.robot.subsystems.roller.io.RollerIOSpark;
import com.team1165.robot.subsystems.vision.apriltag.ATVision;
import com.team1165.robot.subsystems.vision.apriltag.ATVision.CameraConfig;
import com.team1165.robot.subsystems.vision.apriltag.io.ATVisionIO;
import com.team1165.robot.subsystems.vision.apriltag.io.ATVisionIOPhoton;
import com.team1165.robot.subsystems.vision.apriltag.io.ATVisionIOPhotonSim;
import com.team1165.robot.subsystems.vision.apriltag.io.ATVisionIOPhotonSim.ATVisionIOPhotonSimConfig;
import com.team1165.robot.util.TeleopDashboard;
import com.team1165.robot.util.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

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
  private final Flywheel flywheel;
  private final Funnel funnel;
  private final ATVision apriltagVision;
  private final OdysseusManager robot;

  private final TeleopDashboard teleopDash = TeleopDashboard.getInstance();

  // Driver Controllers
  private final CommandXboxController driverController = new CommandXboxController(0);

  // Testing, likely will be changed later
  private final SwerveRequest.FieldCentric fieldCentric;
  private final SwerveRequest.SwerveDriveBrake brake;
  private final AutoBuilder autoBuilder;

  private double manualPosition = 0.0;

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    switch (Constants.robotMode) {
      case REAL -> {
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new DriveIOReal(
                    DriveConstants.drivetrainConstants, DriveConstants.getModuleConstants()));

        elevator =
            new Elevator(
                new ElevatorIOTalonFX(
                    ElevatorConstants.Motors.primaryMotorConfig,
                    ElevatorConstants.Motors.secondaryMotorConfig),
                drive::getPose,
                () -> TeleopDashboard.getInstance().getReefLocation().getPose());

        flywheel =
            new Flywheel(
                new RollerIOSpark(
                    FlywheelConstants.primaryMotorConfig, FlywheelConstants.secondaryMotorConfig));

        funnel =
            new Funnel(
                new RollerIOSpark(
                    FunnelConstants.primaryMotorConfig, FunnelConstants.secondaryMotorConfig));

        apriltagVision =
            new ATVision(
                drive::addVisionMeasurement,
                drive::getRotation,
                new CameraConfig(
                    new ATVisionIOPhoton("Right Camera"),
                    new Transform3d(
                        0.197,
                        -0.286,
                        0.2,
                        new Rotation3d(Degrees.zero(), Degrees.of(-20), Degrees.of(10)))),
                new CameraConfig(
                    new ATVisionIOPhoton("Left Camera"),
                    new Transform3d(
                        0.197,
                        0.286,
                        0.2,
                        new Rotation3d(Degrees.zero(), Degrees.of(-20), Degrees.of(-10)))));
      }

      case SIM -> {
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new DriveIOMapleSim(
                    DriveConstants.drivetrainConstants,
                    DriveConstants.simConfig,
                    DriveConstants.getModuleConstants()));

        // TODO: Add simulation IO for Elevator
        elevator =
            new Elevator(
                new ElevatorIO() {},
                drive::getPose,
                () -> TeleopDashboard.getInstance().getReefLocation().getPose());

        flywheel = new Flywheel(new RollerIOSim(FunnelConstants.simConfig, Amps.of(50)) {});

        funnel = new Funnel(new RollerIOSim(FunnelConstants.simConfig, Amps.of(40)));

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
                                    0.197,
                                    -0.286,
                                    0.2,
                                    new Rotation3d(
                                        Degrees.zero(), Degrees.of(-20), Degrees.of(10))))
                            .withCalibError(0.15, 0.1)
                            .withLatency(0, 0)
                            .withFPS(150),
                        drive::getSimulationPose),
                    new Transform3d(
                        0.197,
                        -0.286,
                        0.2,
                        new Rotation3d(Degrees.zero(), Degrees.of(-20), Degrees.of(10)))));
      }

      default -> {
        // Replayed robot, disable IO implementations
        drive = new Drive(new DriveIO() {});
        elevator =
            new Elevator(
                new ElevatorIO() {},
                drive::getPose,
                () -> TeleopDashboard.getInstance().getReefLocation().getPose());
        flywheel = new Flywheel(new RollerIO() {});
        funnel = new Funnel(new RollerIO() {});
        apriltagVision =
            new ATVision(
                drive::addVisionMeasurement,
                drive::getRotation,
                new CameraConfig(new ATVisionIO() {}, new Transform3d()));
      }
    }

    robot = new OdysseusManager(OdysseusState.IDLE, elevator, flywheel, funnel);

    fieldCentric = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);

    brake = new SwerveDriveBrake();

    autoBuilder = AutoBuilder.getInstance();

    configureButtonBindings();
    configureDefaultCommands();
  }

  /** Use this method to define your button->command mappings. */
  private void configureButtonBindings() {
    driverController.a().onTrue(robot.stateCommand(OdysseusState.INTAKE));
    driverController.b().onTrue(robot.stateCommand(OdysseusState.SCORE_L1));
    driverController.y().onTrue(robot.stateCommand(OdysseusState.L1));
    driverController.x().onTrue(robot.stateCommand(OdysseusState.IDLE));
    driverController.leftBumper().whileTrue(flywheel.overrideState(FlywheelState.CUSTOM_MANUAL));
    driverController.rightBumper().whileTrue(funnel.overrideState(FunnelState.CUSTOM_MANUAL));
  }

  /** Use this method to define default commands for subsystems. */
  private void configureDefaultCommands() {
    drive.setDefaultCommand(
        DriveCommands.teleopManualDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> (driverController.getLeftTriggerAxis() - driverController.getRightTriggerAxis()),
            true));
  }

  public Command getAutonomousCommand() {
    //    return new InstantCommand(
    //            () ->
    //                drive.resetRotation(
    //                    DriverStation.getAlliance().isPresent()
    //                            && DriverStation.getAlliance().get() == Alliance.Red
    //                        ? Rotation2d.kZero
    //                        : Rotation2d.kPi))
    //        .andThen(autoBuilder.buildAutoCommand(drive, elevator, flywheels, funnel));
    return Commands.none();
  }
}
