/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;
import com.team1165.robot.commands.elevator.ElevatorPosition;
import com.team1165.robot.commands.flywheels.FlywheelsPercenmt;
import com.team1165.robot.subsystems.drive.Drive;
import com.team1165.robot.subsystems.drive.constants.DriveConstants;
import com.team1165.robot.subsystems.drive.constants.TunerConstants;
import com.team1165.robot.subsystems.drive.io.DriveIO;
import com.team1165.robot.subsystems.drive.io.DriveIOMapleSim;
import com.team1165.robot.subsystems.drive.io.DriveIOReal;
import com.team1165.robot.subsystems.elevator.Elevator;
import com.team1165.robot.subsystems.elevator.io.ElevatorIO;
import com.team1165.robot.subsystems.elevator.io.ElevatorIOSim;
import com.team1165.robot.subsystems.elevator.io.ElevatorIOTalonFX;
import com.team1165.robot.subsystems.flywheels.Flywheels;
import com.team1165.robot.subsystems.flywheels.io.FlywheelsIO;
import com.team1165.robot.subsystems.flywheels.io.FlywheelsIOSparkMax;
import com.team1165.robot.subsystems.roller.funnel.Funnel;
import com.team1165.robot.subsystems.roller.funnel.FunnelState;
import com.team1165.robot.subsystems.roller.funnel.constants.FunnelConstants;
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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
  private final OdysseusManager robot;

  private final TeleopDashboard teleopDash = TeleopDashboard.getInstance();

  // Driver Controllers
  private final CommandXboxController driverController = new CommandXboxController(0);

  // Testing, likely will be changed later
  private final DoubleSupplier MaxSpeed;
  private final DoubleSupplier MaxAngularRate;
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
        elevator = new Elevator(new ElevatorIOTalonFX());
        flywheels = new Flywheels(new FlywheelsIOSparkMax());
        funnel =
            new Funnel(
                new RollerIOSpark(
                    FunnelConstants.Configurations.primaryMotorConfig,
                    FunnelConstants.Configurations.secondaryMotorConfig));
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
        elevator = new Elevator(new ElevatorIOSim());
        flywheels = new Flywheels(new FlywheelsIO() {});
        funnel = new Funnel(new RollerIOSim(FunnelConstants.Configurations.simConfig, Amps.of(50)));
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
        elevator = new Elevator(new ElevatorIO() {});
        flywheels = new Flywheels(new FlywheelsIO() {});
        funnel = new Funnel(new RollerIO() {});
        apriltagVision =
            new ATVision(
                drive::addVisionMeasurement,
                drive::getRotation,
                new CameraConfig(new ATVisionIO() {}, new Transform3d()));
      }
    }

    robot = new OdysseusManager(FunnelState.IDLE, funnel);

    MaxSpeed =
        () ->
            elevator.getPosition() >= 6.5
                ? TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / 3
                : TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    MaxAngularRate =
        () ->
            elevator.getPosition() >= 6.5
                ? RotationsPerSecond.of(2).in(RadiansPerSecond) / 3
                : RotationsPerSecond.of(2).in(RadiansPerSecond);

    fieldCentric = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);

    brake = new SwerveDriveBrake();

    autoBuilder = AutoBuilder.getInstance();

    configureButtonBindings();
    configureDefaultCommands();
  }

  /** Use this method to define your button->command mappings. */
  private void configureButtonBindings() {
    // Bumpers
    driverController
        .leftBumper()
        .whileTrue(
            robot
                .stateCommand(FunnelState.MANUAL_REVERSE)
                .alongWith(new FlywheelsPercenmt(flywheels, () -> -0.15)))
        .onFalse(robot.stateCommand(FunnelState.IDLE));
    driverController
        .rightBumper()
        .whileTrue(
            robot
                .stateCommand(FunnelState.MANUAL_FORWARD)
                .alongWith(new FlywheelsPercenmt(flywheels, () -> 0.15)))
        .onFalse(robot.stateCommand(FunnelState.IDLE));
    // Ideally, if both of the above subsystems were state-based, maybe do something like this?:
    // driverController
    // .onTrue(
    //    funnel.stateCommand(FunnelState.MANUAL_FORWARD)
    //      .alongWith(flywheels.stateCommand(FlywheelsState.MANUAL_FORWARD))
    // .onFalse(
    //    funnel.stateCommand(FunnelState.IDLE)
    //      .alongWith(flywheels.stateCommand(FlywheelsState.IDLE));
    // This is longer than the original pre-state machine code, but it is also MUCH more clear in
    // exactly what it is doing, and allows more control outside of the command

    // Center buttons
    driverController.start().onTrue(new InstantCommand(drive::seedFieldCentric));
    driverController
        .back()
        .onTrue(
            new InstantCommand(() -> elevator.setEmergencyStop(!elevator.getEmergencyStopState())));

    // ABXY
    // driverController.a().onTrue(new FlywheelsPercenmt(flywheels, () -> 0.3).withTimeout(0.05));
    // driverController.b().onTrue(new Intake(elevator, flywheels, funnel));
    driverController.a().onTrue(robot.stateCommand(FunnelState.INTAKE));
    driverController.b().whileTrue(funnel.overrideState(FunnelState.MANUAL_FORWARD));
    driverController.x().onTrue(robot.stateCommand(FunnelState.IDLE));
    driverController
        .y()
        .onTrue(new ElevatorPosition(elevator, () -> teleopDash.getLevel().getElevatorHeight()));
    // driverController
    //     .x()
    //    .whileTrue(new DriveToPose(drive, () -> teleopDash.getReefLocation().getPose()));

    // RobotModeTriggers.teleop().onTrue(new InstantCommand(apriltagVision::enableSingleTagTrig));
  }

  /** Use this method to define default commands for subsystems. */
  private void configureDefaultCommands() {
    drive.setDefaultCommand(
        drive.run(
            () -> {
              var joystickX = -driverController.getLeftY();
              var joystickY = -driverController.getLeftX();
              var joystickRotation =
                  (driverController.getLeftTriggerAxis() - driverController.getRightTriggerAxis());

              // Apply deadband
              double linearMagnitude =
                  MathUtil.applyDeadband(Math.hypot(joystickX, joystickY), 0.05);
              Rotation2d linearDirection =
                  new Rotation2d(Math.atan2(joystickX, joystickY))
                      .plus(
                          DriverStation.getAlliance().isPresent()
                                  && DriverStation.getAlliance().get() == Alliance.Red
                              ? Rotation2d.kPi
                              : Rotation2d.kZero);

              linearMagnitude = linearMagnitude * linearMagnitude;

              // Return new linear velocity
              Translation2d linearVelocity =
                  new Pose2d(Translation2d.kZero, linearDirection)
                      .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
                      .getTranslation();

              drive.setControl(
                  fieldCentric
                      .withVelocityX(linearVelocity.getY() * MaxSpeed.getAsDouble())
                      .withVelocityY(linearVelocity.getX() * MaxSpeed.getAsDouble())
                      .withRotationalRate(
                          MathUtil.applyDeadband(joystickRotation, 0.05)
                              * MaxAngularRate.getAsDouble()));
            }));
  }

  public Command getAutonomousCommand() {
    return new InstantCommand(
            () ->
                drive.resetRotation(
                    DriverStation.getAlliance().isPresent()
                            && DriverStation.getAlliance().get() == Alliance.Red
                        ? Rotation2d.kZero
                        : Rotation2d.kPi))
        .andThen(autoBuilder.buildAutoCommand(drive, elevator, flywheels, funnel));
  }
}
