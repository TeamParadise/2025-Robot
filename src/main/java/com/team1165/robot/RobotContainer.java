/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team1165.robot.subsystems.drive.Drive;
import com.team1165.robot.subsystems.drive.constants.TunerConstants;
import com.team1165.robot.subsystems.drive.io.DriveIO;
import com.team1165.robot.subsystems.drive.io.DriveIOMapleSim;
import com.team1165.robot.subsystems.drive.io.DriveIOMapleSim.MapleSimConfig;
import com.team1165.robot.subsystems.drive.io.DriveIOReal;
import com.team1165.robot.subsystems.vision.apriltag.ATVision;
import com.team1165.robot.subsystems.vision.apriltag.io.ATVisionIOPhoton.ATVisionIOPhotonConfig;
import com.team1165.robot.subsystems.vision.apriltag.io.ATVisionIOPhotonSim;
import com.team1165.robot.subsystems.vision.apriltag.io.ATVisionIOPhotonSim.ATVisionIOPhotonSimConfig;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
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
  private final ATVision apriltagVision;

  // Driver Controllers
  private final CommandXboxController driverController = new CommandXboxController(0);

  // Testing, likely will be changed later
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  private final SwerveRequest.FieldCentric fieldCentric =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1)
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.robotMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new DriveIOReal(
                    TunerConstants.DrivetrainConstants,
                    TunerConstants.FrontLeft,
                    TunerConstants.FrontRight,
                    TunerConstants.BackLeft,
                    TunerConstants.BackRight));
        apriltagVision =
            new ATVision(
                drive::addVisionMeasurement,
                new ATVisionIOPhotonSim("test", new Transform3d(), drive::getSimulationPose));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new DriveIOMapleSim(
                    TunerConstants.DrivetrainConstants,
                    new MapleSimConfig(
                        Seconds.of(0.002),
                        Pounds.of(115),
                        Inches.of(30),
                        Inches.of(30),
                        DCMotor.getKrakenX60Foc(1),
                        DCMotor.getFalcon500(1),
                        1.5),
                    TunerConstants.FrontLeft,
                    TunerConstants.FrontRight,
                    TunerConstants.BackLeft,
                    TunerConstants.BackRight));
        apriltagVision =
            new ATVision(
                drive::addVisionMeasurement,
                new ATVisionIOPhotonSim(
                    new ATVisionIOPhotonConfig("test", new Transform3d()),
                    new ATVisionIOPhotonSimConfig(960, 720)
                        .withCalibError(1, 0.1)
                        .withLatency(0, 0)
                        .withFPS(150),
                    drive::getSimulationPose));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive = new Drive(new DriveIO() {});
        apriltagVision =
            new ATVision(
                drive::addVisionMeasurement,
                new ATVisionIOPhotonSim("test", new Transform3d(), drive::getSimulationPose));
        break;
    }

    configureButtonBindings();
    configureDefaultCommands();
  }

  /** Use this method to define your button->command mappings. */
  private void configureButtonBindings() {}

  /** Use this method to define default commands for subsystems. */
  private void configureDefaultCommands() {
    drive.setDefaultCommand(
        drive.applyRequest(
            () ->
                fieldCentric
                    .withVelocityX(-driverController.getLeftY() * MaxSpeed)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate)));
  }
}
