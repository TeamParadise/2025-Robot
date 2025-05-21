/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.util.auto;

import static edu.wpi.first.units.Units.Seconds;

import com.team1165.robot.FieldConstants.*;
import com.team1165.robot.commands.Intake;
import com.team1165.robot.commands.drivetrain.DriveToPose;
import com.team1165.robot.commands.elevator.ElevatorPosition;
import com.team1165.robot.commands.flywheels.FlywheelsPercenmt;
import com.team1165.robot.subsystems.drive.Drive;
import com.team1165.robot.subsystems.elevator.Elevator;
import com.team1165.robot.subsystems.flywheels.Flywheels;
import com.team1165.robot.subsystems.roller.funnel.Funnel;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/** Class that represents a full auto routine. */
public class AutoRoutine {
  private final Time delayBeforeStart;
  private final boolean pushPartner;
  private final AutoSegmentConfig[] segments;
  private static final double translationToleranceBeforeScoring = 0.4;
  private static final double rotationToleranceBeforeScoring = 0.78539816339;

  public AutoRoutine(Time delayBeforeStart, boolean pushPartner, AutoSegmentConfig... segments) {
    this.delayBeforeStart = delayBeforeStart;
    this.pushPartner = pushPartner;
    this.segments = segments;
  }

  public AutoRoutine(Time delayBeforeStart, AutoSegmentConfig... segments) {
    this(delayBeforeStart, false, segments);
  }

  public AutoRoutine(boolean pushPartner, AutoSegmentConfig... segments) {
    this(Seconds.zero(), pushPartner, segments);
  }

  public AutoRoutine(AutoSegmentConfig... segments) {
    this(Seconds.zero(), false, segments);
  }

  public Command getAutoCommand(
      Drive drive, Elevator elevator, Flywheels flywheels, Funnel funnel) {
    var initialDriveCommand =
        new DriveToPose(
            drive,
            () ->
                segments[0]
                    .reefLocation()
                    .getPose()
                    .transformBy(new Transform2d(-0.3, 0.0, Rotation2d.kZero)));
    var nextDriveCommand = new DriveToPose(drive, () -> segments[0].reefLocation().getPose());
    var secondLineupCommand =
        new DriveToPose(
            drive,
            () ->
                segments[1]
                    .reefLocation()
                    .getPose()
                    .transformBy(new Transform2d(-0.3, 0.0, Rotation2d.kZero)));
    var secondScoreCommand = new DriveToPose(drive, () -> segments[1].reefLocation().getPose());
    var intake = new Intake(elevator, flywheels, funnel);

    return new ElevatorPosition(elevator, () -> segments[0].reefLevel().getElevatorHeight())
        .alongWith(
            initialDriveCommand
                .until(() -> elevator.getAtPosition(segments[0].reefLevel().getElevatorHeight(), 1))
                .andThen(
                    nextDriveCommand
                        .alongWith(
                            new WaitCommand(2).andThen(new FlywheelsPercenmt(flywheels, () -> 0.3)))
                        .withTimeout(3)))
        .withTimeout(5)
        .andThen(
            new DriveToPose(
                    drive,
                    () ->
                        segments[0]
                            .reefLocation()
                            .getPose()
                            .transformBy(new Transform2d(-0.3, 0.0, Rotation2d.kZero)))
                .withTimeout(0.75)
                .andThen(new DriveToPose(drive, () -> segments[0].coralStation().getPose()))
                .alongWith(intake))
        .until(intake::isFinished)
        .andThen(
            new ElevatorPosition(elevator, () -> segments[1].reefLevel().getElevatorHeight())
                .alongWith(
                    secondLineupCommand
                        .until(
                            () ->
                                elevator.getAtPosition(
                                    segments[1].reefLevel().getElevatorHeight(), 1))
                        .andThen(
                            secondScoreCommand
                                .alongWith(
                                    new WaitCommand(2.3)
                                        .andThen(new FlywheelsPercenmt(flywheels, () -> 0.3)))
                                .withTimeout(3)))
                .withTimeout(4))
        .andThen(
            new DriveToPose(
                    drive,
                    () ->
                        segments[1]
                            .reefLocation()
                            .getPose()
                            .transformBy(new Transform2d(-0.3, 0.0, Rotation2d.kZero)))
                .alongWith(new ElevatorPosition(elevator, () -> 0)));
  }
}
