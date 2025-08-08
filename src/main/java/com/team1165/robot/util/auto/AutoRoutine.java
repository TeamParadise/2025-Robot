/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.util.auto;

import static edu.wpi.first.units.Units.Seconds;

import com.team1165.robot.OdysseusManager;
import com.team1165.robot.commands.RobotCommands;
import com.team1165.robot.commands.drivetrain.DriveToPose;
import com.team1165.robot.subsystems.drive.Drive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** Class that represents a full auto routine. */
public class AutoRoutine {
  private final AutoSegmentConfig[] segments;

  public AutoRoutine(AutoSegmentConfig... segments) {
    this.segments = segments;
  }

  public Command getAutoCommand(OdysseusManager robot, Drive drive) {
    var autoCommand = Commands.none();

    for (AutoSegmentConfig segment : segments) {
      autoCommand =
          autoCommand.andThen(
              RobotCommands.autoScore(robot, drive, segment::reefLocation, segment::reefLevel)
                  .andThen(new DriveToPose(drive, () -> segment.coralStation().getPose())));
    }
    //    var initialDriveCommand =
    //        new DriveToPose(
    //            drive,
    //            () ->
    //                segments[0]
    //                    .reefLocation()
    //                    .getPose()
    //                    .transformBy(new Transform2d(-0.3, 0.0, Rotation2d.kZero)));
    //    var nextDriveCommand = new DriveToPose(drive, () -> segments[0].reefLocation().getPose());
    //    var secondLineupCommand =
    //        new DriveToPose(
    //            drive,
    //            () ->
    //                segments[1]
    //                    .reefLocation()
    //                    .getPose()
    //                    .transformBy(new Transform2d(-0.3, 0.0, Rotation2d.kZero)));
    //    var secondScoreCommand = new DriveToPose(drive, () ->
    // segments[1].reefLocation().getPose());
    //    var intake = new Intake(elevator, flywheel, funnel);
    //
    //    return new ElevatorPosition(elevator, () -> segments[0].reefLevel().getElevatorHeight())
    //        .alongWith(
    //            initialDriveCommand
    //                .until(() ->
    // elevator.getAtPosition(segments[0].reefLevel().getElevatorHeight(), 1))
    //                .andThen(
    //                    nextDriveCommand
    //                        .alongWith(
    //                            new WaitCommand(2).andThen(new FlywheelsPercenmt(flywheels, () ->
    // 0.2)))
    //                        .withTimeout(3)))
    //        .withTimeout(5)
    //        .andThen(
    //            new DriveToPose(
    //                    drive,
    //                    () ->
    //                        segments[0]
    //                            .reefLocation()
    //                            .getPose()
    //                            .transformBy(new Transform2d(-0.3, 0.0, Rotation2d.kZero)))
    //                .withTimeout(0.75)
    //                .andThen(new DriveToPose(drive, () -> segments[0].coralStation().getPose()))
    //                .alongWith(intake))
    //        .until(intake::isFinished)
    //        .andThen(
    //            new ElevatorPosition(elevator, () -> segments[1].reefLevel().getElevatorHeight())
    //                .alongWith(
    //                    secondLineupCommand
    //                        .until(
    //                            () ->
    //                                elevator.getAtPosition(
    //                                    segments[1].reefLevel().getElevatorHeight(), 1))
    //                        .andThen(
    //                            secondScoreCommand
    //                                .alongWith(
    //                                    new WaitCommand(2.3)
    //                                        .andThen(new FlywheelsPercenmt(flywheels, () ->
    // 0.25)))
    //                                .withTimeout(3)))
    //                .withTimeout(4))
    //        .andThen(
    //            new DriveToPose(
    //                    drive,
    //                    () ->
    //                        segments[1]
    //                            .reefLocation()
    //                            .getPose()
    //                            .transformBy(new Transform2d(-0.3, 0.0, Rotation2d.kZero)))
    //                .alongWith(new ElevatorPosition(elevator, () -> 0)));
  }
}
