/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.util.auto;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team1165.robot.OdysseusManager;
import com.team1165.robot.commands.Intake;
import com.team1165.robot.commands.RobotCommands;
import com.team1165.robot.commands.drivetrain.DriveToPose;
import com.team1165.robot.subsystems.drive.Drive;
import com.team1165.robot.util.constants.RobotMode;
import com.team1165.robot.util.constants.RobotMode.Mode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/** Class that represents a full auto routine. */
public class AutoRoutine {
  private final boolean pushPartner;
  private final AutoSegmentConfig[] segments;
  private final Command command;

  public AutoRoutine(OdysseusManager robot, Drive drive, boolean pushPartner, AutoSegmentConfig... segments) {
    this.pushPartner = pushPartner;
    this.segments = segments;

    var autoCommand =
        pushPartner
            ? drive
            .applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(-1.5))
            .withTimeout(0.75)
            : Commands.none();

    for (AutoSegmentConfig segment : segments) {
      autoCommand =
          autoCommand.andThen(
              RobotCommands.autoScore(robot, drive, segment::reefLocation, segment::reefLevel)
                  .andThen(
                      new DriveToPose(drive, () -> segment.coralStation().getPose())
                          .raceWith(
                              new Intake(robot)
                                  // Every thing below is just for simulating intake in sim.
                                  .until(
                                      () ->
                                          RobotMode.get() == Mode.SIM
                                              && drive
                                              .getPose()
                                              .getTranslation()
                                              .getDistance(
                                                  segment
                                                      .coralStation()
                                                      .getPose()
                                                      .getTranslation())
                                              < 0.05)))
                  .andThen(RobotMode.get() == Mode.SIM ? new WaitCommand(0.7) : Commands.none()));
    }

      this.command = autoCommand;
  }

  public Command getAutoCommand() {
    return command;
  }
}
