/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.util.auto;

import com.team1165.robot.OdysseusManager;
import com.team1165.robot.commands.Intake;
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
                  .andThen(new DriveToPose(drive, () -> segment.coralStation().getPose()))
                  .alongWith(new Intake(robot)));
    }

    return Commands.none();
  }
}
