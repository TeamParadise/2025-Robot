/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import com.team1165.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

public class DriveToPose extends Command {
  private final Drive drive;
  private final Supplier<Pose2d> pose;

  public DriveToPose(Drive drive, Supplier<Pose2d> pose) {
    this.drive = drive;
    this.pose = pose;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.drive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    drive.goToPose(pose.get());
  }

  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
