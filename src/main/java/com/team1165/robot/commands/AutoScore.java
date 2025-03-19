/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.commands;

import com.team1165.robot.FieldConstants.Reef;
import com.team1165.robot.subsystems.drive.Drive;
import com.team1165.robot.subsystems.elevator.Elevator;
import com.team1165.robot.subsystems.flywheels.Flywheels;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoScore extends Command {
  private final Drive drive;
  private final Elevator elevator;
  private final Flywheels flywheels;

  public AutoScore(
      Reef.Location reefLocation,
      Reef.Level reefLevel,
      Drive drive,
      Elevator elevator,
      Flywheels flywheels) {
    this.drive = drive;
    this.elevator = elevator;
    this.flywheels = flywheels;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.elevator, this.flywheels);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
