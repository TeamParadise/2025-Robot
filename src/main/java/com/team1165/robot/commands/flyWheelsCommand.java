/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.commands;

import com.team1165.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;

public class flyWheelsCommand extends Command {
  private double speed1;
  private double speed2;

  public flyWheelsCommand(double speed1, double speed2) {
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    this.speed1 = speed1;
    this.speed2 = speed2;
    addRequirements(RobotContainer.neoSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    RobotContainer.neoSubsystem.runBothNeos(speed1, speed2);
  }

  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.neoSubsystem.stopBothNeos();
  }
}
