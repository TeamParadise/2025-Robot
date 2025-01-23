/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.Commands;

import com.team1165.robot.RobotContainer;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorCommand extends Command {
  private Distance height;
  private Double h;

  public ElevatorCommand(Distance height) {
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(RobotContainer.elevator);
    this.height = height;
    //    SmartDashboard.putNumber("Elevator speed:", 1.0); can't do that if we use set position,
    // figruring it out.
  }

  public ElevatorCommand(Double h) {
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(RobotContainer.elevator);
    this.h = h;
    //    SmartDashboard.putNumber("Elevator speed:", 1.0); can't do that if we use set position,
    // figruring it out.
  }

  @Override
  public void initialize() {
    RobotContainer.elevator.setPID(0.3, 0.02, 0.05);
  }

  @Override
  public void execute() {
    RobotContainer.elevator.setPosition(h, 3.0);
    // don't forget to add wait timeout to this
  }

  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
