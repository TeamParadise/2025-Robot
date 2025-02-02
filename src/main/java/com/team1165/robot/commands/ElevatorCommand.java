/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.commands;

import com.team1165.robot.RobotContainer;
import edu.wpi.first.units.Units;
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
    System.out.println("ElevatorCommand started, target height: " + h);
    RobotContainer.elevator.setPID(0.3, 0.02, 0.05);
  }

  @Override
  public void execute() {
    if (h != null) {
      RobotContainer.elevator.setPosition(h, 0.2);
    } else if (height != null) {
      RobotContainer.elevator.setPosition(height.in(Units.Inches), 1.0);
    } else {
      System.err.println("Error: ElevatorCommand received no valid height!");
    }
    double currentPosition = RobotContainer.elevator.getLastDesiredPosition();
    if (Math.abs(currentPosition - h) < 1) System.out.println("finished");
    else {
      System.out.println(currentPosition);
    }
  }

  @Override
  public boolean isFinished() {
    double currentPosition = RobotContainer.elevator.getLastDesiredPosition();

    return Math.abs(currentPosition - h) < 0.01; // Stop when close to target
  }

  @Override
  public void end(boolean interrupted) {}
}
