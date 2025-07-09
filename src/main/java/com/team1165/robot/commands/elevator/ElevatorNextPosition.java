/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.commands.elevator;

import com.team1165.robot.globalconstants.FieldConstants.Reef;
import com.team1165.robot.subsystems.elevator.Elevator;
import com.team1165.robot.subsystems.elevator.constants.ElevatorConstants;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorNextPosition extends Command {
  private final Elevator elevator;
  private double position = 0.0;

  public ElevatorNextPosition(Elevator elevator) {
    this.elevator = elevator;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.elevator);
  }

  @Override
  public void initialize() {
    if (elevator.getAtPosition(ElevatorConstants.intakePosition, 0.2)) {
      position = Reef.Level.L1.getElevatorHeight();
    } else if (elevator.getAtPosition(Reef.Level.L1.getElevatorHeight(), 0.2)) {
      position = Reef.Level.L2.getElevatorHeight();
    } else if (elevator.getAtPosition(Reef.Level.L2.getElevatorHeight(), 0.2)) {
      position = Reef.Level.L3.getElevatorHeight();
    } else if (elevator.getAtPosition(Reef.Level.L3.getElevatorHeight(), 0.2)) {
      position = Reef.Level.L4.getElevatorHeight();
    } else {
      position = ElevatorConstants.intakePosition;
    }
  }

  @Override
  public void execute() {
    elevator.runPosition(position);
  }

  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
