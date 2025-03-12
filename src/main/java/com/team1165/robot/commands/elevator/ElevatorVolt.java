/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.commands.elevator;

import com.team1165.robot.subsystems.elevator.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class ElevatorVolt extends Command {
  private final Elevator elevator;
  private final DoubleSupplier position;

  public ElevatorVolt(Elevator elevator, DoubleSupplier position) {
    this.elevator = elevator;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.elevator);
    this.position = position;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    elevator.runVolts(position.getAsDouble());
  }

  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
