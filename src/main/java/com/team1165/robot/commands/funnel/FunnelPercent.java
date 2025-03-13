/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.commands.funnel;

import com.team1165.robot.subsystems.funnel.Funnel;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class FunnelPercent extends Command {
  private final Funnel funnel;
  private final DoubleSupplier supplier;

  public FunnelPercent(Funnel funnel, DoubleSupplier supplier) {
    this.funnel = funnel;
    this.supplier = supplier;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.funnel);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    funnel.runPercent(supplier.getAsDouble());
  }

  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    funnel.runPercent(0);
  }
}
