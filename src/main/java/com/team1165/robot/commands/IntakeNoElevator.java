/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.commands;

import com.team1165.robot.subsystems.flywheels.Flywheels;
import com.team1165.robot.subsystems.funnel.Funnel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeNoElevator extends Command {
  private final Flywheels flywheels;
  private final Funnel funnel;
  private final double elapsedTime = 0.13;

  private double lastTimestamp = 0.0;
  private double timeElapsed = 0.0;
  private boolean didDrawHighCurrent = false;
  private boolean endCommand = false;

  public IntakeNoElevator(Flywheels flywheels, Funnel funnel) {
    this.flywheels = flywheels;
    this.funnel = funnel;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.flywheels, this.funnel);
  }

  @Override
  public void initialize() {
    timeElapsed = 0.0;
    didDrawHighCurrent = false;
    endCommand = false;
  }

  @Override
  public void execute() {
    if (flywheels.isDrawingHighCurrent()) {
      didDrawHighCurrent = true;
      timeElapsed += Timer.getTimestamp() - lastTimestamp;
      if (timeElapsed > elapsedTime) {
        endCommand = true;
        flywheels.stop();
        funnel.stop();
      }
    } else if (!flywheels.isDrawingHighCurrent() && didDrawHighCurrent) {
      didDrawHighCurrent = false;
      timeElapsed = 0;
    }
    lastTimestamp = Timer.getTimestamp();

    flywheels.runPercent(0.2);
    funnel.runPercent(0.15);
  }

  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return endCommand;
  }

  @Override
  public void end(boolean interrupted) {
    flywheels.stop();
    funnel.stop();
  }
}
