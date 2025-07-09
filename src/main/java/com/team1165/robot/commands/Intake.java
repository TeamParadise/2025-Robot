/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.commands;

import com.team1165.robot.subsystems.elevator.Elevator;
import com.team1165.robot.subsystems.roller.flywheel.Flywheel;
import com.team1165.robot.subsystems.roller.funnel.Funnel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class Intake extends Command {
  private final Elevator elevator;
  private final Flywheel flywheel;
  private final Funnel funnel;
  private final Timer timer = new Timer();
  private final double elapsedTime = 0.04;

  private double startingTimestampCurrent = 0.0;
  private boolean didDrawHighCurrent = false;
  private boolean endCommand = false;

  public Intake(Elevator elevator, Flywheel flywheel, Funnel funnel) {
    this.elevator = elevator;
    this.flywheel = flywheel;
    this.funnel = funnel;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.elevator, /* this.flywheels,*/ this.funnel);
  }

  @Override
  public void initialize() {
    timer.restart();
    startingTimestampCurrent = 0.0;
    didDrawHighCurrent = false;
    endCommand = false;
  }

  @Override
  public void execute() {
    //    if (timer.hasElapsed(0.15)) {
    //      if (flywheels.isDrawingHighCurrent() && !didDrawHighCurrent) {
    //        startingTimestampCurrent = timer.get();
    //        didDrawHighCurrent = true;
    //      }
    //      if (flywheels.isDrawingHighCurrent()) {
    //        didDrawHighCurrent = true;
    //        if (timer.get() - startingTimestampCurrent > elapsedTime) {
    //          endCommand = true;
    //          flywheels.stop();
    //          // funnel.setState(FunnelState.IDLE);
    //        }
    //      } else if (!flywheels.isDrawingHighCurrent() && didDrawHighCurrent) {
    //        didDrawHighCurrent = false;
    //      }
    //    }
    //
    //    flywheels.runPercent(0.22);
    //    // funnel.setState(FunnelState.INTAKE); // Ideally, this might go in init, to avoid repeat
    //    // calls?
    //    elevator.runToIntakePosition();
  }

  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return endCommand;
  }

  @Override
  public void end(boolean interrupted) {
    //    flywheels.stop();
    // funnel.setState(FunnelState.IDLE);
  }
}
