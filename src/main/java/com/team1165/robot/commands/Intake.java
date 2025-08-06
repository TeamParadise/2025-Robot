/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.commands;

import com.team1165.robot.OdysseusManager;
import com.team1165.robot.OdysseusState;
import com.team1165.robot.util.logging.LoggedTunableNumber;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class Intake extends Command {
  private static final LoggedTunableNumber currentThreshold =
      new LoggedTunableNumber("Commands/Intake/OverCurrentThreshold", 11.5);
  private static final LoggedTunableNumber elapsedTime =
      new LoggedTunableNumber("Commands/Intake/OverCurrentTime", 0.04);
  private static final LoggedTunableNumber spinupTime =
      new LoggedTunableNumber("Commands/Intake/SpinupTime", 0.15);

  private final OdysseusManager robot;
  private final Timer timer = new Timer();
  private double startingTimestampCurrent = 0.0;
  private boolean didDrawHighCurrent = false;
  private boolean endCommand = false;

  public Intake(OdysseusManager robot) {
    this.robot = robot;
    addRequirements(this.robot);
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
    // TODO: Add a check to make sure elevator is down before fully intaking? Additional state.
    var drawingHighCurrent = Math.abs(robot.getFlywheelCurrent()) > currentThreshold.get();

    if (timer.hasElapsed(spinupTime.get())) {
      if (drawingHighCurrent && !didDrawHighCurrent) {
        startingTimestampCurrent = timer.get();
        didDrawHighCurrent = true;
      }

      if (drawingHighCurrent) {
        if (timer.get() - startingTimestampCurrent > elapsedTime.get()) {
          endCommand = true;
          robot.setState(OdysseusState.IDLE);
        }
      } else if (didDrawHighCurrent) {
        didDrawHighCurrent = false;
      }
    }

    robot.setState(OdysseusState.INTAKE);
  }

  @Override
  public boolean isFinished() {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return endCommand;
  }

  @Override
  public void end(boolean interrupted) {
    robot.setState(OdysseusState.IDLE);
  }
}
