/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.commands;

import com.team1165.robot.OdysseusManager;
import com.team1165.robot.OdysseusState;
import com.team1165.robot.subsystems.roller.flywheel.sensor.DetectionMode;
import com.team1165.robot.util.logging.LoggedTunableNumber;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class Intake extends Command {

  private static final LoggedTunableNumber currentThreshold =
      new LoggedTunableNumber("Commands/Intake/OverCurrentThreshold", 14);
  private static final LoggedTunableNumber currentElapsedTime =
      new LoggedTunableNumber("Commands/Intake/OverCurrentTime", 0.04);
  private static final LoggedTunableNumber sensorElapsedTime = new LoggedTunableNumber(
      "Commands/Intake/SensorElapsedTime", 0.04);
  private static final LoggedTunableNumber spinupTime =
      new LoggedTunableNumber("Commands/Intake/SpinupTime", 0.15);

  private final OdysseusManager robot;
  private final Timer timer = new Timer();
  private double startingDetectionTimestamp = 0.0;
  private boolean endCommand = false;

  public Intake(OdysseusManager robot) {
    this.robot = robot;
    addRequirements(this.robot);
  }

  @Override
  public void initialize() {
    timer.restart();
    endCommand = false;
    robot.setState(OdysseusState.INTAKE);
  }

  @Override
  public void execute() {
    // TODO: Add a check to make sure elevator is down before fully intaking? Additional state.
    var detectionMode = robot.getFlywheelDetectionMode();
    var isCoralDetected =
        detectionMode == DetectionMode.DISTANCE_SENSOR ? robot.getFlywheelSensorHold()
            : robot.getFlywheelCurrentHold(currentThreshold.get());

    if (timer.hasElapsed(spinupTime.get())) {
      if (isCoralDetected && startingDetectionTimestamp == 0.0) {
        startingDetectionTimestamp = timer.get();
      }

      if (isCoralDetected && timer.get() - startingDetectionTimestamp > (
          detectionMode == DetectionMode.DISTANCE_SENSOR ? sensorElapsedTime.get()
              : currentElapsedTime.get())) {
        endCommand = true;
        robot.setState(OdysseusState.IDLE);
      } else {
        startingDetectionTimestamp = 0.0;
      }
    }
  }

  @Override
  public boolean isFinished() {
    return endCommand;
  }

  @Override
  public void end(boolean interrupted) {
    robot.setState(OdysseusState.IDLE);
  }
}
