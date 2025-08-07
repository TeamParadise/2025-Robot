/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.elevator.io;

import com.team1165.robot.util.logging.MotorData.GenericMotorData;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class ElevatorIOTempSim implements ElevatorIO {
  private final GenericMotorData primary;
  private double setpoint = 0.0;
  private final SlewRateLimiter slewRateLimiter = new SlewRateLimiter(10);

  public ElevatorIOTempSim() {
    primary = new GenericMotorData();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    primary.update(0.0, 0.0, slewRateLimiter.calculate(setpoint), 0, 0);
    inputs.primaryMotor = primary;
  }

  @Override
  public void runPosition(double position) {
    setpoint = position;
  }
}
