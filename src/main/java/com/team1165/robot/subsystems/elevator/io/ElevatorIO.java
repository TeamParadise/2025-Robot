/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.elevator.io;

import com.ctre.phoenix6.configs.Slot0Configs;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  class ElevatorIOInputs {
    public boolean leftMotorConnected = true;
    public boolean rightMotorConnected = true;

    public double leftPositionInches = 0.0;
    public double leftVelocityRpm = 0.0;
    public double leftAppliedVolts = 0.0;
    public double leftSupplyCurrentAmps = 0.0;
    public double leftTempCelsius = 0.0;

    public double rightPositionInches = 0.0;
    public double rightVelocityRpm = 0.0;
    public double rightAppliedVolts = 0.0;
    public double rightSupplyCurrentAmps = 0.0;
    public double rightTempCelsius = 0.0;
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  /** Stop both Elevator Motors */
  default void stop() {}

  default void runCurrent(double current) {}

  default void runVolts(double volts) {}

  default void runPosition(double positionInches) {}

  default void setBrakeMode(boolean enabled) {}

  /** Config PID values for both motors */
  default void setPID(Slot0Configs gains) {}
}
