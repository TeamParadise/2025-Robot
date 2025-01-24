/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  class ElevatorIOInputs {
    public boolean leftMotorConnected = true;
    public boolean rightMotorConnected = true;

    Distance currentLeftPosition = Units.Inches.of(0);
    public double leftVelocityRpm = 0.0;
    public double leftAppliedVolts = 0.0;
    public double leftSupplyCurrentAmps = 0.0;
    public double leftTempCelsius = 0.0;

    Distance currentRightPosition = Units.Inches.of(0);
    public double rightVelocityRpm = 0.0;
    public double rightAppliedVolts = 0.0;
    public double rightSupplyCurrentAmps = 0.0;
    public double rightTempCelsius = 0.0;
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void resetSensorPosition(Distance setpoint) {}

  /** Run both motors at voltage */
  default void setPosition(Distance height) {}

  /** Stop both Elevator Motors */
  default void stop() {}

  default void runVolts(double leftVoltage, double rightVoltage) {}

  default void setPosition(Double height, double velocity) {}

  /** Config PID values for both motors */
  default void setPID(double kP, double kI, double kD) {}

  /** Config FF values for both motors */
  default void setFF(double kS, double kV, double kA) {}

  /** Run left flywheels at voltage */
  default void runCharacterizationLeft(double input) {}

  /** Run right flywheels at voltage */
  default void runCharacterizationRight(double input) {}
}
