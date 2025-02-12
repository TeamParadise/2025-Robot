/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.Pivot;


import org.littletonrobotics.junction.AutoLog;

  public interface PivotIO {
    @AutoLog
    class PivotIOInputs {
      public boolean MotorConnected = true;

      public double PositionRads = 0.0;
      public double VelocityRpm = 0.0;
      public double AppliedVolts = 0.0;
      public double SupplyCurrentAmps = 0.0;
      public double TorqueCurrentAmps = 0.0;
      public double TempCelsius = 0.0;

    }

    /** Update inputs */
    default void updateInputs(PivotIOInputs inputs) {}

    /** Run both motors at voltage */
    default void runVolts(double angle) {}

    default void setPivotAngle(double angle) {};

    /** Stop both flywheels */
    default void stop() {}

    /** Run left and right flywheels at velocity in rpm */
    default void runVelocity(
        double leftRpm, double rightRpm, double leftFeedforward, double rightFeedforward) {}

    /** Config PID values for both motors */
    default void setPID(double kP, double kI, double kD) {}

    /** Config FF values for both motors */
    default void setFF(double kS, double kV, double kA) {}

    /** Run left flywheels at voltage */
    default void runCharacterizationLeft(double input) {}

    /** Run right flywheels at voltage */
    default void runCharacterizationRight(double input) {}
  }


