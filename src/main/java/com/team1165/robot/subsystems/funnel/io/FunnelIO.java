// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1165.robot.subsystems.funnel.io;

import org.littletonrobotics.junction.AutoLog;

public interface FunnelIO {
  @AutoLog
  class FunnelIOInputs {
    public double leftPositionRads = 0.0;
    public double leftVelocityRpm = 0.0;
    public double leftAppliedVolts = 0.0;
    public double leftSupplyCurrentAmps = 0.0;
    public double leftTempCelsius = 0.0;

    public double rightPositionRads = 0.0;
    public double rightVelocityRpm = 0.0;
    public double rightAppliedVolts = 0.0;
    public double rightSupplyCurrentAmps = 0.0;
    public double rightTempCelsius = 0.0;
  }

  /** Update inputs */
  default void updateInputs(FunnelIOInputs inputs) {}

  /** Run both motors at voltage */
  default void runVolts(double leftVolts, double rightVolts) {}

  /** Run both motors at a percent */
  default void runPercent(double leftPercent, double rightPercent) {}

  /** Stop both flywheels */
  default void stop() {}
}
