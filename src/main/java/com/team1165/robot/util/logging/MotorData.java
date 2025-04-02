/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.util.logging;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.team1165.robot.util.vendor.TalonFXFaults;
import com.team1165.robot.util.vendor.rev.SparkFaults;
import java.util.function.Function;

/**
 * Record that provides easy logging of all the values needed from a motor to log through
 * AdvantageKit.
 */
public record MotorData(
    boolean faultActive,
    SparkFaults sparkFaults,
    TalonFXFaults talonFaults,
    double positionRotations,
    double velocityRpm,
    double appliedVolts,
    double supplyCurrentAmps,
    // Torque current output (positive and negative) of a Talon FX, just motor current output (just
    // positive) of a SPARK.
    double outputCurrentAmps,
    double temperatureCelsius,
    boolean connected) {
  public static final MotorData empty =
      new MotorData(false, null, null, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, false);

  public static MotorData getFromSpark(MotorData previousData, SparkBase spark, RelativeEncoder encoder, Function<Boolean, Boolean> getConnected) {
    return new MotorData(
        spark.hasActiveFault(),
        SparkFaults.getFromFaults(spark.getFaults()),
        null,
        encoder.getPosition(),
        encoder.getVelocity()
    )
  }
}
