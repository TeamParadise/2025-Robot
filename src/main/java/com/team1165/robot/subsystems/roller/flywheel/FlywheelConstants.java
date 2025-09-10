/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.roller.flywheel;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.team1165.robot.globalconstants.CANConstants.IDs.RIO;
import com.team1165.robot.util.vendor.ctre.PhoenixDeviceConfigs.CANrangeConfig;
import com.team1165.robot.util.vendor.rev.SparkConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;

/** Class to store constants for the {@link Flywheel} subsystem of the robot. */
public class FlywheelConstants {
  // Base SPARK configuration
  public static final SparkBaseConfig primaryMotor =
      new SparkMaxConfig().smartCurrentLimit(50).idleMode(IdleMode.kBrake);
  public static final SparkBaseConfig secondaryMotor =
      new SparkMaxConfig().smartCurrentLimit(50).idleMode(IdleMode.kBrake).inverted(true);

  // Individual SPARK MAX configurations
  public static final SparkConfig primaryMotorConfig =
      SparkConfig.sparkMax(
          "FlywheelPrimary", RIO.flywheelPrimary, MotorType.kBrushless, primaryMotor);
  public static final SparkConfig secondaryMotorConfig =
      SparkConfig.sparkMax(
          "FlywheelSecondary", RIO.flywheelSecondary, MotorType.kBrushless, secondaryMotor);

  // Simulation motor configuration
  public static final SimMotorConfigs simConfig =
      new SimMotorConfigs(DCMotor.getNEO(1), 1, KilogramSquareMeters.of(0.002), Volts.of(0.05));

  // Sensor configuration
  public static final CANrangeConfiguration baseSensorConfig = new CANrangeConfiguration();
  public static final CANrangeConfig sensorConfig = new CANrangeConfig("FlywheelSensor", RIO.flywheelSensor, "rio", baseSensorConfig);
}
