/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.roller.funnel;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.team1165.robot.globalconstants.Constants;
import com.team1165.robot.util.vendor.rev.SparkConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;

/** Class to store constants for the {@link Funnel} subsystem of the robot. */
public class FunnelConstants {
  // SPARK MAX configurations
  private static final SparkBaseConfig baseMotorConfig =
      new SparkMaxConfig().smartCurrentLimit(50).idleMode(IdleMode.kBrake);
  public static final SparkConfig primaryMotorConfig =
      SparkConfig.sparkMax("FunnelPrimary", 3, MotorType.kBrushless, baseMotorConfig);
  public static final SparkConfig secondaryMotorConfig =
      SparkConfig.sparkMax(
          "FunnelSecondary",
          Constants.CANIDs,
          MotorType.kBrushless,
          baseMotorConfig.follow(primaryMotorConfig.canId(), true));

  // Simulation motor configuration
  public static final SimMotorConfigs simConfig =
      new SimMotorConfigs(DCMotor.getNEO(1), 1, KilogramSquareMeters.of(0.002), Volts.of(0.05));
}
