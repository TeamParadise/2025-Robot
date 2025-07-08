/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.roller.flywheel.constants;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.team1165.robot.util.vendor.rev.SparkConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;

/** Class to store constants for the Flywheel subsystem of the robot. */
public class FlywheelConstants {
  /** Class to store constants required for initial configuration of the Flywheel. */
  public static final class Configurations {
    // SPARK MAX configurations
    private static final SparkBaseConfig baseMotorConfig =
        new SparkMaxConfig().smartCurrentLimit(50).idleMode(IdleMode.kBrake);
    public static final SparkConfig primaryMotorConfig =
        SparkConfig.sparkMax("FlywheelPrimary", 1, MotorType.kBrushless, baseMotorConfig);
    public static final SparkConfig secondaryMotorConfig =
        SparkConfig.sparkMax(
            "FlywheelSecondary",
            2,
            MotorType.kBrushless,
            baseMotorConfig); // TODO: Likely will need to invert this motor?

    // Simulation motor configuration
    public static final SimMotorConfigs simConfig =
        new SimMotorConfigs(DCMotor.getNEO(1), 1, KilogramSquareMeters.of(0.002), Volts.of(0.05));
  }
}
