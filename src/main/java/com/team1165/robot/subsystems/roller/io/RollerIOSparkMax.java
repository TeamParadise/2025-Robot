/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.roller.io;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.team1165.robot.util.vendor.rev.SparkFullConfigs.SparkFullConfig;
import com.team1165.robot.util.vendor.rev.SparkModel;
import com.team1165.robot.util.vendor.rev.SparkUtil;

/**
 * A hardware interface/implementation layer for a basic wheel/roller subsystem powered by two
 * motors attached to SPARK MAX/FLEX motor controllers. These two motors are usually controlled
 * together, but they can be controlled separately if needed.
 */
public class RollerIOSparkMax implements RollerIO {
  private final SparkBase primaryMotor;
  private final SparkBase secondaryMotor;
  private final SparkBaseConfig primaryConfig;
  private final SparkBaseConfig secondaryConfig;
  private final RelativeEncoder primaryEncoder;
  private final RelativeEncoder secondaryEncoder;

  public RollerIOSparkMax(SparkFullConfig primaryFullConfig, SparkFullConfig secondaryFullConfig) {
    // Assign motor variables
    primaryMotor =
        primaryFullConfig.model == SparkModel.SparkFlex
            ? new SparkFlex(primaryFullConfig.canId, primaryFullConfig.motorType)
            : new SparkMax(primaryFullConfig.canId, primaryFullConfig.motorType);
    secondaryMotor =
        secondaryFullConfig.model == SparkModel.SparkFlex
            ? new SparkFlex(secondaryFullConfig.canId, secondaryFullConfig.motorType)
            : new SparkMax(secondaryFullConfig.canId, secondaryFullConfig.motorType);

    // Get the encoders from the motors and store them
    primaryEncoder = primaryMotor.getEncoder();
    secondaryEncoder = secondaryMotor.getEncoder();

    // Assign the configurations to variables
    primaryConfig = primaryFullConfig.config;
    secondaryConfig = secondaryFullConfig.config;

    // Configure the motors
    SparkUtil.tryUntilOk(
        5,
        () ->
            primaryMotor.configure(
                primaryConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    SparkUtil.tryUntilOk(
        5,
        () ->
            secondaryMotor.configure(
                secondaryConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  /**
   * Updates a {@link RollerIOInputs} instance with the latest updates from this {@link RollerIO}.
   *
   * @param inputs A {@link RollerIOInputs} instance to update.
   */
  @Override
  public void updateInputs(RollerIOInputs inputs) {
    // Update the motor data inputs with the debouncers passed in to check connection status
    inputs.primaryMotor.updateFromSpark(primaryMotor, primaryEncoder);
    inputs.secondaryMotor.updateFromSpark(secondaryMotor, secondaryEncoder);
  }

  /**
   * Run the motors together at a specific voltage.
   *
   * @param voltage The voltage to run the rollers at.
   */
  @Override
  public void runVolts(double voltage) {
    primaryMotor.setVoltage(voltage);
    secondaryMotor.setVoltage(voltage);
  }

  /**
   * Run the motors separately at different voltages. This should only be used if the motors are not
   * physically coupled by any means.
   *
   * @param primaryVoltage The voltage to run the primary motor at.
   * @param secondaryVoltage The voltage to run the secondary motor at.
   */
  @Override
  public void runVolts(double primaryVoltage, double secondaryVoltage) {
    primaryMotor.setVoltage(primaryVoltage);
    secondaryMotor.setVoltage(secondaryVoltage);
  }

  /** Stops both of the motors (sets the output to zero). */
  @Override
  public void stop() {
    primaryMotor.set(0);
    secondaryMotor.set(0);
  }

  /**
   * Enables or disables brake mode on both of the roller motors.
   *
   * @param enabled Whether to enable brake mode.
   */
  @Override
  public void setBrakeMode(boolean enabled) {
    SparkUtil.tryUntilOk(
        5,
        () ->
            primaryMotor.configure(
                primaryConfig.idleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters));
    SparkUtil.tryUntilOk(
        5,
        () ->
            secondaryMotor.configure(
                secondaryConfig.idleMode(enabled ? IdleMode.kBrake : IdleMode.kCoast),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters));
  }
}
