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
import com.team1165.robot.util.logging.MotorData;
import com.team1165.robot.util.vendor.rev.SparkFullConfigs.SparkFullConfig;
import com.team1165.robot.util.vendor.rev.SparkModel;
import com.team1165.robot.util.vendor.rev.SparkUtil;
import edu.wpi.first.math.filter.Debouncer;

public class RollerIOSparkMax implements RollerIO {
  private final SparkBase primaryMotor;
  private final SparkBase secondaryMotor;
  private final RelativeEncoder primaryEncoder;
  private final RelativeEncoder secondaryEncoder;

  private final Debouncer primaryConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer secondaryConnectedDebouncer = new Debouncer(0.5);

  public RollerIOSparkMax(SparkFullConfig primaryFullConfig, SparkFullConfig secondaryFullConfig) {
    // Assign motor variables
    primaryMotor = primaryFullConfig.model == SparkModel.SparkFlex ? new SparkFlex(primaryFullConfig.canId, primaryFullConfig.motorType) : new SparkMax(primaryFullConfig.canId, primaryFullConfig.motorType);
    secondaryMotor = secondaryFullConfig.model == SparkModel.SparkFlex ? new SparkFlex(secondaryFullConfig.canId, secondaryFullConfig.motorType) : new SparkMax(secondaryFullConfig.canId, secondaryFullConfig.motorType);

    // Get the encoders from the motors and store them
    primaryEncoder = primaryMotor.getEncoder();
    secondaryEncoder = secondaryMotor.getEncoder();

    // Configure the motors
    SparkUtil.tryUntilOk(primaryMotor, 5, spark -> spark.configure(primaryFullConfig.config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    SparkUtil.tryUntilOk(secondaryMotor, 5, spark -> spark.configure(secondaryFullConfig.config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  /**
   * Updates a {@link RollerIOInputs} instance with the latest updates from this {@link RollerIO}.
   *
   * @param inputs A {@link RollerIOInputs} instance to update.
   */
  @Override
  public void updateInputs(RollerIOInputs inputs) {
    // Update the motor data inputs with the debouncers passed in to check connection status
    inputs.primaryMotor = MotorData.getFromSpark(inputs.primaryMotor, primaryMotor, primaryEncoder, fault -> primaryConnectedDebouncer.calculate(!fault));
    inputs.secondaryMotor = MotorData.getFromSpark(inputs.secondaryMotor, secondaryMotor, secondaryEncoder, fault -> secondaryConnectedDebouncer.calculate(!fault));
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
   * Run the motors separately at different voltages. This should only be used if the motors are not physically coupled by any means.
   *
   * @param primaryVoltage The voltage to run the primary motor at.
   * @param secondaryVoltage The voltage to run the secondary motor at.
   */
  @Override
  public void runVolts(double primaryVoltage, double secondaryVoltage) {
    primaryMotor.setVoltage(primaryVoltage);
    secondaryMotor.setVoltage(secondaryVoltage);
  }

  /**
   * Stops both of the motors (sets the output to zero).
   */
  @Override
  public void stop() {
    primaryMotor.stopMotor();
    secondaryMotor.stopMotor();
  }
}
