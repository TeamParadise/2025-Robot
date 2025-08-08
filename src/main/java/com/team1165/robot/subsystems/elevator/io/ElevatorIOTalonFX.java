/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.elevator.io;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team1165.robot.util.constants.CANFrequency;
import com.team1165.robot.util.logging.MotorData.TalonFXMotorData;
import com.team1165.robot.util.vendor.ctre.PhoenixDeviceConfigs.TalonFXConfig;
import com.team1165.robot.util.vendor.ctre.PhoenixUtil;

/**
 * A hardware interface/implementation layer for a basic elevator subsystem powered by two motors
 * with Talon FX motor controllers.
 */
public class ElevatorIOTalonFX implements ElevatorIO {
  // Motor (controllers)
  private final TalonFX primaryMotor;
  private final TalonFX secondaryMotor;

  // Motor data to log
  private final TalonFXMotorData primaryMotorData;
  private final TalonFXMotorData secondaryMotorData;

  // Basic open loop control
  private final VoltageOut voltageControl = new VoltageOut(0);
  private final TorqueCurrentFOC torqueControl = new TorqueCurrentFOC(0);

  // Motion Magic control
  // TODO: Try TorqueCurrentFOC or just FOC on the elevator
  private final MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0);

  public ElevatorIOTalonFX(TalonFXConfig primaryConfig, TalonFXConfig secondaryConfig) {
    // Assign motor variables
    primaryMotor = PhoenixUtil.createNewTalonFX(primaryConfig);
    secondaryMotor = PhoenixUtil.createNewTalonFX(secondaryConfig);

    // Set the secondary motor to follow and oppose the primary
    secondaryMotor.setControl(new Follower(primaryMotor.getDeviceID(), true));

    // Create motor data
    primaryMotorData = new TalonFXMotorData(primaryMotor, primaryConfig);
    secondaryMotorData = new TalonFXMotorData(secondaryMotor, secondaryConfig);

    // Set the values used in code to update faster
    PhoenixUtil.setFrequencyAndRegister(
        primaryConfig.canBus(),
        CANFrequency.FAST,
        primaryMotor.getPosition(),
        primaryMotor.getTorqueCurrent());
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // Update the motor data
    primaryMotorData.update();
    secondaryMotorData.update();

    // Put the motor data values in inputs
    inputs.primaryMotor = primaryMotorData;
    inputs.secondaryMotor = secondaryMotorData;
  }

  @Override
  public void runCurrent(double amps) {
    primaryMotor.setControl(torqueControl.withOutput(amps));
  }

  @Override
  public void runPosition(double position) {
    primaryMotor.setControl(motionMagicControl.withPosition(position));
  }

  @Override
  public void runVolts(double volts) {
    primaryMotor.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void stop() {
    primaryMotor.stopMotor();
    secondaryMotor.stopMotor();
  }

  @Override
  public void setPID(Slot0Configs gains) {
    primaryMotor.getConfigurator().apply(gains);
    secondaryMotor.getConfigurator().apply(gains);
  }

  @Override
  public void setMotionProfiling(MotionMagicConfigs config) {
    primaryMotor.getConfigurator().apply(config);
    secondaryMotor.getConfigurator().apply(config);
  }

  @Override
  public void setPosition(double position) {
    primaryMotor.setPosition(position);
    secondaryMotor.setPosition(position);
  }
}
