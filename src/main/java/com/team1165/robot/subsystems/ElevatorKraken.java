/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class ElevatorKraken implements ElevatorIO {
  public TalonFX leftTalon;
  public TalonFX rightTalon;

  public StatusSignal<Double> leftPosition;
  public StatusSignal<Double> leftVelocity;
  public StatusSignal<Double> leftAppliedVolts;
  public StatusSignal<Double> leftSupplyCurrent;
  public StatusSignal<Double> leftTorqueCurrent;

  public StatusSignal<Double> rightPosition;
  public StatusSignal<Double> rightVelocity;
  public StatusSignal<Double> rightAppliedVolts;
  public StatusSignal<Double> rightSupplyCurrent;
  public StatusSignal<Double> rightTorqueCurrent;

  public Slot0Configs controllerConfig = new Slot0Configs();
  public VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0.0);
  public VelocityVoltage velocityControl = new VelocityVoltage(0).withUpdateFreqHz(0.0);
  public NeutralOut neutralControl = new NeutralOut().withUpdateFreqHz(0.0);

  public static final ElevatorConfig ELEVATOR_CONFIG =
      new ElevatorConfig(4, 0, (1.0 / 2.0), 9000.0);

  public static final Gains gains =
      new Gains(1, 0.01, 0.01, 0.5, 0, 1.2); // don't really know what to do here

  public ElevatorKraken() {
    leftTalon = new TalonFX(ELEVATOR_CONFIG.leftID(), "rio");
    rightTalon = new TalonFX(ELEVATOR_CONFIG.rightID(), "rio");

    // General config
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 60.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Feedback.SensorToMechanismRatio = ELEVATOR_CONFIG.reduction();

    controllerConfig.kP = gains.kP();
    controllerConfig.kI = gains.kI();
    controllerConfig.kD = gains.kD();
    controllerConfig.kS = gains.kS();
    controllerConfig.kV = gains.kV();
    controllerConfig.kA = gains.kA();

    // Apply configs
    leftTalon.getConfigurator().apply(config, 1.0);
    rightTalon.getConfigurator().apply(config, 1.0);
    leftTalon.getConfigurator().apply(controllerConfig, 1.0);
    rightTalon.getConfigurator().apply(controllerConfig, 1.0);

    leftTalon.getConfigurator().apply(config, 1.0);
    rightTalon.getConfigurator().apply(config, 1.0);
    leftTalon.getConfigurator().apply(controllerConfig, 1.0);
    rightTalon.getConfigurator().apply(controllerConfig, 1.0);

    leftTalon.setInverted(true);
    rightTalon.setInverted(false);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.leftMotorConnected =
        BaseStatusSignal.refreshAll(
                leftPosition, leftVelocity, leftAppliedVolts, leftSupplyCurrent, leftTorqueCurrent)
            .isOK();
    inputs.rightMotorConnected =
        BaseStatusSignal.refreshAll(
                rightPosition,
                rightVelocity,
                rightAppliedVolts,
                rightSupplyCurrent,
                rightTorqueCurrent)
            .isOK();

    inputs.leftPositionRads = Units.rotationsToRadians(leftPosition.getValueAsDouble());
    inputs.leftVelocityRpm = leftVelocity.getValueAsDouble() * 60.0;
    inputs.leftAppliedVolts = leftAppliedVolts.getValueAsDouble();
    inputs.leftSupplyCurrentAmps = leftSupplyCurrent.getValueAsDouble();
    inputs.leftTorqueCurrentAmps = leftTorqueCurrent.getValueAsDouble();

    inputs.rightPositionRads = Units.rotationsToRadians(rightPosition.getValueAsDouble());
    inputs.rightVelocityRpm = rightVelocity.getValueAsDouble() * 60.0;
    inputs.rightAppliedVolts = rightAppliedVolts.getValueAsDouble();
    inputs.rightSupplyCurrentAmps = rightSupplyCurrent.getValueAsDouble();
    inputs.rightTorqueCurrentAmps = rightTorqueCurrent.getValueAsDouble();
  }

  @Override
  public void runVolts(double leftVolts, double rightVolts) {
    leftTalon.setControl(voltageControl.withOutput(leftVolts));
    rightTalon.setControl(voltageControl.withOutput(rightVolts));
  }

  @Override
  public void stop() {
    leftTalon.setControl(neutralControl);
    rightTalon.setControl(neutralControl);
  }

  @Override
  public void runVelocity(
      double leftRpm, double rightRpm, double leftFeedforward, double rightFeedforward) {
    leftTalon.setControl(
        velocityControl.withVelocity(leftRpm / 60.0).withFeedForward(leftFeedforward));
    rightTalon.setControl(
        velocityControl.withVelocity(rightRpm / 60.0).withFeedForward(rightFeedforward));
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    controllerConfig.kP = kP;
    controllerConfig.kI = kI;
    controllerConfig.kD = kD;
    leftTalon.getConfigurator().apply(controllerConfig);
    rightTalon.getConfigurator().apply(controllerConfig);
  }

  @Override
  public void runCharacterizationLeft(double input) {
    leftTalon.setControl(voltageControl.withOutput(input));
  }

  @Override
  public void runCharacterizationRight(double input) {
    rightTalon.setControl(voltageControl.withOutput(input));
  }

  public record ElevatorConfig(
      int leftID, int rightID, double reduction, double maxAcclerationRpmPerSec) {}

  public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {}
}
