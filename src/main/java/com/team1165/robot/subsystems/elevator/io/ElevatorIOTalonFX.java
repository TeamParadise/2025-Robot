/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.elevator.io;

import static com.team1165.robot.subsystems.elevator.constants.ElevatorConstants.gearRatio;
import static com.team1165.robot.subsystems.elevator.constants.ElevatorConstants.sprocketRadiusInches;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.team1165.robot.subsystems.elevator.constants.ElevatorConstants;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorIOTalonFX implements ElevatorIO {
  private final StatusSignal<Voltage> leftAppliedVolts;
  private final StatusSignal<Current> leftSupplyCurrent;
  private final StatusSignal<Temperature> leftTempCelsius;
  private final StatusSignal<Angle> leftPosition;
  private final StatusSignal<AngularVelocity> leftVelocity;

  private final StatusSignal<Voltage> rightAppliedVolts;
  private final StatusSignal<Current> rightSupplyCurrent;
  private final StatusSignal<Temperature> rightTempCelsius;
  private final StatusSignal<Angle> rightPosition;
  private final StatusSignal<AngularVelocity> rightVelocity;

  private final TalonFX leftTalon;
  private final TalonFX rightTalon;

  // Basic open loop
  private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0.0);
  private final TorqueCurrentFOC torqueControl = new TorqueCurrentFOC(0).withUpdateFreqHz(0.0);

  // Basic position, likely will do Motion Magic later?
  private final PositionVoltage positionControl = new PositionVoltage(0).withUpdateFreqHz(0.0);

  // Motion Magic control
  private final MotionMagicVoltage motionMagicControl =
      new MotionMagicVoltage(0).withUpdateFreqHz(0.0);

  public ElevatorIOTalonFX() {
    var ELEVATOR_CONFIG = new TalonFXConfiguration();

    ELEVATOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    ELEVATOR_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    ELEVATOR_CONFIG.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    ELEVATOR_CONFIG.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    // Elevator motors will provide feedback in INCHES the carriage has moved
    ELEVATOR_CONFIG.Feedback.SensorToMechanismRatio =
        (2 * Math.PI * (2 * sprocketRadiusInches))
            / gearRatio; // TODO: Check and make sure this math is right!!!!

    ELEVATOR_CONFIG.CurrentLimits.SupplyCurrentLimit = 60;
    ELEVATOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;

    leftTalon = new TalonFX(ElevatorConstants.LeftMotorConstants.canID, ElevatorConstants.canBus);
    rightTalon = new TalonFX(ElevatorConstants.RightMotorConstants.canID, ElevatorConstants.canBus);
    rightTalon.setControl(new Follower(leftTalon.getDeviceID(), true));

    // Apply configs
    rightTalon.getConfigurator().apply(ELEVATOR_CONFIG, 1);
    leftTalon.getConfigurator().apply(ELEVATOR_CONFIG, 1);
    leftTalon
        .getConfigurator()
        .apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    leftTalon.getConfigurator().apply(ElevatorConstants.gains);
    leftTalon
        .getConfigurator()
        .apply(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(15)
                .withMotionMagicAcceleration(30));

    leftAppliedVolts = leftTalon.getMotorVoltage();
    leftSupplyCurrent = leftTalon.getSupplyCurrent();
    leftTempCelsius = leftTalon.getDeviceTemp();
    leftPosition = leftTalon.getPosition();
    leftVelocity = leftTalon.getVelocity();

    rightAppliedVolts = rightTalon.getMotorVoltage();
    rightSupplyCurrent = rightTalon.getSupplyCurrent();
    rightTempCelsius = rightTalon.getDeviceTemp();
    rightPosition = rightTalon.getPosition();
    rightVelocity = rightTalon.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        leftPosition,
        leftVelocity,
        leftAppliedVolts,
        leftSupplyCurrent,
        leftTempCelsius,
        rightPosition,
        rightVelocity,
        rightAppliedVolts,
        rightSupplyCurrent,
        rightTempCelsius);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.leftMotorConnected =
        BaseStatusSignal.refreshAll(
                leftPosition, leftVelocity, leftAppliedVolts, leftSupplyCurrent, leftTempCelsius)
            .isOK();
    inputs.rightMotorConnected =
        BaseStatusSignal.refreshAll(
                rightPosition,
                rightVelocity,
                rightAppliedVolts,
                rightSupplyCurrent,
                rightTempCelsius)
            .isOK();

    inputs.leftPositionInches = leftPosition.getValueAsDouble();
    inputs.leftVelocityRpm = leftVelocity.getValueAsDouble() * 60.0;
    inputs.leftAppliedVolts = leftAppliedVolts.getValueAsDouble();
    inputs.leftSupplyCurrentAmps = leftSupplyCurrent.getValueAsDouble();
    inputs.leftTempCelsius = leftTempCelsius.getValueAsDouble();

    inputs.rightPositionInches = rightPosition.getValueAsDouble();
    inputs.rightVelocityRpm = rightVelocity.getValueAsDouble() * 60.0;
    inputs.rightAppliedVolts = rightAppliedVolts.getValueAsDouble();
    inputs.rightSupplyCurrentAmps = rightSupplyCurrent.getValueAsDouble();
    inputs.rightTempCelsius = rightTempCelsius.getValueAsDouble();
  }

  @Override
  public void stop() {
    leftTalon.stopMotor();
  }

  @Override
  public void runCurrent(double current) {
    leftTalon.setControl(torqueControl.withOutput(current));
  }

  @Override
  public void runVolts(double volts) {
    leftTalon.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void runPosition(double positionInches) {
    // Set to just basic position control right now, likely will switch to motion magic
    leftTalon.setControl(motionMagicControl.withPosition(positionInches));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    leftTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    rightTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setPID(Slot0Configs gains) {
    leftTalon.getConfigurator().apply(gains);
    rightTalon.getConfigurator().apply(gains);
  }
}
