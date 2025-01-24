package com.team1165.robot.subsystems; /// *

// * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
// *
// * Use of this source code is governed by the MIT License, which can be found in the LICENSE file
// at
// * the root directory of this project.
// */
//
// package com.team1165.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;

public class ElevatorKraken implements ElevatorIO {

  private StatusSignal<Angle> leftP;
  private StatusSignal<AngularVelocity> leftV;
  private StatusSignal<Voltage> leftAppliedVolts;
  private StatusSignal<Current> leftSupplyCurrent;
  private StatusSignal<Temperature> leftTempCelsius;
  private StatusSignal<Angle> rightP;
  private StatusSignal<AngularVelocity> rightV;
  private StatusSignal<Voltage> rightAppliedVolts;
  private StatusSignal<Current> rightSupplyCurrent;
  private StatusSignal<Temperature> rightTempCelsius;

  public TalonFX leftTalon;
  public TalonFX rightTalon;

  Distance lastDesiredPosition;

  public StatusSignal<Distance> leftPosition;
  public StatusSignal<Double> leftVelocity;

  public StatusSignal<Distance> rightPosition;
  public StatusSignal<Double> rightVelocity;

  public VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0.0);
  private final VelocityVoltage velocityControl = new VelocityVoltage(0).withUpdateFreqHz(0.0);

  public ElevatorKraken() {
    if (RobotBase.isReal()) {
      leftTalon = new TalonFX(8, "rio");
      rightTalon = new TalonFX(9, "rio");

      lastDesiredPosition = Units.Inches.of(0);

      // Apply configs
      rightTalon.getConfigurator().apply(ElevatorConstants.ELEVATOR_CONFIG, 1);
      leftTalon.getConfigurator().apply(ElevatorConstants.ELEVATOR_CONFIG, 1);

      leftP = leftTalon.getPosition();
      leftV = leftTalon.getVelocity();
      leftAppliedVolts = leftTalon.getMotorVoltage();
      leftSupplyCurrent = leftTalon.getSupplyCurrent();
      leftTempCelsius = leftTalon.getDeviceTemp();

      rightP = rightTalon.getPosition();
      rightV = rightTalon.getVelocity();
      rightAppliedVolts = rightTalon.getMotorVoltage();
      rightSupplyCurrent = rightTalon.getSupplyCurrent();
      rightTempCelsius = rightTalon.getDeviceTemp();

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
  }

  public Distance getElevatorPosition() {
    return Units.Inches.of(rightTalon.getPosition().getValueAsDouble());
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

    inputs.currentLeftPosition = Units.Inches.of(leftTalon.getPosition().getValueAsDouble());
    inputs.leftVelocityRpm =
        leftVelocity.getValueAsDouble() * 60.0; // probs redundant considering whats above

    inputs.currentRightPosition = Units.Inches.of(rightTalon.getPosition().getValueAsDouble());
    inputs.rightVelocityRpm = rightVelocity.getValueAsDouble() * 60.0;

    inputs.leftVelocityRpm = leftVelocity.getValueAsDouble() * 60.0;
    inputs.leftAppliedVolts = leftAppliedVolts.getValueAsDouble();
    inputs.leftSupplyCurrentAmps = leftSupplyCurrent.getValueAsDouble();
    inputs.leftTempCelsius = leftTempCelsius.getValueAsDouble();

    inputs.rightVelocityRpm = rightVelocity.getValueAsDouble() * 60.0;
    inputs.rightAppliedVolts = rightAppliedVolts.getValueAsDouble();
    inputs.rightSupplyCurrentAmps = rightSupplyCurrent.getValueAsDouble();
    inputs.rightTempCelsius = rightTempCelsius.getValueAsDouble();
  }

  @Override
  public void resetSensorPosition(Distance setpoint) {
    rightTalon.setPosition(setpoint.in(Inches));
    leftTalon.setPosition(setpoint.in(Inches));
  }

  @Override
  public void setPosition(Distance height) {
    rightTalon.setControl(new PositionVoltage(height.in(Units.Inches)));
    leftTalon.setControl(new Follower(rightTalon.getDeviceID(), true));
    lastDesiredPosition = height;
  }

  @Override
  public void stop() {
    leftTalon.setControl(new NeutralOut());
    rightTalon.setControl(new NeutralOut());
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    rightTalon.getConfigurator().apply(ElevatorConstants.ELEVATOR_CONFIG);
    leftTalon.getConfigurator().apply(ElevatorConstants.ELEVATOR_CONFIG);
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
  public void runVolts(double leftVolts, double rightVolts) {
    leftTalon.setControl(voltageControl.withOutput(leftVolts));
    rightTalon.setControl(voltageControl.withOutput(rightVolts));
  }
}
