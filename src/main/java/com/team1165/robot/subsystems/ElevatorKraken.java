package com.team1165.robot.subsystems; /// *

// * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
// *
// * Use of this source code is governed by the MIT License, which can be found in the LICENSE file
// at
// * the root directory of this project.
// */
//
// package com.team1165.robot.subsystems;

import static com.team1165.robot.subsystems.ConstantsElevator.ELEVATOR_CONFIG;
import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team1165.robot.subsystems.ConstantsElevator.Gains;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;

@Logged
public class ElevatorKraken implements ElevatorIO {

  public TalonFX leftTalon;
  public TalonFX rightTalon;

  Distance lastDesiredPosition;

  public StatusSignal<Distance> leftPosition;
  public StatusSignal<Double> leftVelocity;

  public StatusSignal<Distance> rightPosition;
  public StatusSignal<Double> rightVelocity;

  public VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0.0);

  public static final Gains gains =
      new Gains(
          ELEVATOR_CONFIG.Slot0.kP,
          ELEVATOR_CONFIG.Slot0.kI,
          ELEVATOR_CONFIG.Slot0.kD,
          ELEVATOR_CONFIG.Slot0.kS,
          ELEVATOR_CONFIG.Slot0.kV,
          1.2,
          ELEVATOR_CONFIG.Slot0.kG); // don't really know what to do here

  public ElevatorKraken() {
    if (RobotBase.isReal()) {
      leftTalon = new TalonFX(8, "rio");
      rightTalon = new TalonFX(9, "rio");

      lastDesiredPosition = Units.Inches.of(0);

      // Apply configs
      rightTalon.getConfigurator().apply(ConstantsElevator.ELEVATOR_CONFIG);
      leftTalon.getConfigurator().apply(ConstantsElevator.ELEVATOR_CONFIG);
    }
  }

  public Distance getElevatorPosition() {
    return Units.Inches.of(rightTalon.getPosition().getValueAsDouble());
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    //    inputs.leftMotorConnected = BaseStatusSignal.refreshAll(leftPosition,
    // leftVelocity).isOK();
    //    inputs.rightMotorConnected = BaseStatusSignal.refreshAll(rightPosition, - didnt build
    // rightVelocity).isOK();

    inputs.currentLeftPosition = Units.Inches.of(leftTalon.getPosition().getValueAsDouble());
    inputs.leftVelocityRpm = leftVelocity.getValueAsDouble() * 60.0;

    inputs.currentRightPosition = Units.Inches.of(rightTalon.getPosition().getValueAsDouble());
    inputs.rightVelocityRpm = rightVelocity.getValueAsDouble() * 60.0;
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
    rightTalon.getConfigurator().apply(ConstantsElevator.ELEVATOR_CONFIG);
    leftTalon.getConfigurator().apply(ConstantsElevator.ELEVATOR_CONFIG);
  }

  @Override
  public void runCharacterizationLeft(double input) {
    leftTalon.setControl(voltageControl.withOutput(input));
  }

  // these 2 Idek what I was on tbh
  @Override
  public void runCharacterizationRight(double input) {
    rightTalon.setControl(voltageControl.withOutput(input));
  }
}
