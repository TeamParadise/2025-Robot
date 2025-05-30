// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1165.robot.subsystems.flywheels.io;

import com.revrobotics.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.team1165.robot.subsystems.flywheels.constants.FlywheelConstants;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;

public class FlywheelsIOSparkMax implements FlywheelsIO {
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;
  private final DigitalInput beamBreak;

  public FlywheelsIOSparkMax() {
    // Init Hardware
    leftMotor = new SparkMax(FlywheelConstants.leftID, MotorType.kBrushless);
    rightMotor = new SparkMax(FlywheelConstants.rightID, MotorType.kBrushless);
    beamBreak = new DigitalInput(1);

    // Configure motors
    leftMotor.configure(
        FlywheelConstants.motorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    rightMotor.configure(
        FlywheelConstants.motorConfig.inverted(true),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();
  }

  @Override
  public void updateInputs(FlywheelsIOInputs inputs) {
    inputs.leftPositionRads = Units.rotationsToRadians(leftEncoder.getPosition());
    inputs.leftVelocityRpm = leftEncoder.getVelocity();
    inputs.leftAppliedVolts = leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();
    inputs.leftSupplyCurrentAmps = leftMotor.getOutputCurrent();
    inputs.leftTempCelsius = leftMotor.getMotorTemperature();

    inputs.rightPositionRads = Units.rotationsToRadians(rightEncoder.getPosition());
    inputs.rightVelocityRpm = rightEncoder.getVelocity();
    inputs.rightAppliedVolts = rightMotor.getAppliedOutput() * rightMotor.getBusVoltage();
    inputs.rightSupplyCurrentAmps = rightMotor.getOutputCurrent();
    inputs.rightTempCelsius = rightMotor.getMotorTemperature();

    inputs.beamBreakStatus = beamBreak.get();
  }

  @Override
  public void runVolts(double leftVolts, double rightVolts) {
    leftMotor.setVoltage(leftVolts);
    rightMotor.setVoltage(rightVolts);
  }

  @Override
  public void runPercent(double leftPercent, double rightPercent) {
    leftMotor.set(leftPercent);
    rightMotor.set(rightPercent);
  }

  @Override
  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }
}
