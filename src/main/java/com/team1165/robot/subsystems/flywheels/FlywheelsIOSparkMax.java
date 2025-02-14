// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1165.robot.subsystems.flywheels;

import static com.team1165.robot.subsystems.flywheels.FlywheelConstants.*;

import com.revrobotics.*;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

public class FlywheelsIOSparkMax implements FlywheelsIO {
  // Hardware
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  private double lastTimestamp = Timer.getFPGATimestamp();

  // Controllers
  private final PIDController leftController =
      new PIDController(gains.kP(), gains.kI(), gains.kD());
  private final PIDController rightController =
      new PIDController(gains.kP(), gains.kI(), gains.kD());

  public FlywheelsIOSparkMax() {
    // Init Hardware
    leftMotor = new SparkMax(flywheelConfig.leftID(), SparkFlex.MotorType.kBrushless);
    rightMotor = new SparkMax(flywheelConfig.rightID(), SparkFlex.MotorType.kBrushless);
    leftEncoder = leftMotor.getEncoder();
    rightEncoder = rightMotor.getEncoder();

    // Config Hardware

    // Reset encoders
    leftEncoder.setPosition(0.0);
    rightEncoder.setPosition(0.0);

    setPID(gains.kP(), gains.kI(), gains.kD());
    setFF(gains.kS(), gains.kV(), gains.kA());
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
  }

  @Override
  public void runVolts(double leftRpm, double rightRpm) {
    double kS = 0.5; // Adjust based on testing
    double kV = 12.0 / 5676.0; // Approximate for a NEO motor

    double leftVolts = MathUtil.clamp(kS + kV * leftRpm, -12.0, 12.0);
    double rightVolts = MathUtil.clamp(kS + kV * rightRpm, -12.0, 12.0);

    leftMotor.setVoltage(leftVolts);
    rightMotor.setVoltage(rightVolts);
  }

  @Override
  public void runVelocity(
      double leftRpm, double rightRpm, double leftFeedforward, double rightFeedforward) {
    double currentTimestamp = Timer.getFPGATimestamp();
    double dt = currentTimestamp - lastTimestamp;
    lastTimestamp = currentTimestamp;

    double leftVelocity = leftEncoder.getVelocity();
    double rightVelocity = rightEncoder.getVelocity();

    double leftPIDOutput = leftController.calculate(leftVelocity, leftRpm);
    double rightPIDOutput = rightController.calculate(rightVelocity, rightRpm);

    double leftVoltage = leftPIDOutput + leftFeedforward;
    double rightVoltage = rightPIDOutput + rightFeedforward;

    leftMotor.setVoltage(leftVoltage);
    rightMotor.setVoltage(rightVoltage);
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    leftController.setP(kP);
    leftController.setI(kI);
    leftController.setD(kD);
    rightController.setP(kP);
    rightController.setI(kI);
    rightController.setD(kD);
  }

  @Override
  public void runCharacterizationLeft(double input) {
    leftMotor.setVoltage(input);
  }

  @Override
  public void runCharacterizationRight(double input) {
    rightMotor.setVoltage(input);
  }

  @Override
  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }
}
