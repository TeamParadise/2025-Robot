/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.Pivot;

import static com.team1165.robot.subsystems.Pivot.PivotConstants.pivotConfig;
import static com.team1165.robot.subsystems.flywheels.FlywheelConstants.gains;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PivotSparkMax implements PivotIO {
  private final SparkMax Motor;

  private final RelativeEncoder Encoder;

  private static final double Gear_Ratio = 100.00; // 100 to 1 gear ratio

  private double lastTimestamp = Timer.getFPGATimestamp();

  // Controllers
  private final PIDController Controller = new PIDController(gains.kP(), gains.kI(), gains.kD());

  public PivotSparkMax() {
    // Init Hardware
    Motor = new SparkMax(pivotConfig.theID(), SparkFlex.MotorType.kBrushless);

    Encoder = Motor.getEncoder();

    // Config Hardware

    // Reset encoders
    Encoder.setPosition(0.0);

    setPID(gains.kP(), gains.kI(), gains.kD());
    setFF(gains.kS(), gains.kV(), gains.kA());
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.PositionRads = Units.rotationsToRadians(Encoder.getPosition());
    inputs.VelocityRpm = Encoder.getVelocity();
    inputs.AppliedVolts = Motor.getAppliedOutput() * Motor.getBusVoltage();
    inputs.SupplyCurrentAmps = Motor.getOutputCurrent();
    inputs.TempCelsius = Motor.getMotorTemperature();

    SmartDashboard.putNumber("Neo position", getPivotAngle());
  }

  @Override
  public void runVolts(double angle) {
    Motor.setVoltage(angle);
    // figure out

  }

  public double getPivotAngle() {
    double motorRotation = Encoder.getPosition();
    return motorRotation * (360 / Gear_Ratio);
  }

  @Override
  public void setPivotAngle(double targetAngle) {
    double targetRotations = targetAngle / (360.0 / Gear_Ratio); // Convert to rotations

    // Set the target position for the PID controller
    Controller.setSetpoint(targetRotations);

    // In  periodic method (or a separate control loop):
    double currentRotations = Encoder.getPosition();
    double voltage = Controller.calculate(currentRotations); // Use PID for position control
    Motor.setVoltage(voltage);
  }

  @Override
  public void runVelocity(
      double leftRpm, double rightRpm, double leftFeedforward, double rightFeedforward) {
    double currentTimestamp = Timer.getFPGATimestamp();
    double dt = currentTimestamp - lastTimestamp;

    lastTimestamp = currentTimestamp;

    double Velocity = Encoder.getVelocity();

    double PIDOutput = Controller.calculate(Velocity, leftRpm);

    double Voltage = PIDOutput + leftFeedforward;

    Motor.setVoltage(Voltage);
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    Controller.setP(kP);
    Controller.setI(kI);
    Controller.setD(kD);
  }

  @Override
  public void runCharacterizationLeft(double input) {
    Motor.setVoltage(input);
  }

  @Override
  public void runCharacterizationRight(double input) {
    Motor.setVoltage(input);
  }

  @Override
  public void stop() {
    Motor.stopMotor();
  }

  public void periodic() {
    // Telemetry for Neo 1
    SmartDashboard.putNumber("Neo 1 Speed", Motor.get()); // Current speed
    SmartDashboard.putNumber("Neo 1 Current", Motor.getOutputCurrent()); // Current draw
    SmartDashboard.putNumber("Neo 1 Temperature", Motor.getMotorTemperature()); // Temperature
    SmartDashboard.putNumber("Neo 1 Voltage", Motor.getBusVoltage()); // Input Voltage
  }
}
