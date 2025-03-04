/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.elevator.io;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

@Logged
public class ElevatorIOSim implements ElevatorIO {
  private static final double kMetersPerInch = 0.0254;
  private static final double reduction = 32.0 / 10.0;
  private static final double maxLengthMeters = inchesToMeters(40.872);
  private static final double drumRadiusMeters = inchesToMeters(0.5);
  private double lastDesiredPosition = 0.0;
  Alert alert =
      new Alert(
          "Elevator",
          "ElevatorSimIO",
          AlertType.kError); // I saw u removed this, commented out logic until just in case,

  private final ElevatorSim elevatorSim =
      new ElevatorSim(
          DCMotor.getKrakenX60Foc(2),
          5.0,
          0.5,
          drumRadiusMeters,
          0.0,
          maxLengthMeters,
          false,
          0.0);

  private final PIDController leftController = new PIDController((12.0 / 483.0) * 3, 0.0, 0.0);
  private final PIDController rightController = new PIDController((12.0 / 483.0) * 3, 0.0, 0.0);

  private double leftAppliedVolts = 0.0;
  private double rightAppliedVolts = 0.0;

  private Double leftSetpointRpm = null;
  private Double rightSetpointRpm = null;
  private double leftFeedforward = 0.0;
  private double rightFeedforward = 0.0;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      stop();
    }
    leftSim.update(0.02);
    rightSim.update(0.02);

    if (leftSetpointRpm != null && rightSetpointRpm != null) {
      runVolts(
          leftController.calculate(leftSim.getVelocityMetersPerSecond(), leftSetpointRpm)
              + leftFeedforward,
          rightController.calculate(rightSim.getVelocityMetersPerSecond(), rightSetpointRpm)
              + rightFeedforward); // probs wrong
    }
    inputs.currentLeftPosition = Units.Inches.of(meterstoInches(leftSim.getPositionMeters()));

    inputs.leftAppliedVolts = leftAppliedVolts;
    inputs.leftSupplyCurrentAmps = leftSim.getCurrentDrawAmps();

    inputs.currentRightPosition = Units.Inches.of(meterstoInches(rightSim.getPositionMeters()));

    inputs.rightAppliedVolts = rightAppliedVolts;
    inputs.rightSupplyCurrentAmps = rightSim.getCurrentDrawAmps();
  }

  @Override
  public void runVolts(double leftVolts, double rightVolts) {
    leftSetpointRpm = null;
    rightSetpointRpm = null;
    leftAppliedVolts = leftVolts;
    rightAppliedVolts = rightVolts;
    leftSim.setInputVoltage(leftAppliedVolts);
    rightSim.setInputVoltage(rightAppliedVolts);
  }

  @Override
  public void setPosition(double height, double velocity) {

    //    double error = height - rightSim.getPositionMeters();
    //    double kP = 10.0; // Tune this value
    //    double voltage = MathUtil.clamp(error * kP, -12.0, 12.0);
    //    runVolts(voltage, voltage);
    rightSim.setState(inchesToMeters(height), velocity);

    leftSim.setState(inchesToMeters(height), velocity);
  }

  public static double inchesToMeters(double inches) {
    return inches * kMetersPerInch;
  }

  public static double meterstoInches(double meters) {
    return meters / kMetersPerInch;
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    leftController.setPID(kP, kI, kD);
    rightController.setPID(kP, kI, kD);
  }

  @Override
  public void stop() {
    runVolts(0.0, 0.0);
  }

  public void setLastDesiredPosition(double lastDesiredPosition) {
    this.lastDesiredPosition = lastDesiredPosition;
  }

  @Override
  public double getLastDesiredPosition() {
    return meterstoInches(leftSim.getPositionMeters());
  }
}
