/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems;

import com.team1165.robot.Constants;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

@Logged
public class ElevatorSimIO implements ElevatorIO {
  private static final double kMetersPerInch = 0.0254;
  private static final double reduction = 32.0 / 10.0;
  private static final double maxLengthMeters = inchesToMeters(11.872);
  private static final double drumRadiusMeters = inchesToMeters(0.5);
  private double lastDesiredPosition = 0.0;
  Alert alert =
      new Alert(
          "Elevator",
          "ElevatorSimIO",
          AlertType.kError); // I saw u removed this, commented out logic until just in case,

  private final ElevatorSim leftSim =
      new ElevatorSim(
          DCMotor.getKrakenX60Foc(1),
          reduction,
          0.5,
          drumRadiusMeters,
          0.0,
          maxLengthMeters,
          false,
          0.0); // took away ElevatorConfig.reduction()
  private final ElevatorSim rightSim =
      new ElevatorSim(
          DCMotor.getKrakenX60Foc(1),
          reduction,
          0.5,
          drumRadiusMeters,
          0.0,
          maxLengthMeters,
          false,
          0.0); // took away ElevatorConfig.reduction()

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
    leftSim.update(Constants.loopPeriodSecs);
    rightSim.update(Constants.loopPeriodSecs);

    if (leftSetpointRpm != null && rightSetpointRpm != null) {
      runVolts(
          leftController.calculate(leftSim.getVelocityMetersPerSecond(), leftSetpointRpm)
              + leftFeedforward,
          rightController.calculate(rightSim.getVelocityMetersPerSecond(), rightSetpointRpm)
              + rightFeedforward); // probs wrong
    }
    inputs.currentLeftPosition = Units.Inches.of(leftSim.getPositionMeters() / drumRadiusMeters);

    inputs.leftAppliedVolts = leftAppliedVolts;
    inputs.leftSupplyCurrentAmps = leftSim.getCurrentDrawAmps();

    inputs.currentRightPosition = Units.Inches.of(rightSim.getPositionMeters() / drumRadiusMeters);

    inputs.rightAppliedVolts = rightAppliedVolts;
    inputs.rightSupplyCurrentAmps = rightSim.getCurrentDrawAmps();
  }

  @Override
  public void runVolts(double leftVolts, double rightVolts) {
    leftSetpointRpm = null;
    rightSetpointRpm = null;
    leftAppliedVolts = MathUtil.clamp(leftVolts, -12.0, 12.0);
    rightAppliedVolts = MathUtil.clamp(rightVolts, -12.0, 12.0);
    leftSim.setInputVoltage(leftAppliedVolts);
    rightSim.setInputVoltage(rightAppliedVolts);
  }

  @Override
  public void setPosition(Double height, double velocity) {
    //    if (height + lastDesiredPosition < maxLengthMeters) {
    rightSim.setState(height, velocity);
    leftSim.setState(height, velocity);
    lastDesiredPosition = height;
    //    } else {
    //      alert.setText("Elevator not at limit");
    //    }
  }

  public static double inchesToMeters(double inches) {
    return inches * kMetersPerInch;
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
}
