/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.elevator.io;

import com.team1165.robot.subsystems.elevator.constants.ElevatorConstants;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

@Logged
public class ElevatorIOSim implements ElevatorIO {
  private final DCMotor gearbox = DCMotor.getKrakenX60(2);
  private final ElevatorSim elevatorSim =
      new ElevatorSim(
          gearbox,
          ElevatorConstants.gearRatio,
          2.0,
          Units.inchesToMeters(ElevatorConstants.sprocketRadiusInches),
          Units.inchesToMeters(ElevatorConstants.minHeightInches),
          Units.inchesToMeters(ElevatorConstants.maxHeightInches),
          false,
          0.0);

  private final PIDController leftController = new PIDController((12.0 / 483.0) * 3, 0.0, 0.0);
  private final PIDController rightController = new PIDController((12.0 / 483.0) * 3, 0.0, 0.0);

  private double leftAppliedVolts = 0.0;
  private double rightAppliedVolts = 0.0;
  private boolean closedLoop = false;

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
    inputs.leftSupplyCurrentAmps = elevatorSim.

    inputs.currentRightPosition = Units.Inches.of(meterstoInches(rightSim.getPositionMeters()));

    inputs.rightAppliedVolts = rightAppliedVolts;
    inputs.rightSupplyCurrentAmps = rightSim.getCurrentDrawAmps();
  }

  @Override
  public void stop() {
    runVolts(0.0);
  }

  @Override
  public void runCurrent(double current) {

  }

  @Override
  public void runVolts(double volts) {
    elevatorSim.setInputVoltage(volts);
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

  @Override
  public void setPID(double kP, double kI, double kD) {
    leftController.setPID(kP, kI, kD);
    rightController.setPID(kP, kI, kD);
  }

  public void setLastDesiredPosition(double lastDesiredPosition) {
    this.lastDesiredPosition = lastDesiredPosition;
  }

  @Override
  public double getLastDesiredPosition() {
    return meterstoInches(leftSim.getPositionMeters());
  }
}
