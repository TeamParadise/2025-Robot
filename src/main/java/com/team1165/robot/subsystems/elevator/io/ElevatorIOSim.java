/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.elevator.io;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.team1165.robot.subsystems.elevator.constants.ElevatorConstants;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

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

  private final PIDController pidController = new PIDController((12.0 / 483.0) * 3, 0.0, 0.0);
  private final ElevatorFeedforward feedforwardController =
      new ElevatorFeedforward(0.0, 0.0, 0.0, 0.0);

  private Double positionSetpoint = null;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      stop();
    }

    elevatorSim.update(0.02);

    double appliedVolts = 0;
    if (positionSetpoint != null) {
      appliedVolts = pidController.calculate(
                  Units.metersToInches(elevatorSim.getPositionMeters()), positionSetpoint)
              + feedforwardController.calculate(
                  Units.metersToInches(elevatorSim.getVelocityMetersPerSecond()));
      elevatorSim.setInputVoltage(appliedVolts);
    }

    inputs.leftPositionInches = Units.metersToInches(elevatorSim.getPositionMeters());
    inputs.leftVelocityRpm = Units.metersToInches(elevatorSim.getVelocityMetersPerSecond());
    inputs.leftAppliedVolts = appliedVolts;

    inputs.rightPositionInches = Units.metersToInches(elevatorSim.getPositionMeters());
    inputs.rightVelocityRpm = Units.metersToInches(elevatorSim.getVelocityMetersPerSecond());
    inputs.rightAppliedVolts = appliedVolts;
  }

  @Override
  public void stop() {
    positionSetpoint = null;
    runVolts(0.0);
  }

  @Override
  public void runVolts(double volts) {
    positionSetpoint = null;
    elevatorSim.setInputVoltage(volts);
  }

  @Override
  public void runPosition(double positionInches) {
    positionSetpoint = positionInches;
  }

  @Override
  public void setPID(Slot0Configs gains) {
    pidController.setPID(gains.kP, gains.kI, gains.kD);
    feedforwardController.setKs(gains.kS);
    feedforwardController.setKg(gains.kG);
    feedforwardController.setKv(gains.kV);
    feedforwardController.setKa(gains.kA);
  }
}
