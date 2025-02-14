/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.Pivot;

import static com.team1165.robot.subsystems.flywheels.FlywheelConstants.gains;

import com.team1165.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class PivotSIMIO implements PivotIO{
  DCMotor Gearbox = DCMotor.getNEO(1); // One Falcon 500 motor
  double gearing = 1.0; // Example gear ratio
  double momentOfInertia = 0.01; // Example inertia (kg⋅m²)

  // Create the plant model for the flywheel
  LinearSystem<N2, N1, N2> DCMotorSystem =
      LinearSystemId.createDCMotorSystem(Gearbox, momentOfInertia, gearing);
  private final DCMotorSim leftSim = new DCMotorSim(DCMotorSystem, Gearbox);

  private final PIDController Controller =
      new PIDController(gains.kP(), gains.kI(), gains.kD());

  private double leftAppliedVolts = 0.0;
  private double rightAppliedVolts = 0.0;

  private Double leftSetpointRpm = null;
  private Double rightSetpointRpm = null;
  private double leftFeedforward = 0.0;
  private double rightFeedforward = 0.0;

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    leftSim.update(Constants.loopPeriodSecs);


    inputs.PositionRads +=
        Units.radiansToRotations(leftSim.getAngularVelocityRadPerSec() * Constants.loopPeriodSecs);
    inputs.VelocityRpm = leftSim.getAngularVelocityRPM();
    inputs.AppliedVolts = leftAppliedVolts;
    inputs.SupplyCurrentAmps = leftSim.getCurrentDrawAmps();

  }

  @Override
  public void runVolts(double angle) {
    leftSetpointRpm = null;
    rightSetpointRpm = null;
//    leftAppliedVolts = MathUtil.clamp(leftVolts, -12.0, 12.0);
//    rightAppliedVolts = MathUtil.clamp(rightVolts, -12.0, 12.0);
    leftSim.setInputVoltage(leftAppliedVolts);
  }

  @Override
  public void runVelocity(
      double leftRpm, double rightRpm, double leftFeedforward, double rightFeedforward) {
    leftSetpointRpm = leftRpm;
    rightSetpointRpm = rightRpm;
    this.leftFeedforward = leftFeedforward;
    this.rightFeedforward = rightFeedforward;
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    Controller.setPID(kP, kI, kD);
  }

  @Override
  public void stop() {
    runVolts( 0.0);
  }

  @Override
  public void runCharacterizationLeft(double input) {
    leftSetpointRpm = null;
    rightSetpointRpm = null;
    runVolts( 0.0);
  }

  @Override
  public void runCharacterizationRight(double input) {
    leftSetpointRpm = null;
    rightSetpointRpm = null;
    runVolts(0.0);
  }
}
