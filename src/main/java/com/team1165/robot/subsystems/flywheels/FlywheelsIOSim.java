// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1165.robot.subsystems.flywheels;

import static com.team1165.robot.subsystems.flywheels.FlywheelConstants.*;

import com.team1165.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelsIOSim implements FlywheelsIO {
  DCMotor Gearbox = DCMotor.getNEO(1); // One Falcon 500 motor
  double gearing = 50.0; // Example gear ratio
  double momentOfInertia = 0.01; // Example inertia (kg⋅m²)

  // Create the plant model for the flywheel
  LinearSystem<N1, N1, N1> FlywheelPlant =
      LinearSystemId.createFlywheelSystem(Gearbox, momentOfInertia, gearing);
  private final FlywheelSim leftSim = new FlywheelSim(FlywheelPlant, Gearbox);
  private final FlywheelSim rightSim = new FlywheelSim(FlywheelPlant, Gearbox);

  private final PIDController leftController =
      new PIDController(gains.kP(), gains.kI(), gains.kD());
  private final PIDController rightController =
      new PIDController(gains.kP(), gains.kI(), gains.kD());

  private double leftAppliedVolts = 0.0;
  private double rightAppliedVolts = 0.0;

  private Double leftSetpointRpm = null;
  private Double rightSetpointRpm = null;
  private double leftFeedforward = 0.0;
  private double rightFeedforward = 0.0;

  @Override
  public void updateInputs(FlywheelsIOInputs inputs) {
    leftSim.update(Constants.loopPeriodSecs);
    rightSim.update(Constants.loopPeriodSecs);
    // control to setpoint
    //    if (leftSetpointRpm != null && rightSetpointRpm != null) {
    //      runVolts(
    //          leftController.calculate(leftSim.getAngularVelocityRPM(), leftSetpointRpm)
    //              + leftFeedforward,
    //          rightController.calculate(rightSim.getAngularVelocityRPM(), rightSetpointRpm)
    //              + rightFeedforward);
    //    }

    inputs.leftPositionRads +=
        Units.radiansToRotations(leftSim.getAngularVelocityRadPerSec() * Constants.loopPeriodSecs);
    inputs.leftVelocityRpm = leftSim.getAngularVelocityRPM();
    inputs.leftAppliedVolts = leftAppliedVolts;
    inputs.leftSupplyCurrentAmps = leftSim.getCurrentDrawAmps();

    inputs.rightPositionRads +=
        Units.radiansToRotations(rightSim.getAngularVelocityRadPerSec() * Constants.loopPeriodSecs);
    inputs.rightVelocityRpm = rightSim.getAngularVelocityRPM();
    inputs.rightAppliedVolts = rightAppliedVolts;
    inputs.rightSupplyCurrentAmps = rightSim.getCurrentDrawAmps();
  }

  @Override
  public void runVolts(double leftRpm, double rightRpm) {
    leftSetpointRpm = null;
    rightSetpointRpm = null;

    double kS = 0.5; // Adjust based on testing
    double kV = 12.0 / 5676.0; // Approximate for a NEO motor

    double leftAppliedVolts = MathUtil.clamp(kS + kV * leftRpm, -12.0, 12.0);
    double rightAppliedVolts = MathUtil.clamp(kS + kV * rightRpm, -12.0, 12.0);

    System.out.println(leftAppliedVolts);
    if (leftRpm == 0) {
      leftSim.setInputVoltage(0);
      rightSim.setInputVoltage(0);
    } else {
      leftSim.setInputVoltage(leftAppliedVolts);
      rightSim.setInputVoltage(rightAppliedVolts);
    }
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
    leftController.setPID(kP, kI, kD);
    rightController.setPID(kP, kI, kD);
  }

  @Override
  public void stop() {
    runVolts(0.0, 0.0);
  }

  @Override
  public void runCharacterizationLeft(double input) {
    leftSetpointRpm = null;
    rightSetpointRpm = null;
    runVolts(input, 0.0);
  }

  @Override
  public void runCharacterizationRight(double input) {
    leftSetpointRpm = null;
    rightSetpointRpm = null;
    runVolts(0.0, input);
  }
}
