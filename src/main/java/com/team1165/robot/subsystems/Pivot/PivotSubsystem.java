/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.Pivot;


import static com.team1165.robot.subsystems.flywheels.FlywheelConstants.gains;

import com.team1165.robot.Constants;
import com.team1165.robot.subsystems.flywheels.Flywheels.Goal;
import com.team1165.robot.subsystems.flywheels.FlywheelsIO;
import com.team1165.robot.subsystems.flywheels.LinearProfile;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;

public class PivotSubsystem extends SubsystemBase {
  private final PivotIO io;

  private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(gains.kS(), gains.kV(),
      gains.kA());
  private boolean wasClosedLoop = false;
  private boolean closedLoop = false;

  public void setPrepareShootSupplier(BooleanSupplier prepareShootSupplier) {
    this.prepareShootSupplier = prepareShootSupplier;
  }

  private BooleanSupplier prepareShootSupplier = () -> false;

  private final Alert PIVOTDisconnected =
      new Alert("Pivot disconnected!", AlertType.kWarning);



  public PivotSubsystem(PivotIO io) {
    this.io = io;
  }
  public void setPivotAngle(double angle) {
    io.setPivotAngle(angle);
  }

  /** Stop both Elevator Motors */
  public void stop() {
    io.stop();
  }

  public void setPID(double kp, double ki, double kd) {
    io.setPID(kp, ki, kd);
  }



  public void runVelocity(
      double leftRpm, double rightRpm, double leftFeedforward, double rightFeedforward) {
    io.runVelocity(leftRpm, rightRpm, leftFeedforward, rightFeedforward);
  }

}

