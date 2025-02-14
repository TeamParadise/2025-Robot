// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1165.robot.subsystems.flywheels;

import static com.team1165.robot.subsystems.flywheels.FlywheelConstants.*;

import com.team1165.robot.Constants;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheels extends SubsystemBase {
  //  private static final double kP = new double("Flywheels/kP",
  // gains.kP());
  ////  private static final double kI = new double("Flywheels/kI",
  // gains.kI());
  ////  private static final double kD = new double("Flywheels/kD",
  // gains.kD());
  ////  private static final double kS = new double("Flywheels/kS",
  // gains.kS());
  ////  private static final double kV = new double("Flywheels/kV",
  // gains.kV());
  ////  private static final double kA = new double("Flywheels/kA",
  // gains.kA());

  //  private static final double shootingLeftRpm =
  //      new double("Flywheels/ShootingLeftRpm", 5066.0);
  //  private static final double shootingRightRpm =
  //      new double("Flywheels/ShootingRightRpm", 7733.0);
  //  private static final double prepareShootMultiplier =
  //      new double("Flywheels/PrepareShootMultiplier", 1.0);
  //  private static final double intakingRpm =
  //      new double("Flywheels/IntakingRpm", -3000.0);
  //  private static final double demoIntakingRpm =
  //      new double("Flywheels/DemoIntakingRpm", -250.0);
  //  private static final double ejectingRpm =
  //      new double("Flywheels/EjectingRpm", 1000.0);
  //  private static final double poopingRpm =
  //      new double("Flywheels/PoopingRpm", 3000.0);
  //  private static final double maxAcceleration =
  //      new double(
  //          "Flywheels/MaxAccelerationRpmPerSec", flywheelConfig.maxAcclerationRpmPerSec());

  private static final double shootingLeftRpm = 7733.0;
  private static final double shootingRightRpm = 7733.0;
  private static final double prepareShootMultiplier = 1.0;
  private static final double intakingRpm = -3000.0;
  private static final double demoIntakingRpm = -250.0;
  private static final double ejectingRpm = 1000.0;
  private static final double poopingRpm = 3000.0;
  private static final double maxAcceleration = flywheelConfig.maxAcclerationRpmPerSec();

  private final FlywheelsIO io;
  private final FlywheelsIOInputsAutoLogged inputs = new FlywheelsIOInputsAutoLogged();

  private final LinearProfile leftProfile;
  private final LinearProfile rightProfile;
  private SimpleMotorFeedforward ff =
      new SimpleMotorFeedforward(gains.kS(), gains.kV(), gains.kA());
  private boolean wasClosedLoop = false;
  private boolean closedLoop = false;

  public void setPrepareShootSupplier(BooleanSupplier prepareShootSupplier) {
    this.prepareShootSupplier = prepareShootSupplier;
  }

  private BooleanSupplier prepareShootSupplier = () -> false;

  private final Alert leftDisconnected =
      new Alert("Left flywheel disconnected!", AlertType.kWarning);
  private final Alert rightDisconnected =
      new Alert("Right flywheel disconnected!", AlertType.kWarning);

  public enum IdleMode {
    TELEOP,
    AUTO
  }

  public enum Goal {
    IDLE(0.0, 0.0),
    SHOOT(shootingLeftRpm, shootingRightRpm),
    INTAKE(intakingRpm, intakingRpm),
    DEMO_INTAKE(demoIntakingRpm, demoIntakingRpm),
    EJECT(ejectingRpm, ejectingRpm),
    POOP(poopingRpm, poopingRpm);

    private final double leftGoal;
    private final double rightGoal;

    // Constructor to set the left and right goal RPM values
    Goal(double leftGoal, double rightGoal) {
      this.leftGoal = leftGoal;
      this.rightGoal = rightGoal;
    }

    // Getter methods for left and right goal values
    public double getLeftGoal() {
      return -leftGoal;
    }

    public double getRightGoal() {
      return rightGoal;
    }
  }

  @AutoLogOutput(key = "Flywheels/Goal")
  private Goal goal = Goal.IDLE;

  private boolean isDrawingHighCurrent() {
    return Math.abs(inputs.leftSupplyCurrentAmps) > 50.0
        || Math.abs(inputs.rightSupplyCurrentAmps) > 50.0;
  }

  public Flywheels(FlywheelsIO io) {
    this.io = io;

    leftProfile = new LinearProfile(maxAcceleration, Constants.loopPeriodSecs);
    rightProfile = new LinearProfile(maxAcceleration, Constants.loopPeriodSecs);

    io.stop();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheels", inputs);

    // Set alerts
    leftDisconnected.set(!inputs.leftMotorConnected);
    rightDisconnected.set(!inputs.rightMotorConnected);

    // Stop when disabled
    if (DriverStation.isDisabled()) {
      setGoal(Goal.IDLE);
    }

    // Check if profile needs to be reset
    if (!closedLoop && wasClosedLoop) {
      leftProfile.reset();
      rightProfile.reset();
      wasClosedLoop = false;
    }

    // Get goal
    double leftGoal = goal.getLeftGoal();
    double rightGoal = goal.getRightGoal();
    boolean idlePrepareShoot = goal == Goal.IDLE && prepareShootSupplier.getAsBoolean();
    if (idlePrepareShoot) {
      leftGoal = Goal.SHOOT.getLeftGoal() * prepareShootMultiplier;
      rightGoal = Goal.SHOOT.getRightGoal() * prepareShootMultiplier;
    }

    // Run to setpoint
    if (closedLoop || idlePrepareShoot) {
      // Update goals
      leftProfile.setGoal(leftGoal);
      rightProfile.setGoal(rightGoal);
      double leftSetpoint = leftProfile.calculateSetpoint();
      double rightSetpoint = rightProfile.calculateSetpoint();
      io.runVelocity(
          leftSetpoint, rightSetpoint, ff.calculate(leftSetpoint), ff.calculate(rightSetpoint));
    }

    Logger.recordOutput("Flywheels/SetpointLeftRpm", leftProfile.getCurrentSetpoint());
    Logger.recordOutput("Flywheels/SetpointRightRpm", rightProfile.getCurrentSetpoint());
    Logger.recordOutput("Flywheels/GoalLeftRpm", leftGoal);
    Logger.recordOutput("Flywheels/GoalRightRpm", rightGoal);
  }

  /** Set the current goal of the flywheel */
  private void setGoal(Goal goal) {
    // If not already controlling to requested goal
    // set closed loop false
    closedLoop = this.goal == goal;
    // Enable close loop
    if (!closedLoop) {
      leftProfile.setGoal(goal.getLeftGoal(), inputs.leftVelocityRpm);
      rightProfile.setGoal(goal.getRightGoal(), inputs.rightVelocityRpm);
      closedLoop = true;
    }
    this.goal = goal;
  }

  /** Runs flywheels at the commanded voltage or amps. */
  public void runCharacterization(double input) {
    setGoal(Goal.IDLE);
    io.runCharacterizationLeft(input);
    io.runCharacterizationRight(input);
  }

  /** Get characterization velocity */
  public double getCharacterizationVelocity() {
    return (inputs.leftVelocityRpm + inputs.rightVelocityRpm) / 2.0;
  }

  /** Get if velocity profile has ended */
  @AutoLogOutput(key = "Flywheels/AtGoal")
  public boolean atGoal() {
    return goal == Goal.IDLE
        || (leftProfile.getCurrentSetpoint() == goal.getLeftGoal()
            && rightProfile.getCurrentSetpoint() == goal.getRightGoal());
  }

  public Command shootCommand() {
    return startEnd(
            () -> runVolts(Goal.SHOOT.getLeftGoal(), Goal.SHOOT.rightGoal),
            () -> runVolts(Goal.IDLE.getLeftGoal(), Goal.IDLE.rightGoal))
        .withName("Flywheels Shoot");
  }

  public Command intakeCommand() {
    return startEnd(
            () -> runVolts(Goal.INTAKE.getLeftGoal(), Goal.INTAKE.rightGoal),
            () -> runVolts(Goal.IDLE.getLeftGoal(), Goal.IDLE.rightGoal))
        .withName("Flywheels Intake");
  }

  public Command demoIntakeCommand() {
    return startEnd(
            () -> runVolts(Goal.DEMO_INTAKE.getLeftGoal(), Goal.DEMO_INTAKE.rightGoal),
            () -> runVolts(Goal.IDLE.getLeftGoal(), Goal.IDLE.rightGoal))
        .withName("Flywheels Demo Intake");
  }

  public Command ejectCommand() {
    return startEnd(
            () -> runVolts(Goal.EJECT.getLeftGoal(), Goal.EJECT.rightGoal),
            () -> runVolts(Goal.IDLE.getLeftGoal(), Goal.IDLE.rightGoal))
        .withName("Flywheels Eject");
  }

  public Command poopCommand() {
    return startEnd(
            () -> runVolts(Goal.POOP.getLeftGoal(), Goal.POOP.rightGoal),
            () -> runVolts(Goal.IDLE.getLeftGoal(), Goal.IDLE.rightGoal))
        .withName("Flywheels Poop");
  }

  public void runVolts(double shootingLeftRpm, double shootingRightRpm) {
    io.runVolts(shootingLeftRpm, shootingRightRpm);
  }

  public void stop() {
    io.stop();
  }
}
