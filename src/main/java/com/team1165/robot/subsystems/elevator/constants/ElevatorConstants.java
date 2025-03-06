/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.elevator.constants;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorConstants {
  public static final String canBus = "canivore";

  // Intake position
  public static final double intakePosition = 0.0;

  // General characteristics
  public static final double gearRatio = 5.0;
  public static final double sprocketRadiusInches = 0.8755;
  public static final double minHeightInches = 0.0;
  public static final double maxHeightInches = 40.872;

  // PID gains
  public static final Slot0Configs gains =
      new Slot0Configs()
          .withKP(0.0)
          .withKI(0.0)
          .withKD(0.0)
          .withKS(0.0)
          .withKG(0.0)
          .withKV(0.0)
          .withKA(0.0);

  public static final class LeftMotorConstants {
    public static final int canID = 14;
  }

  public static final class RightMotorConstants {
    public static final int canID = 15;
  }

  public static TalonFXConfiguration ELEVATOR_CONFIG = new TalonFXConfiguration();

  static {
    ELEVATOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    ELEVATOR_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Inches.of(66).in(Inches);
    ELEVATOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    ELEVATOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Inches.of(0).in(Inches);

    ELEVATOR_CONFIG.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    // Elevator motors will provide feedback in INCHES the carriage has moved
    ELEVATOR_CONFIG.Feedback.SensorToMechanismRatio =
        (2 * Math.PI * (2 * sprocketRadiusInches))
            / gearRatio; // TODO: Check and make sure this math is right!!!!
    ELEVATOR_CONFIG.Slot0.kG = 0.3;
    ELEVATOR_CONFIG.Slot0.kS = 0.4;
    // ELEVATOR_CONFIG.Slot0.kP = 1;
    ELEVATOR_CONFIG.Slot0.kP = 0.3;

    ELEVATOR_CONFIG.CurrentLimits.SupplyCurrentLimit = 80;
    ELEVATOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
    ELEVATOR_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    ELEVATOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
  }

  //  public static final ElevatorConfig elevatorConfig =
  //      switch (Constants.getRobot()) {
  //        case COMPBOT -> new ElevatorConfig(4, 0, (1.0 / 2.0), 9000.0);
  //        case DEVBOT -> new ElevatorConfig(5, 4, (1.0 / 2.0), 6000.0);
  //        case SIMBOT -> new ElevatorConfig(0, 0, (1.0 / 2.0), 9000.0);
  //      };

  //  public static final Gains gains =
  //      switch (Constants.getRobot()) {
  //        case COMPBOT -> new Gains(0.18, 0, 0.0006, 0.38367, 0.00108, 0);
  //        case DEVBOT -> new Gains(0.0003, 0.0, 0.0, 0.33329, 0.00083, 0.0);
  //        case SIMBOT -> new Gains(0.05, 0.0, 0.0, 0.01, 0.00103, 0.0);
  //      };

}
