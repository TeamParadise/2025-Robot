/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.elevator.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

public class ElevatorConstants {

  public static final Distance CORAL_L1_HEIGHT = Units.Inches.of(9.039062);
  public static final Distance CORAL_L2_HEIGHT = Units.Inches.of(17.946289);
  public static final Distance CORAL_L3_HEIGHT = Units.Inches.of(33.742188);
  public static final Distance CORAL_L4_HEIGHT = Units.Inches.of(58.888916);

  public static final double SimCORAL_L1_HEIGHT = 9.039062;
  public static final double SimCORAL_L2_HEIGHT = 17.946289;
  public static final double SimCORAL_L3_HEIGHT = 33.742188;
  public static final double SimCORAL_L4_HEIGHT = 58.888916;

  public static final Distance ALGAE_PREP_NET = Units.Inches.of(50);
  public static final Distance ALGAE_PREP_PROCESSOR_HEIGHT = Units.Inches.of(1);
  public static final Distance ALGAE_L3_CLEANING = Units.Inches.of(35);
  public static final Distance ALGAE_L2_CLEANING = Units.Inches.of(25);
  public static final Distance ALGAE_GROUND_INTAKE = Units.Inches.of(0);
  public static final Distance PREP_0 = Units.Inches.of(0);

  public static TalonFXConfiguration ELEVATOR_CONFIG = new TalonFXConfiguration();

  static {
    ELEVATOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    ELEVATOR_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Units.Inches.of(66).in(Units.Inches);
    ELEVATOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    ELEVATOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Units.Inches.of(0).in(Units.Inches);

    ELEVATOR_CONFIG.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    // Elevator motors will provide feedback in INCHES the carriage has moved
    ELEVATOR_CONFIG.Feedback.SensorToMechanismRatio = 0.4545;
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
