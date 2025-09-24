/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.team1165.robot.globalconstants.CANConstants.IDs.CANivore;
import com.team1165.robot.util.vendor.ctre.PhoenixDeviceConfigs.TalonFXConfig;

/** Class to store constants for the {@link Elevator} subsystem of the robot. */
public class ElevatorConstants {
  public static final class ElevatorCharacteristics {
    // TODO: Update these to be actually right
    public static final double gearRatio = 5.0;
    public static final double sprocketRadiusInches = 0.8755;
    public static final double minHeightInches = 0.0;
    public static final double maxHeightInches = 40.872;
  }

  public static final class Motors {
    // Base configuration creation
    public static final TalonFXConfiguration baseMotorConfig = new TalonFXConfiguration();

    public static final double primaryZeroPosition = -0.05826724051 - 0.38;
    public static final double secondaryZeroPosition = 0.12982191818 - 0.048;

    static {
      baseMotorConfig.CurrentLimits.SupplyCurrentLimit = 60;
      baseMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

      baseMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      baseMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

      // Elevator motors will provide feedback in INCHES the carriage has moved
      // TODO: Fix this math
      baseMotorConfig.Feedback.SensorToMechanismRatio =
          (2 * Math.PI * (2 * ElevatorCharacteristics.sprocketRadiusInches))
              / ElevatorCharacteristics.gearRatio;

      baseMotorConfig.Slot0 =
          new Slot0Configs()
              .withKP(10.0)
              .withKI(0.0)
              .withKD(0.0)
              .withKS(0.22)
              .withKG(0.33)
              .withKV(0.0)
              .withKA(0.0)
              .withGravityType(GravityTypeValue.Elevator_Static)
              .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

      baseMotorConfig.MotionMagic =
          new MotionMagicConfigs()
              .withMotionMagicCruiseVelocity(15)
              .withMotionMagicAcceleration(30);
    }

    // Individual Talon FX configurations
    public static final TalonFXConfig primaryMotorConfig =
        new TalonFXConfig(
            "ElevatorPrimary", CANivore.elevatorPrimary, CANivore.name, baseMotorConfig);
    public static final TalonFXConfig secondaryMotorConfig =
        new TalonFXConfig(
            "ElevatorSecondary", CANivore.elevatorSecondary, CANivore.name, baseMotorConfig);
  }
}
