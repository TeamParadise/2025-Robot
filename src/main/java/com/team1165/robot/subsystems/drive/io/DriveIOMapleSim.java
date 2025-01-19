/*
 * Copyright (c) 2025 Team Paradise - FRC 1165 (https://github.com/TeamParadise)
 *
 * Use of this source code is governed by the MIT License, which can be found in the LICENSE file at
 * the root directory of this project.
 */

package com.team1165.robot.subsystems.drive.io;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.team1165.robot.subsystems.drive.constants.TunerConstants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.littletonrobotics.junction.Logger;

/**
 * {@link DriveIO} class that implements the CTRE {@link SwerveDrivetrain} class, and makes it
 * partially AdvantageKit compatible. This class utilizes the maple-sim {@link
 * SwerveDriveSimulation} in order for more realistic simulation of the drivetrain.
 */
public class DriveIOMapleSim extends DriveIOReal {
  private SwerveDriveSimulation driveSim;
  private Pigeon2SimState gyroSimState;
  private Notifier simNotifier;

  /**
   * Constructs a {@link DriveIOMapleSim} using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves.
   *
   * <p>Some of these simulation values for the {@link MapleSimConfig} can be grabbed through a
   * PathPlanner {@link com.pathplanner.lib.config.RobotConfig} and {@link
   * com.pathplanner.lib.config.ModuleConfig}, if you are utilizing PathPlanner.
   *
   * <p>Final note, this constructor automatically will regulate certain {@link
   * SwerveModuleConstants} in order to ensure that they work properly with simulation.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive.
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param odometryStandardDeviation The standard deviation for odometry calculation in the form
   *     [x, y, theta]ᵀ, with units in meters and radians.
   * @param visionStandardDeviation The standard deviation for vision calculation in the form [x, y,
   *     theta]ᵀ, with units in meters and radians.
   * @param simConfig The {@link MapleSimConfig} to be used for this drivetrain simulation.
   * @param modules Constants for each specific module.
   */
  public DriveIOMapleSim(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      Matrix<N3, N1> odometryStandardDeviation,
      Matrix<N3, N1> visionStandardDeviation,
      MapleSimConfig simConfig,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(
        drivetrainConstants,
        odometryUpdateFrequency,
        odometryStandardDeviation,
        visionStandardDeviation,
        regulateModuleConstantsForSimulation(modules));
    setupSimulation(simConfig, regulateModuleConstantsForSimulation(modules));
  }

  /**
   * Constructs a {@link DriveIOMapleSim} using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves.
   *
   * <p>Some of these simulation values for the {@link MapleSimConfig} can be grabbed through a
   * PathPlanner {@link com.pathplanner.lib.config.RobotConfig} and {@link
   * com.pathplanner.lib.config.ModuleConfig}, if you are utilizing PathPlanner.
   *
   * <p>Final note, this constructor automatically will regulate certain {@link
   * SwerveModuleConstants} in order to ensure that they work properly with simulation.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive.
   * @param simConfig The {@link MapleSimConfig} to be used for this drivetrain simulation.
   * @param modules Constants for each specific module.
   */
  public DriveIOMapleSim(
      SwerveDrivetrainConstants drivetrainConstants,
      MapleSimConfig simConfig,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, regulateModuleConstantsForSimulation(modules));
    setupSimulation(simConfig, regulateModuleConstantsForSimulation(modules));
  }

  public static SwerveModuleConstants<?, ?, ?>[] regulateModuleConstantsForSimulation(
      SwerveModuleConstants<?, ?, ?>... moduleConstants) {
    for (SwerveModuleConstants<?, ?, ?> moduleConstant : moduleConstants) {
      // Apply simulation-specific adjustments to module constants
      moduleConstant
          // Disable encoder offsets
          .withEncoderOffset(0)
          // Disable motor inversions for drive and steer motors
          .withDriveMotorInverted(false)
          .withSteerMotorInverted(false)
          // Disable CanCoder inversion
          .withEncoderInverted(false)
          // Adjust steer motor PID gains for simulation
          .withSteerMotorGains(
              moduleConstant
                  .SteerMotorGains
                  .withKP(70) // Proportional gain
                  .withKD(4.5)) // Derivative gain
          // Adjust friction voltages
          .withDriveFrictionVoltage(Volts.of(0.1))
          .withSteerFrictionVoltage(Volts.of(0.15))
          // Adjust steer inertia
          .withSteerInertia(KilogramSquareMeters.of(0.05));
    }

    return moduleConstants;
  }

  private void setupSimulation(
      MapleSimConfig simConfig, SwerveModuleConstants<?, ?, ?>... modules) {
    this.gyroSimState = getPigeon2().getSimState();
    var drivetrainSimConfig =
        DriveTrainSimulationConfig.Default()
            .withRobotMass(simConfig.robotMassWithBumpers())
            .withBumperSize(simConfig.bumperLengthX(), simConfig.bumperWidthY())
            .withCustomModuleTranslations(getModuleLocations())
            .withGyro(COTS.ofPigeon2())
            .withSwerveModule(
                new SwerveModuleSimulationConfig(
                    simConfig.driveMotorModel(),
                    simConfig.steerMotorModel(),
                    modules[0].DriveMotorGearRatio,
                    modules[0].SteerMotorGearRatio,
                    Volts.of(modules[0].DriveFrictionVoltage),
                    Volts.of(modules[0].SteerFrictionVoltage),
                    Meters.of(modules[0].WheelRadius),
                    KilogramSquareMeters.of(modules[0].SteerInertia),
                    simConfig.wheelCOF()));

    driveSim = new SwerveDriveSimulation(drivetrainSimConfig, new Pose2d());

    // Make sure all the modules use the Talon FX motor simulations
    SwerveModuleSimulation[] moduleSimulations = driveSim.getModules();
    for (int i = 0; i < moduleSimulations.length; i++) {
      var module = getModule(i);
      moduleSimulations[i].useDriveMotorController(new SimulatedTalonFX(module.getDriveMotor()));
      moduleSimulations[i].useSteerMotorController(
          new SimulatedTalonFXWithCANcoder(module.getSteerMotor(), module.getEncoder()));
    }

    SimulatedArena.overrideSimulationTimings(simConfig.simLoopPeriod(), 1);
    SimulatedArena.getInstance().addDriveTrainSimulation(driveSim);

    simNotifier =
        new Notifier(
            () -> {
              SimulatedArena.getInstance().simulationPeriodic();
              gyroSimState.setRawYaw(
                  driveSim.getSimulatedDriveTrainPose().getRotation().getMeasure());
              gyroSimState.setAngularVelocityZ(
                  RadiansPerSecond.of(
                      driveSim.getDriveTrainSimulatedChassisSpeedsRobotRelative()
                          .omegaRadiansPerSecond));
            });
    simNotifier.startPeriodic(simConfig.simLoopPeriod().in(Seconds));

    // Make sure the robot starts in the field
    resetPose(new Pose2d(new Translation2d(1.5, 1.5), new Rotation2d()));
  }

  /**
   * Updates a {@link DriveIOInputs} instance with the latest updates from this {@link DriveIO}.
   *
   * @param inputs A {@link DriveIOInputs} instance to update.
   */
  @Override
  public void updateInputs(DriveIOInputs inputs) {
    // Update the inputs passed in with the current SwerveDriveState
    inputs.fromSwerveDriveState(getState());

    // Log the drivetrain pose to NetworkTables
    Logger.recordOutput("Drive/SimulatedPose", driveSim.getSimulatedDriveTrainPose());
  }

  @Override
  public void resetPose(Pose2d pose) {
    // Reset the simulated pose as well as the odometry pose
    driveSim.setSimulationWorldPose(pose);
    Timer.delay(0.1);
    super.resetPose(pose);
  }

  public Pose2d getSimulationPose() {
    return driveSim.getSimulatedDriveTrainPose();
  }

  /**
   * Class used to wrap a Talon FX simulation state ({@link TalonFXSimState}) in a {@link
   * SimulatedMotorController}, in order to update the simulation state with the values from a
   * {@link SwerveModuleSimulation}.
   */
  public static class SimulatedTalonFX implements SimulatedMotorController {
    private final TalonFXSimState motorSimState;

    /**
     * Constructs a {@link SimulatedTalonFX} with the provided {@link TalonFX}.
     *
     * @param motor The {@link TalonFX} that this {@link SimulatedTalonFX} will update the
     *     simulation state of.
     */
    public SimulatedTalonFX(TalonFX motor) {
      this.motorSimState = motor.getSimState();
    }

    /**
     * Update the {@link TalonFXSimState} of this {@link SimulatedTalonFX} with the new simulated
     * values.
     *
     * @param mechanismAngle The angle of the mechanism the Talon FX is controlling. This is the
     *     angle reported by the encoder divided by the gear ratio.
     * @param mechanismVelocity The velocity of the mechanism the Talon FX is controlling. This is
     *     the velocity reported by the encoder divided by the gear ratio.
     * @param encoderAngle The angle of the encoder of the Talon FX. This is the direct angle
     *     reported by the encoder.
     * @param encoderVelocity The velocity of the encoder of the Talon FX. This is the direct
     *     velocity reported by the encoder.
     * @return The simulated output voltage of the Talon FX.
     */
    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      // Update motor state with provided values
      motorSimState.setRawRotorPosition(encoderAngle);
      motorSimState.setRotorVelocity(encoderVelocity);
      motorSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());

      // Get the output voltage of the motor
      return motorSimState.getMotorVoltageMeasure();
    }
  }

  /**
   * Class used to wrap a Talon FX simulation state ({@link TalonFXSimState}) and a remote CANcoder
   * simulation state ({@link CANcoderSimState} in a {@link SimulatedMotorController}, in order to
   * update the simulation states with the values from a {@link SwerveModuleSimulation}.
   */
  public static class SimulatedTalonFXWithCANcoder extends SimulatedTalonFX {
    private final CANcoderSimState encoderSimState;

    /**
     * Constructs a {@link SimulatedTalonFX} with the provided {@link TalonFX} and {@link CANcoder}.
     *
     * @param motor The {@link TalonFX} that this {@link SimulatedTalonFXWithCANcoder} will update
     *     the simulation state of.
     * @param encoder The remote {@link CANcoder} that this {@link SimulatedTalonFXWithCANcoder}
     *     will update the simulation state of.
     */
    public SimulatedTalonFXWithCANcoder(TalonFX motor, CANcoder encoder) {
      super(motor);
      this.encoderSimState = encoder.getSimState();
    }

    /**
     * Update the {@link TalonFXSimState} and {@link CANcoderSimState} of this {@link
     * SimulatedTalonFXWithCANcoder} with the new simulated values.
     *
     * @param mechanismAngle The angle of the mechanism the Talon FX is controlling. This is the
     *     angle reported by the encoder divided by the gear ratio.
     * @param mechanismVelocity The velocity of the mechanism the Talon FX is controlling. This is
     *     the velocity reported by the encoder divided by the gear ratio.
     * @param encoderAngle The angle of the encoder of the Talon FX. This is the direct angle
     *     reported by the encoder.
     * @param encoderVelocity The velocity of the encoder of the Talon FX. This is the direct
     *     velocity reported by the encoder.
     * @return The simulated output voltage of the Talon FX.
     */
    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      encoderSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
      encoderSimState.setRawPosition(mechanismAngle);
      encoderSimState.setVelocity(mechanismVelocity);

      return super.updateControlSignal(
          mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
    }
  }

  /**
   * A configuration class for a {@link DriveIOMapleSim}, in order to provide values not provided by
   * the {@link TunerConstants} of the CTRE Swerve Library. A lot of these value's can be
   *
   * @param simLoopPeriod The amount of time between each loop of the simulation.
   * @param robotMassWithBumpers The {@link Mass} of the robot on the competition field.
   * @param bumperLengthX The length of the bumpers in the X direction.
   * @param bumperWidthY The width of the bumpers in the Y direction.
   * @param driveMotorModel The model of the drive motor on this simulated swerve module.
   * @param steerMotorModel The model of the steer motor on this simulated swerve module.
   * @param wheelCOF The COF (coefficient of friction) of the drive wheel. Look at {@link
   *     org.ironmaple.simulation.drivesims.COTS.WHEELS} for some examples.
   */
  public record MapleSimConfig(
      Time simLoopPeriod,
      Mass robotMassWithBumpers,
      Distance bumperLengthX,
      Distance bumperWidthY,
      DCMotor driveMotorModel,
      DCMotor steerMotorModel,
      double wheelCOF) {}
}
