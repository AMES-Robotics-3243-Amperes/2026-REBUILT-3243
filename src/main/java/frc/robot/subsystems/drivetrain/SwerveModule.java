// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.subsystems.drivetrain.SwerveSubsystem.SwerveSysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;
  private final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      constants;

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private final Alert turnEncoderDisconnectedAlert;
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  public SwerveModule(
      ModuleIO io,
      int index,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    this.io = io;
    this.index = index;
    this.constants = constants;
    driveDisconnectedAlert =
        new Alert(
            "Disconnected drive motor on module " + Integer.toString(index) + ".",
            AlertType.kError);
    turnDisconnectedAlert =
        new Alert(
            "Disconnected turn motor on module " + Integer.toString(index) + ".", AlertType.kError);
    turnEncoderDisconnectedAlert =
        new Alert(
            "Disconnected turn encoder on module " + Integer.toString(index) + ".",
            AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drivetrain/Module" + Integer.toString(index), inputs);

    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsRad[i] * constants.WheelRadius;
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    // Update alerts
    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
    turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  public void runSetpoint(SwerveModuleState state) {
    // Optimize velocity setpoint
    Rotation2d azimuthRotation = Rotation2d.fromRadians(getAzimuthAngle().in(Radians));
    state.optimize(azimuthRotation);
    state.cosineScale(azimuthRotation);

    // Apply setpoints
    io.setDriveVelocity(RadiansPerSecond.of(state.speedMetersPerSecond / constants.WheelRadius));
    io.setTurnPosition(state.angle);
  }

  /**
   * Runs the module with the specified setpoint state and feedforwards. Mutates the state to
   * optimize it.
   */
  public void runSetpoint(SwerveModuleState state, LinearAcceleration feedforwards) {
    // Optimize velocity setpoint
    Rotation2d azimuthRotation = Rotation2d.fromRadians(getAzimuthAngle().in(Radians));
    state.optimize(azimuthRotation);
    state.cosineScale(azimuthRotation);

    // Apply setpoints
    io.setDriveSetpoint(
        RadiansPerSecond.of(state.speedMetersPerSecond / constants.WheelRadius),
        RadiansPerSecondPerSecond.of(
            feedforwards.in(MetersPerSecondPerSecond) / constants.WheelRadius));
    io.setTurnPosition(state.angle);
  }

  /** Runs the SysId characterization with the given output. */
  public void runCharacterization(SwerveSysIdRoutine routine, double output) {
    switch (routine) {
      case DRIVE_LINEAR_FEEDFORWARD:
        io.setDriveOpenLoop(output);
        io.setTurnPosition(Rotation2d.kZero);
        break;

      case AZIMUTH_FEEDFORWARD:
        io.setDriveOpenLoop(0.0);
        io.setTurnOpenLoop(output);
        break;
    }
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(0.0);
  }

  /** Returns the current applied voltage of the module. */
  public Voltage getAppliedDriveVoltage() {
    return inputs.driveAppliedVolts;
  }

  /** Returns the current applied turn voltage of the module. */
  public Voltage getAppliedAzimuthVoltage() {
    return inputs.turnAppliedVolts;
  }

  /** Returns the current of the module. */
  public Current getCurrent() {
    return inputs.driveCurrent;
  }

  /** Returns the current turn angle of the module. */
  public Angle getAzimuthAngle() {
    return inputs.turnPosition;
  }

  /** Returns the current turn velocity of the module. */
  public AngularVelocity getAzimuthVelocity() {
    return inputs.turnVelocity;
  }

  /** Returns the current linear drive position of the module. */
  public Distance getLinearDistance() {
    return Meters.of(constants.WheelRadius).times(inputs.drivePosition.in(Radians));
  }

  /** Returns the current linear drive velocity of the module. */
  public LinearVelocity getLinearVelocity() {
    return MetersPerSecond.of(constants.WheelRadius)
        .times(inputs.driveVelocity.in(RadiansPerSecond));
  }

  /** Returns the angular drive position. */
  public Angle getAngularDrivePosition() {
    return inputs.drivePosition;
  }

  /** Returns the angular drive velocity. */
  public AngularVelocity getAngularDriveVelocity() {
    return inputs.driveVelocity;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getLinearDistance(), Rotation2d.fromRadians(getAzimuthAngle().in(Radians)));
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getLinearVelocity(), Rotation2d.fromRadians(getAzimuthAngle().in(Radians)));
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }
}
