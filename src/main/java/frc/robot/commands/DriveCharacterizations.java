// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.swerve.SwerveConstants;
import frc.robot.constants.swerve.SysIdConstants;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import frc.robot.subsystems.drivetrain.SwerveSubsystem.SwerveSysIdRoutine;
import java.text.DecimalFormat;
import java.text.NumberFormat;

public class DriveCharacterizations {
  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(SwerveSubsystem drive) {
    class CharacterizationState {
      Angle[] initialModuleRotations = new Angle[4];
      Rotation2d lastDrivetrainAngle = Rotation2d.kZero;
      double accumulatedDrivetrainRotationDeltaDegrees = 0;
    }

    final AngularVelocity maxVelocity = RadiansPerSecond.of(0.25);
    final AngularAcceleration rampRate = RadiansPerSecondPerSecond.of(0.05);

    SlewRateLimiter accelerationLimiter =
        new SlewRateLimiter(rampRate.in(RadiansPerSecondPerSecond));
    CharacterizationState state = new CharacterizationState();

    return Commands.parallel(
        Commands.sequence( // Drive control sequence
            Commands.runOnce(() -> accelerationLimiter.reset(0.0)),
            Commands.run(
                () -> {
                  double speed = accelerationLimiter.calculate(maxVelocity.in(RadiansPerSecond));
                  drive.drive(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),
        Commands.sequence( // Measurement sequence
            Commands.waitSeconds(1.0), // Wait for modules to fully align
            Commands.runOnce( // Record starting measurement
                () -> {
                  state.initialModuleRotations = drive.getModuleDriveAngles();
                  state.lastDrivetrainAngle = drive.getRotation();
                  state.accumulatedDrivetrainRotationDeltaDegrees = 0;
                }),
            Commands.run( // Update state
                    () -> {
                      Rotation2d rotation = drive.getRotation();
                      state.accumulatedDrivetrainRotationDeltaDegrees +=
                          Math.abs(rotation.minus(state.lastDrivetrainAngle).getDegrees());
                      state.lastDrivetrainAngle = rotation;
                    })
                .finallyDo( // When canceled, calculate and print results
                    () -> {
                      Angle rotationDelta =
                          Degrees.of(state.accumulatedDrivetrainRotationDeltaDegrees);

                      Angle[] moduleRotations = drive.getModuleDriveAngles();
                      double totalWheelDeltaRadians = 0;
                      for (int i = 0; i < 4; i++) {
                        totalWheelDeltaRadians +=
                            moduleRotations[i].minus(state.initialModuleRotations[i]).abs(Radians);
                      }
                      Angle wheelDelta = Radians.of(totalWheelDeltaRadians / 4);

                      Distance wheelRadius =
                          rotationDelta.div(wheelDelta).times(SwerveConstants.driveBaseRadius);

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println("\tWheel Delta: " + wheelDelta.toLongString());
                      System.out.println(
                          "\tGyro Delta: "
                              + formatter.format(rotationDelta.in(Rotations))
                              + " rotations");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius.in(Meters))
                              + " meters, "
                              + formatter.format(wheelRadius.in(Inches))
                              + " inches");
                    })));
  }

  /**
   * Measures the robot's max possible speed by driving full speed robot-relative forwards. Nudges
   * forwards first to indicate the direction it'll go in.
   */
  public static Command maxSpeedCharacterization(SwerveSubsystem drive) {
    class CharacterizationState {
      double maxVelocityMetersPerSecond = Double.MIN_VALUE;
      Timer timeSinceLastMax = new Timer();
    }

    SlewRateLimiter accelerationLimiter = new SlewRateLimiter(15, 25, 0);
    CharacterizationState state = new CharacterizationState();

    return Commands.sequence(
        Commands.runOnce(
            () -> {
              state.maxVelocityMetersPerSecond = Double.MIN_VALUE;
              accelerationLimiter.reset(0);
            }),
        Commands.run(() -> drive.drive(new ChassisSpeeds(0.05, 0, 0))).withTimeout(Seconds.of(0.4)),
        Commands.runOnce(() -> drive.drive(new ChassisSpeeds())),
        new WaitCommand(Seconds.of(1)),
        Commands.run(
                () -> {
                  state.timeSinceLastMax.start();

                  double currentMeasuredVelocityMetersPerSecond =
                      drive.getChassisSpeeds().vxMetersPerSecond;
                  double currentSetpointVelocityMetersPerSecond =
                      accelerationLimiter.calculate(Double.MAX_VALUE);

                  if (currentMeasuredVelocityMetersPerSecond - state.maxVelocityMetersPerSecond
                      > 5e-2) {
                    state.timeSinceLastMax.restart();
                  }

                  state.maxVelocityMetersPerSecond =
                      Double.max(
                          currentMeasuredVelocityMetersPerSecond, state.maxVelocityMetersPerSecond);

                  drive.drive(new ChassisSpeeds(currentSetpointVelocityMetersPerSecond, 0, 0));
                },
                drive)
            .until(
                () ->
                    state.timeSinceLastMax.hasElapsed(Units.millisecondsToSeconds(65))
                        && state.maxVelocityMetersPerSecond > 1e-1),
        Commands.run(
                () -> drive.drive(new ChassisSpeeds(accelerationLimiter.calculate(0), 0, 0)), drive)
            .withTimeout(Seconds.of(1)),
        Commands.runOnce(() -> drive.drive(new ChassisSpeeds()), drive),
        Commands.runOnce(
            () -> {
              NumberFormat formatter = new DecimalFormat("#0.000");
              System.out.println("********** Max Speed Characterization Results **********");
              System.out.println(
                  "\tMax Speed: "
                      + formatter.format(state.maxVelocityMetersPerSecond)
                      + " meters per second");
            }));
  }

  /** Measures the robot's slip current. Place robot in front of a solid wall before running. */
  public static Command slipCurrentCharacterization(SwerveSubsystem drive) {
    class CharacterizationState {
      Current maxCurrent = Amps.of(0);
    }

    SlewRateLimiter accelerationLimiter = new SlewRateLimiter(0.1);
    CharacterizationState state = new CharacterizationState();

    return Commands.sequence(
        Commands.runOnce(
            () -> {
              state.maxCurrent = Amps.of(0);
              accelerationLimiter.reset(0);
            }),
        Commands.run(() -> drive.drive(new ChassisSpeeds(0.08, 0, 0)), drive)
            .withTimeout(Seconds.of(1)),
        Commands.run(
                () ->
                    drive.runCharacterization(
                        SwerveSysIdRoutine.DRIVE_LINEAR_FEEDFORWARD,
                        accelerationLimiter.calculate(Double.MAX_VALUE)),
                drive)
            .withTimeout(Seconds.of(0.5)),
        Commands.run(
                () -> {
                  drive.runCharacterization(
                      SwerveSysIdRoutine.DRIVE_LINEAR_FEEDFORWARD,
                      accelerationLimiter.calculate(Double.MAX_VALUE));

                  Current activeCurrent = drive.getAverageCurrent();
                  if (state.maxCurrent.lt(activeCurrent)) {
                    state.maxCurrent = activeCurrent;
                  }
                },
                drive)
            .until(() -> drive.getChassisSpeeds().vxMetersPerSecond > 3e-2),
        Commands.runOnce(
            () -> {
              NumberFormat formatter = new DecimalFormat("#0.000");
              System.out.println("********** Slip Current Characterization Results **********");
              System.out.println(
                  "\tSlip Current: " + formatter.format(state.maxCurrent.in(Amps)) + " amps");
            }));
  }

  /** Runs all SysId routines. The advanceRoutine supplier ends the routine early. */
  public static Command sysIdCharacterization(SwerveSubsystem drive, Trigger advanceRoutine) {
    return Commands.sequence(
        Commands.run(() -> drive.runCharacterization(SysIdConstants.activeRoutine, 0), drive)
            .withTimeout(Seconds.of(1.5)),
        Commands.waitUntil(advanceRoutine.negate()),
        drive.sysIdDynamic(SysIdRoutine.Direction.kForward).until(advanceRoutine),
        Commands.waitUntil(advanceRoutine.negate()),
        drive.sysIdDynamic(SysIdRoutine.Direction.kReverse).until(advanceRoutine),
        Commands.waitUntil(advanceRoutine.negate()),
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward).until(advanceRoutine),
        Commands.waitUntil(advanceRoutine.negate()),
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse).until(advanceRoutine));
  }
}
