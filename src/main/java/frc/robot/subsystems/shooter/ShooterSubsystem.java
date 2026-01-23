// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.util.FuelTrajectoryCalculator;
import frc.robot.util.FuelTrajectoryCalculator.ShooterSetpoint;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
  private final FlywheelIO flywheelIO;
  private final FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();

  private final HoodIO hoodIO;
  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(FlywheelIO shooterIO, HoodIO hoodIO) {
    this.flywheelIO = shooterIO;
    this.hoodIO = hoodIO;

    hoodIO.resetPosition(ShooterConstants.hoodPhysicalBottomOutRotation);
  }

  @Override
  public void periodic() {
    flywheelIO.updateInputs(flywheelInputs);
    Logger.processInputs("Shooter/Flywheel", flywheelInputs);

    hoodIO.updateInputs(hoodInputs);
    Logger.processInputs("Shooter/Hood", hoodInputs);
  }

  /* Clamps the given angle to the allowed range, sets the hood to the clamped angle, and returns the clamped angle. */
  private Angle setHoodAngle(Angle angle) {
    Angle clampedAngle =
        Degrees.of(
            MathUtil.clamp(
                angle.in(Degrees),
                ShooterConstants.hoodMinRotation.in(Degrees),
                ShooterConstants.hoodMaxRotation.in(Degrees)));

    hoodIO.setAngle(clampedAngle);
    Logger.recordOutput("Shooter/Hood/SetpointAngle", ShooterConstants.hoodMinRotation);

    return clampedAngle;
  }

  private void runFlywheelWithHood(AngularVelocity velocity, Angle targetHoodAngle) {
    Angle clampedHoodTarget = setHoodAngle(targetHoodAngle);

    if (hoodInputs.angle.isNear(clampedHoodTarget, ShooterConstants.hoodToleranceWhenShooting)) {
      flywheelIO.setAngularVelocity(velocity);
      Logger.recordOutput("Shooter/Flywheel/SetpointVelocity", velocity);
    } else {
      flywheelIO.coast();
      Logger.recordOutput("Shooter/Flywheel/SetpointVelocity", RotationsPerSecond.of(0));
    }
  }

  private void runFlywheelAtFuelSpeedWithHood(LinearVelocity fuelVelocity, Angle targetHoodAngle) {
    AngularVelocity angularVelocity =
        RadiansPerSecond.of(
            fuelVelocity.div(ShooterConstants.fuelToFlywheelLinearSpeedRatio).in(MetersPerSecond)
                / ShooterConstants.flywheelRadius.in(Meters));

    runFlywheelWithHood(angularVelocity, targetHoodAngle);
  }

  private void coastFlywheel() {
    flywheelIO.coast();
    Logger.recordOutput("Shooter/Flywheel/SetpointVelocity", RotationsPerSecond.of(0));
  }

  /* Lowers hood and sets shooter to coast. */
  private void reset() {
    setHoodAngle(ShooterConstants.hoodMinRotation);
    coastFlywheel();
  }

  public Command setHoodAngleCommand(Angle angle) {
    return runEnd(
        () -> {
          setHoodAngle(angle);
        },
        this::reset);
  }

  public Command runFlywheelAtHoodAngleCommand(AngularVelocity velocity, Angle angle) {
    return runEnd(
        () -> {
          Angle clampedAngle = setHoodAngle(angle);
          runFlywheelWithHood(velocity, clampedAngle);
        },
        this::reset);
  }

  public Command prepareHoodForShootCommand(
      Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
    return runEnd(
        () -> {
          ShooterSetpoint shooterSetpoint =
              FuelTrajectoryCalculator.calcualteShooterSetpoint(
                  robotPoseSupplier.get(), chassisSpeedsSupplier.get());

          setHoodAngle(shooterSetpoint.hoodAngle());
        },
        this::reset);
  }

  public Command shootCommand(
      Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
    return runEnd(
        () -> {
          ShooterSetpoint shooterSetpoint =
              FuelTrajectoryCalculator.calcualteShooterSetpoint(
                  robotPoseSupplier.get(), chassisSpeedsSupplier.get());

          Angle clampedAngle = setHoodAngle(shooterSetpoint.hoodAngle());
          runFlywheelAtFuelSpeedWithHood(shooterSetpoint.linearFlywheelSpeed(), clampedAngle);
        },
        this::reset);
  }

  public Angle getHoodAngle() {
    return hoodInputs.angle;
  }

  public AngularVelocity getFlywheelVelocity() {
    return flywheelInputs.velocity;
  }

  public LinearVelocity getFuelVelocity() {
    return MetersPerSecond.of(
        flywheelInputs.velocity.in(RadiansPerSecond)
            * ShooterConstants.flywheelRadius.in(Meters)
            * ShooterConstants.fuelToFlywheelLinearSpeedRatio);
  }
}
