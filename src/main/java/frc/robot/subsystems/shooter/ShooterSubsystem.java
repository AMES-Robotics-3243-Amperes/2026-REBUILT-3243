// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.util.FuelTrajectoryCalculator;
import frc.robot.util.FuelTrajectoryCalculator.ShooterSetpoint;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
  private final FlywheelIO shooterIO;
  private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  private final HoodIO hoodIO;
  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(FlywheelIO shooterIO, HoodIO hoodIO) {
    this.shooterIO = shooterIO;
    this.hoodIO = hoodIO;
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(shooterInputs);
    Logger.processInputs("Shooter/Flywheel", shooterInputs);

    hoodIO.updateInputs(hoodInputs);
    Logger.processInputs("Shooter/Hood", hoodInputs);
  }

  public Command runFlywheelAtSpeedCommand(AngularVelocity velocity) {
    return runEnd(
        () -> {
          shooterIO.setAngularVelocity(velocity);
          Logger.recordOutput("Shooter/Flywheel/SetpointVelocity", velocity);
        },
        () -> {
          shooterIO.setAngularVelocity(RPM.of(0));
          Logger.recordOutput("Shooter/Flywheel/SetpointVelocity", RotationsPerSecond.of(0));
        });
  }

  public Command setHoodAngleCommand(Angle angle) {
    // safety mechanism for when someone inevitably puts 0 rotation without thinking
    Angle clampedAngle =
        Degrees.of(
            MathUtil.clamp(
                angle.abs(Degrees),
                ShooterConstants.hoodMinRotation.in(Degrees),
                ShooterConstants.hoodMaxRotation.in(Degrees)));

    return runEnd(
        () -> {
          hoodIO.setAngle(clampedAngle);
          Logger.recordOutput("Shooter/Hood/SetpointAngle", angle);
        },
        () -> {
          hoodIO.setAngle(ShooterConstants.hoodMinRotation);
          Logger.recordOutput("Shooter/Hood/SetpointAngle", ShooterConstants.hoodMinRotation);
        });
  }

  public Command lowerHoodCommand() {
    return setHoodAngleCommand(ShooterConstants.hoodMinRotation);
  }

  public Command prepareHoodForShootCommand(
      Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
    return runEnd(
        () -> {
          ShooterSetpoint shooterSetpoint =
              FuelTrajectoryCalculator.calcualteShooterSetpoint(
                  robotPoseSupplier.get(), chassisSpeedsSupplier.get());

          hoodIO.setAngle(shooterSetpoint.hoodAngle());
          Logger.recordOutput("Shooter/Hood/SetpointAngle", shooterSetpoint.hoodAngle());
        },
        () -> {
          hoodIO.setAngle(ShooterConstants.hoodMinRotation);
          Logger.recordOutput("Shooter/Hood/SetpointAngle", ShooterConstants.hoodMinRotation);
        });
  }

  public Command shootCommand(
      Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
    return runEnd(
        () -> {
          ShooterSetpoint shooterSetpoint =
              FuelTrajectoryCalculator.calcualteShooterSetpoint(
                  robotPoseSupplier.get(), chassisSpeedsSupplier.get());
          AngularVelocity flywheelVelocity =
              RadiansPerSecond.of(
                  shooterSetpoint.linearFlywheelSpeed().in(MetersPerSecond)
                      / ShooterConstants.flywheelRadius.in(Meters));

          shooterIO.setAngularVelocity(flywheelVelocity);
          Logger.recordOutput("Shooter/Flywheel/SetpointVelocity", flywheelVelocity);

          hoodIO.setAngle(shooterSetpoint.hoodAngle());
          Logger.recordOutput("Shooter/Hood/SetpointAngle", shooterSetpoint.hoodAngle());
        },
        () -> {
          shooterIO.setAngularVelocity(RPM.of(0));
          Logger.recordOutput("Shooter/Flywheel/SetpointVelocity", RotationsPerSecond.of(0));

          hoodIO.setAngle(ShooterConstants.hoodMinRotation);
          Logger.recordOutput("Shooter/Hood/SetpointAngle", ShooterConstants.hoodMinRotation);
        });
  }

  public Angle getHoodAngle() {
    return hoodInputs.angle;
  }

  public AngularVelocity getFlywheelVelocity() {
    return shooterInputs.velocity;
  }
}
