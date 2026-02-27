// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.util.FuelTrajectoryCalculator;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
  private final FlywheelIO flywheelIO;
  private final FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();

  private final HoodIO hoodIO;
  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();

  private AngularVelocity flywheelSetpoint = RadiansPerSecond.of(0);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(FlywheelIO shooterIO, HoodIO hoodIO) {
    this.flywheelIO = shooterIO;
    this.hoodIO = hoodIO;
  }

  @Override
  public void periodic() {
    flywheelIO.updateInputs(flywheelInputs);
    Logger.processInputs("Shooter/Flywheel", flywheelInputs);

    hoodIO.updateInputs(hoodInputs);
    Logger.processInputs("Shooter/Hood", hoodInputs);
  }

  /* Clamps the given angle to the allowed range, sets the hood to the clamped angle, and returns the clamped angle. */
  public Angle setHoodAngle(Angle angle) {
    Angle clampedAngle =
        Degrees.of(
            MathUtil.clamp(
                angle.in(Degrees),
                ShooterConstants.hoodMinRotation.in(Degrees),
                ShooterConstants.hoodMaxRotation.in(Degrees)));

    hoodIO.setAngle(clampedAngle);
    Logger.recordOutput("Shooter/Hood/SetpointAngle", clampedAngle);

    return clampedAngle;
  }

  public void runFlywheelWithHood(AngularVelocity velocity, Angle targetHoodAngle) {
    setHoodAngle(targetHoodAngle);

    flywheelIO.setAngularVelocity(velocity);
    flywheelSetpoint = velocity;
  }

  public void coastFlywheel() {
    flywheelIO.coast();
    flywheelSetpoint = RadiansPerSecond.of(0);
  }

  public void reset() {
    coastFlywheel();

    hoodIO.setAngle(ShooterConstants.hoodMinRotation);
    Logger.recordOutput("Shooter/Hood/SetpointAngle", ShooterConstants.hoodMinRotation);
  }

  @AutoLogOutput(key = "Shooter/Hood/Angle")
  public Angle getHoodAngle() {
    return hoodInputs.angle;
  }

  @AutoLogOutput(key = "Shooter/Flywheel/SetpointVelocity")
  public AngularVelocity flywheelSetpointVelocity() {
    return flywheelSetpoint;
  }

  public boolean flywheelSpunUp() {
    return flywheelInputs.velocity.isNear(
        flywheelSetpoint, ShooterConstants.flywheelVelocityTolerance);
  }

  @AutoLogOutput(key = "Shooter/Flywheel/FlywheelVelocity")
  public AngularVelocity getFlywheelVelocity() {
    return flywheelInputs.velocity;
  }

  @AutoLogOutput(key = "Shooter/Flywheel/FuelVelocity")
  public LinearVelocity getFuelVelocity() {
    return MetersPerSecond.of(
        flywheelInputs.velocity.in(RadiansPerSecond)
            * ShooterConstants.flywheelRadius.in(Meters)
            * ShooterConstants.fuelToFlywheelLinearSpeedRatio);
  }

  public Command shootCommand(Supplier<AngularVelocity> flywheelSpeed, Supplier<Angle> hoodAngle) {
    return runEnd(() -> runFlywheelWithHood(flywheelSpeed.get(), hoodAngle.get()), this::reset);
  }

  public Command shootCommand(AngularVelocity flywheelSpeed, Angle hoodAngle) {
    return shootCommand(() -> flywheelSpeed, () -> hoodAngle);
  }

  public Command shootFuelAtSpeedCommand(
      Supplier<LinearVelocity> flywheelSpeed, Supplier<Angle> hoodAngle) {
    return shootCommand(
        () ->
            RadiansPerSecond.of(
                flywheelSpeed.get().in(MetersPerSecond)
                    / (ShooterConstants.flywheelRadius.in(Meters)
                        * ShooterConstants.fuelToFlywheelLinearSpeedRatio)),
        hoodAngle::get);
  }

  public Command shootInHubCommand() {
    return shootFuelAtSpeedCommand(
        () -> FuelTrajectoryCalculator.getHubShot().shooterSetpoint().linearFlywheelSpeed(),
        () -> FuelTrajectoryCalculator.getHubShot().shooterSetpoint().hoodAngle());
  }

  public Command shootInAllianceZoneCommand() {
    return shootFuelAtSpeedCommand(
        () -> FuelTrajectoryCalculator.getAllianceShot().shooterSetpoint().linearFlywheelSpeed(),
        () -> FuelTrajectoryCalculator.getAllianceShot().shooterSetpoint().hoodAngle());
  }

  public Command shootInNeutralZoneCommand() {
    return shootFuelAtSpeedCommand(
        () -> FuelTrajectoryCalculator.getNeutralShot().shooterSetpoint().linearFlywheelSpeed(),
        () -> FuelTrajectoryCalculator.getNeutralShot().shooterSetpoint().hoodAngle());
  }
}
