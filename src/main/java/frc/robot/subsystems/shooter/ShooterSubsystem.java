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
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
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
    Logger.recordOutput("Shooter/Hood/SetpointAngle", clampedAngle);

    return clampedAngle;
  }

  private void runFlywheelWithHood(AngularVelocity velocity, Angle targetHoodAngle) {
    setHoodAngle(targetHoodAngle);

    flywheelIO.setAngularVelocity(velocity);
    Logger.recordOutput("Shooter/Flywheel/SetpointVelocity", velocity);
  }

  private void reset() {
    flywheelIO.coast();
    Logger.recordOutput("Shooter/Flywheel/SetpointVelocity", RadiansPerSecond.of(0));

    hoodIO.setAngle(ShooterConstants.hoodMinRotation);
    Logger.recordOutput("Shooter/Hood/SetpointAngle", ShooterConstants.hoodMinRotation);
  }

  public Command shootCommand(Supplier<AngularVelocity> flywheelSpeed, Supplier<Angle> hoodAngle) {
    return runEnd(() -> runFlywheelWithHood(flywheelSpeed.get(), hoodAngle.get()), this::reset);
  }

  public Command shootCommand(AngularVelocity flywheelSpeed, Angle hoodAngle) {
    return shootCommand(() -> flywheelSpeed, () -> hoodAngle);
  }

  @AutoLogOutput(key = "Shooter/Hood/Angle")
  public Angle getHoodAngle() {
    return hoodInputs.angle;
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
}
