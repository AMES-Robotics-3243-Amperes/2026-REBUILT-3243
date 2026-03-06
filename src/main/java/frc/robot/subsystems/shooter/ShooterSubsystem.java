// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SysIdCommand;
import frc.robot.constants.ShooterConstants;
import frc.robot.util.Container;
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

  private boolean recoverIfSlow = false;

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

    if (flywheelSpunUp()) recoverIfSlow = true;
    flywheelIO.enableRecoverControl(
        recoverIfSlow
            && !flywheelInputs.velocity.isNear(
                flywheelSetpoint, ShooterConstants.flywheelRecoverControlTolerance));
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

    if (!velocity.isNear(flywheelSetpoint, RPM.of(1))) recoverIfSlow = false;

    flywheelIO.setAngularVelocity(velocity);
    flywheelSetpoint = velocity;
  }

  public void coastFlywheel() {
    flywheelIO.coast();
    flywheelSetpoint = RadiansPerSecond.of(0);
    recoverIfSlow = false;
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
    return flywheelInputs.velocity.isNear(flywheelSetpoint, ShooterConstants.flywheelIndexTolerance)
        && !flywheelSetpoint.isEquivalent(RPM.of(0));
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

  //
  // Characterization
  //
  public Command flywheelSysIdCommand(Trigger advanceRoutine) {
    return SysIdCommand.sysIdCommand(
        advanceRoutine,
        "Shooter/SysId/Flywheel",
        voltage -> flywheelIO.runOpenLoop(voltage.in(Volts)),
        () -> flywheelInputs.position,
        () -> flywheelInputs.velocity,
        () -> flywheelInputs.appliedVoltage,
        this);
  }

  public Command torqueCurrentKsCharacterization(Current initialGuess) {
    Container<Current> upperBound = new Container<Current>(initialGuess.times(2));
    Container<Current> lowerBound = new Container<Current>(Amps.of(0));

    return Commands.sequence(
            run(() -> coastFlywheel()).withTimeout(3),
            run(() ->
                    flywheelIO.runOpenLoop(upperBound.inner.plus(lowerBound.inner).div(2).in(Amps)))
                .withTimeout(4),
            runOnce(
                () -> {
                  boolean isRunning = !flywheelInputs.velocity.isNear(RPM.of(0), RPM.of(0.5));

                  if (isRunning) {
                    upperBound.inner = upperBound.inner.plus(lowerBound.inner).div(2);
                  } else {
                    lowerBound.inner = upperBound.inner.plus(lowerBound.inner).div(2);
                  }
                }))
        .repeatedly()
        .until(() -> upperBound.inner.minus(lowerBound.inner).lt(Amps.of(0.05)))
        .finallyDo(
            () -> {
              System.out.println(upperBound.inner.plus(lowerBound.inner).div(2).toLongString());
              coastFlywheel();
            });
  }
}
