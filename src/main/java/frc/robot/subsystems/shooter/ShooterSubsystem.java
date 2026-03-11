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
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.GeneralPurposeCharacterization;
import frc.robot.constants.ShooterConstants;
import frc.robot.util.FuelTrajectoryCalculator;
import frc.robot.util.FuelTrajectoryCalculator.ShootTarget;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
  private final FlywheelIO flywheelIOLeft;
  private final FlywheelIOInputsAutoLogged flywheelLeftInputs = new FlywheelIOInputsAutoLogged();

  private final FlywheelIO flywheelIORight;
  private final FlywheelIOInputsAutoLogged flywheelRightInputs = new FlywheelIOInputsAutoLogged();

  private final HoodIO hoodIO;
  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();

  private AngularVelocity flywheelSetpoint = RadiansPerSecond.of(0);

  private boolean recoverIfSlow = false;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(FlywheelIO flywheelIOLeft, FlywheelIO flywheelIORight, HoodIO hoodIO) {
    this.flywheelIOLeft = flywheelIOLeft;
    this.flywheelIORight = flywheelIORight;
    this.hoodIO = hoodIO;
  }

  @Override
  public void periodic() {
    flywheelIOLeft.updateInputs(flywheelLeftInputs);
    Logger.processInputs("Shooter/Flywheel/Left", flywheelLeftInputs);

    flywheelIORight.updateInputs(flywheelRightInputs);
    Logger.processInputs("Shooter/Flywheel/Right", flywheelRightInputs);

    hoodIO.updateInputs(hoodInputs);
    Logger.processInputs("Shooter/Hood", hoodInputs);

    if (flywheelSpunUpBoolean()) recoverIfSlow = true;

    flywheelIOLeft.enableRecoverControl(
        recoverIfSlow
            && !flywheelLeftInputs.velocity.isNear(
                flywheelSetpoint, ShooterConstants.flywheelRecoverControlTolerance));
    flywheelIORight.enableRecoverControl(
        recoverIfSlow
            && !flywheelRightInputs.velocity.isNear(
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

    flywheelIOLeft.setAngularVelocity(velocity);
    flywheelIORight.setAngularVelocity(velocity);
    flywheelSetpoint = velocity;
  }

  public void coastFlywheel() {
    flywheelIOLeft.coast();
    flywheelIORight.coast();
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

  @AutoLogOutput(key = "Shooter/Flywheel/TargetFlywheelVelocity")
  public AngularVelocity getSetpointFlywheelVelocity() {
    return flywheelSetpoint;
  }

  @AutoLogOutput(key = "Shooter/Flywheel/AverageMeasuredVelocity")
  public AngularVelocity getAverageFlywheelVelocity() {
    return flywheelLeftInputs.velocity.plus(flywheelRightInputs.velocity).div(2);
  }

  @AutoLogOutput(key = "Shooter/Flywheel/TargetFuelVelocity")
  public LinearVelocity getSetpointFuelVelocity() {
    return MetersPerSecond.of(
        flywheelSetpoint.in(RadiansPerSecond)
            * ShooterConstants.flywheelRadius.in(Meters)
            * ShooterConstants.fuelToFlywheelLinearSpeedRatio);
  }

  private boolean flywheelSpunUpBoolean() {
    return flywheelLeftInputs.velocity.isNear(
            flywheelSetpoint, ShooterConstants.flywheelIndexTolerance)
        && flywheelRightInputs.velocity.isNear(
            flywheelSetpoint, ShooterConstants.flywheelIndexTolerance)
        && !flywheelSetpoint.isEquivalent(RPM.of(0));
  }

  public Trigger flywheelSpunUp() {
    return new Trigger(this::flywheelSpunUpBoolean).debounce(1e-1, DebounceType.kRising);
  }

  public Command spinUpFlywheelCommand(ShootTarget target) {
    SlewRateLimiter rateLimiter =
        new SlewRateLimiter(ShooterConstants.spinUpMaxAcceleration.in(RadiansPerSecondPerSecond));
    return runOnce(() -> rateLimiter.reset(getAverageFlywheelVelocity().in(RPM)))
        .andThen(
            runEnd(
                () -> {
                  double linearSpeedMps =
                      FuelTrajectoryCalculator.getShot(target)
                          .shooterSetpoint()
                          .linearFlywheelSpeed()
                          .in(MetersPerSecond);
                  AngularVelocity velocity =
                      RadiansPerSecond.of(
                          rateLimiter.calculate(
                              linearSpeedMps
                                  / (ShooterConstants.flywheelRadius.in(Meters)
                                      * ShooterConstants.fuelToFlywheelLinearSpeedRatio)));
                  flywheelIOLeft.setAngularVelocity(velocity);
                  flywheelIORight.setAngularVelocity(velocity);
                },
                this::coastFlywheel));
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
    return GeneralPurposeCharacterization.sysIdCommand(
        advanceRoutine,
        "Shooter/SysId/Flywheel",
        voltage -> {
          flywheelIOLeft.runOpenLoop(voltage.in(Volts));
          flywheelIORight.runOpenLoop(voltage.in(Volts));
        },
        () -> flywheelLeftInputs.position.plus(flywheelRightInputs.position).div(2),
        () -> getAverageFlywheelVelocity(),
        () -> flywheelLeftInputs.appliedVoltage.plus(flywheelRightInputs.appliedVoltage).div(2),
        this);
  }

  public Command torqueCurrentKsCharacterization(Current initialGuess) {
    return Commands.sequence(
        GeneralPurposeCharacterization.torqueCurrentKsCharacterization(
            initialGuess,
            () -> flywheelLeftInputs.velocity,
            current -> flywheelIOLeft.runOpenLoop(current.in(Amps)),
            this::coastFlywheel,
            "left flywheel",
            this),
        GeneralPurposeCharacterization.torqueCurrentKsCharacterization(
            initialGuess,
            () -> flywheelRightInputs.velocity,
            current -> flywheelIORight.runOpenLoop(current.in(Amps)),
            this::coastFlywheel,
            "right flywheel",
            this));
  }
}
