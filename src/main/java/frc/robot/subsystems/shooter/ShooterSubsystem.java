// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.GeneralPurposeCharacterization;
import frc.robot.constants.ShooterConstants;
import frc.robot.util.FuelTrajectoryCalculator;
import frc.robot.util.FuelTrajectoryCalculator.ShootTarget;
import java.util.Arrays;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
  private final FlywheelIOAndName[] flywheelIO;
  private final FlywheelIOInputsAutoLogged[] flywheelInputs;

  private final HoodIO hoodIO;
  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();

  private AngularVelocity flywheelSetpoint = RadiansPerSecond.of(0);

  public record FlywheelIOAndName(String name, FlywheelIO io) {}

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(HoodIO hoodIO, FlywheelIOAndName... flywheelIO) {
    assert flywheelIO.length > 0;
    this.flywheelIO = flywheelIO;
    this.hoodIO = hoodIO;

    this.flywheelInputs = new FlywheelIOInputsAutoLogged[flywheelIO.length];
    for (int i = 0; i < flywheelInputs.length; i++) {
      flywheelInputs[i] = new FlywheelIOInputsAutoLogged();
    }
  }

  public ShooterSubsystem(HoodIO hoodIO, FlywheelIO flywheelIO) {
    this(hoodIO, new FlywheelIOAndName[] {new FlywheelIOAndName("Main", flywheelIO)});
  }

  @Override
  public void periodic() {
    for (int i = 0; i < flywheelIO.length; i++) {
      flywheelIO[i].io().updateInputs(flywheelInputs[i]);
      Logger.processInputs("Shooter/Flywheel/" + flywheelIO[i].name(), flywheelInputs[i]);
    }

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

  private void setFlyweelVelocity(AngularVelocity velocity) {
    Arrays.stream(flywheelIO).forEach(io -> io.io().setAngularVelocity(velocity));
    flywheelSetpoint = velocity;
  }

  private void runFlywheelOpenLoop(double openLoop) {
    Arrays.stream(flywheelIO).forEach(io -> io.io().runOpenLoop(openLoop));
  }

  private void coastFlywheel() {
    Arrays.stream(flywheelIO).forEach(io -> io.io().coast());
    flywheelSetpoint = RadiansPerSecond.of(0);
  }

  public void runFlywheelWithHood(AngularVelocity velocity, Angle targetHoodAngle) {
    setHoodAngle(targetHoodAngle);
    setFlyweelVelocity(velocity);
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
    double averageDegPerSec =
        Arrays.stream(flywheelInputs)
            .mapToDouble(inputs -> inputs.velocity.in(DegreesPerSecond))
            .average()
            .orElse(0.0);
    return DegreesPerSecond.of(averageDegPerSec);
  }

  public Angle getAverageFlywheelPosition() {
    double averageDegrees =
        Arrays.stream(flywheelInputs)
            .mapToDouble(inputs -> inputs.position.in(Degrees))
            .average()
            .orElse(0.0);
    return Degrees.of(averageDegrees);
  }

  public Voltage getAverageFlywheelVoltage() {
    double averageVolts =
        Arrays.stream(flywheelInputs)
            .mapToDouble(inputs -> inputs.appliedVoltage.in(Volts))
            .average()
            .orElse(0.0);
    return Volts.of(averageVolts);
  }

  @AutoLogOutput(key = "Shooter/Flywheel/TargetFuelVelocity")
  public LinearVelocity getSetpointFuelVelocity() {
    return MetersPerSecond.of(
        flywheelSetpoint.in(RadiansPerSecond)
            * ShooterConstants.flywheelRadius.in(Meters)
            * ShooterConstants.fuelToFlywheelLinearSpeedRatio);
  }

  public boolean flywheelSpunUp() {
    return Arrays.stream(flywheelInputs)
            .allMatch(
                inputs ->
                    inputs.velocity.isNear(
                        flywheelSetpoint, ShooterConstants.flywheelIndexTolerance))
        && !flywheelSetpoint.isEquivalent(RPM.of(0));
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
                  setFlyweelVelocity(velocity);
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
          runFlywheelOpenLoop(voltage.in(Volts));
        },
        () -> getAverageFlywheelPosition(),
        () -> getAverageFlywheelVelocity(),
        () -> getAverageFlywheelVoltage(),
        this);
  }

  public Command torqueCurrentKsCharacterization(Current initialGuess) {
    return GeneralPurposeCharacterization.torqueCurrentKsCharacterization(
        initialGuess,
        () -> getAverageFlywheelVelocity(),
        current -> runFlywheelOpenLoop(current.in(Amps)),
        this::coastFlywheel,
        "Flywheel",
        this);
  }
}
