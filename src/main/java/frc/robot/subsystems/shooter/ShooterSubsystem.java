// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.ShooterConstants;
import frc.robot.util.FuelTrajectoryCalculator.FuelShot;
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
  public void reset() {
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

  public Command shootAtHoodAngleCommand(AngularVelocity velocity, Angle angle) {
    return runEnd(
        () -> {
          Angle clampedAngle = setHoodAngle(angle);
          runFlywheelWithHood(velocity, clampedAngle);
        },
        this::reset);
  }

  public Command angleHoodForShootCommand(Supplier<FuelShot> setpointSupplier) {
    return runEnd(
        () -> {
          FuelShot setpoint = setpointSupplier.get();
          setHoodAngle(setpoint.hoodAngle());
        },
        this::reset);
  }

  public Command shootAtSetpointCommand(Supplier<FuelShot> setpointSupplier) {
    return runEnd(
        () -> {
          FuelShot setpoint = setpointSupplier.get();
          Angle clampedAngle = setHoodAngle(setpoint.hoodAngle());
          runFlywheelAtFuelSpeedWithHood(setpoint.linearFlywheelSpeed(), clampedAngle);
        },
        this::reset);
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

  //
  // SysId
  //

  public Command sysIdCommand(Trigger advanceRoutine) {
    SysIdRoutine routine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                ShooterConstants.sysIdRampRate,
                ShooterConstants.sysIdStepVoltage,
                ShooterConstants.sysIdTimeout,
                (state) -> {
                  Logger.recordOutput("Shooter/SysId/State", state.toString());

                  Logger.recordOutput(
                      "Shooter/SysId/FlywheelPositionRadians", flywheelInputs.position.in(Radians));
                  Logger.recordOutput(
                      "Shooter/SysId/FlywheelVelocityRadiansPerSecond",
                      flywheelInputs.velocity.in(RadiansPerSecond));
                  Logger.recordOutput(
                      "Shooter/SysId/FlywheelAppliedVolts",
                      flywheelInputs.appliedVoltage.in(Volts));
                }),
            new SysIdRoutine.Mechanism(
                (voltage) -> flywheelIO.runOpenLoop(voltage.in(Volts)), null, this));

    Supplier<Command> waitCommand =
        () ->
            Commands.parallel(
                Commands.waitUntil(advanceRoutine.negate()),
                Commands.waitSeconds(4),
                runOnce(flywheelIO::coast));

    return Commands.sequence(
        // TODO: the drivetrain sysid routine looks the exact same. remove code repetition
        waitCommand.get(),
        routine.dynamic(SysIdRoutine.Direction.kForward).until(advanceRoutine),
        waitCommand.get(),
        routine.dynamic(SysIdRoutine.Direction.kReverse).until(advanceRoutine),
        waitCommand.get(),
        routine.quasistatic(SysIdRoutine.Direction.kForward).until(advanceRoutine),
        waitCommand.get(),
        routine.quasistatic(SysIdRoutine.Direction.kReverse).until(advanceRoutine));
  }
}
