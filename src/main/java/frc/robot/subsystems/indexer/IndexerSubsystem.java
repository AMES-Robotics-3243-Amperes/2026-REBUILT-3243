// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.GeneralPurposeCharacterization;
import frc.robot.constants.IndexerConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class IndexerSubsystem extends SubsystemBase {
  private final KickerIO kickerIO;
  private final KickerIOInputsAutoLogged kickerInputs = new KickerIOInputsAutoLogged();

  private final SpindexerIO spindexerIO;
  private final SpindexerIOInputsAutoLogged spindexerInputs = new SpindexerIOInputsAutoLogged();

  /** Creates a new IndexerSubsystem. */
  public IndexerSubsystem(KickerIO kickerIO, SpindexerIO spindexerIO) {
    this.kickerIO = kickerIO;
    this.spindexerIO = spindexerIO;
  }

  @Override
  public void periodic() {
    kickerIO.updateInputs(kickerInputs);
    Logger.processInputs("Indexer/Kicker", kickerInputs);

    spindexerIO.updateInputs(spindexerInputs);
    Logger.processInputs("Indexer/Spindexer", spindexerInputs);
  }

  public AngularVelocity getSpindexerVelocity() {
    return spindexerInputs.velocity;
  }

  public AngularVelocity getKickerVelocity() {
    return kickerInputs.velocity;
  }

  public void setKickerVelocity(AngularVelocity velocity) {
    kickerIO.setAngularVelocity(velocity);
    Logger.recordOutput("Indexer/Kicker/SetpointVelocity", velocity);
  }

  public void setSpindexerVelocity(AngularVelocity velocity) {
    spindexerIO.setAngularVelocity(velocity);
    Logger.recordOutput("Indexer/Spindexer/SetpointVelocity", velocity);
  }

  public void coastKicker() {
    kickerIO.coast();
    Logger.recordOutput("Indexer/Kicker/SetpointVelocity", RadiansPerSecond.of(0));
  }

  public void coastSpindexer() {
    spindexerIO.coast();
    Logger.recordOutput("Indexer/Spindexer/SetpointVelocity", RadiansPerSecond.of(0));
  }

  private AngularVelocity getKickerVelocityFromLinearVelocity(LinearVelocity velocity) {
    double targetRadPerSec =
        velocity.in(MetersPerSecond) / IndexerConstants.kickerWheelRadius.in(Meters);
    return RadiansPerSecond.of(
        Math.min(targetRadPerSec, IndexerConstants.maxKickerSpeed.in(RadiansPerSecond)));
  }

  public Command indexCommand(ShooterSubsystem shooter) {
    return runAtSpeedCommand(
        () -> getKickerVelocityFromLinearVelocity(shooter.getSetpointFuelVelocity()),
        IndexerConstants.spindexerIndexingSpeed);
  }

  public Command spinUpKickerWithFlywheelCommand(ShooterSubsystem shooter) {
    return spinUpKickerCommand(
        () -> getKickerVelocityFromLinearVelocity(shooter.getSetpointFuelVelocity()));
  }

  public Command spinUpKickerCommand(Supplier<AngularVelocity> velocity) {
    return runEnd(
            () -> {
              setKickerVelocity(velocity.get());
              coastSpindexer();
            },
            () -> {
              coastKicker();
              coastSpindexer();
            })
        .until(
            () ->
                kickerInputs.velocity.isNear(
                    velocity.get(), IndexerConstants.kickerSpinUpTolerance));
  }

  public Command runKickerAtSpeedCommand(Supplier<AngularVelocity> kickerVelocity) {
    return runEnd(
        () -> {
          setKickerVelocity(kickerVelocity.get());
          coastSpindexer();
        },
        () -> {
          coastKicker();
          coastSpindexer();
        });
  }

  public Command runAtSpeedCommand(
      Supplier<AngularVelocity> kickerVelocity, AngularVelocity spindexerVelocity) {
    return runEnd(
        () -> {
          setKickerVelocity(kickerVelocity.get());
          setSpindexerVelocity(spindexerVelocity);
        },
        () -> {
          coastKicker();
          coastSpindexer();
        });
  }

  public Command kickerSysIdCommand(Trigger advanceRoutine) {
    return GeneralPurposeCharacterization.sysIdCommand(
        advanceRoutine,
        "Indexer/SysId/Kicker",
        voltage -> kickerIO.runOpenLoop(voltage.in(Volts)),
        () -> kickerInputs.position,
        () -> kickerInputs.velocity,
        () -> kickerInputs.appliedVoltage,
        this);
  }

  public Command spindexerSysIdCommand(Trigger advanceRoutine) {
    return GeneralPurposeCharacterization.sysIdCommand(
        advanceRoutine,
        "Indexer/SysId/Spindexer",
        voltage -> spindexerIO.runOpenLoop(voltage.in(Volts)),
        () -> spindexerInputs.position,
        () -> spindexerInputs.velocity,
        () -> spindexerInputs.appliedVoltage,
        this);
  }
}
