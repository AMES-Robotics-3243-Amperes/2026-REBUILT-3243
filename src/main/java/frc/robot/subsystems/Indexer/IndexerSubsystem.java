// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SysIdCommand;
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

  public Command runAtSpeedCommand(
      AngularVelocity kickerVelocity, AngularVelocity spindexerVelocity) {
    return runEnd(
        () -> {
          setKickerVelocity(kickerVelocity);
          setSpindexerVelocity(spindexerVelocity);
        },
        () -> {
          coastKicker();
          coastSpindexer();
        });
  }

  public Command kickerSysIdCommand(Trigger advanceRoutine) {
    return SysIdCommand.sysIdCommand(
        advanceRoutine,
        "Indexer/SysId/Kicker",
        voltage -> kickerIO.runOpenLoop(voltage.in(Volts)),
        () -> kickerInputs.position,
        () -> kickerInputs.velocity,
        () -> kickerInputs.appliedVoltage,
        this);
  }

  public Command spindexerSysIdCommand(Trigger advanceRoutine) {
    return SysIdCommand.sysIdCommand(
        advanceRoutine,
        "Indexer/SysId/Spindexer",
        voltage -> spindexerIO.runOpenLoop(voltage.in(Volts)),
        () -> spindexerInputs.position,
        () -> spindexerInputs.velocity,
        () -> spindexerInputs.appliedVoltage,
        this);
  }
}
