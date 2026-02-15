// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.IndexerConstants;
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

  // eli was here
  @Override
  public void periodic() {
    kickerIO.updateInputs(kickerInputs);
    Logger.processInputs("Indexer/Kicker", kickerInputs);

    spindexerIO.updateInputs(spindexerInputs);
    Logger.processInputs("Indexer/Spindexer", spindexerInputs);
  }

  public Command runAtSpeedCommand(
      AngularVelocity kickerVelocity, AngularVelocity spindexerVelocity) {
    return runEnd(
        () -> {
          kickerIO.setAngularVelocity(kickerVelocity);
          spindexerIO.setAngularVelocity(spindexerVelocity);
        },
        () -> {
          kickerIO.coast();
          spindexerIO.coast();
        });
  }

  public Command kickerSysIdCommand(Trigger advanceRoutine) {
    SysIdRoutine routine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                IndexerConstants.sysIdRampRate,
                IndexerConstants.sysIdStepVoltage,
                IndexerConstants.sysIdTimeout,
                (state) -> {
                  Logger.recordOutput("Intake/SysId/State", state.toString());

                  Logger.recordOutput(
                      "Intake/SysId/PositionRadians", kickerInputs.position.in(Radians));
                  Logger.recordOutput(
                      "Intake/SysId/VelocityRadiansPerSecond",
                      kickerInputs.velocity.in(RadiansPerSecond));
                  Logger.recordOutput(
                      "Intake/SysId/AppliedVolts", kickerInputs.appliedVoltage.in(Volts));
                }),
            new SysIdRoutine.Mechanism(
                (voltage) -> kickerIO.runOpenLoop(voltage.in(Volts)), null, this));

    Supplier<Command> waitCommand =
        () ->
            Commands.parallel(
                Commands.waitUntil(advanceRoutine.negate()),
                Commands.waitSeconds(0.8),
                runOnce(() -> kickerIO.runOpenLoop(0)));

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
