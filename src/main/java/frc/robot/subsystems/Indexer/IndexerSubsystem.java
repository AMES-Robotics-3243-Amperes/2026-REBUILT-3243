// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
    Logger.processInputs("Shooter/Flywheel", kickerInputs);

    spindexerIO.updateInputs(spindexerInputs);
    Logger.processInputs("Shooter/Indexer", spindexerInputs);
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
}
