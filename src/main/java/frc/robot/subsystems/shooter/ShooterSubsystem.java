// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubystem. */
  private final FlywheelIO flywheelIO;

  private final FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();

  private final IndexerIO indexerIO;
  private final IndexerIOInputsAutoLogged indexerInputs = new IndexerIOInputsAutoLogged();

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(FlywheelIO flywheelIO, IndexerIO indexerIO) {
    this.flywheelIO = flywheelIO;
    this.indexerIO = indexerIO;
  }

  @Override
  public void periodic() {
    flywheelIO.updateInputs(flywheelInputs);
    Logger.processInputs("Shooter/Flywheel", flywheelInputs);

    indexerIO.updateInputs(indexerInputs);
    Logger.processInputs("Shooter/Indexer", indexerInputs);
  }

  public Command runAtSpeedCommand(
      AngularVelocity flywheelVelocity, AngularVelocity indexerVelocity) {
    return runEnd(
        () -> {
          flywheelIO.setAngularVelocity(flywheelVelocity);
          indexerIO.setAngularVelocity(indexerVelocity);
        },
        () -> {
          flywheelIO.coast();
          indexerIO.coast();
        });
  }
}
