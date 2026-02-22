// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Value;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.IndexerConstants;

public class KickerIOSim implements KickerIO {
  public AngularVelocity velocity = RadiansPerSecond.of(0);

  public KickerIOSim() {}

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    inputs.position = Rotations.of(0);
    inputs.velocity = velocity;
    inputs.appliedVoltage =
        Volts.of(
            velocity.in(RadiansPerSecond)
                * IndexerConstants.kickerControl.kV.in(Value.per(RadiansPerSecond)));
  }

  @Override
  public void runOpenLoop(double output) {
    velocity =
        RadiansPerSecond.of(
            output / IndexerConstants.kickerControl.kV.in(Value.per(RadiansPerSecond)));
  }

  @Override
  public void setAngularVelocity(AngularVelocity velocity) {
    this.velocity = velocity;
  }

  @Override
  public void coast() {
    this.velocity = RadiansPerSecond.of(0);
  }
}
