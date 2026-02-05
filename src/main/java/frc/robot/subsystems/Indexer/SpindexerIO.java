// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface SpindexerIO {
  @AutoLog
  class SpindexerIOInputs {
    Angle position = Radians.of(0);
    AngularVelocity velocity = RadiansPerSecond.of(0);
    Voltage appliedVoltage = Volts.of(0);
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(SpindexerIOInputs inputs) {}

  /** Run the motors at the specified open loop value. */
  default void runOpenLoop(double output) {}

  /** Run the motors at the specified velocity. */
  default void setAngularVelocity(AngularVelocity velocity) {}

  default void coast() {}
}
