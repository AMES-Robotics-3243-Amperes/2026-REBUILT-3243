// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface KickerIO {
  @AutoLog
  class KickerIOInputs {
    Angle position = Radians.of(0);
    AngularVelocity velocity = RadiansPerSecond.of(0);
    Voltage appliedVoltage = Volts.of(0);
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(KickerIOInputs inputs) {}

  /** Run the motor at the specified open loop value. */
  default void runOpenLoop(double output) {}

  /** Run the motor at the specified velocity. */
  default void setAngularVelocity(AngularVelocity velocity) {}

  /** Set the motor to coast. */
  default void coast() {}
}
