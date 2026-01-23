// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface HoodIO {
  @AutoLog
  class HoodIOInputs {
    public Angle angle = Rotations.of(0);
    public AngularVelocity angularVelocity = RPM.of(0);
    public Voltage appliedVoltage = Volts.of(0);
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(HoodIOInputs inputs) {}

  /** Resets the tracked position's value. */
  default void resetPosition(Angle angle) {}

  /** Run the hood at the specified open loop value. */
  default void runOpenLoop(double output) {}

  /** Set the hood to a specified angle. An angle of 0 would shoot fuel straight up. */
  default void setAngle(Angle angle) {}
}
