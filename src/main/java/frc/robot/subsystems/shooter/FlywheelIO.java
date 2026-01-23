// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface FlywheelIO {
  @AutoLog
  class ShooterIOInputs {
    public Angle position = Rotations.of(0);
    public AngularVelocity velocity = RotationsPerSecond.of(0);
    public Voltage appliedVoltage = Volts.of(0);
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(ShooterIOInputs inputs) {}

  /** Run the flywheel at the specified open loop value. */
  default void runOpenLoop(double output) {}

  /** Run the flywheel at the specified velocity. */
  default void setAngularVelocity(AngularVelocity velocity) {}

  /** Coasts the flywheel. */
  default void coast() {}
}
