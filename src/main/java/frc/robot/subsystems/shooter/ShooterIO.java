// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ShooterIO {

  @AutoLog
  class ShooterIOInputs {
    public Angle shooterPosition = Rotations.of(0);
    public AngularVelocity shooterVelocity = RPM.of(0);
    public Voltage shooterAppliedVoltage = Volts.of(0);
    public Current shooterStatorCurrent = Amps.of(0);
    public Current shooterSupplyCurrent = Amps.of(0);

    public Angle hoodAngle = Degrees.of(0);
    public AngularVelocity hoodAngularVelocity = DegreesPerSecond.of(0);
    public Voltage hoodAppliedVoltage = Volts.of(0);
    public Current hoodCurrent = Amps.of(0);
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(ShooterIOInputs inputs) {}

  /** Run the motors at the specified open loop value. */
  default void runShooterOpenLoop(double output) {}

  /** Run the motors at the specified velocity. */
  default void setShooterAngularVelocity(AngularVelocity velocity) {}

  /** Run the motors at the specified open loop value. */
  default void runHoodOpenLoop(double output) {}

  default void setHoodAngle(Angle angle) {}
}
