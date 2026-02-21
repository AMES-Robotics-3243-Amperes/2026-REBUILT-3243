// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.ControlConstantsBuilder;

/** Add your docs here. */
public class IndexerConstants {
  public static final int spindexerId = 0;

  public static final double spindexerReduction = 1;

  public static final Distance spindexerRadius = Inches.of(1);

  public static final ControlConstantsBuilder spindexerControl =
      ControlConstantsBuilder.fromRadiansAndSeconds().pid(0.000002, 0, 0).sva(0, 0.0, 0.0);

  public static final int kickerId = 1;

  public static final double kickerReduction = 1.25;

  public static final Distance kickerRadius = Inches.of(1);

  public static final ControlConstantsBuilder kickerControl =
      ControlConstantsBuilder.fromRadiansAndSeconds().pid(0.000002, 0, 0).sva(0, 0.0, 0.0);

  public static final Velocity<VoltageUnit> sysIdRampRate = Volts.per(Second).of(0.8);
  public static final Voltage sysIdStepVoltage = Volts.of(4);
  public static final Time sysIdTimeout = Seconds.of(8);
}
