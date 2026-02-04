// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.ControlConstantsBuilder;

/** Add your docs here. */
public class ShooterConstants {

  public static final int flywheelId = 10;

  public static final double flywheelGearRatio = 1;

  public static final ControlConstantsBuilder flywheelControl =
      ControlConstantsBuilder.fromRadiansAndSeconds().pid(0.000001, 0, 0).sva(0, 0.0, 0.0);

  public static final int indexerId = 11;

  public static final double indexerGearRatio = 0.25;

  public static final ControlConstantsBuilder indexerControl =
      ControlConstantsBuilder.fromRadiansAndSeconds().pid(0.000001, 0, 0).sva(0, 0.0, 0.0);

  public static final Velocity<VoltageUnit> sysIdRampRate = Volts.per(Second).of(0.3);
  public static final Voltage sysIdStepVoltage = Volts.of(1.5);
  public static final Time sysIdTimeout = Seconds.of(8);
}
