// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class ShooterConstants {

  public static final int flywheelLeaderId = 3;
  public static final int flywheelFollowerId = 4;

  public static final int hoodId = 5;

  public static final double flywheelGearRatio = 1;

  public static final double hoodGearRatio = 1 / 5;
  public static final double hoodMaxOutput = 1;

  public static final double flywheelKp = 0.0;
  public static final double flywheelKi = 0;
  public static final double flywheelKd = 0.0;
  public static final double flywheelKs = 0.0;
  public static final double flywheelKv = 0.0;
  public static final double flywheelKa = 0.0;

  public static final double hoodKp = 0.0;
  public static final double hoodKi = 0;
  public static final double hoodKd = 0.0;
  public static final double hoodKs = 0.0;
  public static final double hoodKv = 0.0;
  public static final double hoodKa = 0.0;

  public static final Velocity<VoltageUnit> sysIdRampRate = Volts.per(Second).of(0);
  public static final Voltage sysIdStepVoltage = Volts.of(0);
  public static final Time sysIdTimeout = Seconds.of(0);

  public static final Angle testHoodAngle = Degrees.of(25);
  public static final AngularVelocity testShooterSpeed = RPM.of(4500);

  public static final Angle hoodTolerance = Degrees.of(1.0);
  public static final AngularVelocity shooterTolerance = RPM.of(100);
}
