// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.util.ControlConstantsBuilder;

/** Add your docs here. */
public class ShooterConstants {

  public static final int flywheelLeftId = 3;
  public static final int flywheelRightId = 4;

  public static final int hoodId = 5;

  public static final Current flywheelSupplyCurrentLimit = Amps.of(70);

  public static final Distance flywheelRadius = Inches.of(2);
  public static final double flywheelGearReduction = 4.0 / 3.0;

  public static final Angle hoodMinRotation = Degrees.of(28);
  public static final Angle hoodMaxRotation = Degrees.of(45);

  public static final double hoodGearReduction = 475.0 / 6.0;
  public static final double hoodMaxOutput = 1;

  public static final ControlConstantsBuilder<AngleUnit, VoltageUnit> flywheelControl =
      ControlConstantsBuilder.fromUnits(Radians, Volts, Seconds).pid(0.1, 0, 0).sva(0, 0, 0);

  public static final ControlConstantsBuilder<AngleUnit, VoltageUnit> hoodControl =
      ControlConstantsBuilder.fromUnits(Radians, Volts, Seconds).pid(0.01, 0, 0).sva(0, 0, 0);

  // fuel trajectory calculation
  public static final Distance extraPointHorizontalOffset = Inches.of(10);
  public static final Distance extraPointVerticalOffset = Inches.of(10);

  public static final Transform3d robotToShooter = new Transform3d(0, 0, 0, new Rotation3d());

  public static final int extraLookaheadIterations = 2;

  // only used for simulation
  public static final MomentOfInertia hoodMomentOfIntertia = KilogramSquareMeters.of(0.5);
}
