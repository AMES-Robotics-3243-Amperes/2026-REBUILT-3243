// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.util.ControlConstantsBuilder;

/** Add your docs here. */
public class ShooterConstants {
  // flywheel
  public static final int flywheelLeaderId = 10;
  public static final int flywheelFollowerId = 11;
  public static final InvertedValue shooterFlywheelInverted = InvertedValue.Clockwise_Positive;

  public static final AngularVelocity shootVelocity = RPM.of(3500);

  public static final Distance flywheelRadius = Inches.of(2);
  public static final double flywheelGearReduction = 4.0 / 3.0;

  public static final Current flywheelSupplyCurrentLimit = Amps.of(70);

  public static final double fuelToFlywheelLinearSpeedRatio = 0.5;

  public static final ControlConstantsBuilder flywheelControl =
      ControlConstantsBuilder.fromRadiansAndSeconds().pid(0.85, 0, 0).sva(10.6, 0, 0);

  // hood
  public static final int hoodId = 5;

  public static final Angle hoodAbsoluteEncoderZeroedRotation = Degrees.of(20);
  public static final Angle hoodMinRotation = Degrees.of(20);
  public static final Angle hoodMaxRotation = Degrees.of(45);

  public static final double encoderToHoodReduction = 320.0 / 24.0;
  public static final double motorToEncoderHoodReduction = 15.0 / 1.0;

  public static final Angle hoodTolerance = Degrees.of(0.05);
  public static final ControlConstantsBuilder hoodControl =
      ControlConstantsBuilder.fromRadiansAndSeconds().pid(10, 0.02, 0.05);

  // fuel trajectory calculation
  public static final Distance extraPointHorizontalOffset = Inches.of(4);
  public static final Distance extraPointVerticalOffset = Inches.of(5);

  public static final Transform3d robotToShooter =
      new Transform3d(Units.inchesToMeters(-8), 0, Units.inchesToMeters(20), new Rotation3d());

  public static final int secantMethodIterations = 3;

  // only used for simulation
  public static final MomentOfInertia hoodMomentOfIntertia = KilogramSquareMeters.of(0.5);
}
