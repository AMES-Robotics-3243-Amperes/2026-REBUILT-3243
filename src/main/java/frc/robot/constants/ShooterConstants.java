// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.ControlConstantsBuilder;

/** Add your docs here. */
public class ShooterConstants {
  public static final int flywheelLeaderId = 10;
  public static final int flywheelFollowerId = 11;
  public static final InvertedValue shooterFlywheelInverted = InvertedValue.Clockwise_Positive;

  public static final int hoodId = 10;

  public static final Current flywheelSupplyCurrentLimit = Amps.of(70);

  public static final Distance flywheelRadius = Inches.of(2);
  public static final double flywheelGearReduction = 4.0 / 3.0;

  // the ratio of speed transferred into a fuel to linear speed of the flywheel
  public static final double fuelToFlywheelLinearSpeedRatio = 0.5;

  public static final Angle hoodPhysicalBottomOutRotation = Degrees.of(25);
  public static final Angle hoodMinRotation = Degrees.of(15);
  public static final Angle hoodMaxRotation = Degrees.of(50);

  public static final Angle hoodToleranceWhenShooting = Degrees.of(5);

  public static final double encoderToHoodReduction = 475.0 / 6.0;

  public static final ControlConstantsBuilder flywheelControl =
      ControlConstantsBuilder.fromRadiansAndSeconds()
          .pid(0.1, 0, 0)
          .sva(0.13976, 0.023548, 0.0013083);

  public static final ControlConstantsBuilder hoodControl =
      ControlConstantsBuilder.fromRadiansAndSeconds().pid(1, 0, 0.05).sva(0, 0, 0);

  public static final Velocity<VoltageUnit> sysIdRampRate = Volts.per(Second).of(0.8);
  public static final Voltage sysIdStepVoltage = Volts.of(4);
  public static final Time sysIdTimeout = Seconds.of(8);

  // fuel trajectory calculation
  public static final Distance extraPointHorizontalOffset = Inches.of(4);
  public static final Distance extraPointVerticalOffset = Inches.of(5);

  public static final Transform3d robotToShooter =
      new Transform3d(Units.inchesToMeters(-8), 0, Units.inchesToMeters(20), new Rotation3d());

  public static final int secantMethodIterations = 3;

  // only used for simulation
  public static final MomentOfInertia hoodMomentOfIntertia = KilogramSquareMeters.of(0.5);
}
