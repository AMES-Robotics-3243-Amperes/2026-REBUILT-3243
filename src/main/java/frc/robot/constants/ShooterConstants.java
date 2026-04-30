// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.ControlConstantsBuilder;

/** Add your docs here. */
public class ShooterConstants {
  public static final Translation2d bottomBlueAllianceZoneShot =
      new Translation2d(FieldConstants.barrierLeftX.div(2), FieldConstants.trenchWidth);
  public static final Translation2d bottomBlueNeutralZoneShot =
      new Translation2d(FieldConstants.fieldLength.div(2), FieldConstants.trenchWidth);

  public static final AngularVelocity flywheelIndexTolerance = RPM.of(25);

  public static final AngularAcceleration spinUpMaxAcceleration = RPM.per(Second).of(1500);

  // flywheel
  public static final int flywheelLeftId = 10;
  public static final int flywheelMiddleId = 11;
  public static final int flywheelRightId = 12;

  public static final InvertedValue shooterFlywheelInverted = InvertedValue.Clockwise_Positive;

  public static final Distance flywheelRadius = Inches.of(2);
  public static final double flywheelGearReduction = 4.0 / 3.0;

  public static final Current flywheelStatorCurrentLimit = Amps.of(80);
  public static final Current flywheelSupplyCurrentLimit = Amps.of(40);

  public static final double fuelToFlywheelLinearSpeedRatio = 0.484;

  public static final ClosedLoopOutputType flywheelClosedLoopOutput = ClosedLoopOutputType.Voltage;

  public static final ControlConstantsBuilder flywheelControl =
      ControlConstantsBuilder.fromRadiansAndSeconds()
          .sva(0.12766, 0.023994, 0.0019976)
          .pid(0.012, 0, 0);

  // hood
  public static final int hoodId = 5;

  public static final Angle hoodAbsoluteEncoderZeroedRotation = Degrees.of(23.4);
  public static final Angle hoodMinRotation = Degrees.of(17);
  public static final Angle hoodMaxRotation = Degrees.of(46);

  public static final double encoderToHoodReduction = 320.0 / 24.0;
  public static final double motorToEncoderHoodReduction = 15.0 / 1.0;

  public static final int hoodCurrentLimit = 20;

  public static final ControlConstantsBuilder hoodControl =
      ControlConstantsBuilder.fromRadiansAndSeconds().pid(10, 0.02, 0.05);

  // fuel trajectory calculation
  public static final Distance extraPointHorizontalOffset = Inches.of(3.2);
  public static final Distance extraPointVerticalOffset = Inches.of(5);

  public static final Transform3d robotToShooter =
      new Transform3d(Units.inchesToMeters(-8), 0, Units.inchesToMeters(20), new Rotation3d());

  public static final int secantMethodIterations = 3;
}
