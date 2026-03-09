package frc.robot.constants.choreo;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;

/**
 * Generated file containing variables defined in Choreo. DO NOT MODIFY THIS FILE YOURSELF; instead,
 * change these values in the Choreo GUI.
 */
public final class ChoreoVars {
  public static final Distance F_AllianceLenX = Units.Meters.of(4.022);
  public static final Distance F_KeepOutRadius = Units.Meters.of(12.714);
  public static final Distance F_LenX = Units.Meters.of(16.541);
  public static final Distance F_LenY = Units.Meters.of(8.069);
  public static final Distance F_NeutralLenX = Units.Meters.of(6.083);
  public static final Distance F_NeutralX = Units.Meters.of(5.229);
  public static final Distance F_TrenchLenY = Units.Meters.of(1.279);
  public static final LinearAcceleration M_GlobalMaxAccel = Units.MetersPerSecondPerSecond.of(3);
  public static final LinearVelocity M_MaxCollectVelocity = Units.MetersPerSecond.of(1.8);
  public static final Distance R_BumperLength = Units.Meters.of(0.883);
  public static final Distance R_BumperTolerance = Units.Meters.of(0.114);
  public static final Distance R_DeployedHopperX = Units.Meters.of(0.635);
  public static final double R_DriveReduction = 5.79;
  public static final AngularVelocity R_MaxDriveRPM = Units.RadiansPerSecond.of(544.543);
  public static final Torque R_MaxDriveTorque = Units.NewtonMeters.of(0.8);
  public static final MomentOfInertia R_RobotMOI = Units.KilogramSquareMeters.of(7);
  public static final Mass R_RobotMass = Units.Kilograms.of(52.163);
  public static final Distance R_TrackLength = Units.Meters.of(0.546);
  public static final double R_WheelCOF = 1.5;
  public static final Distance R_WheelRadius = Units.Meters.of(0.05);
  public static final Distance S_RushOutRadius = Units.Meters.of(1.2);

  public static final class Poses {
    public static final Pose2d KeepOutBottom = new Pose2d(4.625, 13.985, Rotation2d.kZero);
    public static final Pose2d KeepOutTop = new Pose2d(4.625, -5.916, Rotation2d.kZero);
    public static final Pose2d SecondCenterCollectEnd =
        new Pose2d(6.14, 4.87, Rotation2d.fromRadians(-2));
    public static final Pose2d Shoot = new Pose2d(2.38, 6.75, Rotation2d.fromRadians(-0.88));

    private Poses() {}
  }

  private ChoreoVars() {}
}
