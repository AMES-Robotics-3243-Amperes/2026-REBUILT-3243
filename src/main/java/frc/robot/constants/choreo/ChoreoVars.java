// spotless:off
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
  public static final Distance F_AllianceLenX = Units.Meters.of(4.021836);
  public static final Distance F_KeepOutRadius = Units.Meters.of(12.7143372);
  public static final Distance F_LenX = Units.Meters.of(16.540988);
  public static final Distance F_LenY = Units.Meters.of(8.069326);
  public static final Distance F_NeutralLenX = Units.Meters.of(6.082792);
  public static final Distance F_NeutralX = Units.Meters.of(5.229098);
  public static final Distance F_TrenchLenY = Units.Meters.of(1.27889);
  public static final LinearVelocity M_BumpSpeed = Units.MetersPerSecond.of(2.5);
  public static final LinearVelocity M_MaxCollectVelocity = Units.MetersPerSecond.of(2);
  public static final Distance R_BumperLength = Units.Meters.of(0.88265);
  public static final Distance R_BumperTolerance = Units.Meters.of(0.1016);
  public static final Distance R_DeployedHopperX = Units.Meters.of(0.635);
  public static final double R_DriveReduction = 5.79;
  public static final AngularVelocity R_MaxDriveRPM = Units.RadiansPerSecond.of(544.5427266);
  public static final Torque R_MaxDriveTorque = Units.NewtonMeters.of(0.75);
  public static final MomentOfInertia R_RobotMOI = Units.KilogramSquareMeters.of(7);
  public static final Mass R_RobotMass = Units.Kilograms.of(58.0598234);
  public static final Distance R_TrackLength = Units.Meters.of(0.5461);
  public static final double R_WheelCOF = 1.2;
  public static final Distance R_WheelRadius = Units.Meters.of(0.0492252);
  public static final Distance S_RushOutRadius = Units.Meters.of(1.2);

  public static final class Poses {
    public static final Pose2d KeepOutBottom =
        new Pose2d(4.625467, 13.984986, Rotation2d.fromRadians(0));
    public static final Pose2d KeepOutTop =
        new Pose2d(4.625467, -5.91566, Rotation2d.fromRadians(0));
    public static final Pose2d SecondCenterCollectEnd =
        new Pose2d(6.14, 4.87, Rotation2d.fromRadians(-2));
    public static final Pose2d Shoot =
        new Pose2d(2.7916443, 7.207139, Rotation2d.fromRadians(-1.0490899));
  }
}
// spotless:on
