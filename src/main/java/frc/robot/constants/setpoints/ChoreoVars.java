package frc.robot.constants.setpoints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;

/**
 * Generated file containing variables defined in Choreo. DO NOT MODIFY THIS FILE YOURSELF; instead,
 * change these values in the Choreo GUI.
 */
public final class ChoreoVars {
  public static final Distance PickupX = Units.Meters.of(7.8);
  public static final Distance TrenchEnterX = Units.Meters.of(3.3);
  public static final Distance TrenchExitX = Units.Meters.of(5.8);

  public static final class Poses {
    public static final Pose2d LeftShoot = new Pose2d(2.5, 5.5, Rotation2d.fromRadians(-0.6));
    public static final Pose2d RightShoot = new Pose2d(2.5, 2.4, Rotation2d.fromRadians(0.6));

    private Poses() {}
  }

  private ChoreoVars() {}
}
