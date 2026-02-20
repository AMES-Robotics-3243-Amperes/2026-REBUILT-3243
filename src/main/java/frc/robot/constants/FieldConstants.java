package frc.robot.constants;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
  public static final double fieldLengthMeters = Units.inchesToMeters(651.22);
  public static final double fieldWidthMeters = Units.inchesToMeters(317.69);

  // the center of the top of the funnel
  public static Translation3d hubPosition =
      new Translation3d(
          Units.inchesToMeters(182.11), Units.inchesToMeters(158.84), Units.inchesToMeters(72));

  public static Rectangle2d rightTrench =
      new Rectangle2d(
          new Translation2d(Units.inchesToMeters(158.34), 0),
          new Translation2d(Units.inchesToMeters(205.87), Units.inchesToMeters(50.59)));
  public static Rectangle2d leftTrench =
      new Rectangle2d(
          new Translation2d(Units.inchesToMeters(158.34), fieldWidthMeters),
          new Translation2d(
              Units.inchesToMeters(205.87), fieldWidthMeters - Units.inchesToMeters(50.59)));
}
