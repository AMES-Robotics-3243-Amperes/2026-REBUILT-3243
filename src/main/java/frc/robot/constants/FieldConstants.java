package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.PointOfInterestManager;
import frc.robot.util.PointOfInterestManager.FlipType;

public class FieldConstants {
  public static final Distance fieldLength = Inches.of(651.22);
  public static final Distance fieldWidth = Inches.of(317.69);

  public static final Distance barrierLeftX = Inches.of(158.34);
  public static final Distance barrierRightX = Inches.of(205.87);

  public static final Distance trenchWidth = Inches.of(50.59);

  public static final Rectangle2d allianceZone =
      new Rectangle2d(new Translation2d(), new Translation2d(barrierLeftX, fieldWidth));
  public static final Rectangle2d neutralZone =
      new Rectangle2d(
          new Translation2d(barrierRightX.in(Meters), 0),
          new Translation2d(
              PointOfInterestManager.flipX(barrierRightX.in(Meters)), fieldWidth.in(Meters)));

  public static final Rectangle2d bottomTrench =
      new Rectangle2d(
          new Translation2d(barrierLeftX.in(Meters), 0),
          new Translation2d(barrierRightX, trenchWidth));
  public static final Rectangle2d topTrench =
      PointOfInterestManager.flipRectangle(bottomTrench, FlipType.REFLECT_Y);

  // the center of the top of the funnel
  public static Translation3d hubPosition =
      new Translation3d(
          barrierLeftX.plus(barrierRightX).div(2).in(Meters),
          Units.inchesToMeters(158.84),
          Units.inchesToMeters(72));
}
