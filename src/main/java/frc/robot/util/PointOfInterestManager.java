package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldConstants;
import java.util.function.Supplier;

public class PointOfInterestManager {
  public enum FlipType {
    REFLECT_FOR_OTHER_ALLIANCE,
    REFLECT_X,
    REFLECT_Y
  }

  public static double flipX(double x) {
    return FieldConstants.fieldLength.in(Meters) - x;
  }

  public static double flipY(double y) {
    return FieldConstants.fieldWidth.in(Meters) - y;
  }

  public static Translation2d flipTranslation(Translation2d translation, FlipType flipType) {
    double newX = flipType != FlipType.REFLECT_Y ? flipX(translation.getX()) : translation.getX();
    double newY = flipType != FlipType.REFLECT_X ? flipY(translation.getY()) : translation.getY();
    return new Translation2d(newX, newY);
  }

  public static Translation2d flipTranslationConditionally(
      Translation2d translation, FlipType flipType) {
    boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    return isBlue ? translation : flipTranslation(translation, flipType);
  }

  /** Exists for conciseness of constructing commands. */
  public static Supplier<Translation2d> translationFlippedConditionally(Translation2d translation) {
    return () -> flipTranslationConditionally(translation, FlipType.REFLECT_FOR_OTHER_ALLIANCE);
  }

  public static Translation3d flipTranslation(Translation3d translation, FlipType flipType) {
    Translation2d new2d = flipTranslation(translation.toTranslation2d(), flipType);
    return new Translation3d(new2d.getX(), new2d.getY(), translation.getZ());
  }

  public static Translation3d flipTranslationConditionally(
      Translation3d translation, FlipType flipType) {
    boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    return isBlue ? translation : flipTranslation(translation, flipType);
  }

  /** Exists for conciseness of constructing commands. */
  public static Supplier<Translation3d> translationFlippedConditionally(Translation3d translation) {
    return () -> flipTranslationConditionally(translation, FlipType.REFLECT_FOR_OTHER_ALLIANCE);
  }

  public static Pose2d flipPose(Pose2d pose, FlipType flipType) {
    Translation2d newTranslation = flipTranslation(pose.getTranslation(), flipType);
    Rotation2d newRotation =
        switch (flipType) {
          case REFLECT_FOR_OTHER_ALLIANCE -> pose.getRotation().plus(Rotation2d.k180deg);
          case REFLECT_X -> Rotation2d.k180deg.minus(pose.getRotation());
          case REFLECT_Y -> pose.getRotation().unaryMinus();
        };

    return new Pose2d(newTranslation, newRotation);
  }

  public static Pose2d flipPoseConditionally(Pose2d pose, FlipType flipType) {
    boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    return isBlue ? pose : flipPose(pose, flipType);
  }

  public static Rectangle2d flipRectangle(Rectangle2d rectangle, FlipType flipType) {
    return new Rectangle2d(
        flipPose(rectangle.getCenter(), flipType), rectangle.getXWidth(), rectangle.getYWidth());
  }

  public static Rectangle2d flipRectangleConditionally(Rectangle2d rectangle, FlipType flipType) {
    boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    return isBlue ? rectangle : flipRectangle(rectangle, flipType);
  }
}
