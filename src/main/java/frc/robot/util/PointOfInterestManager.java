package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.setpoints.FieldConstants;

public class PointOfInterestManager {
  public enum FlipType {
    REFLECT_FOR_OTHER_ALLIANCE,
    REFLECT_ACROSS_LENGTH,
    REFLECT_ACROSS_WIDTH
  }

  public static double flipX(double x) {
    return FieldConstants.fieldLengthMeters - x;
  }

  public static double flipY(double y) {
    return FieldConstants.fieldWidthMeters - y;
  }

  public static Translation2d flipTranslation(Translation2d translation, FlipType flipType) {
    double newX =
        flipType != FlipType.REFLECT_ACROSS_LENGTH ? flipX(translation.getX()) : translation.getX();
    double newY =
        flipType != FlipType.REFLECT_ACROSS_WIDTH ? flipY(translation.getY()) : translation.getY();
    return new Translation2d(newX, newY);
  }

  public static Translation2d flipTranslationConditionally(Translation2d translation) {
    boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    return isBlue ? translation : flipTranslation(translation, FlipType.REFLECT_FOR_OTHER_ALLIANCE);
  }

  public static Translation3d flipTranslation(Translation3d translation, FlipType flipType) {
    Translation2d new2d = flipTranslation(translation.toTranslation2d(), flipType);
    return new Translation3d(new2d.getX(), new2d.getY(), translation.getZ());
  }

  public static Translation3d flipTranslationConditionally(Translation3d translation) {
    boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    return isBlue ? translation : flipTranslation(translation, FlipType.REFLECT_FOR_OTHER_ALLIANCE);
  }

  public static Pose2d flipPose(Pose2d pose, FlipType flipType) {
    Translation2d newTranslation = flipTranslation(pose.getTranslation(), flipType);
    Rotation2d newRotation =
        switch (flipType) {
          case REFLECT_FOR_OTHER_ALLIANCE -> pose.getRotation().plus(Rotation2d.k180deg);
          case REFLECT_ACROSS_WIDTH -> Rotation2d.k180deg.minus(pose.getRotation());
          case REFLECT_ACROSS_LENGTH -> pose.getRotation().unaryMinus();
        };

    return new Pose2d(newTranslation, newRotation);
  }

  public static Pose2d flipPoseConditionally(Pose2d pose) {
    boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    return isBlue ? pose : flipPose(pose, FlipType.REFLECT_FOR_OTHER_ALLIANCE);
  }

  public static Rectangle2d flipRectangle(Rectangle2d rectangle, FlipType flipType) {
    return new Rectangle2d(
        flipPose(rectangle.getCenter(), flipType), rectangle.getXWidth(), rectangle.getYWidth());
  }

  public static Rectangle2d flipRectangleConditionally(Rectangle2d rectangle) {
    boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    return isBlue ? rectangle : flipRectangle(rectangle, FlipType.REFLECT_FOR_OTHER_ALLIANCE);
  }
}
