package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.setpoints.FieldConstants;

public class PointOfInterestManager {
  public static Translation3d flipTranslation(Translation3d translation) {
    // TODO: is it performance heavy to call getAlliance? should I cache this?
    boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    double newX =
        isBlue ? translation.getX() : FieldConstants.fieldLengthMeters - translation.getX();
    double newY =
        isBlue ? translation.getX() : FieldConstants.fieldWidthMeters - translation.getY();

    return new Translation3d(newX, newY, translation.getZ());
  }

  public static Pose2d flipPose(Pose2d pose) {
    boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    double newX = isBlue ? pose.getX() : FieldConstants.fieldLengthMeters - pose.getX();
    double newY = isBlue ? pose.getX() : FieldConstants.fieldWidthMeters - pose.getY();

    Rotation2d newRotation =
        isBlue ? pose.getRotation() : pose.getRotation().plus(Rotation2d.k180deg);

    return new Pose2d(newX, newY, newRotation);
  }

  public static Rectangle2d flipRectangle(Rectangle2d rectangle) {
    return new Rectangle2d(
        flipPose(rectangle.getCenter()), rectangle.getXWidth(), rectangle.getYWidth());
  }
}
