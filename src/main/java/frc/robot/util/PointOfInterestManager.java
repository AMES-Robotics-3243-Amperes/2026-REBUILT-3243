package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class PointOfInterestManager {
  public static final double fieldLengthMeters = Units.inchesToMeters(651.22);
  public static final double fieldWidthMeters = Units.inchesToMeters(317.69);

  public static Translation3d flipTranslation(Translation3d translation) {
    // TODO: is it performance heavy to call getAlliance? should I cache this?
    boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    double newX = isBlue ? translation.getX() : fieldLengthMeters - translation.getX();
    double newY = isBlue ? translation.getX() : fieldWidthMeters - translation.getY();

    return new Translation3d(newX, newY, translation.getZ());
  }

  public static Pose3d flipPose(Pose3d pose) {
    boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    double newX = isBlue ? pose.getX() : fieldLengthMeters - pose.getX();
    double newY = isBlue ? pose.getX() : fieldWidthMeters - pose.getY();

    Rotation3d newRotation =
        isBlue
            ? pose.getRotation()
            : new Rotation3d(
                pose.getRotation().getMeasureX(),
                pose.getRotation().getMeasureY(),
                pose.getRotation().getMeasureZ().plus(Degrees.of(180)));

    return new Pose3d(newX, newY, pose.getZ(), newRotation);
  }
}
