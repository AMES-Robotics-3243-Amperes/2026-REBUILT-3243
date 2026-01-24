package frc.robot.constants.setpoints;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class BluePointsOfInterest {
  // public static Translation3d hubPosition =
  //     new Translation3d(
  //         Units.inchesToMeters(182.11),
  //         Units.inchesToMeters(158.84),
  //         Units.inchesToMeters(72)); // the center of the top of the funnel
  public static Translation3d hubPosition =
      new Translation3d(
          Units.inchesToMeters(400),
          Units.inchesToMeters(0),
          Units.inchesToMeters(-9.6 - 20)); // the center of the top of the funnel
}
