// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Time;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.List;

public class VisionConstants {
  public enum CameraType {
    LimelightTwo,
    LimelightFour
  }

  public record CameraConfiguration(
      String name, Transform3d robotToCamera, double stdDevFactor, CameraType type) {}

  // Map from camera names to their configuration
  public static final List<CameraConfiguration> cameras =
      List.of(
          new CameraConfiguration(
              "limelight-four",
              new Transform3d(
                  -Units.inchesToMeters(1.327779),
                  Units.inchesToMeters(0),
                  Units.inchesToMeters(18.427671 + 1.8),
                  new Rotation3d(0, Units.degreesToRadians(180 - 65 - 90), 0)),
              2,
              CameraType.LimelightFour),
          new CameraConfiguration(
              "limelight-two",
              new Transform3d(
                  -Units.inchesToMeters(11.694),
                  -Units.inchesToMeters(9.937452),
                  Units.inchesToMeters(9.948125 + 1.8),
                  new Rotation3d(
                      0,
                      -Units.degreesToRadians(180 - 145.891948),
                      -Units.degreesToRadians(7.5 + 180))),
              4,
              CameraType.LimelightTwo));

  // Basic filtering thresholds
  public static final Time maxTimestampError = Milliseconds.of(2);
  public static final double maxAmbiguity = 0.2;
  public static final double maxZError = 0.5;

  public static final Vector<N3> calculateStdDev(
      PoseObservation observation, CameraConfiguration config) {
    if (observation.type() == PoseObservationType.MEGATAG_1
        && observation.cameraReportedStdDevs().present()) {
      return VecBuilder.fill(
          observation.cameraReportedStdDevs().x(),
          observation.cameraReportedStdDevs().y(),
          observation.cameraReportedStdDevs().yawRadians());
    } else if (observation.type() == PoseObservationType.MEGATAG_2
        && observation.cameraReportedStdDevs().present()) {
      return VecBuilder.fill(
          observation.cameraReportedStdDevs().x(),
          observation.cameraReportedStdDevs().y(),
          Double.POSITIVE_INFINITY);
    }

    double linearStdDev = Double.MAX_VALUE;
    double angularStdDev = Double.MAX_VALUE;

    if (observation.type() == PoseObservationType.MEGATAG_2) {
      double factor =
          Math.pow(observation.averageTagDistanceMeters(), 1.3) / observation.tagCount();

      linearStdDev = 0.125 * factor;
      angularStdDev = Double.POSITIVE_INFINITY;
    } else {
      double factor =
          Math.pow(observation.averageTagDistanceMeters(), 2.0) / observation.tagCount();

      // even if the real angular deviation is quite small, there's no reason to not let the gyro do
      // the heavy lifting
      linearStdDev = 0.35 * factor;
      angularStdDev = 1 * factor;
    }

    linearStdDev *= config.stdDevFactor();
    angularStdDev *= config.stdDevFactor();

    return VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev);
  }

  // Throttling the limelight while disabled helps with thermals
  public static int limelightFourThrottle = 300;
  public static Time limelightFourThrottleDebounce = Milliseconds.of(100);

  // AprilTag layout
  public static final AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
}
