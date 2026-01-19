// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.VisionConstants.CameraConfiguration;
import org.littletonrobotics.junction.AutoLog;

public abstract class VisionIO {
  public final CameraConfiguration configuration;

  public VisionIO(CameraConfiguration configuration) {
    this.configuration = configuration;
  }

  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public TargetObservation latestTargetObservation =
        new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
  }

  /** Represents the angle to a simple target, not used for pose estimation. */
  public record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  /**
   * Represents the standard deviations represented by a camera. The wpilib Vector class isn't used
   * for logging purposes.
   */
  public record CameraReportedStandardDeviations(
      boolean present, double x, double y, double yawRadians) {
    public static CameraReportedStandardDeviations empty() {
      return new CameraReportedStandardDeviations(
          false, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    public static CameraReportedStandardDeviations present(double x, double y, double yaw) {
      if (Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(yaw)) {
        return empty();
      }

      return new CameraReportedStandardDeviations(
          true,
          MathUtil.clamp(x, 0, Double.POSITIVE_INFINITY),
          MathUtil.clamp(y, 0, Double.MAX_VALUE),
          MathUtil.clamp(yaw, 0, Double.MAX_VALUE));
    }
  }

  /** Represents a robot pose sample used for pose estimation. */
  public record PoseObservation(
      double timestampSeconds,
      Pose3d pose,
      CameraReportedStandardDeviations cameraReportedStdDevs,
      double ambiguity,
      int tagCount,
      double averageTagDistanceMeters,
      PoseObservationType type) {}

  public enum PoseObservationType {
    MEGATAG_1,
    MEGATAG_2,
    PHOTONVISION
  }

  public void updateInputs(VisionIOInputs inputs) {}

  /** Called periodically. Implementation/purpose varies by camera implementation. */
  public void update() {}
}
