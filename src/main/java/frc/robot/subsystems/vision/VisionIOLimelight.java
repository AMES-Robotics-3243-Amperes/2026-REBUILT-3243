// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Microseconds;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.VisionConstants.CameraConfiguration;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

/** IO implementation for real Limelight hardware. */
public class VisionIOLimelight extends VisionIO {
  private final Supplier<Rotation2d> rotationSupplier;
  private final DoubleArrayPublisher orientationPublisher;

  private final DoubleSubscriber latencySubscriber;
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;
  private final DoubleArraySubscriber megatag1Subscriber;
  private final DoubleArraySubscriber megatag2Subscriber;
  private final DoubleArraySubscriber standardDeviationsSubscriber;

  protected final NetworkTable table;

  /**
   * Creates a new VisionIOLimelight.
   *
   * @param name The configured name of the Limelight.
   * @param rotationSupplier Supplier for the current estimated rotation, used for MegaTag 2.
   */
  public VisionIOLimelight(
      CameraConfiguration configuration, Supplier<Rotation2d> rotationSupplier) {
    super(configuration);

    this.table = NetworkTableInstance.getDefault().getTable(this.configuration.name());

    this.rotationSupplier = rotationSupplier;
    orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
    megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    megatag2Subscriber =
        table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
    standardDeviationsSubscriber = table.getDoubleArrayTopic("stddevs").subscribe(new double[] {});
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Update connection status based on whether an update has been seen in the last
    // 250ms
    inputs.connected =
        ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

    // Update target observation
    inputs.latestTargetObservation =
        new TargetObservation(
            Rotation2d.fromDegrees(txSubscriber.get()), Rotation2d.fromDegrees(tySubscriber.get()));

    // Update orientation for MegaTag 2
    orientationPublisher.accept(
        new double[] {rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});
    NetworkTableInstance.getDefault()
        .flush(); // Increases network traffic but recommended by Limelight

    // Read new pose observations from NetworkTables
    Set<Integer> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new ArrayList<>();

    TimestampedDoubleArray[] standardDeviationSamples = standardDeviationsSubscriber.readQueue();

    TimestampedDoubleArray[] megaTagOneSamples = megatag1Subscriber.readQueue();
    for (int i = 0; i < megaTagOneSamples.length; i++) {
      TimestampedDoubleArray rawMegaTagOneSample = megaTagOneSamples[i];

      if (rawMegaTagOneSample.value.length == 0) continue;

      for (int j = 11; j < rawMegaTagOneSample.value.length; j += 7) {
        tagIds.add((int) rawMegaTagOneSample.value[j]);
      }

      Optional<TimestampedDoubleArray> rawStandardDeviations =
          i < standardDeviationSamples.length
              ? Optional.of(standardDeviationSamples[i])
              : Optional.empty();
      rawStandardDeviations =
          rawStandardDeviations.filter(
              deviation ->
                  Math.abs(deviation.timestamp - rawMegaTagOneSample.timestamp)
                      < VisionConstants.maxTimestampError.in(Microseconds));

      poseObservations.add(
          new PoseObservation(
              // Timestamp, based on server timestamp of publish and latency
              rawMegaTagOneSample.timestamp * 1e-6 - rawMegaTagOneSample.value[6] * 1e-3,

              // 3D pose estimate
              parsePose(rawMegaTagOneSample.value),

              // Reported standard deviations
              rawStandardDeviations
                  .<CameraReportedStandardDeviations>map(
                      deviations ->
                          CameraReportedStandardDeviations.present(
                              deviations.value[0],
                              deviations.value[1],
                              Units.degreesToRadians(deviations.value[5])))
                  .orElse(CameraReportedStandardDeviations.empty()),

              // Ambiguity, using only the first tag because ambiguity isn't applicable for
              // multitag
              rawMegaTagOneSample.value.length >= 18 ? rawMegaTagOneSample.value[17] : 0.0,

              // Tag count
              (int) rawMegaTagOneSample.value[7],

              // Average tag distance
              rawMegaTagOneSample.value[9],

              // Observation type
              PoseObservationType.MEGATAG_1));
    }

    TimestampedDoubleArray[] megaTagTwoSamples = megatag2Subscriber.readQueue();
    for (int i = 0; i < megaTagTwoSamples.length; i++) {
      TimestampedDoubleArray rawMegaTagTwoSample = megaTagTwoSamples[i];

      if (rawMegaTagTwoSample.value.length == 0) continue;

      for (int j = 11; j < rawMegaTagTwoSample.value.length; j += 7) {
        tagIds.add((int) rawMegaTagTwoSample.value[j]);
      }

      Optional<TimestampedDoubleArray> rawStandardDeviations =
          i < standardDeviationSamples.length
              ? Optional.of(standardDeviationSamples[i])
              : Optional.empty();
      rawStandardDeviations =
          rawStandardDeviations.filter(
              deviation ->
                  Math.abs(deviation.timestamp - rawMegaTagTwoSample.timestamp)
                      < VisionConstants.maxTimestampError.in(Microseconds));

      poseObservations.add(
          new PoseObservation(
              // Timestamp, based on server timestamp of publish and latency
              rawMegaTagTwoSample.timestamp * 1e-6 - rawMegaTagTwoSample.value[6] * 1e-3,

              // 3D pose estimate
              parsePose(rawMegaTagTwoSample.value),

              // Reported standard deviations
              rawStandardDeviations
                  .<CameraReportedStandardDeviations>map(
                      deviations ->
                          CameraReportedStandardDeviations.present(
                              deviations.value[6],
                              deviations.value[7],
                              Units.degreesToRadians(deviations.value[11])))
                  .orElse(CameraReportedStandardDeviations.empty()),

              // Ambiguity, zeroed because the pose is already disambiguated
              0.0,

              // Tag count
              (int) rawMegaTagTwoSample.value[7],

              // Average tag distance
              rawMegaTagTwoSample.value[9],

              // Observation type
              PoseObservationType.MEGATAG_2));
    }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }

  /** Parses the 3D pose from a Limelight botpose array. */
  private static Pose3d parsePose(double[] rawLLArray) {
    return new Pose3d(
        rawLLArray[0],
        rawLLArray[1],
        rawLLArray[2],
        new Rotation3d(
            Units.degreesToRadians(rawLLArray[3]),
            Units.degreesToRadians(rawLLArray[4]),
            Units.degreesToRadians(rawLLArray[5])));
  }
}
