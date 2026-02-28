package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.VisionConstants.CameraConfiguration;
import java.util.function.Supplier;

public class VisionIOLimelightFour extends VisionIOLimelight {
  private final DoublePublisher imuModePublisher;
  private final DoublePublisher imuAssistPublisher;

  public VisionIOLimelightFour(
      CameraConfiguration configuration,
      Supplier<Rotation2d> rotationSupplier,
      Supplier<AngularVelocity> rotationRateSupplier) {
    super(configuration, rotationSupplier, rotationRateSupplier);

    imuModePublisher = table.getDoubleTopic("imumode_set").publish();
    imuAssistPublisher = table.getDoubleTopic("imuassistalpha_set").publish();
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    super.updateInputs(inputs);

    imuAssistPublisher.accept(VisionConstants.limelightFourImuAssist);
    if (DriverStation.isEnabled()) {
      imuModePublisher.accept(4); // use limelight 4 builtin imu
    } else {
      imuModePublisher.accept(1); // use external rotation estimate (gyro + megatag1 estimate)
    }
  }
}
