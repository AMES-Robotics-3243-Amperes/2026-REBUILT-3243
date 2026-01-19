package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.VisionConstants.CameraConfiguration;
import java.util.function.Supplier;

public class VisionIOLimelightFour extends VisionIOLimelight {
  private final DoublePublisher imuModePublisher;
  private final DoublePublisher throttlePublisher;

  // 100ms picked arbitrarily. Used to avoid any potential issues involving rapid changing of
  // settings on the limelight
  private final Debouncer enabledDebouncer =
      new Debouncer(
          VisionConstants.limelightFourThrottleDebounce.in(Seconds), DebounceType.kFalling);
  private final Debouncer disabledDebouncer =
      new Debouncer(VisionConstants.limelightFourThrottleDebounce.in(Seconds));

  public VisionIOLimelightFour(
      CameraConfiguration configuration, Supplier<Rotation2d> rotationSupplier) {
    super(configuration, rotationSupplier);

    imuModePublisher = table.getDoubleTopic("imumode_set").publish();
    throttlePublisher = table.getDoubleTopic("throttle_set").publish();
  }

  @Override
  public void update() {
    if (enabledDebouncer.calculate(DriverStation.isEnabled())) {
      throttlePublisher.accept(0); // no throttle

      // imuModePublisher.accept(2); // use limelight 4 builtin imu
    } else if (disabledDebouncer.calculate(DriverStation.isDisabled())) {
      throttlePublisher.accept(VisionConstants.limelightFourThrottle);
      imuModePublisher.accept(1); // use external rotation estimate (gyro + megatag1 estimate)
    }
  }
}
