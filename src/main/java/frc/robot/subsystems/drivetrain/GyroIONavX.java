package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;

public class GyroIONavX implements GyroIO {
  private final AHRS imu = new AHRS(NavXComType.kMXP_SPI);

  public GyroIONavX() {
    imu.reset();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.yawPosition = Rotation2d.fromDegrees(-imu.getYaw());
    inputs.yawVelocity = DegreesPerSecond.of(-imu.getRate());

    inputs.odometryYawTimestamps = new double[] {RobotController.getFPGATime() / 1e6};
    inputs.odometryYawPositions = new Rotation2d[] {Rotation2d.fromDegrees(-imu.getYaw())};
  }

  @Override
  public void resetYaw() {
    imu.reset();
  }
}
