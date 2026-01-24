package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.FuelTrajectoryCalculator;
import frc.robot.util.FuelTrajectoryCalculator.FuelShotData;
import java.util.function.Supplier;

public class ShootingCommands {
  public static Command shootHubWithIndependentLinearDriveCommand(
      Supplier<Translation2d> driveLinearStrategy,
      SwerveSubsystem drivetrain,
      ShooterSubsystem shooter) {
    class ShotDataContainer {
      public FuelShotData data =
          FuelTrajectoryCalculator.getFuelShot(drivetrain.getPose(), drivetrain.getChassisSpeeds());
    }

    ShotDataContainer dataContainer = new ShotDataContainer();

    return Commands.parallel(
        Commands.run(
            () ->
                dataContainer.data =
                    FuelTrajectoryCalculator.getFuelShot(
                        drivetrain.getPose(), drivetrain.getChassisSpeeds())),
        drivetrain.driveSetpiontGeneratorCommand(
            driveLinearStrategy,
            drivetrain.rotateAtAngle(() -> dataContainer.data.drivetrainSetpoint())),
        shooter.shootAtSetpointCommand(() -> dataContainer.data.shooterSetpoint()));
  }
}
