package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.Container;
import frc.robot.util.FuelTrajectoryCalculator;
import frc.robot.util.FuelTrajectoryCalculator.FuelShotSetpoints;
import frc.robot.util.FuelTrajectoryCalculator.ShooterSetpoint;
import java.util.function.Supplier;

public class ShootingCommands {
  public static Command shootHubWithIndependentLinearDriveCommand(
      Supplier<Translation2d> driveLinearStrategy,
      SwerveSubsystem drivetrain,
      ShooterSubsystem shooter) {
    Container<FuelShotSetpoints> shotData =
        new Container<FuelShotSetpoints>(
            new FuelShotSetpoints(
                new ShooterSetpoint(MetersPerSecond.of(0), ShooterConstants.hoodMinRotation),
                Rotation2d.kZero));

    return Commands.parallel(
        Commands.run(
            () ->
                shotData.inner =
                    FuelTrajectoryCalculator.getFuelShot(
                        drivetrain.getPose(), drivetrain.getChassisSpeeds())),
        drivetrain.driveSetpiontGeneratorCommand(
            driveLinearStrategy,
            drivetrain.rotateAtAngle(() -> shotData.inner.fuelGroundSpeedRotation())),
        shooter.shootAtSetpointCommand(() -> shotData.inner.shooterSetpoint()));
  }
}
