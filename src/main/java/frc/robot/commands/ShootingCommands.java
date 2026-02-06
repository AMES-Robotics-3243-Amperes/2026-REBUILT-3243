package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.setpoints.FieldConstants;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.Container;
import frc.robot.util.FuelTrajectoryCalculator;
import frc.robot.util.FuelTrajectoryCalculator.FuelShot;
import frc.robot.util.FuelTrajectoryCalculator.FuelShotSetpoints;
import frc.robot.util.PointOfInterestManager;
import java.util.function.Supplier;

public class ShootingCommands {
  public static Command shootHubWithIndependentLinearDriveCommand(
      Supplier<Translation2d> driveLinearStrategy,
      SwerveSubsystem drivetrain,
      ShooterSubsystem shooter) {
    Container<FuelShotSetpoints> shotData =
        new Container<FuelShotSetpoints>(
            new FuelShotSetpoints(
                new FuelShot(
                    MetersPerSecond.of(0), ShooterConstants.hoodMinRotation, Seconds.of(0)),
                Rotation2d.kZero));

    return Commands.parallel(
        Commands.run(
            () ->
                shotData.inner =
                    FuelTrajectoryCalculator.getFuelShot(
                        PointOfInterestManager.flipTranslationConditionally(
                            FieldConstants.hubPosition),
                        drivetrain.getPose(),
                        drivetrain.getChassisSpeeds())),
        drivetrain.driveSetpiontGeneratorCommand(
            driveLinearStrategy,
            drivetrain.rotateAtAngleFeedForward(() -> shotData.inner.fuelGroundSpeedRotation())),
        shooter.shootAtSetpointCommand(() -> shotData.inner.shooterSetpoint()));
  }
}
