package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.swerve.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.FuelTrajectoryCalculator;
import frc.robot.util.FuelTrajectoryCalculator.ShootTarget;
import java.util.function.Supplier;

public class ShootCommands {
  public static Command shootCommand(
      ShootTarget target,
      ShooterSubsystem shooter,
      SwerveSubsystem drivetrain,
      Supplier<Translation2d> linearStrategy) {
    return Commands.parallel(
        shooter.shootFuelAtSpeedCommand(
            () -> FuelTrajectoryCalculator.getShot(target).shooterSetpoint().linearFlywheelSpeed(),
            () -> FuelTrajectoryCalculator.getShot(target).shooterSetpoint().hoodAngle()),
        drivetrain.driveSetpiontGeneratorCommand(
            linearStrategy,
            drivetrain.rotateAtAngle(
                () -> FuelTrajectoryCalculator.getShot(target).fuelGroundSpeedRotation())));
  }

  public static Command shootTeleopDriveCommand(
      ShootTarget type,
      ShooterSubsystem shooter,
      SwerveSubsystem drivetrain,
      CommandXboxController controller) {
    return shootCommand(
        type,
        shooter,
        drivetrain,
        drivetrain.joystickDriveLinear(SwerveConstants.linearTeleopSpeedWhileShooting, controller));
  }

  public static Command indexWhenReadyCommand(
      IndexerSubsystem indexer, SwerveSubsystem drivetrain, ShooterSubsystem shooter) {
    return Commands.sequence(
        Commands.waitUntil(() -> drivetrain.atRotationSetpoint() && shooter.flywheelSpunUp()),
        indexer.spinUpForShootCommand(shooter),
        Commands.waitSeconds(0.2), // TODO: constants
        indexer.indexCommand(shooter));
  }
}
