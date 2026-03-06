package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.swerve.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.FuelTrajectoryCalculator;
import java.util.function.Supplier;

public class ShootCommand {
  // TODO: just put this enum in the calculator
  public enum ShootTarget {
    HUB,
    ALLIANCE,
    NEUTRAL
  }

  public static Command shootCommand(
      ShootTarget target,
      ShooterSubsystem shooter,
      SwerveSubsystem drivetrain,
      Supplier<Translation2d> linearStrategy) {
    Supplier<Rotation2d> angleSupplier = () -> null;
    Command shootCommand = null;

    // TODO: use a switch
    if (target == ShootTarget.HUB) {
      angleSupplier = () -> FuelTrajectoryCalculator.getHubShot().fuelGroundSpeedRotation();
      shootCommand = shooter.shootInHubCommand();
    } else if (target == ShootTarget.ALLIANCE) {
      angleSupplier = () -> FuelTrajectoryCalculator.getAllianceShot().fuelGroundSpeedRotation();
      shootCommand = shooter.shootInAllianceZoneCommand();
    } else if (target == ShootTarget.NEUTRAL) {
      angleSupplier = () -> FuelTrajectoryCalculator.getNeutralShot().fuelGroundSpeedRotation();
      shootCommand = shooter.shootInNeutralZoneCommand();
    }

    return Commands.parallel(
        shootCommand,
        drivetrain.driveSetpiontGeneratorCommand(
            linearStrategy, drivetrain.rotateAtAngle(angleSupplier)));
  }

  public static Command shootCommandTeleopDrive(
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
}
