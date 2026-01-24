package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.swerve.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.FuelTrajectoryCalculator;
import frc.robot.util.FuelTrajectoryCalculator.FuelShotData;
import java.util.function.Supplier;

public class ShootingCommands {
  public static Command shootWithIndependentLinearDriveCommand(
      Supplier<Translation2d> driveLinearStrategy,
      SwerveSubsystem drivetrain,
      ShooterSubsystem shooter) {
    ProfiledPIDController pidController = SwerveConstants.rotationPidRadians.buildProfiled();
    pidController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.runEnd(
        () -> {
          FuelShotData shotData =
              FuelTrajectoryCalculator.calcualteShooterSetpoint(
                  drivetrain.getPose(), drivetrain.getChassisSpeeds());

          Translation2d driveSpeeds = driveLinearStrategy.get();
          double angularOutput =
              pidController.calculate(
                  drivetrain.getRotation().getRadians(),
                  shotData.drivetrainSetpoint().getRadians());

          drivetrain.driveSetpointGenerator(
              new ChassisSpeeds(driveSpeeds.getX(), driveSpeeds.getY(), angularOutput));
          shooter.shootIntoHub(shotData.shooterSetpoint());
        },
        shooter::reset,
        drivetrain,
        shooter);
  }
}
