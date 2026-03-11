package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.swerve.SwerveConstants;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.FuelTrajectoryCalculator;
import frc.robot.util.FuelTrajectoryCalculator.ShootTarget;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class ShootCommands {
  private ShootTarget target;
  private List<Command> commands;

  private ShootCommands(ShootTarget target) {
    this.target = target;
    this.commands = new ArrayList<>();
  }

  public static ShootCommands newBuilder(ShootTarget target) {
    return new ShootCommands(target);
  }

  public ShootCommands rotateDrivetrain(
      SwerveSubsystem drivetrain, Supplier<Translation2d> linearStrategy) {
    commands.add(
        drivetrain.driveSetpiontGeneratorCommand(
            linearStrategy, drivetrain.rotateForShoot(target)));
    return this;
  }

  public ShootCommands rotateDrivetrain(SwerveSubsystem drivetrain) {
    return rotateDrivetrain(drivetrain, () -> Translation2d.kZero);
  }

  public ShootCommands rotateDrivetrainWithTeleopLinear(
      SwerveSubsystem drivetrain, CommandXboxController controller) {
    return rotateDrivetrain(
        drivetrain,
        drivetrain.joystickDriveLinear(SwerveConstants.linearTeleopSpeedWhileShooting, controller));
  }

  public ShootCommands runShooter(ShooterSubsystem shooter) {
    commands.add(
        shooter.shootFuelAtSpeedCommand(
            () -> FuelTrajectoryCalculator.getShot(target).shooterSetpoint().linearFlywheelSpeed(),
            () -> FuelTrajectoryCalculator.getShot(target).shooterSetpoint().hoodAngle()));
    return this;
  }

  /**
   * Included as a static method since it doesn't require a target and should thus have the ability
   * to be made independently of a builder.
   */
  public static Command automaticallyAgitateCommand(IntakeSubsystem intake) {
    return Commands.sequence(
        Commands.waitTime(IntakeConstants.timeBeforeAgitating),
        intake.raisePivotCommand(),
        intake.agitateCommand().withTimeout(IntakeConstants.timeSpentRunningRollers));
  }

  public ShootCommands automaticallyAgitate(IntakeSubsystem intake) {
    commands.add(automaticallyAgitateCommand(intake));
    return this;
  }

  public ShootCommands indexImmediately(IndexerSubsystem indexer, ShooterSubsystem shooter) {
    commands.add(indexer.indexCommand(shooter));
    return this;
  }

  public ShootCommands indexAfterDelay(IndexerSubsystem indexer, ShooterSubsystem shooter) {
    commands.add(
        Commands.sequence(
            indexer
                .spinUpKickerWithFlywheelCommand(shooter)
                .withDeadline(Commands.waitTime(IndexerConstants.idleTimeBeforeIndexing)),
            indexer.indexCommand(shooter)));
    return this;
  }

  /**
   * Included as a static method since it doesn't require a target and should thus have the ability
   * to be made independently of a builder.
   */
  public static Command indexWhenReadyCommand(
      IndexerSubsystem indexer, ShooterSubsystem shooter, SwerveSubsystem drivetrain) {
    return Commands.sequence(
        Commands.waitUntil(
            new Trigger(drivetrain::atRotationSetpoint).and(shooter.flywheelSpunUp())),
        indexer
            .spinUpKickerWithFlywheelCommand(shooter)
            .withDeadline(Commands.waitTime(IndexerConstants.idleTimeBeforeIndexing)),
        indexer.indexCommand(shooter));
  }

  public ShootCommands indexWhenReady(
      IndexerSubsystem indexer, ShooterSubsystem shooter, SwerveSubsystem drivetrain) {
    commands.add(indexWhenReadyCommand(indexer, shooter, drivetrain));
    return this;
  }

  public Command build() {
    return Commands.parallel(commands.toArray(Command[]::new));
  }
}
