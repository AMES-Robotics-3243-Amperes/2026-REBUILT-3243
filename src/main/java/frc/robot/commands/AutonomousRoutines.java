package frc.robot.commands;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.choreo.ChoreoTraj;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.FuelTrajectoryCalculator.ShootTarget;

public class AutonomousRoutines {
  public static void populateAutoChooser(
      AutoChooser autoChooser,
      SwerveSubsystem drivetrain,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      IntakeSubsystem intake) {
    // TODO: logging
    AutoFactory autoFactory =
        new AutoFactory(
            drivetrain::getPose,
            drivetrain::setPose,
            drivetrain::followChoreoTrajecotry,
            true,
            drivetrain);

    autoChooser.addRoutine(
        "deposit side one cycle",
        () -> depotSideOneCycle(autoFactory, drivetrain, shooter, indexer, intake));
    autoChooser.addRoutine(
        "outpost side one cycle",
        () -> outpostSideOneCycle(autoFactory, drivetrain, shooter, indexer, intake));
  }

  //
  // Single cycle autos
  //

  private static AutoRoutine depotSideOneCycle(
      AutoFactory autoFactory,
      SwerveSubsystem drivetrain,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      IntakeSubsystem intake) {
    AutoRoutine routine = autoFactory.newRoutine("one cycle");

    AutoTrajectory collectFromMiddle = routine.trajectory(ChoreoTraj.Collect.name());
    AutoTrajectory returnToShoot = routine.trajectory(ChoreoTraj.ReturnToShoot.name());

    routine.active().onTrue(collectFromMiddle.cmd());

    // collect from middle while intaking
    collectFromMiddle
        .active()
        .whileTrue(intake.intakeAtSpeedCommand(IntakeConstants.rollerIntakeSpeed));
    collectFromMiddle.chain(returnToShoot);

    // while returning, intake a little longer to collect the rest of the fuel. spin up flywheel
    // when approaching shoot position
    returnToShoot
        .active()
        .onTrue(intake.intakeAtSpeedCommand(IntakeConstants.rollerIntakeSpeed).withTimeout(1.5));
    returnToShoot
        .atTimeBeforeEnd(3)
        .onTrue(
            shooter
                .spinUpFlywheelCommand()
                .until(
                    returnToShoot
                        .active()
                        .negate())); // TODO: have a constant for average spinup time?

    // finally, shoot. we also agitate with pivot and rollers at appropriate times
    returnToShoot
        .done()
        .onTrue(
            Commands.parallel(
                ShootCommands.rotateAndShoot(ShootTarget.HUB, shooter, drivetrain),
                ShootCommands.indexWhenReadyCommand(indexer, drivetrain, shooter),
                Commands.sequence(
                    Commands.waitSeconds(0.8),
                    intake.runPivotOpenLoopCommand(3).withTimeout(2), // TODO: once the spin up ramp rate is in contsants, this time can be calculated to be optimal. also test to make sure the trigger activates for paths shorter than X seconds
                    intake
                        .intakeAtSpeedCommand(IntakeConstants.rollerIntakeSpeed)
                        .withTimeout(2) // TODO: have an agitate speed
                    )));

    return routine;
  }

  private static AutoRoutine outpostSideOneCycle(
      AutoFactory autoFactory,
      SwerveSubsystem drivetrain,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      IntakeSubsystem intake) {
    // this is just the depot one cycle flipped along the field width
    AutoRoutine routine = depotSideOneCycle(autoFactory, drivetrain, shooter, indexer, intake);
    routine.active().whileTrue(drivetrain.driveFlippedAlongFieldWidth());
    return routine;
  }
}
