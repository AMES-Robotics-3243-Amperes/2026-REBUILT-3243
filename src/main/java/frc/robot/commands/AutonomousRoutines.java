package frc.robot.commands;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
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

    autoChooser.addRoutine(
        "deposit side two cycle",
        () -> depotSideTwoCycle(autoFactory, drivetrain, shooter, indexer, intake));
    autoChooser.addRoutine(
        "outpost side two cycle",
        () -> outpostSideTwoCycle(autoFactory, drivetrain, shooter, indexer, intake));
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

    AutoTrajectory collectFromMiddle = routine.trajectory(ChoreoTraj.FirstCenterCollect.name(), 0);
    AutoTrajectory returnToShoot = routine.trajectory(ChoreoTraj.FirstCenterCollect.name(), 1);

    routine.active().onTrue(collectFromMiddle.cmd());

    // collect from middle while intaking
    collectFromMiddle
        .atTime(1)
        .onTrue(intake.intakeCommand().until(collectFromMiddle.active().negate()));
    collectFromMiddle.chain(returnToShoot);

    // while returning, intake a little longer to collect the rest of the fuel. spin up flywheel
    // when approaching shoot position
    returnToShoot.active().onTrue(intake.intakeCommand().withTimeout(1));
    returnToShoot
        .atTimeBeforeEnd(
            2) // TODO: once the spin up ramp rate is in contsants, this time can be calculated to
        // be optimal. also test to make sure the trigger activates for paths shorter than X
        // seconds
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
                    intake.raisePivotCommand(),
                    intake.agitateCommand().withTimeout(2) // TODO: have an agitate speed
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

  //
  // Double Cycle
  //

  private static AutoRoutine depotSideTwoCycle(
      AutoFactory autoFactory,
      SwerveSubsystem drivetrain,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      IntakeSubsystem intake) {
    AutoRoutine routine = autoFactory.newRoutine("two cycle");

    AutoTrajectory firstCollectFromMiddle =
        routine.trajectory(ChoreoTraj.FirstCenterCollect.name(), 0);
    AutoTrajectory firstReturnToShoot = routine.trajectory(ChoreoTraj.FirstCenterCollect.name(), 1);
    AutoTrajectory secondCollectFromMiddle =
        routine.trajectory(ChoreoTraj.SecondCenterCollect.name(), 0);
    AutoTrajectory secondReturnToShoot =
        routine.trajectory(ChoreoTraj.SecondCenterCollect.name(), 1);

    routine.active().onTrue(firstCollectFromMiddle.cmd());

    // collect from middle while intaking
    firstCollectFromMiddle
        .atTime(1)
        .onTrue(intake.intakeCommand().until(firstCollectFromMiddle.active().negate()));
    firstCollectFromMiddle.chain(firstReturnToShoot);

    // while returning, intake a little longer to collect the rest of the fuel. spin up flywheel
    // when approaching shoot position
    firstReturnToShoot.active().onTrue(intake.intakeCommand().withTimeout(1));
    firstReturnToShoot
        .atTimeBeforeEnd(
            2) // TODO: once the spin up ramp rate is in contsants, this time can be calculated to
        // be optimal. also test to make sure the trigger activates for paths shorter than X
        // seconds
        .onTrue(shooter.spinUpFlywheelCommand().until(firstReturnToShoot.active().negate()));

    // finally, shoot. we also agitate with pivot and rollers at appropriate times
    firstReturnToShoot
        .done()
        .onTrue(
            Commands.parallel(
                    ShootCommands.rotateAndShoot(ShootTarget.HUB, shooter, drivetrain),
                    ShootCommands.indexWhenReadyCommand(indexer, drivetrain, shooter),
                    Commands.sequence(
                        Commands.waitSeconds(0.8),
                        intake.raisePivotCommand(),
                        intake.agitateCommand().withTimeout(2) // TODO: have an agitate speed
                        ))
                .withTimeout(4));
    firstReturnToShoot.doneDelayed(4).onTrue(secondCollectFromMiddle.cmd());

    secondCollectFromMiddle.active().whileTrue(intake.intakeCommand());
    secondCollectFromMiddle.chain(secondReturnToShoot);

    secondReturnToShoot.active().onTrue(intake.intakeCommand().withTimeout(1));
    secondReturnToShoot
        .atTimeBeforeEnd(
            1) // TODO: once the spin up ramp rate is in contsants, this time can be calculated to
        // be optimal. also test to make sure the trigger activates for paths shorter than X
        // seconds
        .onTrue(shooter.spinUpFlywheelCommand().until(secondReturnToShoot.active().negate()));

    // finally, shoot. we also agitate with pivot and rollers at appropriate times
    secondReturnToShoot
        .done()
        .onTrue(
            Commands.parallel(
                ShootCommands.rotateAndShoot(ShootTarget.HUB, shooter, drivetrain),
                ShootCommands.indexWhenReadyCommand(indexer, drivetrain, shooter),
                Commands.sequence(
                    Commands.waitSeconds(0.8),
                    intake.raisePivotCommand(),
                    intake.agitateCommand().withTimeout(2) // TODO: have an agitate speed
                    )));

    return routine;
  }

  private static AutoRoutine outpostSideTwoCycle(
      AutoFactory autoFactory,
      SwerveSubsystem drivetrain,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      IntakeSubsystem intake) {
    // this is just the depot one cycle flipped along the field width
    AutoRoutine routine = depotSideTwoCycle(autoFactory, drivetrain, shooter, indexer, intake);
    routine.active().whileTrue(drivetrain.driveFlippedAlongFieldWidth());
    return routine;
  }
}
