package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
        "depog side one cycle",
        () -> depotSideOneCycle(autoFactory, drivetrain, shooter, indexer, intake));
    autoChooser.addRoutine(
        "outpost side one cycle",
        () -> outpostSideOneCycle(autoFactory, drivetrain, shooter, indexer, intake));

    autoChooser.addRoutine(
        "depot side two cycle",
        () -> depotSideTwoCycle(autoFactory, drivetrain, shooter, indexer, intake));
    autoChooser.addRoutine(
        "outpost side two cycle",
        () -> outpostSideTwoCycle(autoFactory, drivetrain, shooter, indexer, intake));
  }

  /**
   * Adds a single collect/shoot cycle to an auto. The returned trigger fires once when the cycle is
   * over.
   */
  private static Trigger registerSingleCycle(
      AutoRoutine routine,
      AutoTrajectory collectTrajectory,
      AutoTrajectory returnTrajectory,
      Time timeIntoPathBeforeIntaking,
      Time shootTime,
      SwerveSubsystem drivetrain,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      IntakeSubsystem intake) {
    // collect while intaking
    collectTrajectory
        .atTime(timeIntoPathBeforeIntaking.in(Seconds))
        .onTrue(intake.intakeCommand().until(collectTrajectory.active().negate()));
    collectTrajectory.chain(returnTrajectory);

    // while returning, intake a little longer to collect the rest of the fuel. spin up flywheel
    // when approaching shoot position
    collectTrajectory.doneFor(1).whileTrue(intake.intakeCommand());
    returnTrajectory
        .active()
        .onTrue(
            shooter
                .spinUpFlywheelCommand(ShootTarget.HUB)
                .until(returnTrajectory.active().negate()));

    // finally, shoot
    returnTrajectory
        .done()
        .onTrue(
            ShootCommands.newBuilder(ShootTarget.HUB)
                .rotateDrivetrain(drivetrain)
                .runShooter(shooter)
                .indexWhenReady(indexer, shooter, drivetrain)
                .automaticallyAgitate(intake)
                .build()
                .withTimeout(shootTime));

    return returnTrajectory.doneDelayed(shootTime.in(Seconds));
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

    registerSingleCycle(
        routine,
        collectFromMiddle,
        returnToShoot,
        Seconds.of(0.6),
        Seconds.of(8),
        drivetrain,
        shooter,
        indexer,
        intake);
    routine.active().onTrue(collectFromMiddle.cmd());

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

    Trigger firstCycleDone =
        registerSingleCycle(
            routine,
            firstCollectFromMiddle,
            firstReturnToShoot,
            Seconds.of(0.6),
            Seconds.of(3.5),
            drivetrain,
            shooter,
            indexer,
            intake);
    routine.active().onTrue(firstCollectFromMiddle.cmd());

    registerSingleCycle(
        routine,
        secondCollectFromMiddle,
        secondReturnToShoot,
        Seconds.of(0.1),
        Seconds.of(8),
        drivetrain,
        shooter,
        indexer,
        intake);
    firstCycleDone.onTrue(secondCollectFromMiddle.cmd());

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
