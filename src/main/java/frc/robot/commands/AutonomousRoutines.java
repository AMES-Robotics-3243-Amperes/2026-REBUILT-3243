package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.choreo.ChoreoTraj;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.FuelTrajectoryCalculator.ShootTarget;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

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

    CommandScheduler.getInstance().schedule(autoFactory.warmupCmd());

       LoggedNetworkNumber returnTimeChooserSeconds =
        new LoggedNetworkNumber("Return Time Seconds", 10.0);

    autoChooser.addRoutine(
        "depot side two cycle",
        () -> depotSideTwoCycle(autoFactory, drivetrain, shooter, indexer, intake));
    autoChooser.addRoutine(
        "outpost side two cycle",
        () -> outpostSideTwoCycle(autoFactory, drivetrain, shooter, indexer, intake));

                Time returnTimeInSinglePath = Seconds.of(3.85);
 autoChooser.addRoutine(
        "depot side single",
        () ->
            follow(
                autoFactory,
                drivetrain,
                shooter,
                indexer,
                intake,
                returnTimeInSinglePath,
                returnTimeChooserSeconds,
                false));
    autoChooser.addRoutine(
        "outpost side single",
        () ->
            follow(
                autoFactory,
                drivetrain,
                shooter,
                indexer,
                intake,
                returnTimeInSinglePath,
                returnTimeChooserSeconds,
                true));

    Time returnTimeInFollowPath = Seconds.of(5.0);
    autoChooser.addRoutine(
        "depot side follow",
        () ->
            follow(
                autoFactory,
                drivetrain,
                shooter,
                indexer,
                intake,
                returnTimeInFollowPath,
                returnTimeChooserSeconds,
                false));
    autoChooser.addRoutine(
        "outpost side follow",
        () ->
            follow(
                autoFactory,
                drivetrain,
                shooter,
                indexer,
                intake,
                returnTimeInFollowPath,
                returnTimeChooserSeconds,
                true));

    autoChooser.addRoutine(
        "center depot cycle",
        () -> depotCollect(autoFactory, drivetrain, shooter, indexer, intake));
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
  // Depot
  //

  private static AutoRoutine depotCollect(
      AutoFactory autoFactory,
      SwerveSubsystem drivetrain,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      IntakeSubsystem intake) {
    AutoRoutine routine = autoFactory.newRoutine("depot");

    AutoTrajectory collect = ChoreoTraj.CollectDepot.asAutoTraj(routine);
    routine.active().onTrue(collect.resetOdometry().andThen(collect.cmd()));

    collect.atTime(0.7).onTrue(intake.intakeCommand().until(collect.active().negate()));

    collect
        .done()
        .onTrue(
            ShootCommands.newBuilder(ShootTarget.HUB)
                .rotateDrivetrain(drivetrain)
                .runShooter(shooter)
                .indexWhenReady(indexer, shooter, drivetrain)
                .automaticallyAgitate(intake)
                .build()
                .withTimeout(Seconds.of(6)));

    return routine;
  }

  //
  // Follow
  //

  private static AutoRoutine follow(
      AutoFactory autoFactory,
      SwerveSubsystem drivetrain,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      IntakeSubsystem intake,
      Time returnTimeInpath,
      LoggedNetworkNumber returnTimeChooserSeconds,
      boolean reflectY) {
    AutoRoutine routine = autoFactory.newRoutine("follow");

    AutoTrajectory goIntoMiddle = ChoreoTraj.FollowIntoCenter$0.asAutoTraj(routine);
    AutoTrajectory returnFromMiddle = ChoreoTraj.FollowIntoCenter$1.asAutoTraj(routine);

    if (reflectY) {
      goIntoMiddle = goIntoMiddle.mirrorY();
      returnFromMiddle = returnFromMiddle.mirrorY();
    }

    registerSingleCycle(
        routine,
        goIntoMiddle,
        returnFromMiddle,
        Seconds.of(0.1),
        Seconds.of(9),
        drivetrain,
        shooter,
        indexer,
        intake);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                goIntoMiddle.resetOdometry(),
                Commands.deferredProxy(
                    () ->
                        Commands.waitSeconds(
                            returnTimeChooserSeconds.getAsDouble() - returnTimeInpath.in(Seconds))),
                goIntoMiddle.cmd()));

    return routine;
  }

  //
  // Single Cycle
  //

    private static AutoRoutine singleCycle(
      AutoFactory autoFactory,
      SwerveSubsystem drivetrain,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      IntakeSubsystem intake,
      Time returnTimeInpath,
      LoggedNetworkNumber returnTimeChooserSeconds,
      boolean reflectY) {
    AutoRoutine routine = autoFactory.newRoutine("single cycle");

    AutoTrajectory goIntoMiddle = ChoreoTraj.FirstCenterCollect$0.asAutoTraj(routine);
    AutoTrajectory returnFromMiddle = ChoreoTraj.FirstCenterCollect$1.asAutoTraj(routine);

    if (reflectY) {
      goIntoMiddle = goIntoMiddle.mirrorY();
      returnFromMiddle = returnFromMiddle.mirrorY();
    }

    registerSingleCycle(
        routine,
        goIntoMiddle,
        returnFromMiddle,
        Seconds.of(0.1),
        Seconds.of(9),
        drivetrain,
        shooter,
        indexer,
        intake);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                goIntoMiddle.resetOdometry(),
                Commands.deferredProxy(
                    () ->
                        Commands.waitSeconds(
                            returnTimeChooserSeconds.getAsDouble() - returnTimeInpath.in(Seconds))),
                goIntoMiddle.cmd()));

    return routine;
  }

  //
  // Double Cycle
  //

  private static AutoRoutine twoCycle(
      AutoFactory autoFactory,
      SwerveSubsystem drivetrain,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      IntakeSubsystem intake,
      boolean reflectY) {
    AutoRoutine routine = autoFactory.newRoutine("two cycle");

    AutoTrajectory firstCollectFromMiddle = ChoreoTraj.FirstCenterCollect$0.asAutoTraj(routine);
    AutoTrajectory firstReturnToShoot = ChoreoTraj.FirstCenterCollect$1.asAutoTraj(routine);

    AutoTrajectory secondCollectFromMiddle = ChoreoTraj.SecondCenterCollect$0.asAutoTraj(routine);
    AutoTrajectory secondReturnToShoot = ChoreoTraj.SecondCenterCollect$1.asAutoTraj(routine);

    if (reflectY) {
      firstCollectFromMiddle = firstCollectFromMiddle.mirrorY();
      firstReturnToShoot = firstReturnToShoot.mirrorY();

      secondCollectFromMiddle = secondCollectFromMiddle.mirrorY();
      secondReturnToShoot = secondReturnToShoot.mirrorY();
    }

    Trigger firstCycleDone =
        registerSingleCycle(
            routine,
            firstCollectFromMiddle,
            firstReturnToShoot,
            Seconds.of(0.1),
            Seconds.of(5),
            drivetrain,
            shooter,
            indexer,
            intake);
    routine
        .active()
        .onTrue(firstCollectFromMiddle.resetOdometry().andThen(firstCollectFromMiddle.cmd()));

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

  private static AutoRoutine depotSideTwoCycle(
      AutoFactory autoFactory,
      SwerveSubsystem drivetrain,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      IntakeSubsystem intake) {
    return twoCycle(autoFactory, drivetrain, shooter, indexer, intake, false);
  }

  private static AutoRoutine outpostSideTwoCycle(
      AutoFactory autoFactory,
      SwerveSubsystem drivetrain,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      IntakeSubsystem intake) {
    // this is just the depot one cycle flipped along the field width
    AutoRoutine routine = twoCycle(autoFactory, drivetrain, shooter, indexer, intake, true);
    return routine;
  }
}
