package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.choreo.ChoreoTraj;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.FuelTrajectoryCalculator.ShootTarget;
import frc.robot.util.PointOfInterestManager;
import frc.robot.util.PointOfInterestManager.FlipType;

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
        "depot side two cycle",
        () -> depotSideTwoCycle(autoFactory, drivetrain, shooter, indexer, intake));
    autoChooser.addRoutine(
        "outpost side two cycle",
        () -> outpostSideTwoCycle(autoFactory, drivetrain, shooter, indexer, intake));
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
  // Helper Commands
  //

  public static Command optionallyResetOdometry(SwerveSubsystem swerve, Pose2d resetTo) {
    double distanceForReset = Units.inchesToMeters(8);
    Angle rotationForReset = Degrees.of(20);

    return Commands.runOnce(
        () -> {
          Pose2d pose = swerve.getPose();

          if (pose.getTranslation().getDistance(resetTo.getTranslation()) < distanceForReset
              && Math.abs(
                      MathUtil.angleModulus(
                          pose.getRotation().getRadians() - resetTo.getRotation().getRadians()))
                  < rotationForReset.in(Radians)) {
            return;
          }

          swerve.setPose(resetTo);
        },
        swerve);
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
    routine.active().onTrue(collect.cmd().beforeStarting(collect.resetOdometry()));
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
  // Double Cycle
  //

  private static AutoRoutine twoCycle(
      AutoFactory autoFactory,
      SwerveSubsystem drivetrain,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      IntakeSubsystem intake,
      Pose2d odometryResetPoseBlue) {
    AutoRoutine routine = autoFactory.newRoutine("two cycle");

    AutoTrajectory firstCollectFromMiddle = ChoreoTraj.FirstCenterCollect$0.asAutoTraj(routine);
    AutoTrajectory firstReturnToShoot = ChoreoTraj.FirstCenterCollect$1.asAutoTraj(routine);

    AutoTrajectory secondCollectFromMiddle = ChoreoTraj.SecondCenterCollect$0.asAutoTraj(routine);
    AutoTrajectory secondReturnToShoot = ChoreoTraj.SecondCenterCollect$1.asAutoTraj(routine);

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
        .onTrue(
            firstCollectFromMiddle
                .cmd()
                .beforeStarting(
                    optionallyResetOdometry(
                        drivetrain,
                        PointOfInterestManager.flipPoseConditionally(
                            odometryResetPoseBlue, FlipType.REFLECT_FOR_OTHER_ALLIANCE))));

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
    return twoCycle(
        autoFactory,
        drivetrain,
        shooter,
        indexer,
        intake,
        ChoreoTraj.FirstCenterCollect.initialPoseBlue());
  }

  private static AutoRoutine outpostSideTwoCycle(
      AutoFactory autoFactory,
      SwerveSubsystem drivetrain,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      IntakeSubsystem intake) {
    // this is just the depot one cycle flipped along the field width
    AutoRoutine routine =
        twoCycle(
            autoFactory,
            drivetrain,
            shooter,
            indexer,
            intake,
            PointOfInterestManager.flipPose(
                ChoreoTraj.FirstCenterCollect.initialPoseBlue(), FlipType.REFLECT_Y));
    routine.active().whileTrue(drivetrain.driveFlippedAlongFieldWidth());
    return routine;
  }
}
