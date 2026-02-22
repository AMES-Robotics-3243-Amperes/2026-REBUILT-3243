// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.choreo.ChoreoVars;
import frc.robot.constants.swerve.SwerveConstants;
import frc.robot.subsystems.Indexer.IndexerSubsystem;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.PointOfInterestManager;
import java.util.Map;

public class HoodAngleCharacterization extends Command {
  private double minDistanceFromHub =
      Units.inchesToMeters(158.34) + ChoreoVars.R_BumperLength.in(Inches) + 2;
  private final double distanceStep = 0.5;

  private final Angle bigAngleStep = Degrees.of(3);
  private final Angle smallAngleStep = Degrees.of(0.2);

  private final SwerveSubsystem drivetrain;
  private final ShooterSubsystem shooter;
  private final IndexerSubsystem indexer;

  private final PIDController xController =
      new PIDController(
          SwerveConstants.driveControl.kP,
          SwerveConstants.driveControl.kI,
          SwerveConstants.driveControl.kD);
  private final PIDController yControler =
      new PIDController(
          SwerveConstants.driveControl.kP,
          SwerveConstants.driveControl.kI,
          SwerveConstants.driveControl.kD);
  private final ProfiledPIDController thetaControllerRadians =
      SwerveConstants.rotationControl.profiledPIDController(Radians);

  private int step = 0;
  private Angle targetHoodAngle = ShooterConstants.hoodMinRotation;

  private Map<Integer, Angle> distancesToHoodAngles = Map.of();

  Trigger shoot;
  Trigger nextStep;
  Trigger previousStep;

  public HoodAngleCharacterization(
      SwerveSubsystem drivetrain,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      CommandXboxController joystick) {
    this(
        drivetrain,
        shooter,
        indexer,
        joystick,
        joystick.povUp(),
        joystick.povDown(),
        joystick.povRight(),
        joystick.povLeft(),
        joystick.rightTrigger(),
        joystick.rightBumper(),
        joystick.leftBumper());
  }

  public HoodAngleCharacterization(
      SwerveSubsystem drivetrain,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      CommandXboxController joystick,
      Trigger incrementAngleBig,
      Trigger decrementAngleBig,
      Trigger incrementAngleSmall,
      Trigger decrementAngleSmall,
      Trigger shoot,
      Trigger nextStep,
      Trigger previousStep) {
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.indexer = indexer;

    this.shoot = shoot;
    this.nextStep = nextStep;
    this.previousStep = previousStep;

    new Trigger(() -> Math.hypot(joystick.getLeftX(), joystick.getLeftY()) > 0.5)
        .and(this::isScheduled)
        .onTrue(Commands.runOnce(this::cancel));

    incrementAngleBig
        .and(this::isScheduled)
        .onTrue(Commands.runOnce(() -> targetHoodAngle = targetHoodAngle.plus(bigAngleStep)));
    decrementAngleBig
        .and(this::isScheduled)
        .onTrue(Commands.runOnce(() -> targetHoodAngle = targetHoodAngle.minus(bigAngleStep)));

    incrementAngleSmall
        .and(this::isScheduled)
        .onTrue(Commands.runOnce(() -> targetHoodAngle = targetHoodAngle.plus(smallAngleStep)));
    decrementAngleSmall
        .and(this::isScheduled)
        .onTrue(Commands.runOnce(() -> targetHoodAngle = targetHoodAngle.minus(smallAngleStep)));

    nextStep
        .and(this::isScheduled)
        .onTrue(
            Commands.runOnce(
                () -> {
                  recordData();
                  step++;
                }));
    previousStep
        .and(this::isScheduled)
        .and(() -> step != 0)
        .onTrue(
            Commands.runOnce(
                () -> {
                  recordData();
                  step--;
                }));

    shoot
        .and(this::isScheduled)
        .whileTrue(
            Commands.parallel(
                Commands.runEnd(
                    () ->
                        shooter.runFlywheelWithHood(
                            ShooterConstants.flywheelShootSpeed, targetHoodAngle),
                    shooter::reset),
                Commands.sequence(
                    Commands.waitSeconds(1.5),
                    Commands.runEnd(
                        () -> {
                          indexer.setKickerVelocity(IndexerConstants.kickerShootingSpeed);
                          indexer.setSpindexerVelocity(IndexerConstants.spindexerIndexingSpeed);
                        },
                        () -> {
                          indexer.coastKicker();
                          indexer.coastSpindexer();
                        }))));

    addRequirements(drivetrain, shooter, indexer);
  }

  private void recordData() {
    distancesToHoodAngles.put(step, bigAngleStep);
  }

  @Override
  public void initialize() {
    Pose2d robotPose = drivetrain.getPose();
    xController.reset();
    yControler.reset();
    thetaControllerRadians.reset(robotPose.getRotation().getRadians());
  }

  @Override
  public void execute() {
    Pose2d poseTarget =
        new Pose2d(
            FieldConstants.hubPosition.getX() - minDistanceFromHub - step * distanceStep,
            FieldConstants.hubPosition.getY(),
            Rotation2d.kZero);
    poseTarget = PointOfInterestManager.flipPoseConditionally(poseTarget);

    Pose2d robotPose = drivetrain.getPose();

    if (robotPose.getTranslation().getDistance(poseTarget.getTranslation()) > 0.3) {
      CommandScheduler.getInstance()
          .schedule(
              AutoBuilder.pathfindToPose(
                  poseTarget,
                  new PathConstraints(
                      MetersPerSecond.of(1),
                      MetersPerSecondPerSecond.of(4),
                      RotationsPerSecond.of(0.5),
                      RotationsPerSecondPerSecond.of(2))));
    }

    // TODO: if I end up using drive to point a lot I should just make it a drive strategy
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            xController.calculate(robotPose.getX(), poseTarget.getX()),
            yControler.calculate(robotPose.getY(), poseTarget.getY()),
            thetaControllerRadians.calculate(
                robotPose.getRotation().getRadians(), poseTarget.getRotation().getRadians()));
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, robotPose.getRotation());
    drivetrain.driveSetpointGenerator(speeds);
  }

  @Override
  public void end(boolean interrupted) {
    String csvString = "====================\ndistance,angle_radians\n";
    for (Map.Entry<Integer, Angle> entry : distancesToHoodAngles.entrySet()) {
      csvString +=
          Double.toString(entry.getKey() * distanceStep + minDistanceFromHub)
              + ","
              + Double.toString(entry.getValue().in(Radians))
              + "\n";
    }

    System.out.println(csvString);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return InterruptionBehavior.kCancelIncoming;
  }
}
