// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveUnderTrenchCommand;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ModeConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.VisionConstants.CameraType;
import frc.robot.constants.choreo.ChoreoVars;
import frc.robot.constants.swerve.ModuleConstants;
import frc.robot.constants.swerve.SwerveConstants;
import frc.robot.constants.swerve.TunerConstants;
import frc.robot.subsystems.drivetrain.GyroIO;
import frc.robot.subsystems.drivetrain.GyroIOPigeon2;
import frc.robot.subsystems.drivetrain.GyroIOSim;
import frc.robot.subsystems.drivetrain.ModuleIO;
import frc.robot.subsystems.drivetrain.ModuleIOTalonFXReal;
import frc.robot.subsystems.drivetrain.ModuleIOTalonFXSim;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.indexer.KickerIO;
import frc.robot.subsystems.indexer.KickerIOReal;
import frc.robot.subsystems.indexer.KickerIOSim;
import frc.robot.subsystems.indexer.SpindexerIO;
import frc.robot.subsystems.indexer.SpindexerIOReal;
import frc.robot.subsystems.indexer.SpindexerIOSim;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.PivotIO;
import frc.robot.subsystems.intake.PivotIOReal;
import frc.robot.subsystems.intake.PivotIOSim;
import frc.robot.subsystems.intake.RollerIO;
import frc.robot.subsystems.intake.RollerIOReal;
import frc.robot.subsystems.intake.RollerIOSim;
import frc.robot.subsystems.shooter.FlywheelIO;
import frc.robot.subsystems.shooter.FlywheelIOReal;
import frc.robot.subsystems.shooter.FlywheelIOSim;
import frc.robot.subsystems.shooter.HoodIO;
import frc.robot.subsystems.shooter.HoodIOReal;
import frc.robot.subsystems.shooter.HoodIOSim;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOLimelightFour;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.FuelTrajectoryCalculator;
import frc.robot.util.RobotLocationManager;
import frc.robot.util.TunableControls;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  //
  // JOYSTICKS, SUBSYSTEMS, CONSTRUCTION, AND SIMULATION
  //

  public final CommandXboxController primaryController = new CommandXboxController(0);
  public final CommandXboxController secondaryController = new CommandXboxController(1);

  public final SwerveSubsystem drivetrain;
  public final IntakeSubsystem intake;
  public final IndexerSubsystem indexer;
  public final ShooterSubsystem shooter;

  private final SwerveDriveSimulation driveSimulation;
  private final LoggedDashboardChooser<Command> autoChooser;
  private final RobotLocationManager robotLocationManager;

  public RobotContainer() {
    switch (ModeConstants.robotMode) {
      case REAL_COMPETITION:
      case REAL:
        driveSimulation = null;

        ModuleIOTalonFXReal fl = new ModuleIOTalonFXReal(TunerConstants.FrontLeft);
        ModuleIOTalonFXReal fr = new ModuleIOTalonFXReal(TunerConstants.FrontRight);
        ModuleIOTalonFXReal bl = new ModuleIOTalonFXReal(TunerConstants.BackLeft);
        ModuleIOTalonFXReal br = new ModuleIOTalonFXReal(TunerConstants.BackRight);

        TunableControls.registerTalonFXSlotTuning(
            0,
            "Drivetrain/Drive",
            ModuleConstants.driveControl,
            fl.driveTalon,
            fr.driveTalon,
            bl.driveTalon,
            br.driveTalon);

        TunableControls.registerTalonFXSlotTuning(
            0,
            "Drivetrain/Azimuth",
            ModuleConstants.steerControl,
            fl.turnTalon,
            fr.turnTalon,
            bl.turnTalon,
            br.turnTalon);

        drivetrain = new SwerveSubsystem(new GyroIOPigeon2(), fl, fr, bl, br);
        intake = new IntakeSubsystem(new RollerIOReal(), new PivotIOReal());
        shooter = new ShooterSubsystem(new FlywheelIOReal(), new HoodIOReal());
        indexer = new IndexerSubsystem(new KickerIOReal(), new SpindexerIOReal());

        new VisionSubsystem(
            drivetrain::addVisionMeasurement,
            drivetrain::getChassisSpeeds,
            VisionConstants.cameras.stream()
                .<VisionIO>map(
                    config ->
                        config.type() == CameraType.LimelightFour
                            ? new VisionIOLimelightFour(
                                config,
                                drivetrain::getRotation,
                                () ->
                                    RadiansPerSecond.of(
                                        drivetrain.getChassisSpeeds().omegaRadiansPerSecond))
                            : new VisionIOLimelight(
                                config,
                                drivetrain::getRotation,
                                () ->
                                    RadiansPerSecond.of(
                                        drivetrain.getChassisSpeeds().omegaRadiansPerSecond)))
                .toList());

        break;

      case SIM:
        SimulatedArena.getInstance().placeGamePiecesOnField();

        driveSimulation =
            new SwerveDriveSimulation(
                SwerveSubsystem.mapleSimConfig, new Pose2d(2, 2, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

        drivetrain =
            new SwerveSubsystem(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOTalonFXSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                new ModuleIOTalonFXSim(TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                new ModuleIOTalonFXSim(TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                new ModuleIOTalonFXSim(TunerConstants.BackRight, driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);

        IntakeSimulation intakeSimulation =
            IntakeSimulation.OverTheBumperIntake(
                "Fuel",
                driveSimulation,
                ChoreoVars.R_BumperLength,
                ChoreoVars.R_DeployedHopperX.minus(ChoreoVars.R_BumperLength.div(2)),
                IntakeSimulation.IntakeSide.FRONT,
                40);
        intake =
            new IntakeSubsystem(
                new RollerIOSim(intakeSimulation, driveSimulation),
                new PivotIOSim(intakeSimulation));
        indexer = new IndexerSubsystem(new KickerIOSim(), new SpindexerIOSim());
        shooter = new ShooterSubsystem(new FlywheelIOSim(), new HoodIOSim());

        new VisionSubsystem(
            drivetrain::addVisionMeasurement,
            drivetrain::getChassisSpeeds,
            VisionConstants.cameras.stream()
                .<VisionIO>map(
                    config ->
                        new VisionIOPhotonVisionSim(
                            config, driveSimulation::getSimulatedDriveTrainPose))
                .toList());

        CommandScheduler.getInstance()
            .schedule(
                Commands.runOnce(
                        () -> {
                          if (indexer.getKickerVelocity().isEquivalent(RadiansPerSecond.of(0))
                              || indexer
                                  .getSpindexerVelocity()
                                  .isEquivalent(RadiansPerSecond.of(0))) return;

                          if (!intakeSimulation.obtainGamePieceFromIntake()) return;

                          RebuiltFuelOnFly fuel =
                              new RebuiltFuelOnFly(
                                  driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                                  ShooterConstants.robotToShooter
                                      .getTranslation()
                                      .toTranslation2d(),
                                  driveSimulation
                                      .getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                                  driveSimulation
                                      .getSimulatedDriveTrainPose()
                                      .getRotation()
                                      .plus(
                                          ShooterConstants.robotToShooter
                                              .getRotation()
                                              .toRotation2d()),
                                  ShooterConstants.robotToShooter.getMeasureZ(),
                                  shooter.getFuelVelocity(),
                                  Degrees.of(90).minus(shooter.getHoodAngle()));

                          SimulatedArena.getInstance().addGamePieceProjectile(fuel);
                        })
                    .andThen(Commands.waitSeconds(1.0 / 6.0))
                    .repeatedly()
                    .ignoringDisable(true));

        break;

      case REPLAY:
        driveSimulation = null;

        drivetrain =
            new SwerveSubsystem(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        intake = new IntakeSubsystem(new RollerIO() {}, new PivotIO() {});
        indexer = new IndexerSubsystem(new KickerIO() {}, new SpindexerIO() {});
        shooter = new ShooterSubsystem(new FlywheelIO() {}, new HoodIO() {});

        new VisionSubsystem(
            drivetrain::addVisionMeasurement,
            drivetrain::getChassisSpeeds,
            VisionConstants.cameras.stream()
                .<VisionIO>map(config -> new VisionIO(config) {})
                .toList());

        break;

      default:
        throw new RuntimeException("Invalid robot mode");
    }

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    robotLocationManager = new RobotLocationManager(() -> drivetrain.getPose().getTranslation());

    FuelTrajectoryCalculator.robotPose = drivetrain::getPose;
    FuelTrajectoryCalculator.robotSpeeds = drivetrain::getChassisSpeeds;

    configureBindings();
  }

  //
  // ===== BINDINGS =====
  //

  private void configureBindings() {
    //
    // State management
    //
    secondaryController.a().whileTrue(robotLocationManager.setLocationAsAllianceZone());
    secondaryController
        .x()
        .or(secondaryController.b())
        .whileTrue(robotLocationManager.setLocationAsNeutralZone());
    secondaryController.y().whileTrue(robotLocationManager.setLocationAsOpponentZone());

    //
    // Misc binds
    //
    drivetrain.setDefaultCommand(
        drivetrain.driveSetpiontGeneratorCommand(
            drivetrain.joystickDriveLinear(SwerveConstants.linearTeleopSpeed, primaryController),
            drivetrain.joystickDriveAngular(primaryController::getRightX)));

    // shooter.setDefaultCommand(
    //     Commands.waitTime(ShooterConstants.timeCoastingBeforeIdle)
    //         .andThen(
    //             shooter.shootCommand(
    //                 ShooterConstants.idleFlywheelSpeed, ShooterConstants.hoodMinRotation)));

    primaryController
        .rightBumper()
        .whileTrue(
            DriveUnderTrenchCommand.driveUnderNearestTrenchCommand(
                drivetrain::getPose,
                drivetrain::getChassisSpeeds,
                robotLocationManager::robotLocation));

    primaryController
        .leftTrigger()
        .whileTrue(intake.intakeAtSpeedCommand(IntakeConstants.rollerIntakeSpeed));

    secondaryController
        .leftTrigger()
        .whileTrue(intake.intakeAtSpeedCommand(IntakeConstants.rollerIntakeSpeed));

    //
    // Shooting
    //
    primaryController
        .rightTrigger()
        .and(robotLocationManager.inAllianceZone())
        .whileTrue(
            Commands.parallel(
                shooter.shootInHubCommand(),
                drivetrain.driveSetpiontGeneratorCommand(
                    drivetrain.joystickDriveLinear(
                        SwerveConstants.linearTeleopSpeedWhileShooting, primaryController),
                    drivetrain.rotateAtAngle(
                        () -> FuelTrajectoryCalculator.getHubShot().fuelGroundSpeedRotation()))));

    primaryController
        .rightTrigger()
        .and(robotLocationManager.innNeutralZone())
        .whileTrue(
            Commands.parallel(
                shooter.shootInAllianceZoneCommand(),
                drivetrain.driveSetpiontGeneratorCommand(
                    drivetrain.joystickDriveLinear(
                        SwerveConstants.linearTeleopSpeedWhileShooting, primaryController),
                    drivetrain.rotateAtAngle(
                        () ->
                            FuelTrajectoryCalculator.getAllianceShot()
                                .fuelGroundSpeedRotation()))));

    primaryController
        .rightTrigger()
        .and(robotLocationManager.inOpponentZone())
        .whileTrue(
            Commands.parallel(
                shooter.shootInNeutralZoneCommand(),
                drivetrain.driveSetpiontGeneratorCommand(
                    drivetrain.joystickDriveLinear(
                        SwerveConstants.linearTeleopSpeedWhileShooting, primaryController),
                    drivetrain.rotateAtAngle(
                        () ->
                            FuelTrajectoryCalculator.getNeutralShot().fuelGroundSpeedRotation()))));

    primaryController
        .rightTrigger()
        .whileTrue(
            indexer
                .indexCommand()
                // .onlyIf(drivetrain::atRotationSetpoint)
                .onlyIf(shooter::flywheelSpunUp)
                .repeatedly());

    primaryController.a().onTrue(intake.pivotSysIdCommand(primaryController.a()));
  }

  //
  // ===== MISC =====
  //

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void updateSimulation() {
    if (ModeConstants.robotMode != ModeConstants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
  }

  public void updateComponents() {
    // TODO: replace sim cad with something that has not so terrible offsets
    Logger.recordOutput(
        "ComponentPoses",
        new Pose3d[] {
          new Pose3d(
              Units.inchesToMeters(4.206652 + 7.266 - 0.1),
              0,
              Units.inchesToMeters(17.735146 - 6.980000 - 1.166046 - 0.21),
              new Rotation3d(0, intake.getPivotAngle().times(-1).in(Radians), 0)),
          new Pose3d(
              ChoreoVars.R_DeployedHopperX.in(Meters)
                  + (ChoreoVars.R_DeployedHopperX.minus(ChoreoVars.R_BumperLength.div(2)).in(Meters)
                      * Math.cos(intake.getPivotAngle().times(-1).in(Radians) + 0.15))
                  - 0.5,
              0,
              0,
              new Rotation3d()),
          new Pose3d(
              Units.inchesToMeters(-4.121261 - 0.21),
              0,
              Units.inchesToMeters(18.062500 - 0.27),
              new Rotation3d(0, shooter.getHoodAngle().in(Radians), 0))
        });
  }
}
